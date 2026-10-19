"""Gazebo task scenes as an experimental environment: deterministic resets, ground truth as the verifier."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from lekiwi_rmf.action import RunSkill
from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf import sim_scene
from ros_test_utils import release_graph

SCRIPTS = Path(__file__).parents[1] / 'scripts'


def script(name):
    spec = importlib.util.spec_from_file_location(name.replace('-', '_'), SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


evaluate = script('evaluate-skill')
episodes = script('sim-episodes')


def scene():
    return sim_scene.load_scene('floor_block')


def test_the_tracked_scene_loads_and_trial_poses_are_a_function_of_seed_and_index():
    value, digest = scene()
    assert value['world'] == sim_scene.SIM_WORLD and len(digest) == 64
    first, again, other = (sim_scene.trial_poses(value, trial) for trial in (3, 3, 4))
    assert first == again and first != other
    block = value['objects']['block']
    for trial in range(1, 50):
        x, y, z, yaw = sim_scene.trial_poses(value, trial)['block']
        assert abs(x - block['pose'][0]) <= block['randomize']['x'] and abs(y - block['pose'][1]) <= block['randomize']['y']
        assert z == block['pose'][2] and abs(math.remainder(yaw - block['pose'][3], math.tau)) <= block['randomize']['yaw']
    with pytest.raises(ValueError, match='trials are numbered'):
        sim_scene.trial_poses(value, 0)
    sdf = sim_scene.object_sdf(value, 'block')
    assert '<model name="floor_block_block">' in sdf and '<mass>0.02</mass>' in sdf
    # The arm start is the SRDF state that every torque cut also requires.
    assert set(sim_scene.group_state(value['robot']['arm_start'])) <= set(ARM_JOINTS)


def test_a_scene_cannot_verify_a_variable_it_does_not_model(tmp_path):
    value, _ = scene()
    with pytest.raises(ValueError, match='does not model'):
        sim_scene.sim_predicate('{"object": "block", "min_z": 0.1}', {**value, 'modeled': ['thermal']})
    for bad in ('{"object": "ghost", "min_z": 0}', '{"object": "block"}', '{"object": "block", "region": [1, 0, 0, 1]}',
                '{"object": "block", "colour": "red"}'):
        with pytest.raises(ValueError):
            sim_scene.sim_predicate(bad, value)
    document = {'scenes': {'other': {**copy.deepcopy(value), 'world': 'elsewhere'}}}
    del document['scenes']['other']['id']
    path = tmp_path / 'scenes.yaml'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='launched world'):
        sim_scene.load_scene('other', path)


def test_ground_truth_predicates_and_reset_confirmation():
    value, _ = scene()
    predicate = sim_scene.sim_predicate('{"object": "block", "min_z": 0.05, "min_displacement_m": 0.02}', value)
    start = [-3.72, -2.5, 0.015, 0.0]
    assert not sim_scene.judge(predicate, start, start)[0]
    holds, evidence = sim_scene.judge(predicate, [-3.72, -2.5, 0.08, 0.0], start)
    assert holds and evidence['checks'] == {'above_min_z': True, 'displaced': True}
    assert sim_scene.judge(predicate, None, start) == (False, {'reason': 'the object is absent from ground truth'})
    commanded = sim_scene.trial_poses(value, 1)
    stow = sim_scene.group_state(value['robot']['arm_start'])
    robot = [*value['robot']['start'][:2], 0.0, value['robot']['start'][2]]
    assert sim_scene.confirm(value, commanded, commanded, robot, stow) == []
    moved = {'block': [commanded['block'][0] + .1, *commanded['block'][1:]]}
    reasons = sim_scene.confirm(value, commanded, moved, [0, 0, 0, 0], {**stow, next(iter(stow)): 1.0})
    assert any('settled' in reason for reason in reasons) and any('robot base' in reason for reason in reasons)
    assert any('arm joint' in reason for reason in reasons)


class FakeController:
    """The scene as ground truth reports it; the runner changes it, the evaluator only reads it."""

    def __init__(self, scene_value):
        self.scene, self.poses, self.confirmed, self.resets = scene_value, {}, True, []

    def ready(self):
        pass

    def reset(self, trial):
        self.resets.append(trial)
        self.poses = {'block': sim_scene.trial_poses(self.scene, trial)['block']}
        return {'trial': trial, 'confirmed': self.confirmed, 'reasons': [] if self.confirmed else ['block slid off']}

    def ground_truth(self):
        return dict(self.poses), None, {}, time.monotonic()


class FakeRunner(Node):
    def __init__(self, context, controller):
        super().__init__('fake_runner', context=context)
        self.controller, self.lifts, self.weights = controller, True, 'a' * 64
        ActionServer(self, RunSkill, '/robot/skills/run', self.execute, goal_callback=lambda _goal: GoalResponse.ACCEPT)

    def execute(self, handle):
        if self.lifts:
            x, y, _, yaw = self.controller.poses['block']
            self.controller.poses['block'] = [x, y, .1, yaw]
        handle.succeed()
        # The runner always claims it finished; only ground truth says whether the block moved.
        return RunSkill.Result(outcome='finished', result_json=json.dumps({'artifact_sha256': self.weights}))


@pytest.fixture
def sim_world():
    context = Context()
    rclpy.init(context=context)
    value, digest = scene()
    controller = FakeController(value)
    runner, client = FakeRunner(context, controller), Node('skill_evaluator', context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(runner)
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    skill = SimpleNamespace(skill_id='learned', version=1, artifact_sha256='a' * 64, status='experimental',
                            max_duration_sec=5.0)
    predicate = sim_scene.sim_predicate('{"object": "block", "min_z": 0.05}', value)
    evaluator = evaluate.SimEvaluator(client, skill, predicate, controller, settle_sec=0.0, timeout_sec=5.0)
    args = SimpleNamespace(trials=2, parameters={}, confidence=.95, required_lower_bound=.1, min_trials=1,
                           reset_command=None, observe_sec=1.0, settle_sec=0.0, timeout_sec=5.0)
    try:
        evaluator.ready()
        yield controller, runner, evaluator, evaluate.protocol_of(args, predicate, (value, digest))
    finally:
        release_graph(executor=executor, thread=thread, nodes=(runner, client), context=context)


def test_sim_trials_are_judged_by_ground_truth_and_labeled_as_sim(sim_world):
    controller, runner, evaluator, protocol = sim_world
    report = evaluate.evaluate(evaluator, protocol, 'sim-eval', log=lambda _line: None)
    assert [trial['verdict'] for trial in report['trials']] == ['success', 'success'] and report['passed']
    assert report['domain'] == 'sim' and report['promotable'] is False and controller.resets == [1, 2]
    assert report['verifier']['source'] == 'sim_ground_truth' and report['protocol']['sim_scene']['id'] == 'floor_block'
    # A run the runner calls finished, without the material result, is a failure (reward hacking, T-58).
    runner.lifts = False
    report = evaluate.evaluate(evaluator, protocol, 'sim-eval-2', log=lambda _line: None)
    assert all(trial['runner']['outcome'] == 'finished' and trial['verdict'] == 'failure' for trial in report['trials'])
    assert report['successes'] == 0 and not report['passed']


def test_an_unconfirmed_reset_is_never_a_valid_trial(sim_world):
    controller, runner, evaluator, protocol = sim_world
    controller.confirmed = False
    report = evaluate.evaluate(evaluator, protocol, 'sim-eval-reset', log=lambda _line: None)
    assert all(trial['verdict'] == 'invalid' and 'reset not confirmed' in trial['reason'] for trial in report['trials'])
    assert 'runner' not in report['trials'][0]  # the skill never ran from an unknown start
    assert report['invalid'] == 2 and report['successes'] == 0 and not report['passed']


def test_scripted_programs_are_validated_data(tmp_path):
    path = tmp_path / 'program.json'
    good = {'waypoints': [{'positions': {'arm_shoulder_pan': 0.1}, 'time_from_start_sec': 1.0},
                          {'positions': {'arm_shoulder_pan': 0.2}, 'time_from_start_sec': 2.0}]}
    path.write_text(json.dumps(good))
    assert episodes.load_program(path)[0] == good
    for bad in ({'waypoints': []}, {'waypoints': [{'positions': {'arm_shoulder_pan': 99.0}, 'time_from_start_sec': 1}]},
                {'waypoints': [{'positions': {'base_x': 0.1}, 'time_from_start_sec': 1}]},
                {'waypoints': [good['waypoints'][1], good['waypoints'][0]]}):
        path.write_text(json.dumps(bad))
        with pytest.raises(ValueError):
            episodes.load_program(path)


def test_the_simulated_arm_publishes_its_applied_command_in_ros_units():
    from lekiwi_rmf.sim_arm_controller import SimArmController
    context = Context()
    rclpy.init(context=context)
    controller = SimArmController(context=context, parameter_overrides=[Parameter('permission_timeout', value=1.0)])
    peer = Node('applied_peer', context=context)
    received = []
    peer.create_subscription(JointState, '/hardware/applied_command', received.append, 10)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(controller)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 10
        while not received and time.monotonic() < deadline:
            time.sleep(.02)
        assert received and list(received[-1].name) == list(ARM_JOINTS) and set(received[-1].position) == {0.0}
        controller._publish_positions({'arm_elbow_flex': 0.4})
        count = len(received)
        while len(received) < count + 2 and time.monotonic() < deadline:
            time.sleep(.02)
        latest = dict(zip(received[-1].name, received[-1].position))
        assert latest['arm_elbow_flex'] == pytest.approx(0.4) and latest['arm_shoulder_pan'] == 0.0
    finally:
        release_graph(executor=executor, thread=thread, nodes=(controller, peer), context=context)
