"""Skill registry contract, rate-limited arm targets and the guarded receding-horizon runner."""

import hashlib
import json
import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import yaml

from lekiwi_rmf.action import RunSkill
from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS
from lekiwi_rmf.skills import arm_targets, artifact_digest, catalog, load_registry, skill_parameters, verify_artifact
from lekiwi_rmf.task_gateway import task_goal
from ros_test_utils import release_graph

ACTIONS = [*ARM_JOINTS, 'base_x_vel', 'base_y_vel', 'base_yaw_rate']


def entry(**changes):
    value = {'version': 1, 'description': 'push the object', 'status': 'qualified', 'kind': 'lerobot_policy',
             'checkpoint': 'push/1', 'artifact_sha256': 'a' * 64, 'action_names': ACTIONS, 'cameras': ['front'],
             'fps': 10, 'max_duration_sec': 2.0, 'max_joint_step_rad': .1,
             'parameters': {'force': {'type': 'number', 'min': 0, 'max': 1}}}
    value.update(changes)
    return value


def registry(tmp_path, skills):
    path = tmp_path / 'skills.yaml'
    path.write_text(yaml.safe_dump({'skills': skills}))
    return load_registry(path, tmp_path)


def test_tracked_registry_is_valid_and_empty_until_a_skill_is_acquired():
    assert load_registry() == {}


@pytest.mark.parametrize('change', [
    {'status': 'trusted'}, {'kind': 'script'}, {'checkpoint': '/etc/x'}, {'checkpoint': '../x'},
    {'artifact_sha256': 'A' * 64}, {'weights_sha256': 'a' * 64}, {'action_names': ['base_x_vel']}, {'action_names': ['elbow']},
    {'cameras': ['front', 'front']}, {'fps': 0}, {'max_duration_sec': 1000}, {'max_joint_step_rad': 1.0},
    {'parameters': {'Bad': {'type': 'number'}}}, {'parameters': {'x': {'type': 'object'}}}, {'extra': 1}])
def test_malformed_entries_reject_the_whole_registry(tmp_path, change):
    with pytest.raises(ValueError):
        registry(tmp_path, {'good': entry(), 'push': entry(**change)})


def test_experimental_skills_are_offered_only_to_an_acquisition_protocol(tmp_path):
    skills = registry(tmp_path, {'push': entry(), 'reach': entry(status='experimental')})
    assert [skill['skill_id'] for skill in catalog(skills)] == ['push']
    assert sorted(skill['skill_id'] for skill in catalog(skills, allow_experimental=True)) == ['push', 'reach']


def test_parameters_follow_the_declared_contract(tmp_path):
    skill = registry(tmp_path, {'push': entry()})['push']
    assert skill_parameters(skill, '{"force": 0.5}') == {'force': .5}
    assert skill_parameters(skill, '') == {}
    for bad in ['{"force": 2}', '{"force": true}', '{"speed": 1}', '[]', '{"force": NaN}', 'x' * 5000]:
        with pytest.raises(ValueError):
            skill_parameters(skill, bad)


def test_arm_targets_are_rate_limited_kept_in_limits_and_ignore_base(tmp_path):
    skill = registry(tmp_path, {'push': entry()})['push']
    measured = dict.fromkeys(ARM_JOINTS, 0.0)
    chunk = np.zeros((3, len(ACTIONS)))
    chunk[:, 0] = 1.0  # far target for the pan joint
    chunk[:, ACTIONS.index('base_x_vel')] = 5.0
    rows = arm_targets(skill, measured, chunk)
    assert [row['arm_shoulder_pan'] for row in rows] == pytest.approx([.1, .2, .3])
    assert all(set(row) == set(ARM_JOINTS) for row in rows)
    lower = JOINT_LIMITS['arm_elbow_flex'][0]
    edge = {**measured, 'arm_elbow_flex': lower + .01}
    chunk = np.zeros((1, len(ACTIONS)))
    chunk[0, 2] = lower - 5
    assert arm_targets(skill, edge, chunk)[0]['arm_elbow_flex'] == pytest.approx(lower)
    with pytest.raises(ValueError, match='non-finite'):
        arm_targets(skill, measured, np.full((1, len(ACTIONS)), math.nan))
    with pytest.raises(ValueError, match='malformed'):
        arm_targets(skill, measured, np.zeros((1, 3)))


def lerobot_checkpoint(tmp_path):
    """A LeRobot-shaped checkpoint registered under the digest of exactly these files."""
    directory = tmp_path / 'push/1'
    directory.mkdir(parents=True)
    directory.chmod(0o755)
    files = {'model.safetensors': b'weights', 'config.json': json.dumps({'type': 'act', 'chunk_size': 10}).encode(),
             'policy_preprocessor_step_3_normalizer_processor.safetensors': b'stats',
             'policy_preprocessor.json': json.dumps({'name': 'policy_preprocessor', 'steps': [
                 {'registry_name': 'normalizer_processor', 'config': {'eps': 1e-8},
                  'state_file': 'policy_preprocessor_step_3_normalizer_processor.safetensors'}]}).encode()}
    for name, data in files.items():
        (directory / name).write_bytes(data)
        (directory / name).chmod(0o644)
    return registry(tmp_path, {'push': entry(artifact_sha256=artifact_digest(directory))})['push']


def test_an_old_weights_only_entry_is_refused_rather_than_trusted(tmp_path):
    old = entry()
    old['weights_sha256'] = old.pop('artifact_sha256')
    with pytest.raises(ValueError, match='re-register for artifact_sha256'):
        registry(tmp_path, {'push': old})


@pytest.mark.parametrize('change', [
    lambda d: (d / 'model.safetensors').write_bytes(b'changed'),
    lambda d: (d / 'config.json').write_text('{"type": "act", "chunk_size": 100}'),
    lambda d: (d / 'policy_preprocessor_step_3_normalizer_processor.safetensors').write_bytes(b'other stats'),
    lambda d: (d / 'policy_preprocessor.json').write_text('{"steps": []}'),
    lambda d: (d / 'policy_postprocessor.json').write_text('{"steps": []}'),
    lambda d: (d / 'policy_preprocessor_step_3_normalizer_processor.safetensors').unlink(),
    lambda d: (d / 'config.json').rename(d / 'train_config.json')])
def test_every_file_the_runner_loads_is_pinned(tmp_path, change):
    skill = lerobot_checkpoint(tmp_path)
    change(skill.checkpoint)
    with pytest.raises(ValueError, match='artifact digest'):
        verify_artifact(skill)


def test_the_digest_is_a_sorted_manifest_and_provenance_records_are_outside_it(tmp_path):
    skill = lerobot_checkpoint(tmp_path)
    lines = ''.join(f'{hashlib.sha256((skill.checkpoint / name).read_bytes()).hexdigest()}  {name}\n'
                    for name in sorted(path.name for path in skill.checkpoint.iterdir()))
    assert skill.artifact_sha256 == hashlib.sha256(lines.encode()).hexdigest()
    (skill.checkpoint / 'training-manifest.json').write_text('{"kind": "lekiwi_skill_training"}')
    (skill.checkpoint / 'evaluation-report.json').write_text('{"passed": true}')
    assert verify_artifact(skill) == skill.checkpoint
    (skill.checkpoint / 'model.safetensors').unlink()
    with pytest.raises(ValueError, match='no model.safetensors'):
        verify_artifact(skill)


def test_a_lerobot_checkpoint_of_safetensors_and_registered_processors_loads(tmp_path):
    skill = lerobot_checkpoint(tmp_path)
    assert verify_artifact(skill) == skill.checkpoint


@pytest.mark.parametrize('name', ['pytorch_model.bin', 'model.pt', 'optimizer.pth', 'stats.pkl', 'last.ckpt',
                                  'modeling_act.py', 'notes.txt'])
def test_pickle_weights_and_code_files_are_refused(tmp_path, name):
    skill = lerobot_checkpoint(tmp_path)
    (skill.checkpoint / name).write_bytes(b'\x80\x04.')
    with pytest.raises(ValueError, match='safetensors or JSON'):
        verify_artifact(skill)


@pytest.mark.parametrize('document', [
    {'steps': [{'class': 'subprocess.Popen', 'config': {'args': ['true']}}]},
    {'type': 'act', 'auto_map': {'AutoModel': 'modeling.Model'}},
    {'model': {'_target_': 'os.system'}}, {'trust_remote_code': True}])
def test_configurations_that_name_code_to_import_are_refused(tmp_path, document):
    skill = lerobot_checkpoint(tmp_path)
    (skill.checkpoint / 'policy_preprocessor.json').write_text(json.dumps(document))
    with pytest.raises(ValueError, match='names code to import'):
        verify_artifact(skill)


def test_links_subdirectories_writable_and_malformed_checkpoints_are_refused(tmp_path):
    skill = lerobot_checkpoint(tmp_path)
    (skill.checkpoint / 'extra.json').symlink_to(skill.checkpoint / 'config.json')
    with pytest.raises(ValueError, match='safetensors or JSON'):
        verify_artifact(skill)
    (skill.checkpoint / 'extra.json').unlink()
    (skill.checkpoint / 'nested').mkdir()
    with pytest.raises(ValueError, match='safetensors or JSON'):
        verify_artifact(skill)
    (skill.checkpoint / 'nested').rmdir()
    (skill.checkpoint / 'config.json').chmod(0o666)
    with pytest.raises(ValueError, match='writable'):
        verify_artifact(skill)
    (skill.checkpoint / 'config.json').chmod(0o644)
    (skill.checkpoint / 'config.json').write_text('{"type": ')
    with pytest.raises(ValueError, match='not valid JSON'):
        verify_artifact(skill)


def test_gateway_bounds_the_skill_request_shape_only():
    goal, parameters = task_goal('skill', '{"skill_id": "push", "parameters": {"force": 0.5}}')
    assert goal.skill_id == 'push' and json.loads(goal.parameters_json) == {'force': .5}
    assert json.loads(parameters) == {'parameters': {'force': .5}, 'skill_id': 'push'}
    for bad in ['{}', '{"skill_id": "Push"}', '{"skill_id": "push", "parameters": []}', '{"skill_id": "push", "x": 1}']:
        with pytest.raises(ValueError):
            task_goal('skill', bad)


class FakePolicy:
    delay = 0.

    def __init__(self, path):
        self.path, self.calls = path, 0

    def reset(self):
        self.calls = 0

    def chunk(self, state, images, task):
        self.calls += 1
        time.sleep(self.delay)
        actions = np.zeros((2, len(ACTIONS)))
        actions[:, 0] = state[0] + .05
        return actions


class FakeArm(Node):
    def __init__(self, context, accept=True):
        super().__init__('fake_arm_controller', context=context)
        self.goals, self.accept, self.hold = [], accept, threading.Event()
        self.hold.set()
        ActionServer(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', self.execute,
                     goal_callback=lambda _goal: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
                     cancel_callback=lambda _goal: CancelResponse.ACCEPT)

    def execute(self, handle):
        self.goals.append(handle.request)
        while not self.hold.wait(.02):
            if handle.is_cancel_requested:
                handle.canceled()
                return FollowJointTrajectory.Result()
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)


@pytest.fixture
def ros_runner(tmp_path):
    from lekiwi_rmf.skill_runner import SkillRunner

    (tmp_path / 'push/1').mkdir(parents=True)
    (tmp_path / 'push/1/model.safetensors').write_bytes(b'weights')
    artifact = artifact_digest(tmp_path / 'push/1')
    skills = {'push': entry(artifact_sha256=artifact, cameras=[], max_duration_sec=.6),
              'trial': entry(status='experimental', artifact_sha256=artifact, cameras=[])}
    (tmp_path / 'skills.yaml').write_text(yaml.safe_dump({'skills': skills}))
    context = Context()
    rclpy.init(context=context)
    runner = SkillRunner(policy_factory=FakePolicy, context=context, parameter_overrides=[
        Parameter('registry_file', value=str(tmp_path / 'skills.yaml')),
        Parameter('artifact_directory', value=str(tmp_path))])
    arm, peer = FakeArm(context), Node('skill_peer', context=context)
    joints = peer.create_publisher(JointState, '/joint_states', 10)
    client = ActionClient(peer, RunSkill, '/robot/skills/run')
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    for node in (runner, arm, peer):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    fresh = threading.Event()
    fresh.set()

    def publish():
        while fresh.is_set():
            message = JointState(name=list(ARM_JOINTS), position=[0.0] * len(ARM_JOINTS))
            message.header.stamp = peer.get_clock().now().to_msg()
            joints.publish(message)
            time.sleep(.05)
    publisher = threading.Thread(target=publish, daemon=True)
    publisher.start()
    try:
        yield runner, arm, client, fresh
    finally:
        fresh.clear()
        arm.hold.set()
        publisher.join(5)
        release_graph(executor=executor, thread=thread, waitables=(client,), nodes=(runner, arm, peer), context=context)


def run(client, skill_id, parameters='{}', cancel_after=None):
    assert client.wait_for_server(timeout_sec=5)
    handle_future = client.send_goal_async(RunSkill.Goal(skill_id=skill_id, parameters_json=parameters))
    end = time.monotonic() + 5
    while not handle_future.done() and time.monotonic() < end:
        time.sleep(.02)
    handle = handle_future.result()
    if not handle.accepted:
        return None
    if cancel_after is not None:
        time.sleep(cancel_after)
        handle.cancel_goal_async()
    result_future = handle.get_result_async()
    end = time.monotonic() + 10
    while not result_future.done() and time.monotonic() < end:
        time.sleep(.02)
    return result_future.result().result


def test_runner_executes_rate_limited_chunks_through_the_arm_controller(ros_runner):
    runner, arm, client, _ = ros_runner
    result = run(client, 'push')
    assert result.outcome == 'finished', result.detail
    report = json.loads(result.result_json)
    assert report['chunks_executed'] == len(arm.goals) >= 1 and report['verification'] == 'not performed by the runner'
    trajectory = arm.goals[0].trajectory
    assert list(trajectory.joint_names) == list(ARM_JOINTS) and len(trajectory.points) == 2
    assert trajectory.points[0].positions[0] == pytest.approx(.05)
    assert run(client, 'missing') is None
    assert run(client, 'trial') is None  # experimental needs an acquisition protocol
    assert run(client, 'push', '{"force": 9}') is None


def test_runner_fails_on_stale_state_and_reports_controller_rejection(ros_runner):
    runner, arm, client, fresh = ros_runner
    arm.accept = False
    result = run(client, 'push')
    assert result.outcome == 'failed' and 'rejected the chunk' in result.detail
    arm.accept = True
    fresh.clear()
    time.sleep(.8)
    result = run(client, 'push')
    assert result.outcome == 'failed' and 'stale' in result.detail
    assert not runner._busy


def test_cancel_stops_the_running_chunk_and_releases_the_arm(ros_runner):
    runner, arm, client, _ = ros_runner
    arm.hold.clear()
    result = run(client, 'push', cancel_after=.3)
    assert result.outcome == 'canceled'
    assert not runner._busy and len(arm.goals) == 1


def test_camera_frames_decode_to_rgb_without_cv_bridge():
    from sensor_msgs.msg import Image
    from lekiwi_rmf.skill_runner import rgb_pixels

    pixels = np.zeros((2, 3, 3), np.uint8)
    pixels[..., 0] = 7
    message = Image(height=2, width=3, encoding='bgr8', step=12, data=np.pad(pixels.reshape(2, 9), ((0, 0), (0, 3))).tobytes())
    assert rgb_pixels(message)[0, 0].tolist() == [0, 0, 7]
    message.encoding = 'mono8'
    with pytest.raises(RuntimeError, match='encoding'):
        rgb_pixels(message)


def test_effects_are_declared_or_conservatively_irreversible(tmp_path):
    skills = registry(tmp_path, {'push': entry(), 'wave': entry(effects={
        'reversibility': 'reversible', 'compensation': 'return the arm to its start pose'})})
    assert skills['push'].effects['reversibility'] == 'irreversible'
    assert catalog(skills)[1]['effects']['reversibility'] == 'reversible'
    for effects in ({'reversibility': 'undoable', 'compensation': 'x'}, {'reversibility': 'reversible'},
                    {'reversibility': 'reversible', 'compensation': ''}):
        with pytest.raises(ValueError, match='effects'):
            registry(tmp_path, {'push': entry(effects=effects)})


def test_a_chunk_computed_too_late_never_moves_the_arm(ros_runner, monkeypatch):
    runner, arm, client, _ = ros_runner
    # Longer than max_observation_age_sec (0.5 s) by itself: every chunk is history by the time it exists.
    monkeypatch.setattr(FakePolicy, 'delay', .6)
    result = run(client, 'push')
    assert result.outcome == 'failed' and 'arrived after max_observation_age_sec' in result.detail
    report = json.loads(result.result_json)
    assert report['chunks_executed'] == 0 and report['late_chunks'] >= 1 and not arm.goals
    summary = runner._budget.summary()
    assert summary['late'] == report['late_chunks'] and summary['stages']['inference']['p50_sec'] >= .6
    status = runner._budget.status()
    assert status.name == 'lekiwi/budget/skill_control' and 'control age limit' in status.message


def test_a_timely_run_reports_its_stage_percentiles(ros_runner):
    runner, arm, client, _ = ros_runner
    result = run(client, 'push')
    assert result.outcome == 'finished' and json.loads(result.result_json)['late_chunks'] == 0
    stages = runner._budget.summary()['stages']
    assert stages['execution']['count'] == len(arm.goals) >= 1
    assert stages['sensor_age']['count'] >= 1 and 0 <= stages['sensor_age']['p99_sec'] <= .5


def test_loading_the_policy_does_not_consume_the_motion_budget(ros_runner, monkeypatch):
    runner, arm, client, _ = ros_runner
    load = FakePolicy.__init__

    def slow(self, path):
        time.sleep(1.0)  # longer than the skill's 0.6 s motion limit
        load(self, path)
    monkeypatch.setattr(FakePolicy, '__init__', slow)
    result = run(client, 'push')
    assert result.outcome == 'finished' and json.loads(result.result_json)['chunks_executed'] >= 1
