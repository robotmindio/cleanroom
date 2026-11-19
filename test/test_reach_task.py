"""The arm capabilities through MoveIt's MoveGroup (reach and gripper) and the motion solver's advertisement."""

import json
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS
from lekiwi_rmf.motion_constraints import PLANNING_FRAME, TOOL_LINK
from lekiwi_rmf.srv import TaskApi
from lekiwi_rmf.task_gateway import CAPABILITIES, REACH_JOINTS, TaskGateway, reach_result, task_goal
from lekiwi_rmf.task_journal import Refused
from ros_test_utils import release_graph

TARGET = {'arm_shoulder_pan': 0.2, 'arm_shoulder_lift': 0.5, 'arm_elbow_flex': -0.4,
          'arm_wrist_flex': 0.3, 'arm_wrist_roll': 0.0}
CONFIG = Path(__file__).parents[1] / 'config'


def srdf_state(group, name):
    """The SRDF's own joint values, read here independently of the gateway."""
    state, = ET.parse(CONFIG / 'lekiwi.srdf').getroot().findall(f"group_state[@name='{name}'][@group='{group}']")
    return {joint.get('name'): float(joint.get('value')) for joint in state.findall('joint')}


def wait(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('ROS operation did not complete')


@pytest.mark.parametrize('parameters', [
    {}, {'joints': {}}, {'joints': {**TARGET, 'arm_gripper': 0.0}},
    {'joints': {k: v for k, v in TARGET.items() if k != 'arm_wrist_roll'}},
    {'joints': {**TARGET, 'arm_shoulder_pan': 3.5}}, {'joints': {**TARGET, 'arm_elbow_flex': True}},
    {'joints': TARGET, 'solution_id': 'motion-1'}, {'joints': TARGET, 'speed': 1.0},
    {'named': 'nowhere'}, {'named': 7}, {'named': 'home', 'joints': TARGET}, {'named': 'open'},
])
def test_invalid_reach_targets_are_refused(parameters):
    with pytest.raises(ValueError):
        task_goal('reach', json.dumps(parameters))


@pytest.mark.parametrize('parameters', [
    {}, {'state': 'ajar'}, {'state': 'home'}, {'state': 'open', 'position_rad': 0.1}, {'position_rad': True},
    {'position_rad': JOINT_LIMITS['arm_gripper'][1] + 0.01}, {'position_rad': JOINT_LIMITS['arm_gripper'][0] - 0.01},
    {'state': 'open', 'speed': 1.0}, {'joints': TARGET},
])
def test_invalid_gripper_targets_are_refused(parameters):
    with pytest.raises(ValueError):
        task_goal('gripper', json.dumps(parameters))


@pytest.mark.parametrize('name', ['home', 'travel_stow', 'supported_rest'])
def test_a_named_reach_is_the_srdf_arm_state(name):
    goal, canonical = task_goal('reach', json.dumps({'named': name}))
    assert json.loads(canonical) == {'named': name} and goal.request.group_name == 'arm'
    constraints = goal.request.goal_constraints[0].joint_constraints
    assert {item.joint_name: item.position for item in constraints} == srdf_state('arm', name)
    assert set(srdf_state('arm', name)) == set(REACH_JOINTS)


@pytest.mark.parametrize('parameters,position', [
    ({'state': 'open'}, srdf_state('gripper', 'open')['arm_gripper']),
    ({'state': 'closed'}, srdf_state('gripper', 'closed')['arm_gripper']),
    ({'state': 'travel_stow'}, srdf_state('gripper', 'travel_stow')['arm_gripper']), ({'position_rad': 0.4}, 0.4)])
def test_a_gripper_goal_moves_only_the_claw_through_the_same_moveit_request(parameters, position):
    goal, _ = task_goal('gripper', json.dumps(parameters))
    reach, _ = task_goal('reach', json.dumps({'joints': TARGET}))
    assert goal.request.group_name == 'gripper' and not goal.planning_options.plan_only
    constraint, = goal.request.goal_constraints[0].joint_constraints
    assert (constraint.joint_name, constraint.position) == ('arm_gripper', position)
    assert (constraint.tolerance_above, constraint.tolerance_below) == (0.01, 0.01)
    assert (goal.request.num_planning_attempts, goal.request.allowed_planning_time, goal.request.start_state.is_diff) == (
        reach.request.num_planning_attempts, reach.request.allowed_planning_time, reach.request.start_state.is_diff)


def test_reach_goal_asks_moveit_to_plan_and_execute_to_the_joint_target():
    goal, canonical = task_goal('reach', json.dumps({'joints': TARGET}))
    assert json.loads(canonical)['joints'] == TARGET
    request = goal.request
    assert request.group_name == 'arm' and request.start_state.is_diff and not goal.planning_options.plan_only
    constraints = request.goal_constraints[0].joint_constraints
    assert {item.joint_name: item.position for item in constraints} == pytest.approx(TARGET)
    assert all(item.tolerance_above == item.tolerance_below == 0.01 for item in constraints)


def test_reach_result_keeps_only_moveits_outcome_and_final_joints():
    success = reach_result({'error_code': {'val': MoveItErrorCodes.SUCCESS}, 'planning_time': 0.2,
                            'executed_trajectory': {'joint_trajectory': {
                                'joint_names': list(REACH_JOINTS),
                                'points': [{'positions': [0.0] * 5}, {'positions': list(TARGET.values())}]}}})
    assert success == {'moveit_error_code': 1, 'error_code': 0, 'planning_time_sec': 0.2, 'final_joints': TARGET}
    failure = reach_result({'error_code': {'val': MoveItErrorCodes.PLANNING_FAILED}})
    assert failure['error_code'] == MoveItErrorCodes.PLANNING_FAILED and failure['final_joints'] is None


class FakeMoveGroup(Node):
    def __init__(self, context):
        super().__init__('fake_move_group', context=context)
        self.code = MoveItErrorCodes.SUCCESS
        self.hold = False
        self.goals = []
        self.server = ActionServer(self, MoveGroup, '/move_action', self.execute,
                                   goal_callback=lambda _: GoalResponse.ACCEPT,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())

    def destroy_node(self):
        self.server.destroy()
        return super().destroy_node()

    def execute(self, goal):
        self.goals.append(goal.request)
        result = MoveGroup.Result()
        result.error_code.val = self.code
        result.executed_trajectory.joint_trajectory.joint_names = list(REACH_JOINTS)
        point = JointTrajectoryPoint(positions=[item.position for item in
                                                goal.request.request.goal_constraints[0].joint_constraints])
        result.executed_trajectory.joint_trajectory.points = [point]
        while self.hold:
            goal.publish_feedback(MoveGroup.Feedback(state='MONITOR'))
            time.sleep(0.05)
        goal.succeed()
        return result


@pytest.fixture
def reach(tmp_path):
    context = Context()
    rclpy.init(context=context)
    fake = FakeMoveGroup(context)
    gateway = TaskGateway(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3')), Parameter('allow_reach', value=True),
        Parameter('allow_gripper', value=True)])
    solver = Node('fake_motion_solver', context=context)
    solver.create_service(TaskApi, '/robot/motion/solve', lambda _request, response: response)
    client = Node('reach_client', context=context)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (fake, gateway, solver, client):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    services = {name: client.create_client(TaskApi, f'/robot/tasks/v2/{name}') for name in ('submit', 'status')}
    capabilities = client.create_client(Trigger, '/robot/capabilities')

    def call(service, request):
        future = service.call_async(request)
        wait(future.done)
        return future.result()

    def api(name, values):
        reply = call(services[name], TaskApi.Request(schema=2, request_json=json.dumps(values)))
        return reply.ok, reply.error, json.loads(reply.response_json or 'null')

    def catalog():
        return json.loads(call(capabilities, Trigger.Request()).message)

    wait(lambda: all(item.service_is_ready() for item in services.values())
         and gateway._action_clients['reach'].server_is_ready())
    try:
        yield fake, api, catalog
    finally:
        release_graph(executor=executor, thread=thread, nodes=(client, solver, gateway, fake), context=context)


def test_reach_and_the_motion_solver_are_advertised(reach):
    _, _, catalog = reach
    document = catalog()
    entry = next(item for item in document['capabilities'] if item['name'] == 'reach')
    assert entry['available'] and entry['resources'] == ['arm', 'base'] and entry['units'] == {'joints': 'rad'}
    assert entry['effects']['reversibility'] == 'reversible' and entry['requires_schema'] == 1
    assert entry['joints'] == list(REACH_JOINTS) and entry['solver'] == '/robot/motion/solve'
    wait(lambda: catalog()['motion']['available'])
    motion = catalog()['motion']
    assert motion['operations'] == ['solve', 'verify'] and motion['execute'] == 'reach'
    assert motion['schema'] == 1
    assert (motion['planning_frame'], motion['tool_link']) == (PLANNING_FRAME, TOOL_LINK)
    assert entry['parameters'] == ['joints', 'named'] and entry['required_parameters'] == ['joints']


@pytest.mark.parametrize('code,state', [(MoveItErrorCodes.SUCCESS, 'succeeded'),
                                        (MoveItErrorCodes.PLANNING_FAILED, 'failed')])
def test_reach_terminal_state_is_moveits_error_code(reach, code, state):
    fake, api, _ = reach
    fake.code = code
    ok, error, view = api('submit', {'task_id': f'reach-{code}', 'capability': 'reach', 'owner': 'planner',
                                     'parameters': {'joints': TARGET}})
    assert ok, (error, view)
    assert view['admitted'] and view['task']['resources'] == ['arm', 'base']

    def terminal():
        return api('status', {'task_id': f'reach-{code}'})[2]['task']['state'] in ('succeeded', 'failed')
    wait(terminal)
    view = api('status', {'task_id': f'reach-{code}'})[2]['task']
    assert view['state'] == state
    assert len(fake.goals) == 1


def test_a_running_reach_reports_moveits_own_state(reach):
    fake, api, _ = reach
    fake.hold = True
    try:
        assert api('submit', {'task_id': 'reach-held', 'capability': 'reach', 'owner': 'planner',
                              'parameters': {'joints': TARGET}})[0]
        wait(lambda: api('status', {'task_id': 'reach-held'})[2]['task']['detail'] == 'MONITOR')
        assert api('status', {'task_id': 'reach-held'})[2]['task']['state'] == 'running'
    finally:
        fake.hold = False
    wait(lambda: api('status', {'task_id': 'reach-held'})[2]['task']['state'] == 'succeeded')


def test_a_named_reach_runs_as_the_joints_it_names(reach):
    fake, api, catalog = reach
    entry = next(item for item in catalog()['capabilities'] if item['name'] == 'reach')
    assert entry['named_states']['travel_stow'] == srdf_state('arm', 'travel_stow')
    assert entry['schema']['properties']['named']['enum'] == ['home', 'supported_rest', 'travel_stow']
    ok, error, view = api('submit', {'task_id': 'stow', 'capability': 'reach', 'owner': 'planner',
                                     'parameters': {'named': 'travel_stow'}})
    assert ok, (error, view)
    assert view['task']['parameters'] == {'named': 'travel_stow'}
    assert view['task']['effective_parameters'] == {'joints': srdf_state('arm', 'travel_stow')}
    wait(lambda: api('status', {'task_id': 'stow'})[2]['task']['state'] == 'succeeded')
    request, = fake.goals
    assert request.request.group_name == 'arm'
    assert {item.joint_name: item.position for item in
            request.request.goal_constraints[0].joint_constraints} == srdf_state('arm', 'travel_stow')


def test_the_gripper_is_advertised_and_holds_the_arm_like_a_reach(reach):
    fake, api, catalog = reach
    entry = next(item for item in catalog()['capabilities'] if item['name'] == 'gripper')
    assert entry['available'] and entry['resources'] == ['arm', 'base'] and entry['units'] == {'position_rad': 'rad'}
    assert entry['parameters'] == ['state', 'position_rad'] and entry['required_parameters'] == ['state']
    # Opening releases and closing grasps: not undone by moving back, so it needs the schema that says so.
    assert entry['effects']['reversibility'] == 'compensable' and entry['requires_schema'] == 2
    assert entry['named_states'] == {name: srdf_state('gripper', name)['arm_gripper']
                                     for name in ('open', 'closed', 'travel_stow')}
    properties = entry['schema']['properties']
    assert properties['state'] == {'type': 'string', 'enum': ['closed', 'open', 'travel_stow']}
    assert (properties['position_rad']['min'], properties['position_rad']['max']) == JOINT_LIMITS['arm_gripper']
    assert entry['schema']['preconditions'][1] == {
        'condition': 'action_server_ready', 'action': '/move_action', 'satisfied': True}
    fake.hold = True
    try:
        assert api('submit', {'task_id': 'reach', 'capability': 'reach', 'owner': 'planner',
                              'parameters': {'joints': TARGET}})[0]
        # One arm: the claw waits for the reach, and the reach for the claw.
        ok, error, data = api('submit', {'task_id': 'grip', 'capability': 'gripper', 'owner': 'planner',
                                         'parameters': {'state': 'closed'}})
        assert (ok, error) == (False, 'resource_busy') and data['next_condition'] == 'wait_resource_free'
        assert {holder['resource'] for holder in data['holders']} == {'arm', 'base'}
    finally:
        fake.hold = False
    wait(lambda: api('status', {'task_id': 'reach'})[2]['task']['state'] == 'succeeded')
    ok, error, view = api('submit', {'task_id': 'grip', 'capability': 'gripper', 'owner': 'planner',
                                     'parameters': {'state': 'closed'}, 'resources': ['arm', 'base']})
    assert ok, (error, view)
    assert view['task']['resources'] == ['arm', 'base']
    assert view['task']['effective_parameters'] == {'position_rad': srdf_state('gripper', 'closed')['arm_gripper']}
    wait(lambda: api('status', {'task_id': 'grip'})[2]['task']['state'] == 'succeeded')
    task = api('status', {'task_id': 'grip'})[2]['task']
    assert task['reason_code'] == 'succeeded' and task['result']['moveit_error_code'] == MoveItErrorCodes.SUCCESS
    request = fake.goals[-1].request
    constraint, = request.goal_constraints[0].joint_constraints
    assert request.group_name == 'gripper' and len(fake.goals) == 2
    assert (constraint.joint_name, constraint.position) == ('arm_gripper', srdf_state('gripper', 'closed')['arm_gripper'])


def test_a_failed_gripper_motion_is_moveits_error_and_asks_for_compensation(reach):
    fake, api, _ = reach
    fake.code = MoveItErrorCodes.PLANNING_FAILED
    assert api('submit', {'task_id': 'grip', 'capability': 'gripper', 'owner': 'planner',
                          'parameters': {'position_rad': 0.4}})[0]
    wait(lambda: api('status', {'task_id': 'grip'})[2]['task']['state'] == 'failed')
    task = api('status', {'task_id': 'grip'})[2]['task']
    assert task['reason_code'] == 'executor_error' and task['compensation_required']
    assert task['result']['moveit_error_code'] == MoveItErrorCodes.PLANNING_FAILED


def test_the_arm_capabilities_are_off_unless_the_deployment_enables_them(tmp_path):
    tracked = yaml.safe_load((CONFIG / 'robot_tasks.yaml').read_text())['robot_tasks']['ros__parameters']
    assert {name: value for name, value in tracked.items() if name.startswith('allow_')} == {
        'allow_skills': False, 'allow_reach': False, 'allow_gripper': False}
    context = Context()
    rclpy.init(context=context)
    gateway = TaskGateway(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3'))])
    try:
        reply = gateway._capabilities(None, Trigger.Response())
        document = json.loads(reply.message)
        for name in ('reach', 'gripper', 'skill'):
            entry = next(item for item in document['capabilities'] if item['name'] == name)
            assert not entry['available'] and not entry['schema']['preconditions'][0]['satisfied']
            with pytest.raises(Refused) as refusal:
                gateway._admit('probe', name, json.dumps(
                    {'reach': {'named': 'home'}, 'gripper': {'state': 'open'}, 'skill': {'skill_id': 'any'}}[name]),
                    request={'owner': 'planner'})
            assert refusal.value.code == 'capability_unavailable'
            assert refusal.value.data == {'retryable': False, 'next_condition': 'enable_capability'}
        # The claw's effects are not reversible, so the schema that cannot say so refuses it.
        with pytest.raises(Refused) as refusal:
            gateway._admit('legacy', 'gripper', '{"state": "open"}')
        assert refusal.value.code == 'schema_incompatible'
        assert document['motion']['available'] is False
    finally:
        gateway.destroy_node()
        context.try_shutdown()


def test_the_gripper_uses_the_calibrated_range_moveit_plans_with(tmp_path):
    calibration = tmp_path / 'calibration.json'
    calibration.write_text(json.dumps({'zero_positions': {**dict.fromkeys(ARM_JOINTS, 0.0), 'arm_gripper': 0.1},
                                       'directions': dict.fromkeys(ARM_JOINTS, 1.0)}))
    lower, upper = JOINT_LIMITS['arm_gripper']
    context = Context()
    rclpy.init(context=context)
    gateway = TaskGateway(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3')),
        Parameter('arm_calibration_file', value=str(calibration))])
    try:
        document = json.loads(gateway._capabilities(None, Trigger.Response()).message)
        entry = next(item for item in document['capabilities'] if item['name'] == 'gripper')
        # Zero is jaw contact; the raw servo range past it and past the calibrated opening is not reachable.
        assert entry['named_states'] == {'closed': 0.0, 'open': pytest.approx(upper - 0.1),
                                         'travel_stow': srdf_state('gripper', 'travel_stow')['arm_gripper']}
        position = entry['schema']['properties']['position_rad']
        assert (position['min'], position['max']) == (lower, pytest.approx(upper - 0.1))
        assert gateway._schema('gripper')['properties']['position_rad'] == position
        with pytest.raises(Refused) as refusal:
            gateway._admit('probe', 'gripper', json.dumps({'position_rad': upper}), request={'owner': 'planner'})
        assert refusal.value.code == 'invalid_request' and refusal.value.data['field'] == 'position_rad'
        effective = CAPABILITIES['gripper']['effective']
        assert effective(gateway, {'state': 'closed'}) == {'position_rad': 0.0}
        assert effective(gateway, {'state': 'open'}) == {'position_rad': pytest.approx(upper - 0.1)}
    finally:
        gateway.destroy_node()
        context.try_shutdown()
