"""The reach capability (a solved joint target through MoveIt's MoveGroup) and the motion solver's advertisement."""

import json
import threading
import time

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

from lekiwi_rmf.srv import TaskApi
from lekiwi_rmf.task_gateway import REACH_JOINTS, TaskGateway, reach_result, task_goal
from ros_test_utils import release_graph

TARGET = {'arm_shoulder_pan': 0.2, 'arm_shoulder_lift': 0.5, 'arm_elbow_flex': -0.4,
          'arm_wrist_flex': 0.3, 'arm_wrist_roll': 0.0}


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
    {'joints': TARGET, 'solution_id': 'bad id with spaces'}, {'joints': TARGET, 'speed': 1.0},
])
def test_invalid_reach_targets_are_refused(parameters):
    with pytest.raises(ValueError):
        task_goal('reach', json.dumps(parameters))


def test_reach_goal_asks_moveit_to_plan_and_execute_to_the_joint_target():
    goal, canonical = task_goal('reach', json.dumps({'joints': TARGET, 'solution_id': 'motion-1'}))
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
        goal.succeed()
        return result


@pytest.fixture
def reach(tmp_path):
    context = Context()
    rclpy.init(context=context)
    fake = FakeMoveGroup(context)
    gateway = TaskGateway(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3')), Parameter('allow_reach', value=True)])
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
    assert motion['schema'] == 1 and motion['planning_frame'] == 'base_footprint'


@pytest.mark.parametrize('code,state', [(MoveItErrorCodes.SUCCESS, 'succeeded'),
                                        (MoveItErrorCodes.PLANNING_FAILED, 'failed')])
def test_reach_terminal_state_is_moveits_error_code(reach, code, state):
    fake, api, _ = reach
    fake.code = code
    ok, error, view = api('submit', {'task_id': f'reach-{code}', 'capability': 'reach', 'owner': 'planner',
                                     'parameters': {'joints': TARGET, 'solution_id': 'motion-1'}})
    assert ok, (error, view)
    assert view['admitted'] and view['task']['resources'] == ['arm', 'base']

    def terminal():
        return api('status', {'task_id': f'reach-{code}'})[2]['task']['state'] in ('succeeded', 'failed')
    wait(terminal)
    view = api('status', {'task_id': f'reach-{code}'})[2]['task']
    assert view['state'] == state
    assert len(fake.goals) == 1


def test_reach_is_off_unless_the_deployment_enables_it(tmp_path):
    context = Context()
    rclpy.init(context=context)
    gateway = TaskGateway(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3'))])
    try:
        reply = gateway._capabilities(None, Trigger.Response())
        document = json.loads(reply.message)
        assert not next(item for item in document['capabilities'] if item['name'] == 'reach')['available']
        assert document['motion']['available'] is False
    finally:
        gateway.destroy_node()
        context.try_shutdown()
