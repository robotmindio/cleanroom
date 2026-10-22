"""/robot/motion/solve over stand-in MoveIt services: answers, provenance and every unknown path.

MoveIt itself is exercised in test_motion_solver_moveit_launch.py; here its services are
small fakes driven by a toy 5-joint chain, so each failure path can be forced.
"""

import json
import math
import threading
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetStateValidity
import numpy as np
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectoryPoint

from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.motion_solver import GROUP_JOINTS, SERVICE, MotionSolver
from lekiwi_rmf.srv import QueryObservations, TaskApi
from ros_test_utils import release_graph
from test_motion_constraints import toy_fk

OBSERVATION = 'a' * 64


def quaternion(rotation):
    """A unit quaternion (x, y, z, w) for a proper rotation matrix."""
    w = math.sqrt(max(0.0, 1 + np.trace(rotation))) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2,
                      rotation[2, 1] - rotation[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2,
                      rotation[0, 2] - rotation[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2,
                      rotation[1, 0] - rotation[0, 1])
    return x, y, z, w


class FakeMoveIt(Node):
    def __init__(self, context):
        super().__init__('fake_moveit', context=context)
        group = ReentrantCallbackGroup()
        self.joints = [0.0] * 6
        self.publish_joints = True
        self.collide_below_z = None
        self.plan_succeeds = True
        self.position = {'mode': 'camera', 'xyz': [0.2, 0.0, 0.0], 'frame_id': 'camera_optical',
                         'uncertainty_m': 0.005, 'meaning': 'visible_surface'}
        self.observation_age = 0.5
        self.validity_calls = 0
        self.create_service(GetPositionFK, '/compute_fk', self.fk, callback_group=group)
        self.create_service(GetStateValidity, '/check_state_validity', self.validity, callback_group=group)
        self.create_service(GetMotionPlan, '/plan_kinematic_path', self.plan, callback_group=group)
        self.create_service(QueryObservations, '/robot/observations/query', self.observations, callback_group=group)
        self.joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.05, self.tick, callback_group=group)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, xyz in (('odom', 'base_link', (0.0, 0.0, 0.0)),
                                   ('base_link', 'camera_optical', (0.0, 0.0, 0.1))):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.static.sendTransform(transforms)

    def tick(self):
        if self.publish_joints:
            message = JointState(name=list(ARM_JOINTS), position=list(self.joints))
            message.header.stamp = self.get_clock().now().to_msg()
            self.joint_states.publish(message)

    @staticmethod
    def group(state):
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        return [positions[name] for name in GROUP_JOINTS]

    def fk(self, request, response):
        position, rotation = toy_fk(self.group(request.robot_state))
        item = PoseStamped(header=request.header)
        item.pose.position.x, item.pose.position.y, item.pose.position.z = map(float, position)
        q = quaternion(rotation)
        item.pose.orientation.x, item.pose.orientation.y, item.pose.orientation.z, item.pose.orientation.w = q
        response.pose_stamped.append(item)
        response.fk_link_names = list(request.fk_link_names)
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response

    def validity(self, request, response):
        self.validity_calls += 1
        position, _ = toy_fk(self.group(request.robot_state))
        response.valid = bool(self.collide_below_z is None or position[2] >= self.collide_below_z)
        return response

    def plan(self, request, response):
        result = response.motion_plan_response
        if self.plan_succeeds:
            result.error_code.val = MoveItErrorCodes.SUCCESS
            point = JointTrajectoryPoint(positions=[0.0] * 5)
            point.time_from_start.sec = 2
            result.trajectory.joint_trajectory.points = [point]
            result.planning_time = 0.1
        else:
            result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
        return response

    def observations(self, request, response):
        ids = json.loads(request.request_json).get('observation_ids', [])
        stamp = self.get_clock().now().nanoseconds / 1e9 - self.observation_age
        records = [{'id': OBSERVATION, 'stamp': {'sec': int(stamp), 'nanosec': int((stamp % 1) * 1e9)},
                    'detections': [{'label': 'cup', 'score': 0.9, 'bbox': [0, 0, 1, 1],
                                    **({'position': self.position} if self.position else {})}]}]
        response.success = True
        response.response_json = json.dumps({'schema': 1, 'observations': records if OBSERVATION in ids else []})
        return response


@pytest.fixture
def graph():
    context = Context()
    rclpy.init(context=context)
    fake = FakeMoveIt(context)
    solver = MotionSolver(context=context, parameter_overrides=[
        Parameter('seed_samples', value=128), Parameter('service_timeout_sec', value=2.0),
        Parameter('planning_frame', value='base_link'), Parameter('max_fk_evaluations', value=600)])
    client_node = Node('motion_client', context=context)
    client = client_node.create_client(TaskApi, SERVICE)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (fake, solver, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(values, schema=1, timeout=30):
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(TaskApi.Request(schema=schema, request_json=json.dumps(values)))
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), 'motion solver did not answer'
        reply = future.result()
        return reply if not reply.ok else json.loads(reply.response_json)

    deadline = time.monotonic() + 5
    while solver._joints is None and time.monotonic() < deadline:
        time.sleep(0.02)
    try:
        yield fake, solver, call
    finally:
        release_graph(executor=executor, thread=thread, nodes=(client_node, solver, fake), context=context)


def point(xyz, radius=0.04, frame='base_link'):
    return {'kind': 'position', 'target': {'point': {'frame': frame, 'xyz': xyz}}, 'radius_m': radius}


def observed(radius=0.06, offset=(0.0, 0.0, 0.0)):
    return {'kind': 'position', 'target': {'observation': {'id': OBSERVATION, 'detection': 0}},
            'radius_m': radius, 'offset_m': list(offset)}


DOWN = {'kind': 'orientation', 'axis': [1, 0, 0], 'direction': [0, 0, -1], 'tolerance_rad': 0.2}


def test_solve_returns_a_verified_joint_target_with_its_motion_and_validity(graph):
    fake, _, call = graph
    answer = call({'op': 'solve', 'constraints': [point([0.18, 0.06, 0.05]), DOWN], 'max_time_sec': 10})
    assert answer['status'] == 'feasible', answer
    assert set(answer['joints']) == set(GROUP_JOINTS)
    position, rotation = toy_fk([answer['joints'][name] for name in GROUP_JOINTS])
    assert np.linalg.norm(position - [0.18, 0.06, 0.05]) <= 0.04
    assert math.acos(np.clip(np.dot(rotation[:, 0], [0, 0, -1]), -1, 1)) <= 0.2
    assert answer['motion']['duration_sec'] == 2.0 and fake.validity_calls >= 1
    assert answer['valid_until'] > answer['checked_at'] and answer['joint_state']['positions']


def test_solve_reaches_an_observed_object_through_tf_at_capture_time(graph):
    _, _, call = graph
    answer = call({'op': 'solve', 'constraints': [observed()], 'max_time_sec': 10})
    assert answer['status'] == 'feasible', answer
    target = answer['targets'][0]
    # camera_optical sits 0.1 m above base_link, so the camera point (0.2, 0, 0) is (0.2, 0, 0.1).
    assert np.allclose(target['center'], [0.2, 0.0, 0.1], atol=1e-6)
    assert target['label'] == 'cup' and target['uncertainty_m'] == 0.005
    assert any('capture time' in item for item in answer['assumptions'])


@pytest.mark.parametrize('change,reason', [
    (lambda fake: setattr(fake, 'position', None), 'position_unavailable'),
    (lambda fake: setattr(fake, 'observation_age', 60.0), 'observation_stale'),
])
def test_an_object_without_a_fresh_metric_position_is_unknown_not_infeasible(graph, change, reason):
    fake, _, call = graph
    change(fake)
    answer = call({'op': 'solve', 'constraints': [observed()]})
    assert (answer['status'], answer['reason']) == ('unknown', reason)


def test_stale_joint_states_make_every_answer_unknown(graph):
    fake, _, call = graph
    fake.publish_joints = False
    time.sleep(0.7)
    assert call({'op': 'verify', 'constraints': [DOWN]})['reason'] == 'joint_state_stale'


def test_an_unreachable_region_is_infeasible_with_the_search_bound_stated(graph):
    _, _, call = graph
    answer = call({'op': 'solve', 'constraints': [point([0.0, 0.0, 0.48], 0.05)], 'max_time_sec': 10})
    assert (answer['status'], answer['reason']) == ('infeasible', 'no_kinematic_solution')
    assert 'not complete' in answer['detail']


def test_colliding_solutions_are_rejected(graph):
    fake, _, call = graph
    fake.collide_below_z = 1.0
    answer = call({'op': 'solve', 'constraints': [point([0.18, 0.06, 0.05])], 'max_time_sec': 10})
    assert (answer['status'], answer['reason']) == ('infeasible', 'in_collision')


def test_a_solution_without_a_motion_plan_is_infeasible_unless_planning_is_skipped(graph):
    fake, _, call = graph
    fake.plan_succeeds = False
    assert call({'op': 'solve', 'constraints': [point([0.18, 0.06, 0.05])], 'max_time_sec': 10})['reason'] == 'no_motion_plan'
    assert call({'op': 'solve', 'constraints': [point([0.18, 0.06, 0.05])], 'plan': False,
                 'max_time_sec': 10})['status'] == 'feasible'


def test_verify_judges_the_current_pose(graph):
    fake, _, call = graph
    position, _ = toy_fk([0.0] * 5)
    assert call({'op': 'verify', 'constraints': [point(position.tolist(), 0.02)]})['status'] == 'holds'
    assert call({'op': 'verify', 'constraints': [point((position + [0.1, 0, 0]).tolist(), 0.02)]})['status'] == 'violated'
    # Within the object's uncertainty of the boundary nobody can tell.
    fake.position = {**fake.position, 'xyz': [position[0] - 0.023, position[1], position[2] - 0.1],
                     'uncertainty_m': 0.01}
    assert call({'op': 'verify', 'constraints': [observed(radius=0.02)]})['status'] == 'uncertain'


def test_unknown_frames_and_missing_moveit_are_unknown(graph):
    _, solver, call = graph
    assert call({'op': 'solve', 'constraints': [point([0.2, 0, 0.1], frame='nowhere')]})['reason'] == 'transform_unavailable'
    solver.destroy_client(solver._fk_client)
    solver._fk_client = solver.create_client(GetPositionFK, '/compute_fk_missing')
    assert call({'op': 'verify', 'constraints': [DOWN]})['reason'] == 'service_unavailable'


def test_malformed_requests_and_other_schemas_are_refused(graph):
    _, _, call = graph
    refused = call({'op': 'solve', 'constraints': []})
    assert (refused.ok, refused.error) == (False, 'invalid_request')
    other = call({'op': 'solve', 'constraints': [DOWN]}, schema=2)
    assert (other.ok, other.error) == (False, 'schema_incompatible')


def test_a_search_cut_short_by_its_deadline_is_unknown_not_infeasible(graph):
    _, solver, call = graph
    solver._search.solve = lambda resolved, current, deadline: ([], 3, 'deadline')
    answer = call({'op': 'solve', 'constraints': [point([0.18, 0.06, 0.05])]})
    assert (answer['status'], answer['reason']) == ('unknown', 'search_timeout')
