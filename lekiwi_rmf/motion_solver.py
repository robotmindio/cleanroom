#!/usr/bin/env python3
"""Geometric feasibility and constraint solving for the arm, over MoveIt's own model and scene.

``/robot/motion/solve`` (``lekiwi_rmf/srv/TaskApi``, motion schema 1) answers two questions
for a planner, without moving anything:

- ``solve``: joint values whose tool pose satisfies the constraints, verified by MoveIt's
  forward kinematics and collision check against the current planning scene and, with
  ``plan``, a collision-free motion from the current state;
- ``verify``: whether the current tool pose, from fresh ``/joint_states``, satisfies them.

MoveIt owns kinematics, collision checking and motion planning; this node owns only the
constraint check and the search that proposes joint values to MoveIt. Executing a solution
is an ordinary ``reach`` task through the task gateway, never a call here.
"""

import json
import math
import threading
import time

from geometry_msgs.msg import PointStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetStateValidity
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS
from lekiwi_rmf.motion_constraints import (
    SCHEMA, KinematicSearch, judge, parse_request, resolve, solvable,
)
from lekiwi_rmf.srv import QueryObservations, TaskApi

SERVICE = '/robot/motion/solve'
# The planning group's joints; the gripper is held where it is.
GROUP_JOINTS = tuple(name for name in ARM_JOINTS if name != 'arm_gripper')


class Unavailable(Exception):
    """An input the answer depends on is missing; the answer is ``unknown``, not ``infeasible``."""


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class MotionSolver(Node):
    def __init__(self, **kwargs):
        super().__init__('motion_solver', **kwargs)
        # MoveIt's model frame, so forward kinematics needs no TF.
        self.declare_parameter('planning_frame', 'base_footprint')
        self.declare_parameter('tool_link', 'tool0')
        self.declare_parameter('group', 'arm')
        # Object positions are measured at capture time; TF carries them to now through odometry.
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('joint_state_max_age_sec', 0.5)
        self.declare_parameter('observation_max_age_sec', 10.0)
        self.declare_parameter('validity_sec', 5.0)
        self.declare_parameter('seed_samples', 256)
        self.declare_parameter('max_fk_evaluations', 1500)
        self.declare_parameter('service_timeout_sec', 2.0)
        value = self.get_parameter
        self._frame, self._tool, self._group = (value('planning_frame').value, value('tool_link').value,
                                                value('group').value)
        self._fixed = value('fixed_frame').value
        self._joint_age, self._observation_age = value('joint_state_max_age_sec').value, value('observation_max_age_sec').value
        self._validity, self._timeout = value('validity_sec').value, value('service_timeout_sec').value
        for name, number in (('joint_state_max_age_sec', self._joint_age), ('observation_max_age_sec', self._observation_age),
                             ('validity_sec', self._validity), ('service_timeout_sec', self._timeout)):
            if type(number) not in (int, float) or not math.isfinite(number) or number <= 0:
                raise ValueError(f'{name} must be positive and finite')
        lower = [JOINT_LIMITS[name][0] for name in GROUP_JOINTS]
        upper = [JOINT_LIMITS[name][1] for name in GROUP_JOINTS]
        self._search = KinematicSearch(self._fk, lower, upper, samples=value('seed_samples').value,
                                       max_evaluations=value('max_fk_evaluations').value)
        # One request at a time: the search's sample table and MoveIt's services are shared.
        self._serial = threading.Lock()
        self._joints = None
        clients = ReentrantCallbackGroup()
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk', callback_group=clients)
        self._validity_client = self.create_client(GetStateValidity, '/check_state_validity', callback_group=clients)
        self._plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path', callback_group=clients)
        self._observations = self.create_client(QueryObservations, '/robot/observations/query', callback_group=clients)
        self.create_subscription(JointState, '/joint_states', self._joint_state, 10, callback_group=clients)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self, spin_thread=False)
        self.create_service(TaskApi, SERVICE, self._handle, callback_group=MutuallyExclusiveCallbackGroup())

    # ------------------------------------------------------------ inputs

    def _joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in ARM_JOINTS):
            return
        if not all(math.isfinite(positions[name]) for name in ARM_JOINTS):
            return
        self._joints = ({name: float(positions[name]) for name in ARM_JOINTS}, time.monotonic(),
                        message.header.stamp.sec + message.header.stamp.nanosec / 1e9)

    def _current(self):
        if self._joints is None or time.monotonic() - self._joints[1] > self._joint_age:
            raise Unavailable('joint_state_stale', 'no /joint_states sample fresher than '
                              f'{self._joint_age} s')
        return self._joints

    def _call(self, client, request, what):
        if not client.service_is_ready():
            raise Unavailable('service_unavailable', f'{what} is not available')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout):
            client.remove_pending_request(future)
            raise Unavailable('service_timeout', f'{what} did not answer within {self._timeout} s')
        return future.result()

    def _state(self, joints, gripper):
        request = GetPositionFK.Request()
        request.robot_state.joint_state.name = [*GROUP_JOINTS, 'arm_gripper']
        request.robot_state.joint_state.position = [*map(float, joints), float(gripper)]
        return request.robot_state

    def _fk(self, joints, gripper=0.0):
        request = GetPositionFK.Request()
        request.header.frame_id = self._frame
        request.fk_link_names = [self._tool]
        request.robot_state = self._state(joints, gripper)
        response = self._call(self._fk_client, request, 'MoveIt forward kinematics (/compute_fk)')
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise Unavailable('kinematics_failed', f'/compute_fk returned {response.error_code.val}')
        pose = response.pose_stamped[0].pose
        return np.array([pose.position.x, pose.position.y, pose.position.z]), rotation_matrix(pose.orientation)

    def _target(self, constraint):
        """The target's planning-frame position and uncertainty, with where it came from."""
        target = constraint['target']
        if 'point' in target:
            point = PointStamped()
            point.header.frame_id = target['point']['frame']
            point.point.x, point.point.y, point.point.z = target['point']['xyz']
            try:
                transform = self._tf.lookup_transform(self._frame, point.header.frame_id, Time(),
                                                      timeout=Duration(seconds=0.2))
            except TransformException as error:
                raise Unavailable('transform_unavailable', str(error)[:200]) from error
            moved = do_transform_point(point, transform).point
            return [moved.x, moved.y, moved.z], 0.0, {'point': target['point']}
        reference = target['observation']
        request = QueryObservations.Request(request_json=json.dumps({'observation_ids': [reference['id']], 'limit': 1}))
        response = self._call(self._observations, request, 'the observation store')
        records = json.loads(response.response_json).get('observations', []) if response.success else []
        if len(records) != 1:
            raise Unavailable('observation_unavailable', 'the observation is unknown or expired')
        record = records[0]
        detections = record.get('detections', [])
        if reference['detection'] >= len(detections):
            raise Unavailable('observation_unavailable', 'the observation has no such detection')
        position = detections[reference['detection']].get('position')
        if not isinstance(position, dict) or position.get('mode') != 'camera':
            raise Unavailable('position_unavailable', 'the detection has no calibrated metric position')
        stamp = Time(seconds=record['stamp']['sec'], nanoseconds=record['stamp']['nanosec'],
                     clock_type=self.get_clock().clock_type)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not -1.0 <= age <= self._observation_age:
            # A frozen estimate is never used as if it were current (spec 17.4).
            raise Unavailable('observation_stale', f'the observation is {age:.1f} s old')
        point = PointStamped()
        point.header.frame_id, point.header.stamp = position['frame_id'], stamp.to_msg()
        point.point.x, point.point.y, point.point.z = position['xyz']
        try:
            transform = self._tf.lookup_transform_full(self._frame, Time(), point.header.frame_id, stamp,
                                                       self._fixed, timeout=Duration(seconds=0.2))
        except TransformException as error:
            raise Unavailable('transform_unavailable', str(error)[:200]) from error
        moved = do_transform_point(point, transform).point
        source = {'observation': reference, 'label': detections[reference['detection']].get('label'),
                  'captured_at': record['stamp'], 'meaning': position.get('meaning')}
        return [moved.x, moved.y, moved.z], float(position.get('uncertainty_m', 0.0)), source

    # ------------------------------------------------------------ operations

    def _resolve(self, request):
        resolved, sources = [], []
        for constraint in request['constraints']:
            if constraint['kind'] == 'orientation':
                resolved.append(resolve(constraint))
                sources.append(None)
                continue
            center, uncertainty, source = self._target(constraint)
            resolved.append(resolve(constraint, center, uncertainty))
            sources.append({**source, 'center': [float(item) for item in resolved[-1].center],
                            'uncertainty_m': uncertainty})
        return resolved, sources

    @staticmethod
    def _pose(position, rotation):
        return {'position': [float(item) for item in position],
                'axes': {name: [float(item) for item in rotation[:, index]] for index, name in enumerate('xyz')}}

    def _verify(self, request, joints, sources, resolved):
        position, rotation = self._fk([joints[name] for name in GROUP_JOINTS], joints['arm_gripper'])
        verdict, details = judge(resolved, position, rotation)
        return {'status': verdict, 'reason': verdict, 'constraints': details, 'targets': sources,
                'pose': self._pose(position, rotation)}

    def _valid(self, joints, gripper):
        request = GetStateValidity.Request()
        request.group_name = self._group
        request.robot_state = self._state(joints, gripper)
        request.robot_state.is_diff = True
        response = self._call(self._validity_client, request, 'MoveIt state validity (/check_state_validity)')
        return response.valid, sorted({f'{item.contact_body_1}|{item.contact_body_2}' for item in response.contacts})[:8]

    def _plan(self, joints, gripper):
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = self._group
        plan.num_planning_attempts, plan.allowed_planning_time = 1, 2.0
        plan.start_state = self._state([self._current()[0][name] for name in GROUP_JOINTS], gripper)
        plan.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for name, value in zip(GROUP_JOINTS, joints)])]
        response = self._call(self._plan_client, request, 'MoveIt motion planning (/plan_kinematic_path)')
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        points = result.trajectory.joint_trajectory.points
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9 if points else 0.0
        return {'planning_time_sec': result.planning_time, 'waypoints': len(points), 'duration_sec': duration}

    def _solve(self, request, joints, sources, resolved, deadline):
        refusal = solvable(resolved)
        if refusal:
            return {'status': 'unknown', 'reason': 'uncertainty_exceeds_tolerance', 'detail': refusal, 'targets': sources}
        current = np.array([joints[name] for name in GROUP_JOINTS])
        candidates, used, stopped = self._search.solve(resolved, current, deadline)
        if not candidates and stopped == 'deadline':
            # Cut short by the caller's time: nothing was learned about feasibility.
            return {'status': 'unknown', 'reason': 'search_timeout', 'targets': sources, 'evaluations': used,
                    'detail': f'the search stopped at max_time_sec after {used} kinematic evaluations'}
        if not candidates:
            return {'status': 'infeasible', 'reason': 'no_kinematic_solution', 'targets': sources,
                    'detail': f'no joint values within limits satisfy the constraints after {used} kinematic '
                              'evaluations; the search is not complete'}
        rejected = []
        for candidate in candidates[:4]:
            gripper = joints['arm_gripper']
            # MoveIt's own FK, without the solver's margin, must find every constraint satisfied.
            position, rotation = self._fk(candidate, gripper)
            verdict, details = judge(resolved, position, rotation)
            if verdict != 'holds':
                rejected.append({'reason': 'constraints_not_held', 'constraints': details})
                continue
            valid, contacts = self._valid(candidate, gripper)
            if not valid:
                rejected.append({'reason': 'in_collision_or_out_of_bounds', 'contacts': contacts})
                continue
            motion = None
            if request['plan']:
                motion = self._plan(candidate, gripper)
                if motion is None:
                    rejected.append({'reason': 'no_motion_plan'})
                    continue
            return {'status': 'feasible', 'reason': 'verified', 'targets': sources, 'constraints': details,
                    'joints': {name: float(item) for name, item in zip(GROUP_JOINTS, candidate)},
                    'pose': self._pose(position, rotation), 'motion': motion, 'evaluations': used}
        reasons = {item['reason'] for item in rejected}
        reason = 'in_collision' if reasons == {'in_collision_or_out_of_bounds'} else (
            'no_motion_plan' if reasons == {'no_motion_plan'} else 'no_verified_solution')
        return {'status': 'infeasible', 'reason': reason, 'targets': sources, 'rejected': rejected,
                'evaluations': used}

    def _handle(self, request, response):
        response.schema = SCHEMA
        if request.schema != SCHEMA:
            response.ok, response.error = False, 'schema_incompatible'
            response.detail = f'motion schema {SCHEMA} only'
            return response
        try:
            values = json.loads(request.request_json)
            parsed = parse_request(values)
        except (ValueError, RecursionError) as error:
            response.ok, response.error, response.detail = False, 'invalid_request', str(error)[:300]
            return response
        started = time.monotonic()
        with self._serial:
            deadline = started + parsed['max_time_sec']
            answer = {'schema': SCHEMA, 'op': parsed['op'], 'planning_frame': self._frame, 'tool_link': self._tool}
            try:
                joints, _, stamp = self._current()
                answer['joint_state'] = {'stamp_sec': stamp, 'positions': joints}
                resolved, sources = self._resolve(parsed)
                if parsed['op'] == 'verify':
                    answer.update(self._verify(parsed, joints, sources, resolved))
                else:
                    answer.update(self._solve(parsed, joints, sources, resolved, deadline))
            except Unavailable as error:
                answer.update({'status': 'unknown', 'reason': error.args[0], 'detail': error.args[1]})
            now = self.get_clock().now().nanoseconds / 1e9
            # An answer describes this scene and this state; a plan rechecks before it moves (spec 17.3).
            answer.update({'checked_at': now, 'valid_until': now + self._validity,
                           'elapsed_sec': time.monotonic() - started,
                           'assumptions': ['the planning scene as MoveIt holds it now; recheck before motion',
                                           *(['object positions are visible-surface points measured at capture time']
                                             if any(source and 'observation' in source for source in answer.get('targets') or [])
                                             else [])]})
        response.ok, response.response_json = True, json.dumps(answer, allow_nan=False)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionSolver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
