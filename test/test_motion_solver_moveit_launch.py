"""/robot/motion/solve against the repository's real move_group: KDL kinematics, FCL collisions, OMPL.

Only move_group runs; no driver, controller or hardware. The test publishes joint states
itself, so nothing can move. It shows that solutions come from MoveIt's own model of this
body, that the answer is verified by MoveIt's collision check and planner, and that the
search refuses what this arm cannot do.
"""

import json
import math
from pathlib import Path
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.srv import GetPositionFK, GetStateValidity
import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.motion_solver import GROUP_JOINTS, SERVICE, MotionSolver, rotation_matrix
from lekiwi_rmf.srv import TaskApi
from ros_test_utils import release_graph

# Outside the production keep-out, as in the MoveIt/driver end-to-end test.
START = [-0.006, 1.0, -1.0, 0.0, -0.02]
OTHER = [0.5, 0.6, -0.4, 0.4, 0.3]


@pytest.mark.rostest
def generate_test_description():
    root = Path(__file__).parents[1]
    config = (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_rmf")
        .robot_description(file_path=str(root / "urdf" / "lekiwi.urdf.xacro"), mappings={"sim": "false"})
        .robot_description_semantic(file_path="config/lekiwi.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    move_group = launch_ros.actions.Node(
        package="moveit_ros_move_group", executable="move_group", name="move_group", output="screen",
        parameters=[config.to_dict(), {"octomap_resolution": 0.1}])
    return launch.LaunchDescription([move_group, launch_testing.actions.ReadyToTest()]), {"move_group": move_group}


class TestMotionSolverOverMoveIt(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.solver = MotionSolver(parameter_overrides=[
            Parameter('service_timeout_sec', value=5.0)])
        cls.node = rclpy.create_node('motion_solver_moveit_client')
        cls.client = cls.node.create_client(TaskApi, SERVICE)
        cls.fk_client = cls.node.create_client(GetPositionFK, '/compute_fk')
        cls.validity = cls.node.create_client(GetStateValidity, '/check_state_validity')
        cls.joint_states = cls.node.create_publisher(JointState, '/joint_states', 10)
        cls.positions = [*START, 0.0]
        cls.timer = cls.node.create_timer(0.05, cls._publish)
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.solver)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        for client in (cls.client, cls.fk_client, cls.validity):
            assert client.wait_for_service(timeout_sec=30)
        assert cls.solver._fk_client.wait_for_service(timeout_sec=30)

    @classmethod
    def _publish(cls):
        message = JointState(name=list(ARM_JOINTS), position=list(cls.positions))
        message.header.stamp = cls.node.get_clock().now().to_msg()
        cls.joint_states.publish(message)

    @classmethod
    def tearDownClass(cls):
        release_graph(executor=cls.executor, thread=cls.thread, nodes=(cls.node, cls.solver))
        rclpy.shutdown()

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def call(self, values, timeout=30):
        reply = self._wait(self.client.call_async(TaskApi.Request(schema=1, request_json=json.dumps(values))), timeout)
        self.assertTrue(reply.ok, reply.detail)
        return json.loads(reply.response_json)

    def fk(self, joints):
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_footprint'
        request.fk_link_names = ['tool0']
        request.robot_state.joint_state.name = list(GROUP_JOINTS)
        request.robot_state.joint_state.position = list(joints)
        pose = self._wait(self.fk_client.call_async(request), 10).pose_stamped[0].pose
        return np.array([pose.position.x, pose.position.y, pose.position.z]), rotation_matrix(pose.orientation)

    def valid(self, joints):
        request = GetStateValidity.Request()
        request.group_name = 'arm'
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(GROUP_JOINTS)
        request.robot_state.joint_state.position = list(joints)
        return self._wait(self.validity.call_async(request), 10).valid

    def test_a_reachable_pose_is_solved_verified_and_planned_by_moveit(self):
        self.assertTrue(self.valid(START) and self.valid(OTHER))
        position, rotation = self.fk(OTHER)
        started = time.monotonic()
        answer = self.call({'op': 'solve', 'max_time_sec': 10, 'constraints': [
            {'kind': 'position', 'target': {'point': {'frame': 'base_footprint', 'xyz': position.tolist()}}, 'radius_m': 0.03},
            {'kind': 'orientation', 'axis': [0, 0, 1], 'direction': rotation[:, 2].tolist(), 'tolerance_rad': 0.25}]})
        self.assertEqual(answer['status'], 'feasible', answer)
        self.assertLess(time.monotonic() - started, 12)
        joints = [answer['joints'][name] for name in GROUP_JOINTS]
        # Independently of the solver: MoveIt's FK and collision check agree with its answer.
        reached, turned = self.fk(joints)
        self.assertLessEqual(np.linalg.norm(reached - position), 0.03)
        self.assertLessEqual(math.acos(np.clip(np.dot(turned[:, 2], rotation[:, 2]), -1, 1)), 0.25)
        self.assertTrue(self.valid(joints))
        self.assertGreater(answer['motion']['waypoints'], 0)

    def test_a_point_beyond_the_arm_is_infeasible(self):
        answer = self.call({'op': 'solve', 'max_time_sec': 6, 'constraints': [
            {'kind': 'position', 'target': {'point': {'frame': 'base_footprint', 'xyz': [1.2, 0.0, 0.3]}}, 'radius_m': 0.05}]})
        self.assertEqual((answer['status'], answer['reason']), ('infeasible', 'no_kinematic_solution'), answer)

    def test_the_current_pose_verifies_and_a_distant_region_does_not(self):
        position, _ = self.fk(START)
        here = self.call({'op': 'verify', 'constraints': [
            {'kind': 'position', 'target': {'point': {'frame': 'base_footprint', 'xyz': position.tolist()}}, 'radius_m': 0.01}]})
        self.assertEqual(here['status'], 'holds', here)
        there = self.call({'op': 'verify', 'constraints': [
            {'kind': 'position', 'target': {'point': {'frame': 'base_footprint', 'xyz': (position + [0, 0, 0.2]).tolist()}},
             'radius_m': 0.05}]})
        self.assertEqual(there['status'], 'violated', there)
