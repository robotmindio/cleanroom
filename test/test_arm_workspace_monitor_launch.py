"""Live graph test for continuous MoveIt state-validity gating."""

from __future__ import annotations

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetStateValidity
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


@pytest.mark.rostest
def generate_test_description():
    monitor = launch_ros.actions.Node(
        package="lekiwi_rmf",
        executable="arm_workspace_monitor",
        name="arm_workspace_monitor_launch_test",
        parameters=[{
            "joint_names": ["joint_a", "joint_b"],
            "check_frequency": 40.0,
            "joint_timeout": 0.20,
            "planning_scene_timeout": 0.20,
            "validity_timeout": 0.15,
            "service_timeout": 0.10,
            "joint_state_topic": "/test/arm_workspace/joints",
            "planning_scene_topic": "/test/arm_workspace/scene",
            "state_validity_service": "/test/arm_workspace/check_state_validity",
            "output_topic": "/test/arm_workspace/clear",
        }],
        output="screen",
    )
    return launch.LaunchDescription([
        monitor,
        launch_testing.actions.ReadyToTest(),
    ]), {"monitor": monitor}


class TestArmWorkspaceMonitorGraph(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("arm_workspace_monitor_graph_client")
        self.valid = True
        self.requests = []
        self.joints = self.node.create_publisher(
            JointState, "/test/arm_workspace/joints", 10
        )
        self.scene = self.node.create_publisher(
            PlanningScene, "/test/arm_workspace/scene", 10
        )
        self.service = self.node.create_service(
            GetStateValidity,
            "/test/arm_workspace/check_state_validity",
            self._check,
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clear = []
        self.diagnostics = []
        self.node.create_subscription(
            Bool, "/test/arm_workspace/clear", lambda message: self.clear.append(message.data), latched
        )
        self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self.diagnostics.append, 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def _check(self, request, response):
        self.requests.append(request)
        response.valid = self.valid
        return response

    def _publish_inputs(self):
        joints = JointState()
        joints.header.stamp = self.node.get_clock().now().to_msg()
        joints.name = ["joint_a", "joint_b"]
        joints.position = [0.1, -0.2]
        self.joints.publish(joints)
        scene = PlanningScene()
        scene.world.octomap.octomap.header.stamp = self.node.get_clock().now().to_msg()
        self.scene.publish(scene)

    def _until(self, predicate, timeout=5.0, publish=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                self._publish_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if predicate():
                return True
        return False

    def test_collision_and_silence_withdraw_workspace_permission(self):
        self.assertTrue(self._until(
            lambda: self.joints.get_subscription_count() == 1
            and self.scene.get_subscription_count() == 1
            and bool(self.clear)
        ))
        self.assertTrue(self._until(lambda: any(self.clear) and bool(self.requests)))
        request = self.requests[-1]
        self.assertEqual(request.group_name, "arm")
        self.assertEqual(list(request.robot_state.joint_state.name), ["joint_a", "joint_b"])

        self.valid = False
        start = len(self.clear)
        self.assertTrue(self._until(lambda: False in self.clear[start:]))

        self.valid = True
        self.assertTrue(self._until(lambda: self.clear and self.clear[-1] is True))
        start = len(self.clear)
        self.assertTrue(self._until(
            lambda: False in self.clear[start:], timeout=2.0, publish=False
        ))
        self.assertTrue(any(
            status.name == "lekiwi/arm_workspace_monitor"
            for message in self.diagnostics for status in message.status
        ))


@launch_testing.post_shutdown_test()
class TestArmWorkspaceMonitorExit(unittest.TestCase):
    def test_clean_exit(self, proc_info, monitor):
        launch_testing.asserts.assertExitCodes(proc_info, process=monitor)
