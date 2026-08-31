"""Exercise the installed command mux as a live ROS graph.

This deliberately launches the real executable instead of constructing its
class directly.  It verifies the default-deny safety subscription, including
its transient-local delivery contract, and that revoking permission stops an
otherwise fresh navigation command without physical hardware.
"""

from __future__ import annotations

import time
import unittest

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from ros_test_utils import spin_until


@pytest.mark.rostest
def generate_test_description():
    mux = launch_ros.actions.Node(
        package="lekiwi_rmf",
        executable="cmd_vel_mux",
        name="cmd_vel_mux_launch_test",
        parameters=[{
            "manual_topic": "/test/cmd_vel_manual",
            "navigation_topic": "/test/cmd_vel_navigation",
            "output_topic": "/test/cmd_vel_muxed",
            "permission_topic": "/test/base_motion_permitted",
            "manual_timeout": 0.5,
            "navigation_timeout": 0.5,
            "permission_timeout": 0.2,
            "publish_frequency": 40.0,
        }],
        output="screen",
    )
    return launch.LaunchDescription([
        mux,
        launch_testing.actions.ReadyToTest(),
    ]), {"mux": mux}


class TestCmdVelMuxInterlock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("cmd_vel_mux_graph_client")
        self.received: list[Twist] = []
        permission_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.permission = self.node.create_publisher(
            Bool, "/test/base_motion_permitted", permission_qos
        )
        self.navigation = self.node.create_publisher(Twist, "/test/cmd_vel_navigation", 10)
        self.output = self.node.create_subscription(
            Twist, "/test/cmd_vel_muxed", self.received.append, 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def _publish_permission(self, allowed: bool) -> None:
        message = Bool()
        message.data = allowed
        self.permission.publish(message)

    def _publish_navigation(self, speed: float) -> None:
        message = Twist()
        message.linear.x = speed
        self.navigation.publish(message)

    def test_permission_is_a_live_default_deny_interlock(self):
        self.assertTrue(spin_until(self.node,
            lambda: self.permission.get_subscription_count() == 1
            and self.navigation.get_subscription_count() == 1
            and self.node.count_publishers("/test/cmd_vel_muxed") == 1
        ))

        self._publish_permission(False)
        for _ in range(4):
            self._publish_navigation(0.2)
            time.sleep(0.03)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(spin_until(self.node, lambda: bool(self.received)))
        self.assertTrue(all(message.linear.x == 0.0 for message in self.received))

        self.received.clear()
        self._publish_permission(True)
        self.assertTrue(spin_until(self.node, lambda: self.permission.get_subscription_count() == 1))
        self.assertTrue(spin_until(self.node,
            lambda: self._publish_navigation(0.2) is None
            and any(message.linear.x == 0.2 for message in self.received)
        ))

        # A latched true value is not an unbounded authorization.  Stop
        # refreshing it and verify that the live mux expires to zero.
        self.received.clear()
        self.assertTrue(spin_until(self.node,
            lambda: any(message.linear.x == 0.0 for message in self.received),
            timeout=2.0,
        ))

        self.received.clear()
        self._publish_permission(False)
        self.assertTrue(spin_until(self.node,
            lambda: any(message.linear.x == 0.0 for message in self.received)
        ))
