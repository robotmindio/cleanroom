"""Verify continuous safety withdrawal and reset in a live ROS graph."""

from __future__ import annotations

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.asserts
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


@pytest.mark.rostest
def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package="lekiwi_rmf",
        executable="safety_supervisor",
        namespace="safety_integration",
        name="safety_supervisor",
        parameters=[{
            "publish_frequency": 40.0,
            "sensor_timeout": 0.20,
            "state_timeout": 1.0,
            "require_driver_state": True,
            "require_acceptance": False,
            "require_scan": True,
            "require_full_scan": True,
            "require_depth": False,
            "require_bumper": False,
            "require_estop": False,
            "require_battery": False,
            "require_motor_health": False,
            "require_odometry": False,
            "require_imu": False,
            "require_joint_states": False,
            "require_arm_workspace": False,
        }],
        output="screen",
    )
    return launch.LaunchDescription([
        supervisor,
        launch_testing.actions.ReadyToTest(),
    ]), {"supervisor": supervisor}


class TestSafetySupervisorGraph(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("safety_supervisor_graph_client")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.driver = self.node.create_publisher(
            String, "/safety_integration/safety/driver_state", latched
        )
        self.scan = self.node.create_publisher(LaserScan, "/scan", 10)
        self.states: list[str] = []
        self.base: list[bool] = []
        self.arm: list[bool] = []
        self.node.create_subscription(
            String,
            "/safety_integration/safety/supervisor_state",
            lambda message: self.states.append(message.data),
            latched,
        )
        self.node.create_subscription(
            Bool,
            "/safety_integration/safety/base_motion_permitted",
            lambda message: self.base.append(message.data),
            latched,
        )
        self.node.create_subscription(
            Bool,
            "/safety_integration/safety/arm_motion_permitted",
            lambda message: self.arm.append(message.data),
            latched,
        )
        self.reset = self.node.create_client(
            Trigger, "/safety_integration/safety/reset_fault"
        )

    def tearDown(self):
        self.node.destroy_node()

    def _until(self, predicate, timeout: float = 4.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.04)
            if predicate():
                return True
        return False

    def _publish_until(self, driver_state: str, predicate, timeout: float = 4.0) -> bool:
        """Survive normal best-effort discovery loss while reaching a test state."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._driver(driver_state)
            self._scan()
            rclpy.spin_once(self.node, timeout_sec=0.04)
            if predicate():
                return True
        return False

    def _driver(self, state: str) -> None:
        message = String()
        message.data = state
        self.driver.publish(message)

    def _scan(self) -> None:
        message = LaserScan()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "laser"
        message.angle_min = -math.pi
        message.angle_max = math.pi
        message.angle_increment = 2.0 * math.pi / 359.0
        message.range_min = 0.05
        message.range_max = 12.0
        message.ranges = [2.0] * 360
        self.scan.publish(message)

    def _reset(self):
        self.assertTrue(self.reset.wait_for_service(timeout_sec=2.0))
        future = self.reset.call_async(Trigger.Request())
        self.assertTrue(self._until(future.done, timeout=2.0))
        return future.result()

    def test_stale_input_withdraws_permission_and_latches(self):
        self.assertTrue(self._until(
            lambda: self.driver.get_subscription_count() == 1
            and self.scan.get_subscription_count() == 1
            and bool(self.states)
            and bool(self.base)
            and bool(self.arm)
        ))
        self.assertEqual(self.states[-1], "BOOT")
        self.assertFalse(self.base[-1])
        self.assertFalse(self.arm[-1])

        became_ready = self._publish_until(
            "DISARMED",
            lambda: bool(self.states) and bool(self.arm) and bool(self.base)
            and self.states[-1] == "READY" and self.arm[-1] and self.base[-1],
        )
        self.assertTrue(
            became_ready,
            f"states={self.states[-8:]}, base={self.base[-8:]}, arm={self.arm[-8:]}",
        )

        self.assertTrue(self._publish_until(
            "ARMED",
            lambda: bool(self.states) and bool(self.arm) and bool(self.base)
            and self.states[-1] == "ARMED" and self.base[-1] and self.arm[-1],
        ))

        self.assertTrue(self._until(
            lambda: bool(self.states) and bool(self.arm) and bool(self.base)
            and self.states[-1] == "FAULT_LATCHED"
            and not self.base[-1]
            and not self.arm[-1],
            timeout=2.0,
        ))

        self._driver("DISARMED")
        self._scan()
        time.sleep(0.02)
        self.assertEqual(self.states[-1], "FAULT_LATCHED")
        result = self._reset()
        self.assertTrue(result.success, result.message)
        self.assertTrue(self._until(
            lambda: bool(self.states) and bool(self.arm) and bool(self.base)
            and self.states[-1] == "READY" and self.arm[-1] and self.base[-1]
        ))


@launch_testing.post_shutdown_test()
class TestSafetySupervisorExit(unittest.TestCase):
    def test_clean_exit(self, proc_info, supervisor):
        launch_testing.asserts.assertExitCodes(proc_info, process=supervisor)
