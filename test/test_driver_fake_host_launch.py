"""Exercise the installed driver against the repository fake motor host.

This is deliberately a live ROS graph and real ZeroMQ transport test.  The
fake binds only ephemeral loopback ports and never imports a motor library, so
the test cannot discover or actuate physical hardware.
"""

from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest

pytest.importorskip("zmq")

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from lekiwi_rmf.fake_host import FakeLeKiwiHost, ObservationFault


@pytest.mark.rostest
def generate_test_description():
    fake_host = FakeLeKiwiHost()
    fake_host.start(period_s=0.02)
    driver = launch_ros.actions.Node(
        package="lekiwi_rmf",
        executable="lekiwi_driver",
        namespace="driver_integration",
        name="lekiwi_driver",
        parameters=[{
            "remote_ip": "127.0.0.1",
            "remote_command_port": fake_host.command_endpoint_port,
            "remote_observation_port": fake_host.observation_endpoint_port,
            "torque_control_port": fake_host.torque_endpoint_port,
            "torque_control_timeout_ms": 500,
            "link_timeout": 0.30,
            "command_timeout": 2.0,
            "permission_timeout": 0.20,
            "cmd_vel_topic": "/test/driver/cmd_vel_safe",
            "odom_topic": "/test/driver/wheel_odometry",
            "base_motion_permission_topic": "/test/driver/base_permitted",
            "arm_motion_permission_topic": "/test/driver/arm_permitted",
            "arm_calibration_file": "/test/intentionally-missing-calibration.json",
            "auto_arm_on_startup": False,
            # The host/driver protocol is tested here. This development image's
            # diagnostic_msgs C extension aborts while serializing any
            # DiagnosticStatus (also reproducible outside this package), so
            # publishing is disabled only for this unrelated launch fixture.
            "publish_motor_health": False,
        }],
        remappings=[("safety/state", "/test/driver/state")],
        output="screen",
    )
    return launch.LaunchDescription([
        driver,
        launch_testing.actions.ReadyToTest(),
    ]), {"driver": driver, "fake_host": fake_host}


class TestDriverFakeHostGraph(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("driver_fake_host_graph_client")
        self.odom: list[Odometry] = []
        self.states: list[str] = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base_permission = self.node.create_publisher(
            Bool, "/test/driver/base_permitted", latched
        )
        self.arm_permission = self.node.create_publisher(
            Bool, "/test/driver/arm_permitted", latched
        )
        self.commands = self.node.create_publisher(
            Twist, "/test/driver/cmd_vel_safe", 10
        )
        self.node.create_subscription(
            Odometry, "/test/driver/wheel_odometry", self.odom.append, 10
        )
        self.node.create_subscription(
            String, "/test/driver/state", lambda message: self.states.append(message.data), latched
        )
        self.arm_client = self.node.create_client(
            Trigger, "/driver_integration/safety/arm"
        )

    def tearDown(self):
        self.node.destroy_node()

    def _until(self, predicate, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _permission(self, publisher, allowed: bool) -> None:
        message = Bool()
        message.data = allowed
        publisher.publish(message)

    def _arm(self):
        self.assertTrue(self.arm_client.wait_for_service(timeout_sec=3.0))
        future = self.arm_client.call_async(Trigger.Request())
        self.assertTrue(self._until(future.done, timeout=3.0))
        return future.result()

    def test_default_deny_motion_restart_and_link_loss(self, fake_host):
        self.assertTrue(self._until(
            lambda: self.commands.get_subscription_count() == 1
            and self.base_permission.get_subscription_count() == 1
            and self.arm_permission.get_subscription_count() == 1
            and bool(self.odom)
            and "DISARMED" in self.states
        ))

        denied = self._arm()
        self.assertFalse(denied.success)
        self.assertFalse(fake_host.torque_enabled)

        self._permission(self.arm_permission, True)
        self.assertTrue(self._until(
            lambda: self.arm_permission.get_subscription_count() == 1
        ))
        # Re-publish while spinning so the test cannot race discovery against
        # the first transient-local sample on a slow CI runner.
        self.assertTrue(self._until(
            lambda: self._permission(self.arm_permission, True) is None
            and self._arm().success,
            timeout=4.0,
        ))
        self.assertTrue(fake_host.torque_enabled)
        self.assertTrue(self._until(lambda: "ARMED" in self.states))

        # Arm permission is a receive-time lease.  A transient-local true
        # sample must not keep the actuator enabled after the supervisor stops.
        state_index = len(self.states)
        self.assertTrue(self._until(
            lambda: not fake_host.torque_enabled
            and "DISARMED" in self.states[state_index:],
            timeout=2.0,
        ))
        self.assertFalse(fake_host.torque_enabled)

        self._permission(self.arm_permission, True)
        self.assertTrue(self._until(lambda: self._arm().success, timeout=4.0))
        self.assertTrue(fake_host.torque_enabled)

        self._permission(self.base_permission, True)
        start = len(fake_host.actions)
        command = Twist()
        command.linear.x = 0.2
        self.assertTrue(self._until(
            lambda: self.commands.publish(command) is None
            and any(action.get("x.vel") == pytest.approx(0.2)
                    for action in fake_host.actions[start:])
        ))

        # Keep arm permission alive while allowing base permission to expire.
        # The command watchdog is deliberately longer than this lease so the
        # zero action demonstrates the permission watchdog itself.
        start = len(fake_host.actions)
        self.assertTrue(self._until(
            lambda: self._permission(self.arm_permission, True) is None
            and any(action.get("x.vel") == 0.0 for action in fake_host.actions[start:]),
            timeout=2.0,
        ))

        start = len(fake_host.actions)
        self._permission(self.base_permission, False)
        self.assertTrue(self._until(
            lambda: any(action.get("x.vel") == 0.0 for action in fake_host.actions[start:])
        ))

        # A host process restart has a new telemetry session and begins with
        # physical torque off.  The driver must not preserve its ARMED state or
        # auto-arm when the first packet from that session arrives.
        state_index = len(self.states)
        fake_host.restart_session()
        self.assertTrue(self._until(
            lambda: not fake_host.torque_enabled
            and "DISARMED" in self.states[state_index:],
            timeout=4.0,
        ))
        self.assertFalse(fake_host.torque_enabled)

        self._permission(self.arm_permission, True)
        self.assertTrue(self._until(lambda: self._arm().success, timeout=4.0))
        self.assertTrue(fake_host.torque_enabled)

        # Silence the source for longer than the driver's link watchdog.  This
        # must cut torque and surface LINK_LOST, even though the ROS graph and
        # cached observation object remain alive.
        state_index = len(self.states)
        fake_host.queue_observation_fault(ObservationFault.DROP, count=30)
        self.assertTrue(self._until(
            lambda: not fake_host.torque_enabled
            and "LINK_LOST" in self.states[state_index:],
            timeout=4.0,
        ))


@launch_testing.post_shutdown_test()
class TestDriverProcessExit(unittest.TestCase):
    def test_driver_exited_and_fake_can_close(self, proc_info, driver, fake_host):
        try:
            launch_testing.asserts.assertExitCodes(proc_info, process=driver)
        finally:
            fake_host.close()
