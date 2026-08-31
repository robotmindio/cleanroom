"""MoveIt-to-driver end-to-end test using only a loopback fake motor host.

The test deliberately launches the installed ``lekiwi_driver`` and its real
FollowJointTrajectory action server.  ``FakeLeKiwiHost`` owns ephemeral
loopback ZMQ endpoints and copies accepted arm commands into later telemetry,
which closes the execution feedback loop without permitting hardware access.
"""

from __future__ import annotations

import json
import math
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest

pytest.importorskip("zmq")

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import AllowedCollisionEntry, Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.fake_host import FakeLeKiwiHost
from lekiwi_rmf.moveit_config import moveit_config_builder
from ros_test_utils import spin_until


# At the deterministic fake pose below (pan/roll/gripper=0, lift=-55°, elbow=85°,
# flex=-30°), MoveIt reports these conservative CAD-envelope contacts.  They
# are allowed only in this test scene so it can cover transport/execution; they
# remain enabled in the production collision model pending a physically
# verified collision-free calibration pose.
TEST_ONLY_COLLISION_PAIRS = frozenset({
    frozenset(pair) for pair in (
        ("arm_pedestal_collision_proxy", "front_camera_collision_proxy"),
        ("arm_pedestal_collision_proxy", "shoulder_collision_proxy"),
        ("arm_pedestal_collision_proxy", "shoulder_lift_servo_collision_proxy"),
        ("arm_pedestal_collision_proxy", "upper_arm_collision_proxy"),
        ("forearm_collision_proxy", "upper_arm_collision_proxy"),
        ("forearm_collision_proxy", "wrist_collision_proxy"),
        ("forearm_collision_proxy", "wrist_roll_servo_collision_proxy"),
        ("gripper_collision_proxy", "tool0"),
        ("gripper_collision_proxy", "wrist_collision_proxy"),
        ("gripper_servo_collision_proxy", "tool0"),
        ("gripper_servo_collision_proxy", "wrist_collision_proxy"),
        ("gripper_servo_collision_proxy", "wrist_roll_servo_collision_proxy"),
        ("roll_collision_proxy", "wrist_collision_proxy"),
        ("shoulder_collision_proxy", "upper_arm_collision_proxy"),
        ("tool0", "wrist_collision_proxy"),
        ("tool0", "wrist_roll_servo_collision_proxy"),
        ("wrist_flex_servo_collision_proxy", "wrist_roll_servo_collision_proxy"),
        # A packet-boundary variant of the same fake startup pose can retain
        # the host's all-zero feedback for one driver cycle.  These are the
        # exact additional contacts MoveIt reports in that case.
        ("arm_pedestal_collision_proxy", "base_link"),
        ("arm_pedestal_collision_proxy", "shoulder_pan_servo_collision_proxy"),
        ("base_link", "front_camera_collision_proxy"),
        ("elbow_servo_collision_proxy", "forearm_collision_proxy"),
        ("elbow_servo_collision_proxy", "upper_arm_collision_proxy"),
        ("forearm_collision_proxy", "wrist_flex_servo_collision_proxy"),
        ("gripper_collision_proxy", "gripper_servo_collision_proxy"),
        ("gripper_collision_proxy", "roll_collision_proxy"),
        ("gripper_servo_collision_proxy", "roll_collision_proxy"),
        ("roll_collision_proxy", "tool0"),
        ("roll_collision_proxy", "wrist_roll_servo_collision_proxy"),
        ("shoulder_collision_proxy", "shoulder_lift_servo_collision_proxy"),
        ("shoulder_collision_proxy", "shoulder_pan_servo_collision_proxy"),
        ("shoulder_lift_servo_collision_proxy", "upper_arm_collision_proxy"),
        ("wrist_collision_proxy", "wrist_flex_servo_collision_proxy"),
        ("wrist_collision_proxy", "wrist_roll_servo_collision_proxy"),
    )
})


def _identity_calibration() -> tuple[str, tempfile.TemporaryDirectory]:
    """Create an explicit valid calibration without reading or writing HOME."""
    temporary_directory = tempfile.TemporaryDirectory(prefix="lekiwi-moveit-e2e-")
    directory = Path(temporary_directory.name)
    calibration = directory / "arm-calibration.json"
    calibration.write_text(json.dumps({
        "zero_positions": dict.fromkeys(ARM_JOINTS, 0.0),
        "directions": dict.fromkeys(ARM_JOINTS, 1.0),
    }))
    return str(calibration), temporary_directory


@pytest.mark.rostest
def generate_test_description():
    fake_host = FakeLeKiwiHost()
    # The CAD collision proxies intentionally reject the folded all-zero pose.
    # Begin from this measured, unfolded arm pose so MoveIt can validate the
    # real current state before planning the small pan motion below.
    fake_host.set_state(**{
        "arm_shoulder_lift.pos": -55.0,
        "arm_elbow_flex.pos": 85.0,
        "arm_wrist_flex.pos": -30.0,
    })
    fake_host.start(period_s=0.02)
    calibration, calibration_directory = _identity_calibration()

    # This is the repository MoveIt configuration, except that perception is
    # omitted solely for this isolated test.  The local test image intentionally
    # lacks moveit_ros_perception; the production launch retains sensors_3d.
    moveit_config = moveit_config_builder("false").to_moveit_configs()
    moveit_parameters = moveit_config.to_dict()
    robot_links = tuple(
        link.attrib["name"] for link in ET.fromstring(moveit_parameters["robot_description"])
        if "name" in link.attrib
    )
    move_group = launch_ros.actions.Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_parameters, {"octomap_resolution": 0.1}],
    )
    driver = launch_ros.actions.Node(
        package="lekiwi_rmf",
        executable="lekiwi_driver",
        name="lekiwi_driver",
        output="screen",
        parameters=[{
            "remote_ip": "127.0.0.1",
            "remote_command_port": fake_host.command_endpoint_port,
            "remote_observation_port": fake_host.observation_endpoint_port,
            "torque_control_port": fake_host.torque_endpoint_port,
            "torque_control_timeout_ms": 500,
            "link_timeout": 1.0,
            "command_timeout": 2.0,
            "permission_timeout": 0.30,
            "arm_motion_permission_topic": "/test/moveit/arm_permitted",
            "base_motion_permission_topic": "/test/moveit/base_permitted",
            "arm_calibration_file": calibration,
            "auto_arm_on_startup": False,
        }],
    )
    success = threading.Event()
    return launch.LaunchDescription([
        driver,
        move_group,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "calibration": calibration,
        "calibration_directory": calibration_directory,
        "driver": driver,
        "fake_host": fake_host,
        "move_group": move_group,
        "robot_links": robot_links,
        "success": success,
    }


class TestMoveItDriverEndToEnd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("moveit_driver_e2e_client")
        lease_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.arm_permission = self.node.create_publisher(
            Bool, "/test/moveit/arm_permitted", lease_qos
        )
        self.joint_states: list[JointState] = []
        self.node.create_subscription(JointState, "/joint_states", self.joint_states.append, 10)
        self.arm_client = self.node.create_client(Trigger, "/safety/arm")
        self.apply_scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")
        # Permissions are receive-time leases, so this timer models the safety
        # supervisor's continuous authorization for the whole action.
        self.permission_timer = self.node.create_timer(0.05, self._publish_permission)

    def tearDown(self):
        self.node.destroy_timer(self.permission_timer)
        self.node.destroy_node()

    def _publish_permission(self):
        permission = Bool()
        permission.data = True
        self.arm_permission.publish(permission)

    def _spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _arm(self):
        self.assertTrue(self.arm_client.wait_for_service(timeout_sec=10.0))
        # Publish while discovery settles; the timer keeps this lease fresh
        # once the service call has succeeded.
        self._publish_permission()
        future = self.arm_client.call_async(Trigger.Request())
        self.assertTrue(spin_until(self.node, future.done, timeout=10.0))
        self.assertTrue(future.result().success, future.result().message)

    def _allow_test_only_self_collisions(self, robot_links):
        self.assertTrue(self.apply_scene.wait_for_service(timeout_sec=15.0))
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        matrix = request.scene.allowed_collision_matrix
        involved_links = tuple(sorted({
            link for pair in TEST_ONLY_COLLISION_PAIRS for link in pair
        }))
        self.assertTrue(set(involved_links) <= set(robot_links))
        matrix.entry_names = list(involved_links)
        matrix.entry_values = [
            AllowedCollisionEntry(enabled=[
                frozenset((first, second)) in TEST_ONLY_COLLISION_PAIRS
                for second in involved_links
            ])
            for first in involved_links
        ]
        future = self.apply_scene.call_async(request)
        self.assertTrue(spin_until(self.node, future.done, timeout=10.0))
        self.assertTrue(future.result().success)

    def test_move_group_executes_joint_space_motion(self, fake_host, robot_links, success):
        self.assertTrue(spin_until(self.node,
            lambda: bool(self.joint_states)
            and self.arm_permission.get_subscription_count() == 1,
            timeout=15.0,
        ))
        self.assertTrue(spin_until(self.node, self.move_group.server_is_ready, timeout=15.0))
        # The CAD-backed collision envelopes conservatively overlap in every
        # deterministic fake feedback pose.  Make only this test graph's
        # planning scene permissive so it exercises plan-to-controller
        # transport without modifying production collision or sensor settings.
        self._allow_test_only_self_collisions(robot_links)
        # Drain the fake transport's pre-arm torque-off telemetry, then arm on
        # a fresh sample.  This models the explicit operator re-arm required
        # after a host state transition and avoids treating queued feedback as
        # permission to move.
        self._arm()
        self._spin_for(0.30)
        self._arm()
        self.assertTrue(fake_host.torque_enabled)

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "arm"
        request.num_planning_attempts = 1
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.15
        request.max_acceleration_scaling_factor = 0.15
        target_positions = {
            "arm_shoulder_pan": 0.12,
            "arm_shoulder_lift": math.radians(-55.0),
            "arm_elbow_flex": math.radians(85.0),
            "arm_wrist_flex": math.radians(-30.0),
            "arm_wrist_roll": 0.0,
        }
        request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, position in target_positions.items()
        ])]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        goal_future = self.move_group.send_goal_async(goal)
        self.assertTrue(spin_until(self.node, goal_future.done, timeout=15.0))
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted, "move_group rejected the plan-and-execute goal")
        result_future = goal_handle.get_result_async()
        self.assertTrue(spin_until(self.node, result_future.done, timeout=30.0))
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.error_code.val, MoveItErrorCodes.SUCCESS)

        target_degrees = math.degrees(0.12)
        self.assertTrue(spin_until(self.node,
            lambda: any(
                action.get("arm_shoulder_pan.pos", 0.0) == pytest.approx(target_degrees, abs=1.0)
                for action in fake_host.actions
            ),
            timeout=10.0,
        ))
        self.assertTrue(spin_until(self.node,
            lambda: any(
                "arm_shoulder_pan" in state.name
                and state.position[state.name.index("arm_shoulder_pan")] == pytest.approx(0.12, abs=0.03)
                for state in self.joint_states
            ),
            timeout=10.0,
        ))
        success.set()


@launch_testing.post_shutdown_test()
class TestMoveItDriverTeardown(unittest.TestCase):
    def test_processes_exit_after_success(
        self, proc_info, driver, fake_host, calibration_directory, success
    ):
        # This test qualifies plan-to-driver transport, not MoveIt's shutdown.
        # The separate scripts/moveit-shutdown-probe.py qualification must
        # report move_group_exit_code=0 and clean_shutdown=true; it deliberately
        # fails on the Jazzy 2.12.4 destructor crash instead of suppressing it.
        # Still require the repository driver itself to exit cleanly here.
        try:
            if success.is_set():
                launch_testing.asserts.assertExitCodes(proc_info, process=driver)
        finally:
            fake_host.close()
            calibration_directory.cleanup()
