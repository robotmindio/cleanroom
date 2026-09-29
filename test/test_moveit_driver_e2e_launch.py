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
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest

pytest.importorskip("zmq")

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.fake_host import FakeLeKiwiHost


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
    # Start outside the production keep-out so planning can test a valid transition.
    fake_host.set_state(
        **{
            "arm_shoulder_pan.pos": math.degrees(-0.006),
            "arm_shoulder_lift.pos": math.degrees(1.0),
            "arm_elbow_flex.pos": math.degrees(-1.0),
            "arm_wrist_flex.pos": math.degrees(0.0),
            "arm_wrist_roll.pos": math.degrees(-0.02),
        }
    )
    fake_host.start(period_s=0.02)
    calibration, calibration_directory = _identity_calibration()

    # This is the repository MoveIt configuration, except that perception is
    # omitted solely for this isolated test.  The local test image intentionally
    # lacks moveit_ros_perception; the production launch retains sensors_3d.
    moveit_config = (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_rmf")
        .robot_description(
            file_path=str(Path(__file__).parents[1] / "urdf" / "lekiwi.urdf.xacro"),
            mappings={"sim": "false"},
        )
        .robot_description_semantic(file_path="config/lekiwi.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()
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
        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")
        self.trajectory_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
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

    def _until(self, predicate, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_moveit_state_validity_distinguishes_floor_contact(self):
        client = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.assertTrue(self._until(client.service_is_ready, timeout=15.0))

        below_floor_contacts = []
        for shoulder_lift in (-1.74533, 1.82):
            request = GetStateValidity.Request()
            request.group_name = "arm"
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = list(ARM_JOINTS)
            request.robot_state.joint_state.position = [
                0.0, shoulder_lift, 0.0, 0.0, 0.0, 0.0,
            ]
            future = client.call_async(request)
            self.assertTrue(self._until(future.done, timeout=10.0))
            below_floor_contacts.extend(future.result().contacts)
        self.assertTrue(any(
            "arm_ground_keepout_proxy" in (c.contact_body_1, c.contact_body_2)
            for c in below_floor_contacts
        ), "MoveIt did not report a below-floor arm collision")

        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(ARM_JOINTS)
        # The broad roll capsule falsely hit the keepout here; exact CAD
        # geometry admits this reachable resting pose without removing the floor.
        request.robot_state.joint_state.position = [
            -0.01995, 1.81054, -1.05871, -0.19486, -0.01995, -0.00264,
        ]
        future = client.call_async(request)
        self.assertTrue(self._until(future.done, timeout=10.0))
        response = future.result()
        self.assertTrue(response.valid, response.contacts)

    def _arm(self):
        self.assertTrue(self.arm_client.wait_for_service(timeout_sec=10.0))
        # Publish while discovery settles; the timer keeps this lease fresh
        # once the service call has succeeded.
        self._publish_permission()
        future = self.arm_client.call_async(Trigger.Request())
        self.assertTrue(self._until(future.done, timeout=10.0))
        self.assertTrue(future.result().success, future.result().message)

    def test_move_group_executes_joint_space_motion(self, fake_host, success):
        self.assertTrue(self._until(
            lambda: bool(self.joint_states)
            and self.arm_permission.get_subscription_count() == 1,
            timeout=15.0,
        ))
        self.assertTrue(self._until(self.move_group.server_is_ready, timeout=15.0))

        fk_client = self.node.create_client(GetPositionFK, "/compute_fk")
        ik_client = self.node.create_client(GetPositionIK, "/compute_ik")
        self.assertTrue(self._until(fk_client.service_is_ready, timeout=10.0))
        self.assertTrue(self._until(ik_client.service_is_ready, timeout=10.0))
        fk = GetPositionFK.Request()
        fk.fk_link_names = ["tool0"]
        fk.robot_state.joint_state.name = [
            "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
            "arm_wrist_flex", "arm_wrist_roll",
        ]
        fk.robot_state.joint_state.position = [-0.006, 1.0, -1.0, 0.0, -0.02]
        fk_future = fk_client.call_async(fk)
        self.assertTrue(self._until(fk_future.done, timeout=10.0))
        fk_result = fk_future.result()
        self.assertEqual(fk_result.error_code.val, MoveItErrorCodes.SUCCESS)

        ik = GetPositionIK.Request()
        ik.ik_request.group_name = "arm"
        ik.ik_request.ik_link_name = "tool0"
        ik.ik_request.pose_stamped = fk_result.pose_stamped[0]
        ik.ik_request.robot_state = fk.robot_state
        ik.ik_request.avoid_collisions = True
        ik.ik_request.timeout.sec = 1
        ik_future = ik_client.call_async(ik)
        self.assertTrue(self._until(ik_future.done, timeout=10.0))
        self.assertEqual(ik_future.result().error_code.val, MoveItErrorCodes.SUCCESS)

        # Use the production collision matrix without test-only exemptions.
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
            "arm_shoulder_lift": 1.0,
            "arm_elbow_flex": -1.0,
            "arm_wrist_flex": 0.0,
            "arm_wrist_roll": -0.02,
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
        self.assertTrue(self._until(goal_future.done, timeout=15.0))
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted, "move_group rejected the plan-and-execute goal")
        result_future = goal_handle.get_result_async()
        self.assertTrue(self._until(result_future.done, timeout=30.0))
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.error_code.val, MoveItErrorCodes.SUCCESS)

        target_degrees = math.degrees(0.12)
        self.assertTrue(self._until(
            lambda: any(
                action.get("arm_shoulder_pan.pos", 0.0) == pytest.approx(target_degrees, abs=1.0)
                for action in fake_host.actions
            ),
            timeout=10.0,
        ))
        self.assertTrue(self._until(
            lambda: any(
                "arm_shoulder_pan" in state.name
                and state.position[state.name.index("arm_shoulder_pan")] == pytest.approx(0.12, abs=0.03)
                for state in self.joint_states
            ),
            timeout=10.0,
        ))
        success.set()

    def test_canceling_a_trajectory_keeps_feedback_live(self, fake_host):
        self.assertTrue(self._until(
            lambda: bool(self.joint_states), timeout=15.0
        ))
        self.assertTrue(self._until(
            self.trajectory_client.server_is_ready, timeout=15.0
        ))
        self._arm()
        joint = "arm_shoulder_pan"
        latest = self.joint_states[-1]
        start = latest.position[latest.name.index(joint)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[start + 0.1],
            time_from_start=Duration(sec=3),
        )]

        before = len(self.joint_states)
        goal_future = self.trajectory_client.send_goal_async(goal)
        self.assertTrue(self._until(goal_future.done, timeout=5.0))
        handle = goal_future.result()
        self.assertTrue(handle.accepted)
        self._spin_for(0.25)
        self.assertGreater(len(self.joint_states), before + 2)

        cancel_future = handle.cancel_goal_async()
        self.assertTrue(self._until(cancel_future.done, timeout=5.0))
        self.assertTrue(cancel_future.result().goals_canceling)
        result_future = handle.get_result_async()
        self.assertTrue(self._until(result_future.done, timeout=5.0))
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self._until(
            lambda: len(self.joint_states) > before + 5, timeout=3.0
        ))


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
