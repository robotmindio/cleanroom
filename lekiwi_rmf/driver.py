#!/usr/bin/env python3
import dataclasses
import math
import os
import signal
import threading
import time
import uuid

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from lekiwi_rmf.arm_trajectory import (
    ARM_JOINTS, action_positions, joint_positions,
    raw_joint_positions, load_calibration,
    duration_seconds, position_tolerances, prepare_trajectory, sample_trajectory,
    stamp_nanoseconds, trajectory_rows,
)
from lekiwi_rmf.host_protocol import STATE_KEYS, TorqueCommand
from lekiwi_rmf.motion_guards import (
    Lease, bounded_test_speed_limits, inside_base_test_boundary, load_base_speed_limits, twist_is_finite,
)
from lekiwi_rmf.odometry import BASE_XY_SCALE, BASE_YAW_SCALE, HostPoseTracker
from lekiwi_rmf.torque_control import TorqueControlClient
from lekiwi_rmf.zmq_client import LeKiwiZmqClient
from lekiwi_rmf.zmq_security import CurveClientCredentials

# MoveIt stamps zero (start now). Anything scheduled further ahead than this is
# a clock mismatch or a mistake, not a plan to wait.
MAX_TRAJECTORY_START_DELAY_NS = 2_000_000_000


@dataclasses.dataclass(frozen=True)
class DriverSettings:
    """Validated driver parameters; each default is the ROS parameter default."""

    # Base speed caps from the tracked Nav2 controller limits.
    max_linear: float
    max_angular: float
    # Bool permissions have no source timestamp.  The receive-time lease
    # makes a transient-local sample a restart convenience, not an
    # unbounded authorization.
    permission_timeout: float
    xy_scale: float = BASE_XY_SCALE
    yaw_scale: float = BASE_YAW_SCALE
    bounded_base_test: bool = False
    command_timeout: float = 0.4
    link_timeout: float = 1.0
    trajectory_path_tolerance: float = 0.20
    # Match the production travel-stow gate: a successful MoveIt action
    # must not leave the arm outside its base-motion stow tolerance.
    trajectory_tolerance: float = 0.02
    gripper_trajectory_tolerance: float = 0.005
    trajectory_timeout: float = 5.0
    # This is velocity-integrated odometry, not an encoder/SLAM pose
    # measurement. Never publish the ROS all-zero covariance (perfect
    # certainty); these conservative values let consumers fuse it honestly.
    odom_xy_stddev: float = 0.05
    odom_yaw_stddev: float = 0.10
    twist_xy_stddev: float = 0.10
    twist_yaw_stddev: float = 0.20
    cmd_vel_topic: str = "/cmd_vel_safe"
    publish_odom_tf: bool = False
    auto_arm_on_startup: bool = True
    # By default the robot stays armed: a failure (link loss, stale telemetry, host
    # restart, withdrawn permission) stops the base and freezes the arm with torque on,
    # and the driver re-arms itself once telemetry and permission are healthy again.
    # Only an operator's safety/disarm stays disarmed. Setting this restores the strict
    # behaviour for larger robots: every failure disarms, cuts torque, latches
    # TORQUE_FAULT if the cut is unconfirmed, and waits for an explicit safety/arm.
    disarm_on_failure: bool = False
    publish_motor_health_enabled: bool = True

    def __post_init__(self):
        """Reject values that would make a command unsafe or undefined."""
        positive = {
            "max_linear": self.max_linear,
            "max_angular": self.max_angular,
            "xy_velocity_scale": self.xy_scale,
            "yaw_velocity_scale": self.yaw_scale,
            "command_timeout": self.command_timeout,
            "link_timeout": self.link_timeout,
            "permission_timeout": self.permission_timeout,
            "trajectory_path_tolerance": self.trajectory_path_tolerance,
            "trajectory_tolerance": self.trajectory_tolerance,
            "gripper_trajectory_tolerance": self.gripper_trajectory_tolerance,
            "trajectory_timeout": self.trajectory_timeout,
            "odom_xy_stddev": self.odom_xy_stddev,
            "odom_yaw_stddev": self.odom_yaw_stddev,
            "twist_xy_stddev": self.twist_xy_stddev,
            "twist_yaw_stddev": self.twist_yaw_stddev,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if not isinstance(self.cmd_vel_topic, str) or not self.cmd_vel_topic.strip():
            raise ValueError("cmd_vel_topic must be a non-empty topic name")


# ROS parameter name -> DriverSettings field.
SETTING_PARAMETERS = {
    "xy_velocity_scale": "xy_scale",
    "yaw_velocity_scale": "yaw_scale",
    "bounded_base_test": "bounded_base_test",
    "command_timeout": "command_timeout",
    "link_timeout": "link_timeout",
    "trajectory_path_tolerance": "trajectory_path_tolerance",
    "trajectory_tolerance": "trajectory_tolerance",
    "gripper_trajectory_tolerance": "gripper_trajectory_tolerance",
    "trajectory_timeout": "trajectory_timeout",
    "odom_xy_stddev": "odom_xy_stddev",
    "odom_yaw_stddev": "odom_yaw_stddev",
    "twist_xy_stddev": "twist_xy_stddev",
    "twist_yaw_stddev": "twist_yaw_stddev",
    "cmd_vel_topic": "cmd_vel_topic",
    "publish_odom_tf": "publish_odom_tf",
    "auto_arm_on_startup": "auto_arm_on_startup",
    "disarm_on_failure": "disarm_on_failure",
    "publish_motor_health": "publish_motor_health_enabled",
}


def odometry_message(stamp, pose, velocity, pose_stddev, twist_stddev):
    """Planar odom->base_footprint odometry; each stddev pair is (xy, yaw)."""
    x, y, yaw = pose
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
    # Indices follow ROS's row-major [x, y, z, roll, pitch, yaw] convention.
    # z/roll/pitch are unobserved by this planar driver, so their deliberately
    # large variance prevents a 3D estimator mistaking them for measurements.
    for covariance, (xy_stddev, yaw_stddev) in (
        ("pose", pose_stddev), ("twist", twist_stddev),
    ):
        values = [0.0] * 36
        values[0] = values[7] = xy_stddev ** 2
        values[14] = values[21] = values[28] = 1e6
        values[35] = yaw_stddev ** 2
        getattr(odom, covariance).covariance = values
    return odom


def odometry_transform(odom):
    """The odom->base_footprint transform matching an odometry message."""
    transform = TransformStamped()
    transform.header = odom.header
    transform.child_frame_id = odom.child_frame_id
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.rotation.z = odom.pose.pose.orientation.z
    transform.transform.rotation.w = odom.pose.pose.orientation.w
    return transform


def arm_joint_state(stamp, positions):
    joints = JointState()
    joints.header.stamp = stamp
    joints.name = list(ARM_JOINTS)
    joints.position = [positions[name] for name in ARM_JOINTS]
    return joints


def diagnostics_message(stamp, statuses):
    """Republish validated motor-health statuses as ROS diagnostics."""
    message = DiagnosticArray()
    message.header.stamp = stamp
    message.status = []
    for source in statuses:
        status = DiagnosticStatus()
        status.name = source.name
        status.level = bytes((source.level,))
        status.message = source.message
        status.hardware_id = "lekiwi_servo_bus"
        status.values = []
        for key, value in source.values:
            item = KeyValue()
            item.key, item.value = key, value
            status.values.append(item)
        message.status.append(status)
    return message


@dataclasses.dataclass
class ArmGoal:
    """One FollowJointTrajectory goal, executed by the motor host's local executor."""

    host_id: int
    host_session: str
    # Monotonic time at which the host starts the first segment.
    start: float
    names: tuple = ()
    start_positions: dict = dataclasses.field(default_factory=dict)
    points: list = dataclasses.field(default_factory=list)
    host_elapsed: float = 0.0
    outcome: str | None = None
    result_code: int | None = None
    done: threading.Event = dataclasses.field(default_factory=threading.Event)

    def finish(self, outcome, result_code=None):
        self.outcome = outcome
        self.result_code = result_code
        self.done.set()


class ArmTrajectoryBridge:
    """The arm_controller action: validate goals, upload them to the motor host and
    report its progress. Arming and permission state stay with the driver, which
    holds the current goal in ``driver.trajectory`` under ``driver.trajectory_lock``.
    """

    def __init__(self, driver):
        self.driver = driver

    def accept(self, goal):
        node = self.driver
        if not node.armed or (
            not node._arm_permission_is_current() and not node._hold_feedback_gap()
        ):
            # MoveIt surfaces this only as an unexplained "goal was rejected"; say why here.
            node.get_logger().warn(
                "Rejecting arm trajectory: driver is disarmed or arm safety permission is absent"
            )
            return GoalResponse.REJECT
        if not node.arm_calibrated:
            node.get_logger().warn(
                "Rejecting arm trajectory: no valid arm calibration is installed"
            )
            return GoalResponse.REJECT
        try:
            points = trajectory_rows(goal.trajectory)
            with node.trajectory_lock:
                start_positions = node.arm_positions.copy()
            prepare_trajectory(
                goal.trajectory.joint_names, points, start_positions,
                node.arm_zero_positions, node.arm_directions,
            )
            self.requested_tolerances(goal, goal.trajectory.joint_names)
            stamp_nanoseconds(goal.trajectory.header.stamp)
            if goal.multi_dof_trajectory.joint_names or goal.multi_dof_trajectory.points:
                raise ValueError("multi-DOF trajectories are unsupported")
        except (IndexError, ValueError) as error:
            node.get_logger().warn(f"Rejecting arm trajectory: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute(self, goal_handle):
        node = self.driver
        names = goal_handle.request.trajectory.joint_names
        requested_points = trajectory_rows(goal_handle.request.trajectory)
        path_tolerances, goal_tolerances, goal_time_tolerance = self.requested_tolerances(
            goal_handle.request, names
        )
        scheduled_ns = stamp_nanoseconds(goal_handle.request.trajectory.header.stamp)
        # The header is ROS time, but execution is timed on the monotonic clock so a
        # wall-clock step cannot stretch or skip a trajectory. Read both together and
        # convert the offset once.
        now_ns = node.get_clock().now().nanoseconds
        now_monotonic = time.monotonic()
        start_delay_ns = scheduled_ns - now_ns if scheduled_ns else 0
        if start_delay_ns < -100_000_000:
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP,
                error_string="trajectory header timestamp is in the past",
            )
        if start_delay_ns > MAX_TRAJECTORY_START_DELAY_NS:
            # A far-future stamp would hold the arm and block this goal for as long.
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string=(
                    "trajectory header timestamp is more than "
                    f"{MAX_TRAJECTORY_START_DELAY_NS / 1e9:.0f}s in the future"
                ),
            )
        goal = ArmGoal(
            host_id=(uuid.uuid4().int % (2**48 - 1)) + 1,
            host_session=node.last_observation_token[1],
            start=now_monotonic + max(0, start_delay_ns) / 1e9,
        )
        waiting_since = time.monotonic()
        while node.armed and not node._arm_permission_is_current() and node._hold_feedback_gap():
            # The plan has not started. Wait for measured joints and the full
            # supervisor permission instead of failing a goal queued in the gap.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.SUCCESSFUL
                )
            await self._yield_for_control(0.05)
        goal.start += time.monotonic() - waiting_since
        with node.state_lock:
            if not node.armed or not node._arm_permission_is_current():
                goal_handle.abort()
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                    error_string="driver was disarmed before trajectory execution",
                )
            with node.trajectory_lock:
                start_positions = {name: node.arm_positions[name] for name in names}
                try:
                    # Feedback can change between goal acceptance and this callback.
                    # Recheck the first segment against its actual execution start.
                    points = prepare_trajectory(
                        names, requested_points, start_positions,
                        node.arm_zero_positions, node.arm_directions,
                    )
                except ValueError as error:
                    goal_handle.abort()
                    return FollowJointTrajectory.Result(
                        error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                        error_string=str(error),
                    )
                if node.trajectory:
                    node.trajectory.finish("preempted", FollowJointTrajectory.Result.INVALID_GOAL)
                goal.start_positions = start_positions
                goal.names = tuple(names)
                goal.points = points
                node.trajectory = goal
            node._zero_command()

        try:
            # Serialize uploads with torque transitions. A preempted callback
            # must never upload its older goal after the replacement's upload.
            with node.torque_lock:
                with node.trajectory_lock:
                    current = node.trajectory is goal
                if current:
                    response = node.torque.trajectory_request(
                        TorqueCommand.TRAJECTORY_START, session=goal.host_session, trajectory={
                            "id": goal.host_id, "names": list(names), "points": requested_points,
                            "zeros": node.arm_zero_positions, "directions": node.arm_directions,
                            "path": path_tolerances, "goal": goal_tolerances,
                            "settling": goal_time_tolerance, "delay": max(0.0, goal.start - time.monotonic()),
                        },
                    )
                    if response.get("trajectory", {}).get("id") != goal.host_id:
                        raise RuntimeError("motor host did not acknowledge this arm goal")
        except Exception as error:
            with node.trajectory_lock:
                if node.trajectory is goal:
                    goal.finish(f"local arm upload failed: {error}", FollowJointTrajectory.Result.INVALID_GOAL)
                    node.trajectory = None

        while not goal.done.is_set():
            if goal_handle.is_cancel_requested:
                with node.trajectory_lock:
                    if node.trajectory is goal:
                        node.trajectory = None
                    goal.finish("canceled")
                goal_handle.canceled()
                self._cancel_on_host(goal)
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
            status = node.robot.arm_trajectory_status
            if status is not None and status["id"] == goal.host_id:
                goal.host_elapsed = status["elapsed"]
                if status["state"] in {"succeeded", "aborted", "canceled"}:
                    with node.trajectory_lock:
                        if node.trajectory is goal and not goal.done.is_set():
                            goal.finish(
                                "succeeded" if status["state"] == "succeeded" else status["detail"],
                                status["code"],
                            )
                            node.trajectory = None
            self.publish_feedback(goal_handle, goal)
            # Keep control-loop timers, safety updates, and action cancellation
            # serviceable while this goal waits for measured servo feedback.
            await self._yield_for_control(0.05)

        if goal.outcome == "succeeded":
            final = goal.points[-1].positions
            with node.state_lock:
                if node.arm_hold_action is not None:
                    node.arm_hold_action.update({f"{name}.pos": value for name, value in action_positions(
                        final.keys(), final.values(), node.arm_zero_positions, node.arm_directions,
                    ).items()})
            goal_handle.succeed()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
        self._cancel_on_host(goal)
        goal_handle.abort()
        return FollowJointTrajectory.Result(
            error_code=(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                if goal.result_code is None else goal.result_code
            ),
            error_string="trajectory preempted" if goal.outcome is None else goal.outcome,
        )

    def _cancel_on_host(self, goal):
        try:
            self.driver.torque.trajectory_request(
                TorqueCommand.TRAJECTORY_CANCEL, id=goal.host_id, session=goal.host_session,
            )
        except Exception as error:
            # The bounded Pi lease has already stopped advancement; report the
            # failed cancellation rather than assuming its retained goal is gone.
            self.driver.get_logger().error(f"Local arm cancellation was not confirmed: {error}")

    def requested_tolerances(self, goal, names):
        node = self.driver
        if goal.component_path_tolerance or goal.component_goal_tolerance:
            raise ValueError("component tolerances are unsupported for revolute arm joints")
        default_path = dict.fromkeys(names, node.trajectory_path_tolerance)
        path = position_tolerances(names, goal.path_tolerance, default_path)
        default_goal = {
            name: node.gripper_trajectory_tolerance if name == "arm_gripper"
            else node.trajectory_tolerance
            for name in names
        }
        goal_tolerances = position_tolerances(names, goal.goal_tolerance, default_goal)
        goal_time = duration_seconds(goal.goal_time_tolerance)
        return path, goal_tolerances, goal_time or node.trajectory_timeout

    def publish_feedback(self, goal_handle, goal):
        with self.driver.trajectory_lock:
            actual = {name: self.driver.arm_positions[name] for name in goal.names}
        goal_handle.publish_feedback(trajectory_feedback(goal, actual))

    async def _yield_for_control(self, delay):
        node = self.driver
        future = Future(executor=node.executor)
        timer = None

        def resume():
            timer.cancel()
            future.set_result(None)

        timer = node.create_timer(delay, resume)
        try:
            await future
        finally:
            timer.cancel()
            node.destroy_timer(timer)


def trajectory_feedback(goal, actual):
    """FollowJointTrajectory feedback at the host's reported elapsed time."""
    elapsed = goal.host_elapsed
    desired, velocities, accelerations = sample_trajectory(
        goal.names, goal.start_positions, goal.points, elapsed,
    )
    feedback = FollowJointTrajectory.Feedback()
    feedback.joint_names = list(goal.names)
    feedback.desired.positions = [desired[name] for name in goal.names]
    feedback.desired.velocities = [velocities[name] for name in goal.names]
    feedback.desired.accelerations = [accelerations[name] for name in goal.names]
    feedback.actual.positions = [actual[name] for name in goal.names]
    feedback.error.positions = [desired[name] - actual[name] for name in goal.names]
    seconds = int(elapsed)
    nanoseconds = int((elapsed - seconds) * 1e9)
    for point in (feedback.desired, feedback.actual, feedback.error):
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
    return feedback


class LeKiwiDriver(Node):
    def __init__(self):
        super().__init__("lekiwi_driver")
        remote_ip = self.declare_parameter("remote_ip", "127.0.0.1").value
        command_port = self.declare_parameter("remote_command_port", 5555).value
        observation_port = self.declare_parameter("remote_observation_port", 5556).value
        torque_control_port = self.declare_parameter("torque_control_port", 5557).value
        # A disable writes all nine servos and reads every one back: measured at 0.6-1.3 s
        # on the Pi, so a one-second timeout failed about a third of explicit disarms.
        torque_control_timeout_ms = self.declare_parameter("torque_control_timeout_ms", 4000).value
        curve_client_secret = self.declare_parameter("curve_client_secret_key_file", "").value
        curve_server_public = self.declare_parameter("curve_server_public_key_file", "").value
        defaults = {field.name: field.default for field in dataclasses.fields(DriverSettings)}
        values = {
            field: self.declare_parameter(name, defaults[field]).value
            for name, field in SETTING_PARAMETERS.items()
        }
        # Both have no code default: every launcher passes the tracked value.
        nav2_file = self.declare_parameter("nav2_params_file", Parameter.Type.STRING).value
        permission_timeout = self.declare_parameter("permission_timeout", Parameter.Type.DOUBLE).value
        if not nav2_file or permission_timeout is None:
            raise ValueError("nav2_params_file and permission_timeout parameters are required")
        speed_limits = load_base_speed_limits(nav2_file)
        max_linear, max_angular = speed_limits
        if values["bounded_base_test"]:
            linear, angular = bounded_test_speed_limits(
                self.declare_parameter("base_test_linear_limit", 0.03).value,
                self.declare_parameter("base_test_angular_limit", 0.20).value,
                speed_limits,
            )
            max_linear = min(max_linear, linear)
            max_angular = min(max_angular, angular)
        settings = DriverSettings(
            max_linear=max_linear, max_angular=max_angular,
            permission_timeout=permission_timeout, **values,
        )
        odom_topic = self.declare_parameter("odom_topic", "/wheel/odometry").value
        base_permission_topic = self.declare_parameter(
            "base_motion_permission_topic", "/safety/base_motion_permitted"
        ).value
        arm_permission_topic = self.declare_parameter(
            "arm_motion_permission_topic", "/safety/arm_motion_permitted"
        ).value
        calibration_file = self.declare_parameter(
            "arm_calibration_file", os.path.expanduser("~/.ros/lekiwi_arm_calibration.json")
        ).value
        # Raw odometry is a continuous local frame. Localization owns the
        # global map->odom placement and must never be baked into wheel odom.
        initial_pose = (
            self.declare_parameter("initial_x", 0.0).value,
            self.declare_parameter("initial_y", 0.0).value,
            self.declare_parameter("initial_yaw", 0.0).value,
        )

        for name, port in (
            ("remote_command_port", command_port),
            ("remote_observation_port", observation_port),
            ("torque_control_port", torque_control_port),
        ):
            if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
                raise ValueError(f"{name} must be an integer from 1 through 65535")

        curve_credentials = CurveClientCredentials(
            curve_client_secret, curve_server_public
        ).validate()
        robot = LeKiwiZmqClient(
            remote_ip, command_port, observation_port, STATE_KEYS,
            curve_credentials=curve_credentials,
        )
        robot.connect()
        torque = TorqueControlClient(
            remote_ip, torque_control_port, torque_control_timeout_ms,
            client_secret_key_file=curve_client_secret,
            server_public_key_file=curve_server_public,
        )
        self._init_state(
            settings, robot=robot, torque=torque, arm_calibration_file=calibration_file,
            initial_pose=initial_pose, now=self.get_clock().now(),
        )
        if not self.arm_calibrated:
            self.get_logger().warn(
                f"No URDF arm calibration at {calibration_file}; arm trajectories are disabled"
            )

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.raw_joint_pub = self.create_publisher(JointState, "arm/raw_joint_states", 10)
        self.motor_health_pub = self.create_publisher(DiagnosticArray, "/hardware/diagnostics", 10)
        safety_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.create_publisher(String, "safety/state", safety_qos)
        self.safety_marker_pub = self.create_publisher(Marker, "safety/marker", safety_qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_command, 10)
        # Permission callbacks must be able to revoke state while an arm RPC is
        # waiting on the motor host. Their callbacks update the guarded state
        # before waiting for action_lock, so an in-flight enable rolls back.
        self.safety_callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            Bool, base_permission_topic, self.on_base_permission, safety_qos,
            callback_group=self.safety_callback_group,
        )
        self.create_subscription(
            Bool, arm_permission_topic, self.on_arm_permission, safety_qos,
            callback_group=self.safety_callback_group,
        )
        self.create_subscription(
            Bool, "/safety/arm_workspace_collision", self.on_arm_collision, safety_qos,
            callback_group=self.safety_callback_group,
        )
        # A torque RPC blocks for up to torque_control_timeout_ms per attempt, twice.
        # Arm, disarm and the automatic arm run in their own group so they serialize
        # with each other but never stall update(), its safety/state heartbeat or
        # permission-lease enforcement; update() skips a send while one is in flight.
        self.transition_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "safety/arm", self.arm,
            callback_group=self.transition_callback_group,
        )
        self.create_service(
            Trigger, "safety/disarm", self.disarm,
            callback_group=self.transition_callback_group,
        )
        self.create_timer(
            0.1, self.auto_arm_tick, callback_group=self.transition_callback_group
        )
        self.trajectory_server = ActionServer(
            self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory",
            execute_callback=self.arm_trajectories.execute, goal_callback=self.arm_trajectories.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(0.05, self.update)
        self.get_logger().info(f"Connected to LeKiwi host at {remote_ip}")
        self.get_logger().info(f"Accepting guarded base commands from {self.cmd_vel_topic}")
        self.publish_safety("DISARMED")

    def _init_state(self, settings, *, robot, torque, arm_calibration_file, initial_pose, now):
        """Set the plain runtime state: no ROS entities, so tests reuse it unchanged."""
        for field in dataclasses.fields(settings):
            setattr(self, field.name, getattr(settings, field.name))
        self.robot = robot
        self.torque = torque
        self.torque_lock = threading.Lock()
        # Serializes physical actuator transitions and command submission. Local
        # state uses a separate lock so callbacks remain responsive while ZeroMQ
        # or the torque service is delayed.
        self.action_lock = threading.Lock()
        self.command = Twist()
        self.command_stamp = now
        self.pose = initial_pose
        self._base_test_center = self.pose[:2]
        self.host_pose = HostPoseTracker()
        self.observation_stamp = None
        self.last_observation = None
        self.last_observation_token = None
        self.last_fresh = now
        self.arm_workspace_collision = False
        self.link_lost = False
        # When update() last accepted telemetry that passed the host-session and
        # torque-readback checks; the automatic arm acts only on such telemetry.
        self._healthy_telemetry_at = None
        self.armed = False
        self.torque_fault = False
        permission_timeout_ns = int(settings.permission_timeout * 1_000_000_000)
        self.base_permission = Lease(permission_timeout_ns)
        self.arm_permission = Lease(permission_timeout_ns)
        self._arm_permission_expired = False
        # Arming waits for a complete, fresh observation and current supervisor
        # permission. By default the robot arms at startup whatever auto_arm_on_startup
        # says, and the flag is set again after every failure so it re-arms itself.
        # Only with disarm_on_failure does auto_arm_on_startup decide the startup arm,
        # and then a link loss or an explicit disarm clears this one-shot flag, so
        # recovery never resumes movement without an operator arming again.
        self.auto_arm_pending = bool(self.auto_arm_on_startup) or not self.disarm_on_failure
        # Set by an operator's safety/disarm and cleared only by a successful safety/arm,
        # so no later failure or telemetry recovery can re-arm a robot the operator disarmed.
        self.operator_disarmed = False
        # A failed automatic arm is retried no earlier than this monotonic time.
        self._next_rearm_at = 0.0
        self.arm_calibrated = os.path.isfile(os.path.expanduser(arm_calibration_file))
        self.stop_pending = True
        # Bumped by every disarm. An arm commits only if no disarm happened since
        # it checked its preconditions, so a disarm never waits behind its RPC.
        self._disarm_epoch = 0
        # Strict mode: a torque cut the control loop handed to the transition group.
        self._deferred_cut = False
        self.state_lock = threading.Lock()
        self.arm_zero_positions, self.arm_directions = load_calibration(arm_calibration_file)
        self.arm_positions = {name: 0.0 for name in ARM_JOINTS}
        self.arm_hold_action = None
        # The current arm goal (ArmGoal or None), guarded by trajectory_lock.
        self.trajectory = None
        self.trajectory_lock = threading.Lock()
        self.arm_trajectories = ArmTrajectoryBridge(self)
        self.safety_publish_lock = threading.Lock()
        self.safety_state = "DISARMED"

    def on_base_permission(self, message):
        permitted = bool(message.data)
        received_at_ns = time.monotonic_ns()
        with self.state_lock:
            self.base_permission.grant(permitted, received_at_ns)
            if not permitted:
                self._zero_command()
            armed = self.armed
            arm_current = self.arm_permission.current(received_at_ns)
        if not permitted and armed and not arm_current and not self._hold_feedback_gap():
            self.get_logger().error(
                "All motion capability permissions withdrawn; disarming"
            )
            self.set_disarmed("DISARMED")

    def on_arm_permission(self, message):
        received_at_ns = time.monotonic_ns()
        with self.state_lock:
            permitted = bool(message.data) and not self.arm_workspace_collision
            was_permitted = self.arm_permission.value
            was_expired = bool(self._arm_permission_expired)
            self.arm_permission.grant(permitted, received_at_ns)
            self._arm_permission_expired = not permitted
            armed = self.armed
            base_current = self.base_permission.current(received_at_ns)
        newly_withdrawn = not permitted and (was_permitted or not was_expired)
        if newly_withdrawn and armed:
            if self._hold_feedback_gap():
                return
            self.cancel_trajectory("arm safety permission withdrawn")
            if not base_current:
                self.get_logger().error(
                    "All motion capability permissions withdrawn; disarming"
                )
                self.set_disarmed("DISARMED")
            else:
                self.get_logger().error(
                    "Arm safety permission withdrawn; canceling arm motion while base remains enabled"
                )

    def on_arm_collision(self, message):
        with self.state_lock:
            self.arm_workspace_collision = bool(message.data)
        if message.data:
            self.on_arm_permission(Bool(data=False))

    def _zero_command(self):
        """Drop the pending base command; the caller holds state_lock."""
        self.command = Twist()
        self.command_stamp = self.get_clock().now()

    def _hold_feedback_gap(self):
        """Retain an arm goal while feedback or collision checking is unavailable.

        No trajectory setpoint is sent without fresh feedback and permission.
        The Pi freezes the arm if commands stop. A confirmed MoveIt collision
        retains the ordinary immediate cancel path, even during a feedback gap.
        """
        with self.state_lock:
            return not self.disarm_on_failure and self.armed and not self.arm_workspace_collision

    def enforce_permission_leases(self, now_monotonic_ns=None):
        """Expire stale supervisor decisions independently of DDS delivery."""
        current = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        arm_expired = False
        with self.state_lock:
            base_current = self.base_permission.current(current)
            if not base_current and self.base_permission.value:
                self.base_permission.value = False
                self._zero_command()

            arm_current = self.arm_permission.current(current)
            if not arm_current:
                if not self._arm_permission_expired:
                    arm_expired = True
                self.arm_permission.value = False
                self._arm_permission_expired = True
            else:
                self._arm_permission_expired = False
            was_armed = self.armed

        holding_gap = was_armed and not arm_current and self._hold_feedback_gap()
        if arm_expired and was_armed and not holding_gap:
            self.cancel_trajectory("arm safety permission lease expired")
        if was_armed and not base_current and not arm_current and not holding_gap:
            self.get_logger().error(
                "All motion capability permission leases expired; disarming"
            )
            self.set_disarmed("DISARMED", defer_cut=True)
        return arm_expired

    def on_command(self, message):
        if not twist_is_finite(message):
            self.get_logger().error("Rejecting non-finite base command and disarming")
            # Shares update()'s callback group, so it must not wait on a torque RPC.
            self.set_disarmed("DISARMED", defer_cut=True)
            return
        with self.state_lock:
            if not self.armed or not self.base_permission.current():
                return
            self.command = message
            self.command_stamp = self.get_clock().now()

    def publish_safety(self, state=None, disarm_epoch=None):
        # The supervisor treats this as a live driver heartbeat. Serialize
        # state transitions with refreshes so an older ARMED refresh can never
        # overtake a concurrent DISARMED/LINK_LOST transition.
        with self.safety_publish_lock:
            if disarm_epoch is not None:
                # An arm publishes ARMED only if no disarm has happened since it
                # committed; that disarm publishes after us or already did.
                with self.state_lock:
                    if self._disarm_epoch != disarm_epoch:
                        return
            if state is not None:
                self.safety_state = state
            state = self.safety_state
            message = String()
            message.data = state
            self.safety_pub.publish(message)

            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "safety"
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = {
                "ARMED": (0.0, 1.0, 0.0),
                "DISARMED": (1.0, 0.75, 0.0),
                "LINK_LOST": (1.0, 0.0, 0.0),
                "TORQUE_FAULT": (1.0, 0.0, 1.0),
            }[state]
            marker.text = state
            self.safety_marker_pub.publish(marker)

    def cancel_trajectory(self, outcome, result_code=None):
        with self.trajectory_lock:
            if self.trajectory:
                self.trajectory.finish(
                    outcome,
                    FollowJointTrajectory.Result.INVALID_GOAL if result_code is None else result_code,
                )
                self.trajectory = None

    def set_disarmed(
        self, state, publish=True, clear_torque_fault=False, deliberate=False, defer_cut=False
    ):
        """Disarm now; cut torque here, or with ``defer_cut`` in the transition group.

        The logical disarm never waits for an in-flight arm/disarm RPC: bumping
        ``_disarm_epoch`` makes that arm roll back instead of committing. The
        control loop passes ``defer_cut`` so its heartbeat never waits on a torque
        RPC; in strict mode ``_run_deferred_cut`` then performs the cut. Returns
        None when the cut was deferred.
        """
        with self.state_lock:
            was_armed = self.armed
            self.armed = False
            self._disarm_epoch += 1
            # An operator's disarm must hold, and in the strict mode so must a failure.
            # Otherwise the driver re-arms itself once telemetry and permission allow.
            if deliberate:
                self.operator_disarmed = True
            self.auto_arm_pending = not self.operator_disarmed and not self.disarm_on_failure
            self._zero_command()
            self.stop_pending = True
            self.arm_hold_action = None
            if defer_cut:
                # Only strict mode cuts torque after a failure.
                self._deferred_cut = self._deferred_cut or bool(self.disarm_on_failure)
                published_state = "TORQUE_FAULT" if self.torque_fault else state
        if defer_cut:
            self.cancel_trajectory("safety disarmed")
            if was_armed:
                self.get_logger().warn(f"Robot disarmed: {published_state}")
            if publish:
                self.publish_safety(published_state)
            return None

        with self.action_lock:
            with self.state_lock:
                # This cut supersedes one the control loop deferred.
                self._deferred_cut = False
            # Never hold state_lock across a service/network operation.
            torque_cut = self.cut_torque_after_failure() if not deliberate else self.set_servo_torque(False)
            with self.state_lock:
                if not torque_cut:
                    # An operator's failed cut must be visible and block re-arming
                    # even when automatic failure handling normally holds torque.
                    self.torque_fault = self.disarm_on_failure or deliberate
                elif clear_torque_fault:
                    # Only a deliberate disarm request may acknowledge recovery;
                    # incidental watchdog cuts cannot silently clear this latch.
                    self.torque_fault = False
                published_state = "TORQUE_FAULT" if self.torque_fault else state

            self.cancel_trajectory("safety disarmed")
            if was_armed:
                self.get_logger().warn(f"Robot disarmed: {published_state}")
            if publish:
                self.publish_safety(published_state)
            return torque_cut

    def cut_torque_after_failure(self):
        """Cut servo torque after a failure, unless the operator chose to hold it.

        Returns True when torque is confirmed off or deliberately left alone, so no
        fault latches and arming is never blocked by a failed cut.
        """
        if not self.disarm_on_failure:
            return True
        return self.set_servo_torque(False)

    def _run_deferred_cut(self):
        """Perform a strict-mode cut the control loop deferred; caller holds action_lock."""
        with self.state_lock:
            pending = self._deferred_cut
            self._deferred_cut = False
        if not pending:
            return
        if self.set_servo_torque(False):
            return
        with self.state_lock:
            self.torque_fault = True
        self.publish_safety("TORQUE_FAULT")

    def set_servo_torque(self, enabled):
        """Synchronously require the serial-bus owner to change physical torque."""
        can_log = rclpy.ok(context=self.context)
        try:
            with self.torque_lock:
                self.torque.set_enabled(enabled)
        except Exception as error:
            if can_log:
                self.get_logger().error(
                    f"Could not {'enable' if enabled else 'cut'} servo torque: {error}"
                )
            return False
        if can_log:
            self.get_logger().info(
                f"Servo torque {'enabled' if enabled else 'cut'} by motor host"
            )
        return True

    def _arm_permission_is_current(self):
        return self.arm_permission.current()

    def _enable_torque_and_arm(self, permission_is_current, operator, disarm_epoch):
        """Enable servo torque, then commit ARMED only if it is still safe.

        The caller holds action_lock and has checked its own preconditions, reading
        ``disarm_epoch`` with them; any disarm since then makes the arm unsafe. Returns
        ``(outcome, torque_cut)`` with outcome ``"armed"``, ``"enable_failed"`` or
        ``"unsafe"``; ``torque_cut`` reports the fail-safe disable after a failure.
        """
        outcome = "enable_failed"
        if self.set_servo_torque(True):
            with self.state_lock:
                still_safe = (
                    permission_is_current()
                    and not self.link_lost
                    and not self.torque_fault
                    and self._disarm_epoch == disarm_epoch
                )
                if still_safe:
                    self.armed = True
                    if operator:
                        self.operator_disarmed = False
                    self._zero_command()
            if still_safe:
                self.publish_safety("ARMED", disarm_epoch=disarm_epoch)
                return "armed", True
            outcome = "unsafe"
        # An enable reply can be lost after the host applied it. A separate
        # disable transaction resolves that ambiguous state toward torque-off
        # (strict mode only) before reporting the arm request failed.
        torque_cut = self.cut_torque_after_failure()
        with self.state_lock:
            if not torque_cut:
                self.torque_fault = True
            state = "TORQUE_FAULT" if self.torque_fault else "DISARMED"
        self.publish_safety(state)
        return outcome, torque_cut

    def auto_arm_tick(self):
        """Attempt the automatic arm off the control loop: its torque RPC can take seconds."""
        with self.state_lock:
            deferred_cut = self._deferred_cut
        if deferred_cut:
            with self.action_lock:
                self._run_deferred_cut()
        with self.state_lock:
            # Checked first so an idle tick never contends for action_lock.
            if not self.auto_arm_pending or self.armed or self._healthy_telemetry_at is None:
                return False
            healthy_at = self._healthy_telemetry_at
        age = (self.get_clock().now() - healthy_at).nanoseconds / 1e9
        if not 0.0 <= age <= self.link_timeout:
            return False
        return self.arm_after_startup_telemetry()

    def arm_after_startup_telemetry(self):
        """Arm after validated telemetry: at startup, and by default after every failure."""
        with self.action_lock:
            self._run_deferred_cut()
            with self.state_lock:
                if (
                    not self.auto_arm_pending
                    or time.monotonic() < self._next_rearm_at
                    or self.link_lost
                    or self.armed
                    or not self._arm_permission_is_current()
                    or self.torque_fault
                ):
                    return False
                self.auto_arm_pending = False
                disarm_epoch = self._disarm_epoch
            outcome, _torque_cut = self._enable_torque_and_arm(
                self._arm_permission_is_current, operator=False, disarm_epoch=disarm_epoch
            )
            if outcome == "armed":
                self.get_logger().info("Armed after initial healthy LeKiwi telemetry")
                return True
            if outcome == "enable_failed":
                self.get_logger().error(
                    "Initial telemetry is healthy, but the motor host would not enable torque"
                )
            self._retry_rearm_soon()
            return False

    def _retry_rearm_soon(self):
        """Unless strict, a failed automatic arm is retried, at a bounded rate."""
        if not self.disarm_on_failure:
            with self.state_lock:
                self.auto_arm_pending = not self.operator_disarmed
                self._next_rearm_at = time.monotonic() + 2.0

    def arm(self, request, response):
        del request
        with self.action_lock:
            # A deferred strict-mode cut runs first, so its fault latch is seen here.
            self._run_deferred_cut()
            # Measured after any transition ahead of this one has finished, so
            # a wait for action_lock cannot make stale telemetry look fresh.
            telemetry_age = (self.get_clock().now() - self.last_fresh).nanoseconds / 1e9
            with self.state_lock:
                if self.torque_fault:
                    response.success = False
                    response.message = (
                        "servo torque state is fault-latched; call safety/disarm "
                        "and confirm torque-off before rearming"
                    )
                    return response
                if not self._arm_permission_is_current():
                    response.success = False
                    response.message = "continuous safety supervisor has not granted arm permission"
                    return response
                if (
                    self.link_lost
                    or self.last_observation is None
                    or telemetry_age < 0.0
                    or telemetry_age > self.link_timeout
                ):
                    response.success = False
                    response.message = "no fresh LeKiwi telemetry"
                    return response
                disarm_epoch = self._disarm_epoch
            outcome, torque_cut = self._enable_torque_and_arm(
                self._arm_permission_is_current, operator=True, disarm_epoch=disarm_epoch
            )
            response.success = outcome == "armed"
            if outcome == "armed":
                response.message = "armed at current measured position; send a new command"
            elif outcome == "unsafe":
                response.message = "safety state changed while enabling torque"
            else:
                response.message = (
                    "motor host did not confirm servo torque enabled; "
                    + (
                        "torque was left unchanged"
                        if not self.disarm_on_failure
                        else "the fail-safe disable was not confirmed"
                        if not torque_cut
                        else "a fail-safe disable was confirmed"
                    )
                )
            return response

    def disarm(self, request, response):
        del request
        torque_cut = self.set_disarmed("DISARMED", clear_torque_fault=True, deliberate=True)
        response.success = torque_cut
        response.message = (
            "commands disabled and all servo torque cut"
            if torque_cut
            else "commands disabled, but the motor host did not confirm servo torque cut; use the physical emergency stop"
        )
        return response

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def clamp_planar(x, y, limit):
        magnitude = math.hypot(x, y)
        if magnitude <= limit or magnitude == 0.0:
            return x, y
        scale = limit / magnitude
        return x * scale, y * scale

    def observation_is_fresh(self, observation):
        # A stationary robot reports identical numeric values in every packet.
        # Comparing those values mistakes healthy, fresh telemetry for a dropout.
        # The client sequence advances only when its ZMQ socket consumed a new
        # observation; cached observations leave it unchanged, which is the
        # signal the driver actually needs for link safety.
        token = self.robot.observation_token
        if token is None:
            # The client has not accepted a packet yet; its initial zero state is not data.
            return False
        fresh = token != self.last_observation_token
        self.last_observation_token = token
        self.last_observation = observation
        return fresh

    def handle_host_session_change(self):
        if not self.robot.observation_session_changed:
            return False
        # A new host process always starts torque-off. Keep logical state aligned
        # even if downtime was shorter than the telemetry watchdog threshold; the
        # robot then re-arms itself, or waits for an operator in the strict mode.
        self.set_disarmed("DISARMED", defer_cut=True)
        self.get_logger().error(
            "LeKiwi host session changed; "
            + (
                "robot remains disarmed until an explicit safety/arm request"
                if self.disarm_on_failure
                else "re-arming automatically once telemetry and permission are healthy"
            )
        )
        return True

    def enforce_reported_torque_state(self):
        """Keep logical arming synchronized with authenticated host readback."""
        reported = self.robot.observation_torque_enabled
        if self.action_lock.locked():
            # An arm or disarm transaction is changing torque right now and
            # settles the logical state itself once the host confirms.
            return False
        with self.state_lock:
            logical = self.armed
        if reported == logical:
            return False
        if reported and not self.disarm_on_failure:
            # Torque was deliberately left on after a failure; the disarmed driver
            # keeps the arm frozen and the base stopped.
            return False
        self.get_logger().error(
            "Motor host torque state changed outside the driver's arm/disarm transaction; disarming"
        )
        self.set_disarmed("DISARMED", defer_cut=True)
        return True

    @staticmethod
    def observation_is_valid(observation, missing_state_keys=()):
        """Require complete, finite arm and base feedback before commanding motion."""
        if missing_state_keys or not isinstance(observation, dict):
            return False
        try:
            values = [observation[f"{joint}.pos"] for joint in ARM_JOINTS]
            values.extend(observation[name] for name in ("x.vel", "y.vel", "theta.vel"))
            values.extend(
                value for name, value in observation.items() if name.endswith((".pos", ".vel"))
            )
            return all(math.isfinite(float(value)) for value in values)
        except (KeyError, TypeError, ValueError):
            return False

    def record_link_loss(self, reason):
        if self.link_lost:
            return
        self.get_logger().error(reason)
        self.link_lost = True
        if (not self.disarm_on_failure
                and reason.startswith("No fresh LeKiwi telemetry")):
            # The Pi's lease expires and freezes motion independently. Stay armed
            # and keep its goal through transport silence; invalid telemetry, motor
            # faults, explicit disarm, and host-session changes still cancel normally.
            self.publish_safety("LINK_LOST")
            return
        self.set_disarmed("LINK_LOST", defer_cut=True)

    def update(self):
        # Run before telemetry polling so a silent supervisor still revokes
        # arm torque even when the motor host is returning cached data.
        self.enforce_permission_leases()
        now = self.get_clock().now()
        polled = self._poll_telemetry(now)
        if polled is None:
            return
        observation, velocity = polled
        with self.state_lock:
            armed = self.armed
        if armed:
            self._send_armed_command(now, observation, velocity)
        else:
            self._send_pending_stop(now, observation, velocity)

    def _poll_telemetry(self, now):
        """Accept one fresh, valid observation; return it with the base velocity."""
        try:
            observation = self.robot.get_observation()
        except Exception as error:
            self.record_link_loss(f"LeKiwi telemetry failed: {error}")
            return None

        missing_state_keys = self.robot.missing_state_keys
        if not self.observation_is_valid(observation, missing_state_keys):
            reason = "LeKiwi telemetry is incomplete or non-finite"
            if missing_state_keys:
                reason = f"LeKiwi telemetry is missing: {', '.join(missing_state_keys)}"
            self.record_link_loss(reason)
            return None

        if not self.observation_is_fresh(observation):
            quiet = (now - self.last_fresh).nanoseconds / 1e9
            if quiet > self.link_timeout:
                self.record_link_loss(
                    f"No fresh LeKiwi telemetry for {quiet:.1f}s; waiting for recovery"
                )
            return None

        odometry = self.robot.observation_odometry
        if any(not math.isclose(a, b, rel_tol=1e-6) for a, b in zip(
                odometry["scales"], (self.xy_scale, self.yaw_scale))):
            self.record_link_loss("Pi and compute wheel calibration differ; sync calibration before motion")
            return None
        stamp_ns = odometry["stamp_ns"]
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age > self.link_timeout or age < -0.1:
            self.record_link_loss(f"Host odometry capture timestamp is stale or unsynchronized ({age:.3f}s)")
            return None
        self.observation_stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
        self.last_fresh = now
        self.publish_motor_health(now.to_msg())
        arm_positions = joint_positions(
            observation, self.arm_zero_positions, self.arm_directions
        )
        with self.trajectory_lock:
            self.arm_positions = arm_positions
        self.handle_host_session_change()
        self.enforce_reported_torque_state()
        with self.state_lock:
            self._healthy_telemetry_at = now
        if self.link_lost:
            self.link_lost = False
            with self.state_lock:
                recovered_state = "TORQUE_FAULT" if self.torque_fault else "ARMED" if self.armed else "DISARMED"
            self.publish_safety(recovered_state)
            self.get_logger().warn(
                "LeKiwi telemetry recovered; inspect robot, then call safety/arm"
                if self.disarm_on_failure
                else "LeKiwi telemetry recovered; re-arming automatically"
            )

        velocity = (
            float(observation["x.vel"]) * self.xy_scale,
            float(observation["y.vel"]) * self.xy_scale,
            math.radians(float(observation["theta.vel"])) * self.yaw_scale,
        )
        # The host integrates its own measured velocity, so samples dropped in
        # transport lose no motion; only its pose is aligned to the local frame.
        self.pose = self.host_pose.update(self.last_observation_token[1], odometry["pose"], self.pose)
        return observation, velocity

    @staticmethod
    def _hold_action(observation):
        return {
            f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS
        }

    def _send_pending_stop(self, now, observation, velocity):
        """While disarmed, send one zero-velocity hold after each disarm."""
        send_error = None
        # A torque transition in flight owns the actuators: skip this cycle's
        # stop (it stays pending) rather than stall the loop behind its RPC.
        if self.action_lock.acquire(blocking=False):
            try:
                with self.state_lock:
                    # An arm request may have completed after the snapshot.
                    # In that case, skip this cycle rather than sending a
                    # stale zero action after torque was enabled.
                    if self.armed:
                        return
                    send_stop = self.stop_pending
                if send_stop:
                    self.robot.send_action({
                        **self._hold_action(observation),
                        "x.vel": 0.0,
                        "y.vel": 0.0,
                        "theta.vel": 0.0,
                    })
                    with self.state_lock:
                        if not self.armed:
                            self.stop_pending = False
            except Exception as error:
                send_error = error
            finally:
                self.action_lock.release()
        if send_error is not None:
            self.record_link_loss(f"LeKiwi stop command failed: {send_error}")
            return
        # Torque state is not motion state: retain measured odometry if the
        # robot is pushed or coasts while commands are inhibited.
        self.publish_state(self.observation_stamp, observation, velocity)
        self.publish_safety()

    def _send_armed_command(self, now, observation, velocity):
        with self.state_lock:
            if not self.armed:
                return
            stale = (now - self.command_stamp).nanoseconds / 1e9 > self.command_timeout
            cmd = Twist() if stale or not self.base_permission.current() else self.command
            # Latch the first measured pose after torque is enabled. Reusing each
            # new observation as the goal lets gravity walk an idle arm down.
            if self.arm_hold_action is None:
                self.arm_hold_action = self._hold_action(observation)
            hold_action = self.arm_hold_action.copy()
        measured_hold = self._hold_action(observation)
        action = dict(hold_action)
        trajectory_ran = bool(self.trajectory)
        if trajectory_ran:
            cmd = Twist()
        # The scales divide here and multiply below: LeRobot's kinematics use a nominal
        # base_radius of 0.125 m, so a robot whose wheels sit elsewhere both under-turns
        # what it is asked for and over-reports what it did, by the same factor. Fixing
        # only the odometry would leave Nav2 asking for rotations it never gets.
        linear_x, linear_y = self.clamp_planar(
            cmd.linear.x, cmd.linear.y, self.max_linear
        )
        action.update({
            "x.vel": linear_x / self.xy_scale,
            "y.vel": linear_y / self.xy_scale,
            "theta.vel": math.degrees(
                self.clamp(cmd.angular.z, self.max_angular) / self.yaw_scale
            ),
        })
        # Serialize the final armed check with disarm. Once disarm returns, no
        # in-flight update can submit a previously prepared non-zero action. Any
        # other holder is an arm or disarm transaction: skip this cycle's send
        # rather than stall the loop behind its torque RPC (a disarm queues a stop).
        if not self.action_lock.acquire(blocking=False):
            self.publish_state(self.observation_stamp, observation, velocity)
            self.publish_safety()
            return
        send_error = None
        try:
            with self.state_lock:
                if not self.armed:
                    return
                arm_permitted = self._arm_permission_is_current()
                base_permitted = self.base_permission.current()
            if not arm_permitted:
                if not self._hold_feedback_gap():
                    self.cancel_trajectory("arm safety permission withdrawn")
                # Withdrawn permission holds the measured pose at the host.
                action.update(measured_hold)
            if not base_permitted:
                action["x.vel"] = action["y.vel"] = action["theta.vel"] = 0.0
            if self.bounded_base_test and not inside_base_test_boundary(self.pose, self._base_test_center):
                action["x.vel"] = action["y.vel"] = action["theta.vel"] = 0.0
            with self.trajectory_lock:
                remote_goal = self.trajectory.host_id if self.trajectory else None
            if remote_goal is not None:
                self.robot.send_action(action, arm_goal_id=remote_goal, arm_permitted=arm_permitted)
            else:
                self.robot.send_action(action)
            if trajectory_ran or not arm_permitted:
                with self.state_lock:
                    if self.armed:
                        self.arm_hold_action = {
                            key: action[key] for key in measured_hold
                        }
        except Exception as error:
            send_error = error
        finally:
            self.action_lock.release()
        if send_error is not None:
            self.record_link_loss(f"LeKiwi command failed: {send_error}")
            return

        self.publish_state(self.observation_stamp, observation, velocity)
        self.publish_safety()

    def publish_state(self, stamp, observation, velocity):
        odom = odometry_message(
            stamp, self.pose, velocity,
            (self.odom_xy_stddev, self.odom_yaw_stddev),
            (self.twist_xy_stddev, self.twist_yaw_stddev),
        )
        self.odom_pub.publish(odom)
        if self.publish_odom_tf:
            self.tf.sendTransform(odometry_transform(odom))
        self.joint_pub.publish(arm_joint_state(stamp, self.arm_positions))
        self.raw_joint_pub.publish(arm_joint_state(stamp, raw_joint_positions(observation)))

    def publish_motor_health(self, stamp):
        """Publish only the snapshot validated with this fresh host observation."""
        if not self.publish_motor_health_enabled:
            return
        # The client rejects absent or malformed health telemetry before a
        # sample reaches update(), so a fresh sample always carries a snapshot.
        self.motor_health_pub.publish(diagnostics_message(stamp, self.robot.observation_motor_health))

    def destroy_node(self):
        try:
            # ROS-stack shutdown disarms logically: commands stop and any
            # trajectory is canceled. Like any other failure it cuts servo
            # torque only with disarm_on_failure; by default the separately
            # supervised host keeps the arm held with torque on and its command
            # watchdog stops the base. SIGINT may already have invalidated the
            # rcl context, so publish only while it is still valid; publishing
            # through a dead context would turn a clean shutdown into exit code 1.
            # The check races the signal handler, so a publish that fails because
            # the context died in between is part of a clean shutdown too.
            try:
                self.set_disarmed(
                    "DISARMED", publish=rclpy.ok(context=self.context)
                )
            except Exception:
                if rclpy.ok(context=self.context):
                    raise
        finally:
            self.trajectory_server.destroy()
            self.robot.disconnect()
        return super().destroy_node()


def main():
    rclpy.init()

    def on_sigterm(_signum, _frame):
        # launch escalates to SIGTERM when a node ignores SIGINT for 5s; the
        # default disposition dies uncleanly and skips robot.disconnect().
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
    node = LeKiwiDriver()
    # One thread each for update(), the arm/disarm transition group and a running
    # trajectory goal, plus two so permission callbacks can revoke while those block.
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
