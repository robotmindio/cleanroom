#!/usr/bin/env python3
import math
import os
import signal
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from lekiwi_rmf.arm_trajectory import (
    ARM_JOINTS, action_positions, joint_positions, load_calibration,
    position_tolerances, prepare_trajectory, sample_trajectory, validate_trajectory,
)
from lekiwi_rmf.odometry import (
    OdometrySampleClock, integrate_pose,
)
from lekiwi_rmf.torque_control import TorqueControlClient
from lekiwi_rmf.zmq_client import LeKiwiZmqClient
from lekiwi_rmf.zmq_security import CurveClientCredentials


class LeKiwiDriver(Node):
    def __init__(self):
        super().__init__("lekiwi_driver")
        remote_ip = self.declare_parameter("remote_ip", "127.0.0.1").value
        self.declare_parameter("robot_id", "lekiwi_1")  # Retained wire/config compatibility.
        command_port = self.declare_parameter("remote_command_port", 5555).value
        observation_port = self.declare_parameter("remote_observation_port", 5556).value
        torque_control_port = self.declare_parameter("torque_control_port", 5557).value
        torque_control_timeout_ms = self.declare_parameter("torque_control_timeout_ms", 1000).value
        curve_client_secret = self.declare_parameter("curve_client_secret_key_file", "").value
        curve_server_public = self.declare_parameter("curve_server_public_key_file", "").value
        allow_legacy_telemetry = self.declare_parameter("allow_legacy_telemetry", False).value
        self.xy_scale = self.declare_parameter("xy_velocity_scale", 1.0).value
        self.yaw_scale = self.declare_parameter("yaw_velocity_scale", 1.0).value
        self.max_linear = self.declare_parameter("max_linear_speed", 0.3).value
        self.max_angular = self.declare_parameter("max_angular_speed", math.pi / 2).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.4).value
        self.link_timeout = self.declare_parameter("link_timeout", 1.0).value
        # Bool permissions have no source timestamp.  The receive-time lease
        # makes a transient-local sample a restart convenience, not an
        # unbounded authorization.
        self.permission_timeout = self.declare_parameter("permission_timeout", 0.5).value
        self.trajectory_path_tolerance = self.declare_parameter(
            "trajectory_path_tolerance", 0.20
        ).value
        self.trajectory_tolerance = self.declare_parameter("trajectory_tolerance", 0.05).value
        self.trajectory_timeout = self.declare_parameter("trajectory_timeout", 5.0).value
        # This is velocity-integrated odometry, not an encoder/SLAM pose
        # measurement. Never publish the ROS all-zero covariance (perfect
        # certainty); these conservative values let consumers fuse it honestly.
        self.odom_xy_stddev = self.declare_parameter("odom_xy_stddev", 0.05).value
        self.odom_yaw_stddev = self.declare_parameter("odom_yaw_stddev", 0.10).value
        self.twist_xy_stddev = self.declare_parameter("twist_xy_stddev", 0.10).value
        self.twist_yaw_stddev = self.declare_parameter("twist_yaw_stddev", 0.20).value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe").value
        self.odom_topic = self.declare_parameter("odom_topic", "/wheel/odometry").value
        self.publish_odom_tf = self.declare_parameter("publish_odom_tf", False).value
        self.auto_arm_on_startup = self.declare_parameter("auto_arm_on_startup", True).value
        self.publish_motor_health_enabled = self.declare_parameter(
            "publish_motor_health", True
        ).value
        self.base_permission_topic = self.declare_parameter(
            "base_motion_permission_topic", "/safety/base_motion_permitted"
        ).value
        self.arm_permission_topic = self.declare_parameter(
            "arm_motion_permission_topic", "/safety/arm_motion_permitted"
        ).value
        calibration_file = self.declare_parameter(
            "arm_calibration_file", os.path.expanduser("~/.ros/lekiwi_arm_calibration.json")
        ).value
        # Raw odometry is a continuous local frame. Localization owns the
        # global map->odom placement and must never be baked into wheel odom.
        initial_x = self.declare_parameter("initial_x", 0.0).value
        initial_y = self.declare_parameter("initial_y", 0.0).value
        initial_yaw = self.declare_parameter("initial_yaw", 0.0).value

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
        state_keys = tuple(f"{joint}.pos" for joint in ARM_JOINTS) + (
            "x.vel", "y.vel", "theta.vel",
        )
        self.robot = LeKiwiZmqClient(
            remote_ip, command_port, observation_port, state_keys,
            curve_credentials=curve_credentials,
            require_metadata=not allow_legacy_telemetry,
        )
        self.robot.connect()
        self.torque = TorqueControlClient(
            remote_ip, torque_control_port, torque_control_timeout_ms,
            client_secret_key_file=curve_client_secret,
            server_public_key_file=curve_server_public,
        )
        self.torque_lock = threading.Lock()
        # Serializes physical actuator transitions and command submission. Local
        # state uses a separate lock so callbacks remain responsive while ZeroMQ
        # or the torque service is delayed.
        self.action_lock = threading.Lock()
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
        self.pose = (initial_x, initial_y, initial_yaw)
        now = self.get_clock().now()
        self.odom_samples = OdometrySampleClock()
        self.last_observation = None
        self.last_observation_token = None
        self.last_fresh = now
        self.link_lost = False
        self.armed = False
        self.torque_fault = False
        self.base_motion_permitted = False
        self.arm_motion_permitted = False
        self._base_permission_received_at_ns = None
        self._arm_permission_received_at_ns = None
        self._base_permission_expired = False
        self._arm_permission_expired = False
        # Startup is allowed to arm only after a complete, fresh observation.  A link
        # loss or an explicit disarm clears this one-shot flag, so recovery can never
        # resume movement without an operator deliberately arming again.
        self.auto_arm_pending = bool(self.auto_arm_on_startup)
        calibration_path = os.path.expanduser(calibration_file)
        self.arm_calibrated = os.path.isfile(calibration_path)
        self.stop_pending = True
        self.state_lock = threading.Lock()
        self.arm_zero_positions, self.arm_directions = load_calibration(calibration_file)
        if not self.arm_calibrated:
            self.get_logger().warn(
                f"No URDF arm calibration at {calibration_file}; arm trajectories are disabled"
            )
        self.arm_positions = {name: 0.0 for name in ARM_JOINTS}
        self.trajectory = None
        self.trajectory_lock = threading.Lock()
        self.safety_publish_lock = threading.Lock()
        self.safety_state = "DISARMED"
        self.validate_motion_parameters()
        self.permission_timeout_ns = int(self.permission_timeout * 1_000_000_000)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
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
            Bool, self.base_permission_topic, self.on_base_permission, safety_qos,
            callback_group=self.safety_callback_group,
        )
        self.create_subscription(
            Bool, self.arm_permission_topic, self.on_arm_permission, safety_qos,
            callback_group=self.safety_callback_group,
        )
        self.create_service(Trigger, "safety/arm", self.arm)
        self.create_service(Trigger, "safety/disarm", self.disarm)
        self.trajectory_server = ActionServer(
            self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_trajectory, goal_callback=self.accept_trajectory,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(0.05, self.update)
        self.get_logger().info(f"Connected to LeKiwi host at {remote_ip}")
        self.get_logger().info(f"Accepting guarded base commands from {self.cmd_vel_topic}")
        self.publish_safety("DISARMED")

    def on_base_permission(self, message):
        permitted = bool(message.data)
        received_at_ns = time.monotonic_ns()
        with self.state_lock:
            self.base_motion_permitted = permitted
            self._base_permission_received_at_ns = received_at_ns
            self._base_permission_expired = not permitted
            if not permitted:
                self.command = Twist()
                self.command_stamp = self.get_clock().now()
            armed = self.armed
            arm_current = self._permission_is_current(
                self.arm_motion_permitted,
                getattr(self, "_arm_permission_received_at_ns", None),
                received_at_ns,
            )
        if not permitted and armed and not arm_current:
            self.get_logger().error(
                "All motion capability permissions withdrawn; cutting actuator torque"
            )
            self.set_disarmed("DISARMED")

    def on_arm_permission(self, message):
        permitted = bool(message.data)
        received_at_ns = time.monotonic_ns()
        with self.state_lock:
            was_permitted = bool(self.arm_motion_permitted)
            was_expired = bool(self._arm_permission_expired)
            self.arm_motion_permitted = permitted
            self._arm_permission_received_at_ns = received_at_ns
            self._arm_permission_expired = not permitted
            armed = self.armed
            base_current = self._permission_is_current(
                self.base_motion_permitted,
                getattr(self, "_base_permission_received_at_ns", None),
                received_at_ns,
            )
        newly_withdrawn = not permitted and (was_permitted or not was_expired)
        if newly_withdrawn and armed:
            self.cancel_trajectory("arm safety permission withdrawn")
            if not base_current:
                self.get_logger().error(
                    "All motion capability permissions withdrawn; cutting actuator torque"
                )
                self.set_disarmed("DISARMED")
            else:
                self.get_logger().error(
                    "Arm safety permission withdrawn; canceling arm motion while base remains enabled"
                )

    @staticmethod
    def _permission_is_fresh(received_at_ns, timeout_ns, now_ns=None):
        """Return false when a Bool permission lease was not refreshed."""
        if received_at_ns is None:
            return False
        current = time.monotonic_ns() if now_ns is None else now_ns
        age = current - received_at_ns
        return 0 <= age <= timeout_ns

    def _permission_is_current(self, permitted, received_at_ns, now_ns=None):
        return bool(permitted) and self._permission_is_fresh(
            received_at_ns, getattr(self, "permission_timeout_ns", 0), now_ns
        )

    def _capability_permission_is_current(self, now_ns=None):
        return self._permission_is_current(
            getattr(self, "base_motion_permitted", False),
            getattr(self, "_base_permission_received_at_ns", None),
            now_ns,
        ) or self._permission_is_current(
            getattr(self, "arm_motion_permitted", False),
            getattr(self, "_arm_permission_received_at_ns", None),
            now_ns,
        )

    def enforce_permission_leases(self, now_monotonic_ns=None):
        """Expire stale supervisor decisions independently of DDS delivery."""
        current = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        arm_expired = False
        with self.state_lock:
            base_current = self._permission_is_current(
                getattr(self, "base_motion_permitted", False),
                getattr(self, "_base_permission_received_at_ns", None),
                current,
            )
            if not base_current:
                if getattr(self, "base_motion_permitted", False):
                    self.base_motion_permitted = False
                    self.command = Twist()
                    self.command_stamp = self.get_clock().now()
                self._base_permission_expired = True
            else:
                self._base_permission_expired = False

            arm_current = self._permission_is_current(
                getattr(self, "arm_motion_permitted", False),
                getattr(self, "_arm_permission_received_at_ns", None),
                current,
            )
            if not arm_current:
                if not getattr(self, "_arm_permission_expired", False):
                    arm_expired = True
                self.arm_motion_permitted = False
                self._arm_permission_expired = True
            else:
                self._arm_permission_expired = False
            was_armed = bool(getattr(self, "armed", False))

        if arm_expired and was_armed:
            self.cancel_trajectory("arm safety permission lease expired")
        if was_armed and not base_current and not arm_current:
            self.get_logger().error(
                "All motion capability permission leases expired; cutting actuator torque"
            )
            self.set_disarmed("DISARMED")
        return arm_expired

    def on_command(self, message):
        if not self.twist_is_finite(message):
            self.get_logger().error("Rejecting non-finite base command and disarming")
            self.set_disarmed("DISARMED")
            return
        with self.state_lock:
            if not self.armed or not self._permission_is_current(
                self.base_motion_permitted, getattr(self, "_base_permission_received_at_ns", None)
            ):
                return
            self.command = message
            self.command_stamp = self.get_clock().now()

    def publish_safety(self, state=None):
        # The supervisor treats this as a live driver heartbeat. Serialize
        # state transitions with refreshes so an older ARMED refresh can never
        # overtake a concurrent DISARMED/LINK_LOST transition.
        with self.safety_publish_lock:
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
                self.trajectory["outcome"] = outcome
                self.trajectory["result_code"] = (
                    FollowJointTrajectory.Result.INVALID_GOAL
                    if result_code is None else result_code
                )
                self.trajectory["done"].set()
                self.trajectory = None

    def set_disarmed(self, state, publish=True, clear_torque_fault=False):
        with self.action_lock:
            with self.state_lock:
                was_armed = self.armed
                self.armed = False
                # A manual disarm or link loss must override a still-pending startup arm.
                self.auto_arm_pending = False
                self.command = Twist()
                self.command_stamp = self.get_clock().now()
                self.stop_pending = True

            # Never hold state_lock across a service/network operation. action_lock
            # still guarantees that an arm or command cannot overtake this cut.
            torque_cut = self.set_servo_torque(False)
            with self.state_lock:
                if not torque_cut:
                    self.torque_fault = True
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

    def set_servo_torque(self, enabled):
        """Synchronously require the serial-bus owner to change physical torque."""
        can_log = not hasattr(self, "context") or rclpy.ok(context=self.context)
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

    def arm_after_startup_telemetry(self):
        """Perform the configured, one-shot startup arm after validated telemetry."""
        with self.action_lock:
            with self.state_lock:
                if (
                    not self.auto_arm_pending
                    or self.link_lost
                    or self.armed
                    or not self._permission_is_current(
                        self.arm_motion_permitted,
                        getattr(self, "_arm_permission_received_at_ns", None),
                    )
                    or self.torque_fault
                ):
                    return False
                self.auto_arm_pending = False
            if not self.set_servo_torque(True):
                # A reply can be lost after the host applied the enable.  A
                # separate disable transaction resolves that ambiguous state
                # toward torque-off before reporting the arm request failed.
                torque_cut = self.set_servo_torque(False)
                with self.state_lock:
                    if not torque_cut:
                        self.torque_fault = True
                    state = "TORQUE_FAULT" if self.torque_fault else "DISARMED"
                self.publish_safety(state)
                self.get_logger().error("Initial telemetry is healthy, but the motor host would not enable torque")
                return False
            with self.state_lock:
                still_safe = (
                    not self.link_lost
                    and self._permission_is_current(
                        self.arm_motion_permitted,
                        getattr(self, "_arm_permission_received_at_ns", None),
                    )
                    and not self.torque_fault
                )
            if not still_safe:
                torque_cut = self.set_servo_torque(False)
                with self.state_lock:
                    if not torque_cut:
                        self.torque_fault = True
                    state = "TORQUE_FAULT" if self.torque_fault else "DISARMED"
                self.publish_safety(state)
                return False
            with self.state_lock:
                self.armed = True
                self.command = Twist()
                self.command_stamp = self.get_clock().now()
            self.publish_safety("ARMED")
            self.get_logger().info("Armed after initial healthy LeKiwi telemetry")
            return True

    def arm(self, request, response):
        del request
        now = self.get_clock().now()
        telemetry_age = (now - self.last_fresh).nanoseconds / 1e9
        with self.action_lock:
            with self.state_lock:
                if self.torque_fault:
                    response.success = False
                    response.message = (
                        "servo torque state is fault-latched; call safety/disarm "
                        "and confirm torque-off before rearming"
                    )
                    return response
                if not self._capability_permission_is_current():
                    response.success = False
                    response.message = "continuous safety supervisor has not granted any motion capability"
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
            if not self.set_servo_torque(True):
                # Treat an enable timeout as physically ambiguous: the host
                # may have completed the write before its reply was lost.
                torque_cut = self.set_servo_torque(False)
                with self.state_lock:
                    if not torque_cut:
                        self.torque_fault = True
                    state = "TORQUE_FAULT" if self.torque_fault else "DISARMED"
                self.publish_safety(state)
                response.success = False
                response.message = (
                    "motor host did not confirm servo torque enabled; "
                    + (
                        "the fail-safe disable was not confirmed"
                        if not torque_cut
                        else "a fail-safe disable was confirmed"
                    )
                )
                return response
            with self.state_lock:
                still_safe = (
                    self._capability_permission_is_current()
                    and not self.link_lost
                    and not self.torque_fault
                )
            if not still_safe:
                torque_cut = self.set_servo_torque(False)
                with self.state_lock:
                    if not torque_cut:
                        self.torque_fault = True
                    state = "TORQUE_FAULT" if self.torque_fault else "DISARMED"
                self.publish_safety(state)
                response.success = False
                response.message = "safety state changed while enabling torque"
                return response
            with self.state_lock:
                self.armed = True
                self.command = Twist()
                self.command_stamp = now
            self.publish_safety("ARMED")
            response.success = True
            response.message = "armed at current measured position; send a new command"
            return response

    def disarm(self, request, response):
        del request
        torque_cut = self.set_disarmed("DISARMED", clear_torque_fault=True)
        response.success = torque_cut
        response.message = (
            "commands disabled and all servo torque cut"
            if torque_cut
            else "commands disabled, but the motor host did not confirm servo torque cut; use the physical emergency stop"
        )
        return response

    def accept_trajectory(self, goal):
        if not self.armed or not self._permission_is_current(
            self.arm_motion_permitted,
            getattr(self, "_arm_permission_received_at_ns", None),
        ):
            # MoveIt surfaces this only as an unexplained "goal was rejected"; say why here.
            self.get_logger().warn(
                "Rejecting arm trajectory: driver is disarmed or arm safety permission is absent"
            )
            return GoalResponse.REJECT
        if not self.arm_calibrated:
            self.get_logger().warn(
                "Rejecting arm trajectory: no valid arm calibration is installed"
            )
            return GoalResponse.REJECT
        try:
            points = [
                (
                    self.trajectory_time(point), point.positions, point.velocities,
                    point.accelerations, point.effort,
                )
                for point in goal.trajectory.points
            ]
            with self.trajectory_lock:
                start_positions = self.arm_positions.copy()
            validate_trajectory(
                goal.trajectory.joint_names, points, start_positions
            )
            self.requested_tolerances(goal, goal.trajectory.joint_names)
            self.trajectory_stamp_ns(goal.trajectory.header.stamp)
            if goal.multi_dof_trajectory.joint_names or goal.multi_dof_trajectory.points:
                raise ValueError("multi-DOF trajectories are unsupported")
        except (IndexError, ValueError) as error:
            self.get_logger().warn(f"Rejecting arm trajectory: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        names = goal_handle.request.trajectory.joint_names
        requested_points = [
            (
                self.trajectory_time(point), point.positions, point.velocities,
                point.accelerations, point.effort,
            )
            for point in goal_handle.request.trajectory.points
        ]
        path_tolerances, goal_tolerances, goal_time_tolerance = self.requested_tolerances(
            goal_handle.request, names
        )
        scheduled_ns = self.trajectory_stamp_ns(goal_handle.request.trajectory.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        if scheduled_ns and scheduled_ns < now_ns - 100_000_000:
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP,
                error_string="trajectory header timestamp is in the past",
            )
        start_delay = max(0.0, (scheduled_ns - now_ns) / 1e9) if scheduled_ns else 0.0
        trajectory = {
            "start": time.monotonic() + start_delay,
            "done": threading.Event(),
            "path_tolerances": path_tolerances,
            "goal_tolerances": goal_tolerances,
            "goal_time_tolerance": goal_time_tolerance,
        }
        with self.state_lock:
            if not self.armed or not self._permission_is_current(
                self.arm_motion_permitted,
                getattr(self, "_arm_permission_received_at_ns", None),
            ):
                goal_handle.abort()
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                    error_string="driver was disarmed before trajectory execution",
                )
            with self.trajectory_lock:
                start_positions = {name: self.arm_positions[name] for name in names}
                try:
                    # Feedback can change between goal acceptance and this callback.
                    # Recheck the first segment against its actual execution start.
                    points = prepare_trajectory(names, requested_points, start_positions)
                except ValueError as error:
                    goal_handle.abort()
                    return FollowJointTrajectory.Result(
                        error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                        error_string=str(error),
                    )
                if self.trajectory:
                    self.trajectory["outcome"] = "preempted"
                    self.trajectory["result_code"] = FollowJointTrajectory.Result.INVALID_GOAL
                    self.trajectory["done"].set()
                trajectory["start_positions"] = start_positions
                trajectory["names"] = tuple(names)
                trajectory["points"] = points
                self.trajectory = trajectory
        self.command = Twist()
        self.command_stamp = self.get_clock().now()

        while not trajectory["done"].wait(0.05):
            if goal_handle.is_cancel_requested:
                with self.trajectory_lock:
                    if self.trajectory is trajectory:
                        self.trajectory = None
                    trajectory["outcome"] = "canceled"
                    trajectory["done"].set()
                goal_handle.canceled()
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
            self.publish_trajectory_feedback(goal_handle, trajectory)

        if trajectory.get("outcome") == "succeeded":
            goal_handle.succeed()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
        goal_handle.abort()
        return FollowJointTrajectory.Result(
            error_code=trajectory.get(
                "result_code", FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            ),
            error_string=trajectory.get("outcome", "trajectory preempted"),
        )

    def requested_tolerances(self, goal, names):
        if goal.component_path_tolerance or goal.component_goal_tolerance:
            raise ValueError("component tolerances are unsupported for revolute arm joints")
        default_path = dict.fromkeys(names, self.trajectory_path_tolerance)
        path = position_tolerances(names, goal.path_tolerance, default_path)
        default_goal = dict.fromkeys(names, self.trajectory_tolerance)
        goal_tolerances = position_tolerances(names, goal.goal_tolerance, default_goal)
        goal_time = self.duration_seconds(goal.goal_time_tolerance)
        return path, goal_tolerances, goal_time or self.trajectory_timeout

    def publish_trajectory_feedback(self, goal_handle, trajectory):
        elapsed = max(0.0, time.monotonic() - trajectory["start"])
        with self.trajectory_lock:
            actual = {
                name: self.arm_positions[name] for name in trajectory["names"]
            }
        desired, velocities, accelerations = sample_trajectory(
            trajectory["names"], trajectory["start_positions"],
            trajectory["points"], elapsed,
        )
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory["names"])
        feedback.desired.positions = [desired[name] for name in trajectory["names"]]
        feedback.desired.velocities = [velocities[name] for name in trajectory["names"]]
        feedback.desired.accelerations = [accelerations[name] for name in trajectory["names"]]
        feedback.actual.positions = [actual[name] for name in trajectory["names"]]
        feedback.error.positions = [
            desired[name] - actual[name] for name in trajectory["names"]
        ]
        seconds = int(elapsed)
        nanoseconds = int((elapsed - seconds) * 1e9)
        for point in (feedback.desired, feedback.actual, feedback.error):
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
        goal_handle.publish_feedback(feedback)

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

    @staticmethod
    def twist_is_finite(message):
        return all(math.isfinite(value) for value in (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ))

    @staticmethod
    def trajectory_time(point):
        seconds = point.time_from_start.sec
        nanoseconds = point.time_from_start.nanosec
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError("trajectory time is malformed")
        value = seconds + nanoseconds / 1e9
        if not math.isfinite(value):
            raise ValueError("trajectory time must be finite")
        return value

    @staticmethod
    def duration_seconds(duration):
        if duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000:
            raise ValueError("duration is malformed")
        value = duration.sec + duration.nanosec / 1e9
        if not math.isfinite(value):
            raise ValueError("duration must be finite")
        return value

    @staticmethod
    def trajectory_stamp_ns(stamp):
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("trajectory header timestamp is malformed")
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def validate_motion_parameters(self):
        """Reject values that would make a command unsafe or undefined."""
        positive = {
            "xy_velocity_scale": self.xy_scale,
            "yaw_velocity_scale": self.yaw_scale,
            "command_timeout": self.command_timeout,
            "link_timeout": self.link_timeout,
            "permission_timeout": self.permission_timeout,
            "trajectory_path_tolerance": self.trajectory_path_tolerance,
            "trajectory_tolerance": self.trajectory_tolerance,
            "trajectory_timeout": self.trajectory_timeout,
            "odom_xy_stddev": self.odom_xy_stddev,
            "odom_yaw_stddev": self.odom_yaw_stddev,
            "twist_xy_stddev": self.twist_xy_stddev,
            "twist_yaw_stddev": self.twist_yaw_stddev,
        }
        nonnegative = {
            "max_linear_speed": self.max_linear,
            "max_angular_speed": self.max_angular,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and greater than zero")
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not isinstance(self.cmd_vel_topic, str) or not self.cmd_vel_topic.strip():
            raise ValueError("cmd_vel_topic must be a non-empty topic name")

    def observation_is_fresh(self, observation):
        # A stationary robot with cameras intentionally disabled reports identical
        # numeric values in every packet. Comparing those values mistakes healthy,
        # fresh telemetry for a dropout. The client sequence advances only when its ZMQ
        # socket consumed a new multipart observation; cached observations leave it
        # unchanged, which is the signal the driver actually needs for link safety.
        robot = getattr(self, "robot", None)
        token_marker = object()
        token = getattr(robot, "observation_token", token_marker)
        if token is None:
            # The repository client exposes this attribute before receiving its
            # first valid packet. Its initialized/cached zero state is not data.
            return False
        if token is token_marker:
            sequence = getattr(robot, "observation_sequence", None)
            token = ("zmq", sequence) if sequence is not None else None
        if token is None:
            # Keep the function useful for lightweight tests and alternate clients that
            # have not implemented the transport sequence yet.
            token = tuple(sorted(
                (name, float(value))
                for name, value in observation.items()
                if name.endswith((".pos", ".vel"))
            ))
        fresh = token != getattr(self, "last_observation_token", None)
        self.last_observation_token = token
        self.last_observation = observation
        return fresh

    def handle_host_session_change(self):
        if not getattr(self.robot, "observation_session_changed", False):
            return False
        # A new host process always starts torque-off. Keep logical state
        # aligned and require a deliberate re-arm even if downtime was shorter
        # than the telemetry watchdog threshold.
        self.odom_samples.reset()
        self.set_disarmed("DISARMED")
        self.get_logger().error(
            "LeKiwi host session changed; robot remains disarmed until an explicit safety/arm request"
        )
        return True

    def enforce_reported_torque_state(self):
        """Keep logical arming synchronized with authenticated host readback."""
        reported = getattr(self.robot, "observation_torque_enabled", None)
        if reported is None:
            # Explicit legacy compatibility has no physical-state field.
            return False
        with self.state_lock:
            logical = self.armed
        if reported == logical:
            return False
        self.get_logger().error(
            "Motor host torque state changed outside the driver's arm/disarm transaction; disarming"
        )
        self.set_disarmed("DISARMED")
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

    def update(self):
        # Run before telemetry polling so a silent supervisor still revokes
        # arm torque even when the motor host is returning cached data.
        self.enforce_permission_leases()
        now = self.get_clock().now()
        try:
            observation = self.robot.get_observation()
        except Exception as error:
            if not self.link_lost:
                self.get_logger().error(f"LeKiwi telemetry failed: {error}")
                self.link_lost = True
                self.odom_samples.reset()
                self.set_disarmed("LINK_LOST")
            return

        missing_state_keys = getattr(self.robot, "missing_state_keys", ())
        if not self.observation_is_valid(observation, missing_state_keys):
            if not self.link_lost:
                reason = "LeKiwi telemetry is incomplete or non-finite"
                if missing_state_keys:
                    reason = f"LeKiwi telemetry is missing: {', '.join(missing_state_keys)}"
                self.get_logger().error(reason)
                self.link_lost = True
                self.odom_samples.reset()
                self.set_disarmed("LINK_LOST")
            return

        if not self.observation_is_fresh(observation):
            quiet = (now - self.last_fresh).nanoseconds / 1e9
            if quiet > self.link_timeout:
                if not self.link_lost:
                    self.get_logger().error(
                        f"No fresh LeKiwi telemetry for {quiet:.1f}s; waiting for recovery"
                    )
                    self.link_lost = True
                    self.odom_samples.reset()
                    self.set_disarmed("LINK_LOST")
            return

        self.last_fresh = now
        self.publish_motor_health(now.to_msg())
        arm_positions = joint_positions(
            observation, self.arm_zero_positions, self.arm_directions
        )
        with self.trajectory_lock:
            self.arm_positions = arm_positions
        self.handle_host_session_change()
        self.enforce_reported_torque_state()
        self.arm_after_startup_telemetry()
        if self.link_lost:
            # The recovered sample establishes a new origin. Never integrate
            # a reported velocity across an interval with no telemetry.
            self.odom_samples.reset()
            self.link_lost = False
            with self.state_lock:
                recovered_state = "TORQUE_FAULT" if self.torque_fault else "DISARMED"
            self.publish_safety(recovered_state)
            self.get_logger().warn("LeKiwi telemetry recovered; inspect robot, then call safety/arm")

        velocity = (
            float(observation["x.vel"]) * self.xy_scale,
            float(observation["y.vel"]) * self.xy_scale,
            math.radians(float(observation["theta.vel"])) * self.yaw_scale,
        )
        sample_dt = self.odom_samples.accept(
            self.last_observation_token,
            now.nanoseconds,
            getattr(self.robot, "observation_sample_monotonic_ns", None),
        )
        if sample_dt is not None:
            self.pose = integrate_pose(self.pose, velocity, sample_dt)
        elif self.odom_samples.discontinuity:
            self.get_logger().warn(
                f"Odometry discontinuity: {self.odom_samples.discontinuity}; not integrating this sample"
            )
        with self.state_lock:
            armed = self.armed
        if not armed:
            send_error = None
            try:
                with self.action_lock:
                    with self.state_lock:
                        # An arm request may have completed after the snapshot.
                        # In that case, skip this cycle rather than sending a
                        # stale zero action after torque was enabled.
                        if self.armed:
                            return
                        send_stop = self.stop_pending
                    if send_stop:
                        self.robot.send_action({
                            **{f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS},
                            "x.vel": 0.0,
                            "y.vel": 0.0,
                            "theta.vel": 0.0,
                        })
                        with self.state_lock:
                            if not self.armed:
                                self.stop_pending = False
            except Exception as error:
                send_error = error
            if send_error is not None:
                self.get_logger().error(f"LeKiwi stop command failed: {send_error}")
                self.link_lost = True
                self.set_disarmed("LINK_LOST")
                return
            # Torque state is not motion state: retain measured odometry if the
            # robot is pushed or coasts while commands are inhibited.
            self.publish_state(now.to_msg(), observation, velocity)
            self.publish_safety()
            return

        with self.state_lock:
            stale = (now - self.command_stamp).nanoseconds / 1e9 > self.command_timeout
            cmd = Twist() if stale or not self._permission_is_current(
                self.base_motion_permitted,
                getattr(self, "_base_permission_received_at_ns", None),
            ) else self.command
        action = {
            f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS
        }
        with self.trajectory_lock:
            trajectory = self.trajectory
            if trajectory:
                elapsed = time.monotonic() - trajectory["start"]
                positions, _velocities, _accelerations = sample_trajectory(
                    trajectory["names"], trajectory["start_positions"],
                    trajectory["points"], elapsed,
                )
                action.update({
                    f"{name}.pos": value for name, value in action_positions(
                        positions.keys(), positions.values(),
                        self.arm_zero_positions, self.arm_directions,
                    ).items()
                })
                final_point = trajectory["points"][-1]
                path_violation = next((
                    name for name, tolerance in trajectory["path_tolerances"].items()
                    if elapsed < final_point.time
                    and abs(self.arm_positions[name] - positions[name]) > tolerance
                ), None)
                if path_violation:
                    trajectory["outcome"] = f"path tolerance exceeded for {path_violation}"
                    trajectory["result_code"] = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    trajectory["done"].set()
                    self.trajectory = None
                elif elapsed >= final_point.time and all(
                    abs(self.arm_positions[name] - final_point.positions[name]) <= tolerance
                    for name, tolerance in trajectory["goal_tolerances"].items()
                ):
                    trajectory["outcome"] = "succeeded"
                    trajectory["done"].set()
                    self.trajectory = None
                elif elapsed > final_point.time + trajectory["goal_time_tolerance"]:
                    trajectory["outcome"] = "goal tolerance exceeded"
                    trajectory["result_code"] = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    trajectory["done"].set()
                    self.trajectory = None
        if trajectory:
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
        try:
            # Serialize the final armed check with disarm. Once disarm returns, no
            # in-flight update can submit a previously prepared non-zero action.
            with self.action_lock:
                with self.state_lock:
                    if not self.armed:
                        return
                    arm_permitted = self._permission_is_current(
                        self.arm_motion_permitted,
                        getattr(self, "_arm_permission_received_at_ns", None),
                    )
                    base_permitted = self._permission_is_current(
                        self.base_motion_permitted,
                        getattr(self, "_base_permission_received_at_ns", None),
                    )
                if not arm_permitted:
                    self.cancel_trajectory("arm safety permission withdrawn")
                if not base_permitted:
                    action["x.vel"] = action["y.vel"] = action["theta.vel"] = 0.0
                self.robot.send_action(action)
        except Exception as error:
            self.get_logger().error(f"LeKiwi command failed: {error}")
            self.link_lost = True
            self.set_disarmed("LINK_LOST")
            return

        self.publish_state(now.to_msg(), observation, velocity)
        self.publish_safety()

    def publish_state(self, stamp, observation, velocity):
        x, y, yaw = self.pose
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
        # Indices follow ROS's row-major [x, y, z, roll, pitch, yaw] convention.
        # z/roll/pitch are unobserved by this planar driver, so their deliberately
        # large variance prevents a 3D estimator mistaking them for measurements.
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = odom.pose.covariance[7] = self.odom_xy_stddev ** 2
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = self.odom_yaw_stddev ** 2
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = odom.twist.covariance[7] = self.twist_xy_stddev ** 2
        odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = self.twist_yaw_stddev ** 2
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        if self.publish_odom_tf:
            self.tf.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(ARM_JOINTS)
        joints.position = [self.arm_positions[name] for name in ARM_JOINTS]
        self.joint_pub.publish(joints)

    def publish_motor_health(self, stamp):
        """Publish only the snapshot validated with this fresh host observation."""
        if not getattr(self, "publish_motor_health_enabled", True):
            return
        statuses = getattr(self.robot, "observation_motor_health", None)
        if not statuses:
            # This path is defensive: the ZeroMQ client rejects absent or
            # malformed health telemetry before a sample reaches ``update``.
            return
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
        self.motor_health_pub.publish(message)

    def destroy_node(self):
        try:
            # ROS-stack shutdown is a disarm event. The separately supervised
            # host otherwise remains alive and could retain servo torque.
            # SIGINT may already have invalidated the rcl context. Physical
            # disarm must still run, but publishing through a dead context
            # would turn a clean shutdown into exit code 1.
            self.set_disarmed(
                "DISARMED", publish=rclpy.ok(context=self.context)
            )
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
    executor = MultiThreadedExecutor(num_threads=3)
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
