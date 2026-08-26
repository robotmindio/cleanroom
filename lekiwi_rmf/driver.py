#!/usr/bin/env python3
import math
import os
import signal
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from lekiwi_rmf.arm_trajectory import (
    ARM_JOINTS, action_positions, interpolate_positions, joint_positions, load_calibration,
    validate_trajectory,
)
from lekiwi_rmf.odometry import integrate_pose


class LeKiwiDriver(Node):
    def __init__(self):
        super().__init__("lekiwi_driver")
        from lerobot.robots.lekiwi.config_lekiwi import LeKiwiClientConfig
        from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient

        class SequencedLeKiwiClient(LeKiwiClient):
            """Expose whether the client actually received a new ZMQ observation."""

            def __init__(self, *args, **kwargs):
                self.observation_sequence = 0
                super().__init__(*args, **kwargs)

            def _poll_and_get_latest_message(self):
                message = super()._poll_and_get_latest_message()
                if message is not None:
                    self.observation_sequence += 1
                return message

        remote_ip = self.declare_parameter("remote_ip", "127.0.0.1").value
        robot_id = self.declare_parameter("robot_id", "lekiwi_1").value
        self.xy_scale = self.declare_parameter("xy_velocity_scale", 1.0).value
        self.yaw_scale = self.declare_parameter("yaw_velocity_scale", 1.0).value
        self.max_linear = self.declare_parameter("max_linear_speed", 0.3).value
        self.max_angular = self.declare_parameter("max_angular_speed", math.pi / 2).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.4).value
        self.link_timeout = self.declare_parameter("link_timeout", 1.0).value
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
        self.auto_arm_on_startup = self.declare_parameter("auto_arm_on_startup", True).value
        calibration_file = self.declare_parameter(
            "arm_calibration_file", os.path.expanduser("~/.ros/lekiwi_arm_calibration.json")
        ).value
        initial_x = self.declare_parameter("initial_x", -4.0).value
        initial_y = self.declare_parameter("initial_y", -2.5).value
        initial_yaw = self.declare_parameter("initial_yaw", 0.0).value

        self.robot = SequencedLeKiwiClient(LeKiwiClientConfig(remote_ip=remote_ip, id=robot_id))
        self.robot.connect()
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
        self.pose = (initial_x, initial_y, initial_yaw)
        self.last_update = self.get_clock().now()
        self.last_observation = None
        self.last_observation_token = None
        self.last_fresh = self.last_update
        self.link_lost = False
        self.armed = False
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
        self.validate_motion_parameters()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        safety_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.create_publisher(String, "safety/state", safety_qos)
        self.safety_marker_pub = self.create_publisher(Marker, "safety/marker", safety_qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_command, 10)
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

    def on_command(self, message):
        if not self.twist_is_finite(message):
            self.get_logger().error("Rejecting non-finite base command and disarming")
            self.set_disarmed("DISARMED")
            return
        with self.state_lock:
            if not self.armed:
                return
            self.command = message
            self.command_stamp = self.get_clock().now()

    def publish_safety(self, state):
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
        }[state]
        marker.text = state
        self.safety_marker_pub.publish(marker)

    def cancel_trajectory(self, outcome):
        with self.trajectory_lock:
            if self.trajectory:
                self.trajectory["outcome"] = outcome
                self.trajectory["done"].set()
                self.trajectory = None

    def set_disarmed(self, state):
        with self.state_lock:
            was_armed = self.armed
            self.armed = False
            # A manual disarm or link loss must override a still-pending startup arm.
            self.auto_arm_pending = False
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
        self.stop_pending = True
        self.cancel_trajectory("safety disarmed")
        if was_armed:
            self.get_logger().warn(f"Robot disarmed: {state}")
        self.publish_safety(state)

    def arm_after_startup_telemetry(self):
        """Perform the configured, one-shot startup arm after validated telemetry."""
        with self.state_lock:
            if not self.auto_arm_pending or self.link_lost or self.armed:
                return False
            self.auto_arm_pending = False
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
        with self.state_lock:
            if (
                self.link_lost
                or self.last_observation is None
                or telemetry_age < 0.0
                or telemetry_age > self.link_timeout
            ):
                response.success = False
                response.message = "no fresh LeKiwi telemetry"
                return response
            self.armed = True
        self.command = Twist()
        self.command_stamp = now
        self.publish_safety("ARMED")
        response.success = True
        response.message = "armed at current measured position; send a new command"
        return response

    def disarm(self, request, response):
        del request
        self.set_disarmed("DISARMED")
        response.success = True
        response.message = "commands disabled; motor torque is controlled by the LeRobot host"
        return response

    def accept_trajectory(self, goal):
        if not self.armed:
            # MoveIt surfaces this only as an unexplained "goal was rejected"; say why here.
            self.get_logger().warn(
                "Rejecting arm trajectory: driver is disarmed -- call the safety/arm service first"
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
        points = [
            (point_time, dict(zip(names, positions)))
            for point_time, positions, _velocities, _accelerations, _effort
            in requested_points
        ]
        trajectory = {"start": time.monotonic(), "points": points, "done": threading.Event()}
        with self.state_lock:
            if not self.armed:
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
                    validate_trajectory(names, requested_points, start_positions)
                except ValueError as error:
                    goal_handle.abort()
                    return FollowJointTrajectory.Result(
                        error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                        error_string=str(error),
                    )
                if self.trajectory:
                    self.trajectory["outcome"] = "preempted"
                    self.trajectory["done"].set()
                trajectory["start_positions"] = start_positions
                self.trajectory = trajectory
        self.command = Twist()
        self.command_stamp = self.get_clock().now()

        while not trajectory["done"].wait(0.05):
            if goal_handle.is_cancel_requested:
                with self.trajectory_lock:
                    if self.trajectory is trajectory:
                        self.trajectory = None
                goal_handle.canceled()
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)

        if trajectory.get("outcome") == "succeeded":
            goal_handle.succeed()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
        goal_handle.abort()
        return FollowJointTrajectory.Result(
            error_code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            error_string=trajectory.get("outcome", "trajectory preempted"),
        )

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

    def validate_motion_parameters(self):
        """Reject values that would make a command unsafe or undefined."""
        positive = {
            "xy_velocity_scale": self.xy_scale,
            "yaw_velocity_scale": self.yaw_scale,
            "command_timeout": self.command_timeout,
            "link_timeout": self.link_timeout,
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
        sequence = getattr(getattr(self, "robot", None), "observation_sequence", None)
        if sequence is None:
            # Keep the function useful for lightweight tests and alternate clients that
            # have not implemented the transport sequence yet.
            token = tuple(sorted(
                (name, float(value))
                for name, value in observation.items()
                if name.endswith((".pos", ".vel"))
            ))
        else:
            token = ("zmq", sequence)
        fresh = token != getattr(self, "last_observation_token", None)
        self.last_observation_token = token
        self.last_observation = observation
        return fresh

    @staticmethod
    def observation_is_valid(observation):
        """Require complete, finite arm and base feedback before commanding motion."""
        if not isinstance(observation, dict):
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
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        try:
            observation = self.robot.get_observation()
        except Exception as error:
            if not self.link_lost:
                self.get_logger().error(f"LeKiwi telemetry failed: {error}")
                self.link_lost = True
                self.set_disarmed("LINK_LOST")
            return

        if not self.observation_is_valid(observation):
            if not self.link_lost:
                self.get_logger().error("LeKiwi telemetry is incomplete or non-finite")
                self.link_lost = True
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
                    self.set_disarmed("LINK_LOST")
            return

        self.last_fresh = now
        arm_positions = joint_positions(
            observation, self.arm_zero_positions, self.arm_directions
        )
        with self.trajectory_lock:
            self.arm_positions = arm_positions
        self.arm_after_startup_telemetry()
        if self.link_lost:
            self.link_lost = False
            self.publish_safety("DISARMED")
            self.get_logger().warn("LeKiwi telemetry recovered; inspect robot, then call safety/arm")
        if not self.armed:
            if self.stop_pending:
                try:
                    self.robot.send_action({
                        **{f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS},
                        "x.vel": 0.0,
                        "y.vel": 0.0,
                        "theta.vel": 0.0,
                    })
                except Exception as error:
                    self.get_logger().error(f"LeKiwi stop command failed: {error}")
                    self.link_lost = True
                    self.publish_safety("LINK_LOST")
                    return
                self.stop_pending = False
            self.publish_state(now.to_msg(), observation, (0.0, 0.0, 0.0))
            return

        with self.state_lock:
            stale = (now - self.command_stamp).nanoseconds / 1e9 > self.command_timeout
            cmd = Twist() if stale else self.command
        action = {
            f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS
        }
        with self.trajectory_lock:
            trajectory = self.trajectory
            if trajectory:
                elapsed = time.monotonic() - trajectory["start"]
                positions = interpolate_positions(
                    trajectory["start_positions"], trajectory["points"], elapsed
                )
                action.update({
                    f"{name}.pos": value for name, value in action_positions(
                        positions.keys(), positions.values(),
                        self.arm_zero_positions, self.arm_directions,
                    ).items()
                })
                final_time, final_positions = trajectory["points"][-1]
                if elapsed >= final_time and all(
                    abs(self.arm_positions[name] - position) <= self.trajectory_tolerance
                    for name, position in final_positions.items()
                ):
                    trajectory["outcome"] = "succeeded"
                    trajectory["done"].set()
                    self.trajectory = None
                elif elapsed > final_time + self.trajectory_timeout:
                    trajectory["outcome"] = "goal tolerance exceeded"
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
            with self.state_lock:
                if not self.armed:
                    return
                self.robot.send_action(action)
        except Exception as error:
            self.get_logger().error(f"LeKiwi command failed: {error}")
            self.link_lost = True
            self.set_disarmed("LINK_LOST")
            return

        velocity = (
            float(observation["x.vel"]) * self.xy_scale,
            float(observation["y.vel"]) * self.xy_scale,
            math.radians(float(observation["theta.vel"])) * self.yaw_scale,
        )
        self.pose = integrate_pose(self.pose, velocity, max(0.0, min(dt, 0.2)))
        self.publish_state(now.to_msg(), observation, velocity)

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
        self.tf.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(ARM_JOINTS)
        joints.position = [self.arm_positions[name] for name in ARM_JOINTS]
        self.joint_pub.publish(joints)

    def destroy_node(self):
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
