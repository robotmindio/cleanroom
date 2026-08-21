#!/usr/bin/env python3
import math
import os
import threading
import time
from copy import deepcopy
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
import yaml

from lekiwi_rmf.arm_trajectory import (
    ARM_JOINTS, action_positions, interpolate_positions, joint_positions, load_calibration,
)
from lekiwi_rmf.odometry import integrate_pose


class LeKiwiDriver(Node):
    def __init__(self):
        super().__init__("lekiwi_driver")
        from lerobot.robots.lekiwi.config_lekiwi import LeKiwiClientConfig
        from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient

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
        calibration_file = self.declare_parameter(
            "arm_calibration_file", os.path.expanduser("~/.ros/lekiwi_arm_calibration.json")
        ).value
        initial_x = self.declare_parameter("initial_x", -4.0).value
        initial_y = self.declare_parameter("initial_y", -2.5).value
        initial_yaw = self.declare_parameter("initial_yaw", 0.0).value
        camera_info_url = self.declare_parameter(
            "camera_info_url", "file://~/.ros/camera_info/lekiwi_front.yaml"
        ).value

        self.robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=remote_ip, id=robot_id))
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
        self.stop_pending = True
        self.state_lock = threading.Lock()
        self.arm_zero_positions, self.arm_directions = load_calibration(calibration_file)
        if not os.path.exists(os.path.expanduser(calibration_file)):
            self.get_logger().warn(
                f"No URDF arm calibration at {calibration_file}; publishing raw LeRobot joint positions"
            )
        self.arm_positions = {name: 0.0 for name in ARM_JOINTS}
        self.trajectory = None
        self.trajectory_lock = threading.Lock()
        self.validate_motion_parameters()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.front_image_pub = self.create_publisher(Image, "/camera/front/image_raw", 10)
        self.front_info_pub = self.create_publisher(CameraInfo, "/camera/front/camera_info", 10)
        self.wrist_image_pub = self.create_publisher(Image, "/camera/wrist/image_raw", 10)
        self.front_camera_info = self.load_camera_info(camera_info_url)
        safety_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.create_publisher(String, "safety/state", safety_qos)
        self.safety_marker_pub = self.create_publisher(Marker, "safety/marker", safety_qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_command, 10)
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
        self.publish_safety("DISARMED")

    def on_command(self, message):
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

    def load_camera_info(self, url):
        """Load a standard ROS camera-calibration YAML file for host-streamed frames."""
        info = CameraInfo()
        path = Path(os.path.expandvars(url.removeprefix("file://"))).expanduser()
        try:
            data = yaml.safe_load(path.read_text())
            info.width = int(data["image_width"])
            info.height = int(data["image_height"])
            info.distortion_model = data["distortion_model"]
            info.d = data["distortion_coefficients"]["data"]
            info.k = data["camera_matrix"]["data"]
            info.r = data["rectification_matrix"]["data"]
            info.p = data["projection_matrix"]["data"]
        except (FileNotFoundError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().warn(f"Could not load front-camera calibration from {path}: {error}")
        return info

    @staticmethod
    def publish_image(stamp, frame, publisher, frame_id):
        if frame is None or getattr(frame, "ndim", 0) != 3 or frame.shape[2] != 3:
            return None
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = frame_id
        image.height, image.width = frame.shape[:2]
        image.encoding = "bgr8"
        image.is_bigendian = False
        image.step = image.width * 3
        image.data = frame.tobytes()
        publisher.publish(image)
        return image

    def publish_front_camera(self, stamp, observation):
        image = self.publish_image(
            stamp, observation.get("front"), self.front_image_pub, "front_camera_optical_frame"
        )
        if image is None:
            return

        info = deepcopy(self.front_camera_info)
        info.header = image.header
        if not info.width:
            info.width, info.height = image.width, image.height
        self.front_info_pub.publish(info)

    def publish_wrist_camera(self, stamp, observation):
        self.publish_image(stamp, observation.get("wrist"), self.wrist_image_pub, "tool")

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
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
        self.stop_pending = True
        self.cancel_trajectory("safety disarmed")
        if was_armed:
            self.get_logger().warn(f"Robot disarmed: {state}")
        self.publish_safety(state)

    def arm(self, request, response):
        del request
        if self.link_lost or self.last_observation is None:
            response.success = False
            response.message = "no fresh LeKiwi telemetry"
            return response
        with self.state_lock:
            self.armed = True
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
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
            return GoalResponse.REJECT
        try:
            if not goal.trajectory.points:
                raise ValueError("trajectory must contain a point")
            previous_time = -1.0
            for point in goal.trajectory.points:
                action_positions(
                    goal.trajectory.joint_names, point.positions,
                    self.arm_zero_positions, self.arm_directions,
                )
                point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                if point_time < 0 or point_time < previous_time:
                    raise ValueError("trajectory times must be ordered")
                previous_time = point_time
        except (IndexError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        names = goal_handle.request.trajectory.joint_names
        points = [
            (point.time_from_start.sec + point.time_from_start.nanosec / 1e9,
             dict(zip(names, point.positions)))
            for point in goal_handle.request.trajectory.points
        ]
        trajectory = {"start": time.monotonic(), "points": points, "done": threading.Event()}
        with self.trajectory_lock:
            if self.trajectory:
                self.trajectory["outcome"] = "preempted"
                self.trajectory["done"].set()
            trajectory["start_positions"] = {name: self.arm_positions[name] for name in names}
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

    def validate_motion_parameters(self):
        """Reject values that would make a command unsafe or undefined."""
        positive = {
            "xy_velocity_scale": self.xy_scale,
            "yaw_velocity_scale": self.yaw_scale,
            "command_timeout": self.command_timeout,
            "link_timeout": self.link_timeout,
            "trajectory_tolerance": self.trajectory_tolerance,
            "trajectory_timeout": self.trajectory_timeout,
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

    def observation_is_fresh(self, observation):
        # LeRobot normally replaces its observation dict for every packet, but some
        # client versions update one cached dict in place. Identity alone therefore
        # turns a healthy, moving robot into a false link-loss. The numeric telemetry
        # and frame identity detect that in-place update while preserving the cheap
        # identity check for the usual case.
        numeric = tuple(sorted(
            (name, float(value))
            for name, value in observation.items()
            if name.endswith((".pos", ".vel"))
        ))
        token = (id(observation), numeric, id(observation.get("front")))
        fresh = token != getattr(self, "last_observation_token", None)
        self.last_observation_token = token
        self.last_observation = observation
        return fresh

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
        if self.link_lost:
            self.link_lost = False
            self.publish_safety("DISARMED")
            self.get_logger().warn("LeKiwi telemetry recovered; inspect robot, then call safety/arm")
        if "front" in observation:
            self.publish_front_camera(now.to_msg(), observation)
        if "wrist" in observation:
            self.publish_wrist_camera(now.to_msg(), observation)
        arm_positions = joint_positions(
            observation, self.arm_zero_positions, self.arm_directions
        )
        with self.trajectory_lock:
            self.arm_positions = arm_positions

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
        action.update(
            {
                "x.vel": self.clamp(cmd.linear.x, self.max_linear) / self.xy_scale,
                "y.vel": self.clamp(cmd.linear.y, self.max_linear) / self.xy_scale,
                "theta.vel": math.degrees(self.clamp(cmd.angular.z, self.max_angular) / self.yaw_scale),
            }
        )
        try:
            self.robot.send_action(action)
        except Exception as error:
            self.get_logger().error(f"LeKiwi command failed: {error}")
            self.link_lost = True
            self.set_disarmed("LINK_LOST")
            return

        velocity = (
            float(observation.get("x.vel", 0.0)) * self.xy_scale,
            float(observation.get("y.vel", 0.0)) * self.xy_scale,
            math.radians(float(observation.get("theta.vel", 0.0))) * self.yaw_scale,
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
