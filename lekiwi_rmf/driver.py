#!/usr/bin/env python3
import math
import os
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
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

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
        self.trajectory_tolerance = self.declare_parameter("trajectory_tolerance", 0.05).value
        self.trajectory_timeout = self.declare_parameter("trajectory_timeout", 5.0).value
        calibration_file = self.declare_parameter(
            "arm_calibration_file", os.path.expanduser("~/.ros/lekiwi_arm_calibration.json")
        ).value
        initial_x = self.declare_parameter("initial_x", -4.0).value
        initial_y = self.declare_parameter("initial_y", -2.5).value
        initial_yaw = self.declare_parameter("initial_yaw", 0.0).value

        self.robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=remote_ip, id=robot_id))
        self.robot.connect()
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
        self.pose = (initial_x, initial_y, initial_yaw)
        self.last_update = self.get_clock().now()
        self.arm_zero_positions, self.arm_directions = load_calibration(calibration_file)
        if not os.path.exists(os.path.expanduser(calibration_file)):
            self.get_logger().warn(
                f"No URDF arm calibration at {calibration_file}; publishing raw LeRobot joint positions"
            )
        self.arm_positions = {name: 0.0 for name in ARM_JOINTS}
        self.trajectory = None
        self.trajectory_lock = threading.Lock()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_command, 10)
        self.trajectory_server = ActionServer(
            self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_trajectory, goal_callback=self.accept_trajectory,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(0.05, self.update)
        self.get_logger().info(f"Connected to LeKiwi host at {remote_ip}")

    def on_command(self, message):
        self.command = message
        self.command_stamp = self.get_clock().now()

    def accept_trajectory(self, goal):
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

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        observation = self.robot.get_observation()
        arm_positions = joint_positions(
            observation, self.arm_zero_positions, self.arm_directions
        )

        stale = (now - self.command_stamp).nanoseconds / 1e9 > self.command_timeout
        cmd = Twist() if stale else self.command
        action = {
            f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS
        }
        with self.trajectory_lock:
            self.arm_positions = arm_positions
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
        self.robot.send_action(action)

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
