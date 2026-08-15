#!/usr/bin/env python3
import math

import rclpy
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import TransformBroadcaster

from lekiwi_rmf.odometry import integrate_pose


ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)


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
        initial_x = self.declare_parameter("initial_x", -4.0).value
        initial_y = self.declare_parameter("initial_y", -2.5).value
        initial_yaw = self.declare_parameter("initial_yaw", 0.0).value
        self.publish_camera = self.declare_parameter("publish_camera", False).value

        self.robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=remote_ip, id=robot_id))
        self.robot.connect()
        self.command = Twist()
        self.command_stamp = self.get_clock().now()
        self.pose = (initial_x, initial_y, initial_yaw)
        self.last_update = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        if self.publish_camera:
            camera_info_url = self.declare_parameter("camera_info_url", "").value
            self.camera_info = CameraInfoManager(
                self, cname="lekiwi_front", url=camera_info_url, namespace="camera/front"
            )
            self.camera_info.loadCameraInfo()
            if self.declare_parameter("require_camera_calibration", False).value and not self.camera_info.isCalibrated():
                self.robot.disconnect()
                raise RuntimeError(
                    "Visual SLAM requires a calibrated front camera; set camera_info_url"
                )
            self.image_pub = self.create_publisher(
                Image, "camera/front/image_raw", qos_profile_sensor_data
            )
            self.camera_info_pub = self.create_publisher(
                CameraInfo, "camera/front/camera_info", qos_profile_sensor_data
            )
            self.cv_bridge = CvBridge()
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_command, 10)
        self.create_timer(0.05, self.update)
        self.get_logger().info(f"Connected to LeKiwi host at {remote_ip}")

    def on_command(self, message):
        self.command = message
        self.command_stamp = self.get_clock().now()

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        observation = self.robot.get_observation()

        stale = (now - self.command_stamp).nanoseconds / 1e9 > self.command_timeout
        cmd = Twist() if stale else self.command
        action = {
            f"{joint}.pos": float(observation.get(f"{joint}.pos", 0.0)) for joint in ARM_JOINTS
        }
        action.update(
            {
                "x.vel": self.clamp(cmd.linear.x, self.max_linear),
                "y.vel": self.clamp(cmd.linear.y, self.max_linear),
                "theta.vel": math.degrees(self.clamp(cmd.angular.z, self.max_angular)),
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
        if self.publish_camera and observation.get("front") is not None:
            self.publish_front_camera(now.to_msg(), observation["front"])

    def publish_front_camera(self, stamp, frame):
        image = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image.header.stamp = stamp
        image.header.frame_id = "front_camera_optical_frame"
        camera_info = self.camera_info.getCameraInfo()
        camera_info.header = image.header
        self.image_pub.publish(image)
        self.camera_info_pub.publish(camera_info)

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
        joints.position = [
            float(observation.get(f"{name}.pos", 0.0)) / 100.0 * math.pi / 2
            if name == "arm_gripper"
            else math.radians(float(observation.get(f"{name}.pos", 0.0)))
            for name in ARM_JOINTS
        ]
        self.joint_pub.publish(joints)

    def destroy_node(self):
        self.robot.disconnect()
        return super().destroy_node()


def main():
    rclpy.init()
    node = LeKiwiDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
