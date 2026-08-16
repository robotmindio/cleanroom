#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from camera_info_manager import CameraInfoManager, default_camera_info_url, resolveURL
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from sensor_msgs.srv import SetCameraInfo
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
            self.replace_set_camera_info_service(camera_info_url)
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

    def replace_set_camera_info_service(self, url):
        """Serve camera/front/set_camera_info ourselves.

        CameraInfoManager registers `setCameraInfo(self, req)`, but rclpy calls service
        handlers with (request, response) -- so the callback raises TypeError and kills
        the node the moment a calibrator presses COMMIT. Its saveCalibrationFile also
        opens the target path without creating its directory, which fails just as
        silently. Both are worked around here rather than patched into /opt/ros.
        """
        self.destroy_service(self.camera_info.svc)
        self.camera_info_url = url
        self.camera_info.svc = self.create_service(
            SetCameraInfo, "camera/front/set_camera_info", self.on_set_camera_info
        )

    def on_set_camera_info(self, request, response):
        resolved = resolveURL(self.camera_info_url or default_camera_info_url, self.camera_info.cname)
        if not resolved.startswith("file://"):
            response.status_message = f"cannot write calibration to {resolved}"
            self.get_logger().error(response.status_message)
            return response

        info = request.camera_info
        # Written here rather than through camera_info_manager.saveCalibration: it hands
        # rclpy's numpy arrays straight to yaml.safe_dump, which cannot represent them,
        # and the RepresenterError escapes the service callback and kills the node.
        calibration = {
            "image_width": info.width,
            "image_height": info.height,
            "camera_name": self.camera_info.cname,
            "distortion_model": info.distortion_model,
            "distortion_coefficients": {"rows": 1, "cols": len(info.d), "data": [float(v) for v in info.d]},
            "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in info.k]},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in info.r]},
            "projection_matrix": {"rows": 3, "cols": 4, "data": [float(v) for v in info.p]},
        }
        path = Path(resolved[len("file://"):])
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(calibration, handle, default_flow_style=None, sort_keys=False)
        except OSError as error:
            response.status_message = f"could not store camera calibration at {path}: {error}"
            self.get_logger().error(response.status_message)
            return response

        self.camera_info.camera_info = info
        response.success = True
        self.get_logger().info(f"stored camera calibration at {path}")
        return response

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
