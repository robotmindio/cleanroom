#!/usr/bin/env python3
"""Expand compressed camera frames from the device machine into canonical topics.

When the cameras live on another machine, pi_cameras.launch.py reads them there
and publishes compressed frames under /pi/camera/...; only that crosses the
network. This node re-creates what a local v4l2_camera would have published --
raw images plus CameraInfo on the /camera/... topics -- so nothing downstream
can tell the topologies apart. Frames keep their original stamps: RTAB-Map
syncs approximately, which ordinary NTP-synced clocks comfortably satisfy.
"""
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraRelay(Node):
    # (camera name, has calibration): the wrist camera carries no calibration,
    # and inventing one would be worse than publishing none.
    CAMERAS = [("front", True), ("wrist", False)]

    def __init__(self):
        super().__init__("camera_relay")
        self.bridge = CvBridge()
        self.last_info = {}
        for name, with_info in self.CAMERAS:
            self.create_subscription(
                CompressedImage, f"/pi/camera/{name}/image_raw/compressed",
                self.make_image_callback(name, with_info), qos_profile_sensor_data)
            if with_info:
                self.create_subscription(
                    CameraInfo, f"/pi/camera/{name}/camera_info",
                    self.make_info_callback(name), qos_profile_sensor_data)

    def make_image_callback(self, name, with_info):
        pub = self.create_publisher(Image, f"/camera/{name}/image_raw", qos_profile_sensor_data)
        info_pub = self.create_publisher(CameraInfo, f"/camera/{name}/camera_info", 10) if with_info else None

        def on_image(msg):
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            except Exception as error:  # a truncated JPEG is data damage, not fatal
                self.get_logger().warn(f"{name}: undecodable frame: {error}")
                return
            image.header = msg.header
            pub.publish(image)
            if info_pub is not None:
                # The info message from the sensor node already carries the right
                # stamp; republishing it here keeps image/info pairs together.
                if self.last_info.get(name) is not None:
                    info = self.last_info[name]
                    info.header = msg.header
                    info_pub.publish(info)

        return on_image

    def make_info_callback(self, name):
        def on_info(msg):
            self.last_info[name] = msg

        return on_info


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
