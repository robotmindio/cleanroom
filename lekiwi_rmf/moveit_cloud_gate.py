#!/usr/bin/env python3
"""Feed MoveIt depth clouds only when the matching arm transform is available."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


MAX_WAIT_NS = 500_000_000


def ready(cloud, received_ns, now_ns, transforms):
    if now_ns - received_ns > MAX_WAIT_NS:
        return False
    return transforms.can_transform(
        "gripper_collision_proxy", cloud.header.frame_id,
        Time.from_msg(cloud.header.stamp),
    )


class MoveItCloudGate(Node):
    def __init__(self):
        super().__init__("moveit_cloud_gate")
        self._transforms = Buffer()
        self._listener = TransformListener(self._transforms, self)
        self._publisher = self.create_publisher(
            PointCloud2, "/moveit/depth/points_ready", qos_profile_sensor_data
        )
        self._pending = None
        self.create_subscription(
            PointCloud2, "/camera/depth/points", self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_timer(0.01, self._flush)

    def _on_cloud(self, cloud):
        self._pending = (cloud, time.monotonic_ns())

    def _flush(self):
        if self._pending is None:
            return
        cloud, received_ns = self._pending
        now_ns = time.monotonic_ns()
        if ready(cloud, received_ns, now_ns, self._transforms):
            self._publisher.publish(cloud)
            self._pending = None
        elif now_ns - received_ns > MAX_WAIT_NS:
            self._pending = None


def main():
    rclpy.init()
    node = MoveItCloudGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
