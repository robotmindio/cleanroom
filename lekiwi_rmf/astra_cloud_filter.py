#!/usr/bin/env python3
"""Publish a compact, valid Astra cloud without sending its raw raster over Wi-Fi."""

from __future__ import annotations

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def compact_cloud(message: PointCloud2, stride: int) -> PointCloud2 | None:
    """Decimate finite XYZ samples while preserving the driver's calibrated fields."""
    if stride < 1 or message.point_step <= 0 or message.row_step < message.width * message.point_step:
        return None
    offsets = {field.name: field.offset for field in message.fields if field.datatype == PointField.FLOAT32}
    if not all(name in offsets for name in ("x", "y", "z")):
        return None
    required = message.height * message.row_step
    if len(message.data) < required:
        return None
    unpack = struct.Struct((">" if message.is_bigendian else "<") + "f")
    output = bytearray()
    for row in range(0, message.height, stride):
        for column in range(0, message.width, stride):
            start = row * message.row_step + column * message.point_step
            xyz = tuple(unpack.unpack_from(message.data, start + offsets[name])[0] for name in ("x", "y", "z"))
            if not all(math.isfinite(value) for value in xyz):
                continue
            output.extend(message.data[start:start + message.point_step])
    if not output:
        return None
    cloud = PointCloud2()
    cloud.header = message.header
    cloud.height = 1
    cloud.width = len(output) // message.point_step
    cloud.fields = message.fields
    cloud.is_bigendian = message.is_bigendian
    cloud.point_step = message.point_step
    cloud.row_step = len(output)
    cloud.data = bytes(output)
    cloud.is_dense = True
    return cloud


class AstraCloudFilter(Node):
    def __init__(self) -> None:
        super().__init__("astra_cloud_filter")
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("max_rate_hz", 5.0)
        self._stride = int(self.get_parameter("pixel_stride").value)
        rate = float(self.get_parameter("max_rate_hz").value)
        if self._stride < 1 or not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("pixel_stride must be positive and max_rate_hz must be finite and positive")
        self._period = 1.0 / rate
        self._last_publish = 0.0
        self._publisher = self.create_publisher(PointCloud2, "/camera/depth/points", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/camera/depth/points_raw", self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, message: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_publish < self._period:
            return
        cloud = compact_cloud(message, self._stride)
        if cloud is None:
            self.get_logger().warning("discarding Astra cloud without finite XYZ data")
            return
        self._last_publish = now
        self._publisher.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstraCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
