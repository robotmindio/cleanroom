#!/usr/bin/env python3
"""Publish a compact, valid Astra cloud without sending its raw raster over Wi-Fi."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField


def compact_cloud(message: PointCloud2, stride: int) -> PointCloud2 | None:
    """Decimate finite XYZ samples while preserving the driver's calibrated fields."""
    if stride < 1 or message.point_step <= 0 or message.row_step < message.width * message.point_step:
        return None
    offsets = {field.name: field.offset for field in message.fields if field.datatype == PointField.FLOAT32}
    if not all(
        name in offsets and 0 <= offsets[name] and offsets[name] + 4 <= message.point_step
        for name in ("x", "y", "z")
    ):
        return None
    required = message.height * message.row_step
    if len(message.data) < required:
        return None
    raster = np.frombuffer(message.data, dtype=np.uint8, count=required).reshape(
        message.height, message.row_step
    )
    samples = raster[:, :message.width * message.point_step].reshape(
        message.height, message.width, message.point_step
    )[::stride, ::stride].reshape(-1, message.point_step)
    float32 = np.dtype(">f4" if message.is_bigendian else "<f4")
    finite = np.ones(len(samples), dtype=bool)
    for name in ("x", "y", "z"):
        column = np.ascontiguousarray(samples[:, offsets[name]:offsets[name] + 4]).view(float32)
        finite &= np.isfinite(column[:, 0])
    output = samples[finite].tobytes()
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
    cloud.data = output
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
        # The driver publishes 4.9 MB clouds at 30 Hz and about one in six is kept, so
        # take them serialised and only deserialise the ones that are used.
        self.create_subscription(
            PointCloud2, "/camera/depth/points_raw", self._on_cloud, qos_profile_sensor_data,
            raw=True,
        )

    def _on_cloud(self, serialized: bytes) -> None:
        now = time.monotonic()
        if now - self._last_publish < self._period:
            return
        cloud = compact_cloud(deserialize_message(serialized, PointCloud2), self._stride)
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
