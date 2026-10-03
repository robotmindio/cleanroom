#!/usr/bin/env python3
"""Publish compact Astra points and rate-limited RGB for the device bridge."""

from __future__ import annotations

import math
import time

import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField


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
        self.declare_parameter("image_max_rate_hz", 2.0)
        self._stride = int(self.get_parameter("pixel_stride").value)
        rate = float(self.get_parameter("max_rate_hz").value)
        image_rate = float(self.get_parameter("image_max_rate_hz").value)
        if self._stride < 1 or not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("pixel_stride must be positive and max_rate_hz must be finite and positive")
        if not math.isfinite(image_rate) or image_rate <= 0:
            raise ValueError("image_max_rate_hz must be finite and positive")
        self._period = 1.0 / rate
        self._last_publish = 0.0
        self._publisher = self.create_publisher(PointCloud2, "/camera/depth/points", qos_profile_sensor_data)
        # QVGA clouds are about 1.2 MB at 30 Hz; decode only published frames.
        self.create_subscription(
            PointCloud2, "/camera/depth/points_raw", self._on_cloud, qos_profile_sensor_data,
            raw=True,
        )
        self._bridge = CvBridge()
        self._image_period = 1.0 / image_rate
        self._last_image_publish = {"color": 0.0, "depth": 0.0}
        # image_transport's lazy republisher does not remain subscribed for the
        # bridge's native DDS reader. Subscribe eagerly, encode only sent frames.
        for camera, extension, encoding in [("color", ".jpg", "jpeg"), ("depth", ".png", "16UC1; png compressed")]:
            publisher = self.create_publisher(CompressedImage, f"/camera/astra/{camera}/image_raw/compressed", 1)
            self.create_subscription(Image, f"/camera/astra/{camera}/image_raw",
                lambda msg,c=camera,e=extension,f=encoding,p=publisher:self._on_image(msg,c,e,f,p),
                # UVC colour publishes reliably; its 0.9 MB raw frame needs
                # fragment recovery while this process also receives clouds.
                1 if camera == "color" else qos_profile_sensor_data, raw=True)

    def _on_image(self, serialized: bytes, camera: str, extension: str, encoding: str, publisher) -> None:
        now = time.monotonic()
        if now - self._last_image_publish[camera] < self._image_period:
            return
        try:
            image = deserialize_message(serialized, Image)
            if camera == "depth" and image.encoding != "16UC1":
                raise ValueError(f"expected depth in millimetres as 16UC1, got {image.encoding}")
            desired = "passthrough" if camera == "depth" else "bgr8"
            ok, compressed = cv2.imencode(extension, self._bridge.imgmsg_to_cv2(image, desired))
            if not ok:
                raise RuntimeError(f"{extension} encoding failed")
        except Exception as error:
            self.get_logger().warning(f"discarding invalid Astra {camera} frame: {error}", throttle_duration_sec=5)
            return
        self._last_image_publish[camera] = now
        publisher.publish(CompressedImage(header=image.header, format=encoding, data=compressed.tobytes()))

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
