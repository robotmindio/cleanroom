#!/usr/bin/env python3
"""Merge whichever range sensors are alive into one cloud for RTAB-Map.

RTAB-Map synchronises every topic it subscribes to, so feeding it the lidar
and the Astra separately would stall SLAM whenever either one dropped off USB.
This node publishes one base_footprint cloud per lidar scan with the latest
Astra points added, and the Astra points alone while no scans arrive. SLAM
therefore stops only when both sensors are gone, and the map gains obstacles
the lidar plane misses: table tops, chair seats, low clutter.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


BASE_FRAME = "base_footprint"
ODOM_FRAME = "odom"
# An Astra cloud older than this relative to the scan it joins is dropped
# rather than smeared across the map.
ASTRA_MAX_AGE_S = 1.0
# With no scan for this long the lidar is treated as gone and Astra clouds are
# published on their own.
SCAN_TIMEOUT_S = 0.5
# Astra points that can block the base: above floor noise, below the height
# where nothing the robot drives under matters.
MIN_HEIGHT_M = 0.03
MAX_HEIGHT_M = 1.0
TF_TIMEOUT = Duration(seconds=0.05)


def scan_points(scan: LaserScan) -> np.ndarray:
    """Valid scan returns as Nx3 points in the scan frame."""
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    ranges, angles = ranges[valid], angles[valid]
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)))


def transform_points(points: np.ndarray, transform: TransformStamped) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return points @ rotation.T + np.array([t.x, t.y, t.z])


def obstacle_band(points: np.ndarray) -> np.ndarray:
    """Keep base-frame points between the floor and the obstacle ceiling."""
    heights = points[:, 2]
    return points[(heights >= MIN_HEIGHT_M) & (heights <= MAX_HEIGHT_M)]


def cloud_points(cloud: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
    return np.asarray(points, dtype=np.float64).reshape(-1, 3)


class SlamCloud(Node):
    def __init__(self) -> None:
        super().__init__("slam_cloud")
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._astra: Optional[PointCloud2] = None
        self._last_scan: Optional[Time] = None
        self._publisher = self.create_publisher(PointCloud2, "/slam/cloud", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/camera/depth/points", self._on_astra, qos_profile_sensor_data)

    def _astra_at(self, stamp: Time) -> np.ndarray:
        """Latest Astra points moved to where the base was at ``stamp``."""
        astra = self._astra
        if astra is None:
            return np.empty((0, 3))
        astra_time = Time.from_msg(astra.header.stamp)
        if abs((stamp - astra_time).nanoseconds) > ASTRA_MAX_AGE_S * 1e9:
            return np.empty((0, 3))
        try:
            # Through odom, so robot motion between the two captures is undone.
            transform = self._tf.lookup_transform_full(
                BASE_FRAME, stamp, astra.header.frame_id, astra_time, ODOM_FRAME, TF_TIMEOUT)
        except TransformException as error:
            self.get_logger().warning(f"skipping Astra cloud: {error}", throttle_duration_sec=10.0)
            return np.empty((0, 3))
        return obstacle_band(transform_points(cloud_points(astra), transform))

    def _publish(self, stamp: Time, points: np.ndarray) -> None:
        if len(points) == 0:
            return
        header = Header(stamp=stamp.to_msg(), frame_id=BASE_FRAME)
        self._publisher.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32)))

    def _on_scan(self, scan: LaserScan) -> None:
        stamp = Time.from_msg(scan.header.stamp)
        self._last_scan = self.get_clock().now()
        try:
            # The lidar is fixed to the base, so its latest mount transform is exact.
            mount = self._tf.lookup_transform(BASE_FRAME, scan.header.frame_id, Time(), TF_TIMEOUT)
        except TransformException as error:
            self.get_logger().warning(f"skipping scan: {error}", throttle_duration_sec=10.0)
            return
        lidar = transform_points(scan_points(scan), mount)
        self._publish(stamp, np.vstack((lidar, self._astra_at(stamp))))

    def _on_astra(self, cloud: PointCloud2) -> None:
        self._astra = cloud
        now = self.get_clock().now()
        if self._last_scan is None or (now - self._last_scan).nanoseconds > SCAN_TIMEOUT_S * 1e9:
            stamp = Time.from_msg(cloud.header.stamp)
            self._publish(stamp, self._astra_at(stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
