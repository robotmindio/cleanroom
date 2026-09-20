import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.qos import ReliabilityPolicy
from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from lekiwi_rmf.slam_cloud import (
    CLOUD_QOS, clamp_to_newest, obstacle_band, scan_points, transform_points,
)


def test_scan_points_keep_only_valid_returns():
    scan = LaserScan(angle_min=0.0, angle_increment=math.pi / 2, range_min=0.05, range_max=5.0)
    scan.ranges = [1.0, float("inf"), 0.01, 2.0]
    points = scan_points(scan)
    np.testing.assert_allclose(points, [[1.0, 0.0, 0.0], [0.0, -2.0, 0.0]], atol=1e-9)


def test_transform_points_rotates_then_translates():
    transform = TransformStamped()
    # 90 degrees about z, then 1 m up.
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    transform.transform.translation.z = 1.0
    np.testing.assert_allclose(
        transform_points(np.array([[1.0, 0.0, 0.0]]), transform), [[0.0, 1.0, 1.0]], atol=1e-9)


def test_obstacle_band_drops_floor_and_overhead_points():
    points = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.5], [1.0, 0.0, 1.5]])
    np.testing.assert_allclose(obstacle_band(points), [[1.0, 0.0, 0.5]])


def test_cloud_reaches_rtabmaps_reliable_subscription():
    assert CLOUD_QOS.reliability == ReliabilityPolicy.RELIABLE


def test_cloud_slightly_newer_than_odometry_uses_the_newest_transform():
    # tf2 hands back its newest time on the system clock, stamps are ROS time.
    newest = Time(seconds=10.000, clock_type=ClockType.SYSTEM_TIME)
    assert clamp_to_newest(Time(seconds=9.5), newest) == Time(seconds=9.5)
    assert clamp_to_newest(Time(seconds=10.001), newest) == Time(seconds=10.000)
    assert clamp_to_newest(Time(seconds=10.19), newest) == Time(seconds=10.000)
    # Odometry that stopped is not papered over.
    assert clamp_to_newest(Time(seconds=10.25), newest) is None
