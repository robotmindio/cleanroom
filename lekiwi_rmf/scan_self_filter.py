#!/usr/bin/env python3
"""Publish /scan with the robot's own body blanked out of the LD06 scan.

Part of the robot's structure sits within 18 cm of the lidar in a fixed sector, so
every scan contains returns from it. Those points fall inside the collision
monitor's stop polygon (which stops the base on a single point) and are drawn into
the costmaps and the SLAM cloud as if they were obstacles. Returns inside the
configured sector that are also closer than the body's reach become "no return"
(+inf); anything farther away in that sector, and everything outside it, is
untouched. Angles are in the laser frame, counter-clockwise from its +x axis.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def blank_body_returns(
    scan: LaserScan, start_deg: float, end_deg: float, max_range: float
) -> LaserScan:
    """Return a copy of scan with finite returns closer than max_range in the sector removed.

    The sector runs counter-clockwise from start_deg to end_deg and may wrap past 360.
    A max_range of zero disables the filter.
    """
    ranges = np.array(scan.ranges, dtype=float)
    angles = np.degrees(scan.angle_min + np.arange(len(ranges)) * scan.angle_increment)
    in_sector = (angles - start_deg) % 360.0 <= (end_deg - start_deg) % 360.0
    ranges[in_sector & np.isfinite(ranges) & (ranges < max_range)] = math.inf
    filtered = LaserScan()
    filtered.header = scan.header
    for field in (
        "angle_min", "angle_max", "angle_increment", "time_increment",
        "scan_time", "range_min", "range_max", "intensities",
    ):
        setattr(filtered, field, getattr(scan, field))
    filtered.ranges = ranges.tolist()
    return filtered


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("scan_self_filter")
        input_topic = self.declare_parameter("input_topic", "/pi/lidar/scan").value
        output_topic = self.declare_parameter("output_topic", "/scan").value
        self.start_deg = float(self.declare_parameter("body_start_deg", 0.0).value)
        self.end_deg = float(self.declare_parameter("body_end_deg", 0.0).value)
        self.max_range = float(self.declare_parameter("body_max_range_m", 0.0).value)
        if not all(math.isfinite(v) for v in (self.start_deg, self.end_deg, self.max_range)):
            raise ValueError("the body sector angles and range must be finite")
        if not 0.0 <= self.max_range <= 1.0:
            raise ValueError("body_max_range_m must be between 0 and 1 m; it only hides the robot itself")
        # Same delivery contract the LD06 driver publishes with, so every existing
        # /scan subscriber keeps connecting; the input side accepts either.
        self.pub = self.create_publisher(
            LaserScan, output_topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.create_subscription(LaserScan, input_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f"{input_topic} -> {output_topic}, blanking returns closer than {self.max_range:.2f} m "
            f"between {self.start_deg:.0f} and {self.end_deg:.0f} deg"
        )

    def on_scan(self, scan: LaserScan) -> None:
        self.pub.publish(
            blank_body_returns(scan, self.start_deg, self.end_deg, self.max_range)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
