#!/usr/bin/env python3
"""Publish /scan with the robot's own body blanked out of the LD06 scan.

Parts of the robot's structure sit within about 20 cm of the lidar in fixed
sectors, so every scan contains returns from them. Those points fall inside the collision
monitor's stop polygon (which stops the base on a single point) and are drawn into
the costmaps and the SLAM cloud as if they were obstacles. Returns inside the
configured sectors that are also closer than that sector's reach become "no return"
(+inf); anything farther away in a sector, and everything outside them, is
untouched. Angles are in the laser frame, counter-clockwise from its +x axis.
Each parameter is one number (one sector) or a list with one entry per sector.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
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
    return blank_body_sectors(scan, [(start_deg, end_deg, max_range)])


def blank_body_sectors(scan: LaserScan, sectors) -> LaserScan:
    """blank_body_returns for several (start_deg, end_deg, max_range) sectors."""
    ranges = np.array(scan.ranges, dtype=float)
    # The LD06 uses NaN for no return; publish the ROS no-return value so
    # downstream consumers can distinguish it from a malformed range.
    ranges[np.isnan(ranges)] = math.inf
    angles = np.degrees(scan.angle_min + np.arange(len(ranges)) * scan.angle_increment)
    finite = np.isfinite(ranges)
    for start_deg, end_deg, max_range in sectors:
        if max_range == 0.0:
            continue
        in_sector = (angles - start_deg) % 360.0 <= (end_deg - start_deg) % 360.0
        body_returns = in_sector & (
            (finite & (ranges < max_range)) | np.isneginf(ranges)
        )
        ranges[body_returns] = math.inf
    filtered = LaserScan()
    filtered.header = scan.header
    for field in (
        "angle_min", "angle_max", "angle_increment", "time_increment",
        "scan_time", "range_min", "range_max", "intensities",
    ):
        setattr(filtered, field, getattr(scan, field))
    filtered.ranges = ranges.tolist()
    return filtered


def propose_sectors(scans, reach_limit=0.30, min_fraction=0.02, gap_deg=5, margin_deg=5.0,
                    range_margin=0.02):
    """Sectors that contain the stationary robot's own returns, from raw scans.

    ``scans`` are (angle_min_rad, angle_increment_rad, ranges) of a stationary robot
    with nothing else within ``reach_limit``. A 1-degree bin belongs to the body when
    at least ``min_fraction`` of the scans return from it closer than that limit;
    bins no more than ``gap_deg`` apart join one sector, which is padded by
    ``margin_deg`` and reaches ``range_margin`` past its farthest return.
    Returns [(start_deg, end_deg, reach_m)] sorted by start angle.
    """
    hits = np.zeros(360)
    farthest = np.zeros(360)
    for angle_min, increment, ranges in scans:
        ranges = np.asarray(ranges, dtype=float)
        bins = np.floor(np.degrees(angle_min + np.arange(len(ranges)) * increment)).astype(int) % 360
        close = np.isfinite(ranges) & (ranges > 0.0) & (ranges < reach_limit)
        seen = np.zeros(360, dtype=bool)
        seen[bins[close]] = True
        hits += seen
        np.maximum.at(farthest, bins[close], ranges[close])
    body = np.flatnonzero(hits >= max(1, min_fraction * len(scans)))
    if not len(body):
        return []
    # Walk the circle starting just after the widest gap, so no sector is split at 0.
    gaps = np.diff(np.append(body, body[0] + 360))
    start = (int(np.argmax(gaps)) + 1) % len(body)
    ordered = np.roll(body, -start)
    groups, current = [], [int(ordered[0])]
    for previous, degree in zip(ordered, ordered[1:]):
        if (degree - previous) % 360 <= gap_deg + 1:
            current.append(int(degree))
        else:
            groups.append(current)
            current = [int(degree)]
    groups.append(current)
    sectors = []
    for group in groups:
        reach = min(float(farthest[group].max()) + range_margin, MAX_REACH_M)
        sectors.append((
            (group[0] - margin_deg) % 360.0,
            (group[-1] + 1 + margin_deg) % 360.0,
            round(reach, 3),
        ))
    return sorted(sectors)


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("scan_self_filter")
        input_topic = self.declare_parameter("input_topic", "/pi/lidar/scan").value
        output_topic = self.declare_parameter("output_topic", "/scan").value
        self.sectors = parse_sectors(*(
            self.declare_parameter(
                name, 0.0, ParameterDescriptor(dynamic_typing=True)
            ).value
            for name in ("body_start_deg", "body_end_deg", "body_max_range_m")
        ))
        # Same delivery contract the LD06 driver publishes with, so every existing
        # /scan subscriber keeps connecting; the input side accepts either.
        self.pub = self.create_publisher(
            LaserScan, output_topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.create_subscription(LaserScan, input_topic, self.on_scan, qos_profile_sensor_data)
        described = ", ".join(
            f"closer than {reach:.2f} m between {start:.0f} and {end:.0f} deg"
            for start, end, reach in self.sectors
        )
        self.get_logger().info(f"{input_topic} -> {output_topic}, blanking returns {described}")

    def on_scan(self, scan: LaserScan) -> None:
        self.pub.publish(blank_body_sectors(scan, self.sectors))


# The robot fits inside this reach, and together the sectors may hide no more of
# the scan than this; anything wider would blind the scan rather than mask the body.
MAX_REACH_M = 1.0
MAX_BLANKED_DEG = 180.0


def parse_sectors(starts, ends, reaches):
    """Validate one sector (numbers) or several (equal-length lists) into tuples."""
    columns = [
        list(value) if isinstance(value, (list, tuple)) else [value]
        for value in (starts, ends, reaches)
    ]
    if len({len(column) for column in columns}) != 1:
        raise ValueError("body_start_deg, body_end_deg and body_max_range_m need one entry per sector")
    sectors = []
    for start, end, reach in zip(*columns):
        start, end, reach = float(start), float(end), float(reach)
        if not all(math.isfinite(v) for v in (start, end, reach)):
            raise ValueError("the body sector angles and range must be finite")
        if not 0.0 <= reach <= MAX_REACH_M:
            raise ValueError("body_max_range_m must be between 0 and 1 m; it only hides the robot itself")
        sectors.append((start, end, reach))
    blanked = sum((end - start) % 360.0 for start, end, reach in sectors if reach > 0.0)
    if blanked > MAX_BLANKED_DEG:
        raise ValueError(f"the body sectors blank {blanked:.0f} deg; at most {MAX_BLANKED_DEG:.0f} is allowed")
    return sectors


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
