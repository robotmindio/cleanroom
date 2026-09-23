#!/usr/bin/env python3
"""Measure the LD06 returns from the robot's own body and propose lidar_self_mask.yaml.

Run on the stationary, stowed robot with nothing else within --reach of the lidar
(it cannot tell a nearby wall from the robot). Reads the raw, unfiltered scan and
never commands motion. Review the proposal, then copy it into
config/lidar_self_mask.yaml together with a note of when and how it was measured.
"""

import argparse
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from lekiwi_rmf.scan_self_filter import parse_sectors, propose_sectors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/pi/lidar/scan", help="raw scan before scan_self_filter")
    parser.add_argument("--scans", type=int, default=200)
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for the scans")
    parser.add_argument("--reach", type=float, default=0.30, help="nearest non-robot object, in metres")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("lidar_self_mask")
    scans = []
    node.create_subscription(
        LaserScan, args.topic,
        lambda scan: scans.append((scan.angle_min, scan.angle_increment, list(scan.ranges))),
        qos_profile_sensor_data,
    )
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while len(scans) < args.scans and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.try_shutdown()
    if len(scans) < args.scans:
        sys.exit(f"received {len(scans)} of {args.scans} scans on {args.topic} in {args.timeout:.0f} s")

    sectors = propose_sectors(scans[:args.scans], reach_limit=args.reach)
    if not sectors:
        print(f"no returns closer than {args.reach} m: body_max_range_m: 0.0 passes the scan through")
        return
    try:
        parse_sectors(*zip(*sectors))
    except ValueError as error:
        sys.exit(f"{error}: something other than the robot is within {args.reach} m; clear it or lower --reach")
    starts, ends, reaches = (list(column) for column in zip(*sectors))
    blanked = sum((end - start) % 360.0 for start, end, _ in sectors)
    print(f"# {args.scans} scans on {args.topic}; blanks {blanked:.0f} deg of the scan")
    print("scan_self_filter:\n  ros__parameters:")
    print(f"    body_start_deg: {starts}\n    body_end_deg: {ends}\n    body_max_range_m: {reaches}")


if __name__ == "__main__":
    main()
