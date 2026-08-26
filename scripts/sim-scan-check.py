#!/usr/bin/env python3
"""Wait for a simulated lidar scan and reject the known all-minimum failure."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class ScanAssessment:
    usable: bool
    message: str


def assess_scan(ranges: list[float], range_min: float) -> ScanAssessment:
    """Require an actual 360-degree-like scan with space beyond its blind zone."""
    if len(ranges) < 180:
        return ScanAssessment(False, f"only {len(ranges)} ranges received (expected at least 180)")

    beyond_minimum = [
        value for value in ranges
        if math.isinf(value) and value > 0 or math.isfinite(value) and value > range_min + 0.001
    ]
    if not beyond_minimum:
        return ScanAssessment(
            False,
            f"all {len(ranges)} ranges are at or below range_min={range_min:.3f} m; renderer/lidar is unusable",
        )

    finite = [value for value in ranges if math.isfinite(value)]
    maximum = max(finite) if finite else math.inf
    return ScanAssessment(
        True,
        f"usable scan: {len(ranges)} ranges, {len(beyond_minimum)} beyond range_min, max={maximum:.3g} m",
    )


class ScanCheck(Node):
    def __init__(self) -> None:
        super().__init__("sim_scan_check")
        self.assessment: ScanAssessment | None = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

    def _on_scan(self, message: LaserScan) -> None:
        self.assessment = assess_scan(list(message.ranges), message.range_min)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0, help="seconds to wait for /scan (default: 20)")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    rclpy.init()
    node = ScanCheck()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.assessment is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
        if node.assessment is None:
            print(f"simulation scan check failed: no /scan received within {args.timeout:g} s")
            return 1
        print(node.assessment.message)
        return 0 if node.assessment.usable else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
