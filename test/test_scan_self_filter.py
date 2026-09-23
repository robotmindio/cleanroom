"""The body mask hides only the robot's own returns and keeps the scan otherwise intact."""

import math
import pathlib

import pytest
import yaml
from sensor_msgs.msg import LaserScan

from lekiwi_rmf.scan_self_filter import blank_body_returns

ROOT = pathlib.Path(__file__).parents[1]


def _scan(ranges, angle_min=0.0, increment=math.radians(30)):
    scan = LaserScan(angle_min=angle_min, angle_increment=increment, range_min=0.02, range_max=12.0)
    scan.angle_max = angle_min + increment * (len(ranges) - 1)
    scan.ranges = list(ranges)
    return scan


def test_close_returns_in_the_sector_become_no_return_and_the_rest_is_untouched():
    # Beams every 30 degrees; the sector is 90..180 degrees.
    scan = _scan([0.1] * 12)
    scan.ranges[4] = 0.5      # 120 deg but beyond the body: a real obstacle
    scan.ranges[5] = math.inf
    out = blank_body_returns(scan, 90.0, 180.0, 0.2)
    expected = [0.1, 0.1, 0.1, math.inf, 0.5, math.inf, math.inf] + [0.1] * 5
    assert list(out.ranges) == pytest.approx(expected)
    assert (out.angle_min, out.angle_increment, out.range_min, out.range_max) == (
        scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
    assert scan.ranges[3] == pytest.approx(0.1), "the input message must not be modified"


def test_sector_may_wrap_past_zero_degrees():
    out = blank_body_returns(_scan([0.1] * 12), 295.0, 35.0, 0.2)
    assert [i for i, r in enumerate(out.ranges) if math.isinf(r)] == [0, 1, 10, 11]


def test_zero_range_disables_the_filter():
    scan = _scan([0.1] * 12)
    assert list(blank_body_returns(scan, 0.0, 359.0, 0.0).ranges) == list(scan.ranges)


def test_the_scan_still_passes_the_supervisors_validity_check():
    from lekiwi_rmf.safety_supervisor import _valid_scan_ranges

    assert _valid_scan_ranges(blank_body_returns(_scan([0.1, 2.0] * 6), 0.0, 90.0, 0.2), 0.05)


def test_the_tracked_mask_covers_the_measured_body_returns_and_nothing_far():
    node = yaml.safe_load((ROOT / "config" / "lidar_self_mask.yaml").read_text())["scan_self_filter"]["ros__parameters"]
    # Measured on the stationary robot: 259-306 deg at up to 0.18 m, edge returns to 340 deg.
    assert node["body_start_deg"] <= 259.0 and node["body_end_deg"] >= 340.0
    assert 0.18 < node["body_max_range_m"] <= 0.25


@pytest.mark.parametrize("value", ["-0.1", "1.5", ".nan"])
def test_the_node_refuses_a_range_that_could_hide_more_than_the_robot(value):
    import rclpy

    from lekiwi_rmf.scan_self_filter import ScanSelfFilter

    rclpy.init(args=["--ros-args", "-p", f"body_max_range_m:={value}"])
    try:
        with pytest.raises(ValueError):
            ScanSelfFilter()
    finally:
        rclpy.shutdown()


def test_several_sectors_each_blank_only_within_their_own_reach():
    from lekiwi_rmf.scan_self_filter import blank_body_sectors

    scan = _scan([0.1] * 12)
    scan.ranges[1] = 0.25     # 30 deg: beyond the first sector's 0.2 m reach
    scan.ranges[8] = 0.25     # 240 deg: inside the second sector's 0.3 m reach
    out = blank_body_sectors(scan, [(0.0, 60.0, 0.2), (210.0, 270.0, 0.3)])
    assert [i for i, r in enumerate(out.ranges) if math.isinf(r)] == [0, 2, 7, 8, 9]
    assert out.ranges[1] == pytest.approx(0.25)


def test_one_number_or_matching_lists_describe_the_sectors():
    from lekiwi_rmf.scan_self_filter import parse_sectors

    assert parse_sectors(255.0, 345.0, 0.2) == [(255.0, 345.0, 0.2)]
    assert parse_sectors([255.0, 85.0], [345.0, 150.0], [0.2, 0.25]) == [
        (255.0, 345.0, 0.2), (85.0, 150.0, 0.25)]
    with pytest.raises(ValueError, match="one entry per sector"):
        parse_sectors([255.0, 85.0], [345.0], [0.2, 0.25])
    with pytest.raises(ValueError, match="at most"):
        parse_sectors([0.0, 120.0], [100.0, 230.0], [0.2, 0.2])


def test_the_node_accepts_a_list_of_sectors():
    import rclpy

    from lekiwi_rmf.scan_self_filter import ScanSelfFilter

    rclpy.init(args=["--ros-args",
                     "-p", "body_start_deg:=[255.0, 85.0]",
                     "-p", "body_end_deg:=[345.0, 150.0]",
                     "-p", "body_max_range_m:=[0.2, 0.25]"])
    try:
        node = ScanSelfFilter()
        assert node.sectors == [(255.0, 345.0, 0.2), (85.0, 150.0, 0.25)]
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_proposed_sectors_cover_steady_body_returns_and_ignore_far_or_rare_ones():
    from lekiwi_rmf.scan_self_filter import parse_sectors, propose_sectors

    increment = math.radians(1.0)
    scans = []
    for index in range(100):
        ranges = [5.0] * 360
        for degree in range(259, 307):           # a steady body part
            ranges[degree] = 0.15
        for degree in (355, 356, 357, 358, 359, 0, 1, 2):  # another, across 0 deg
            ranges[degree] = 0.08
        if index == 0:
            ranges[100] = 0.10                    # one stray return: ignored
        scans.append((0.0, increment, ranges))

    sectors = propose_sectors(scans)
    assert sectors == [(254.0, 312.0, 0.17), (350.0, 8.0, 0.1)]
    parse_sectors(*zip(*sectors))  # the proposal is a valid filter configuration
