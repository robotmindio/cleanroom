"""Position estimates for image detections: each names its method, uncertainty and assumptions."""

import math

import numpy as np
import pytest

from lekiwi_rmf.coverage_search import Pose, camera_from_info, optical_axes
from lekiwi_rmf.object_estimates import (
    Wedge, bearing_only, bearing_wedge, estimates, ray_octomap_hit, rgbd_surface, triangulated,
)
from lekiwi_rmf.search_octomap import KEY_OFFSET, SearchOctomap
from lekiwi_rmf.spatial_anchors import Transform

WIDTH, HEIGHT = 640, 480
CAMERA = camera_from_info([500., 0., 320., 0., 500., 240., 0., 0., 1.], WIDTH, HEIGHT)
RESOLUTION = 0.05
# Mapped free space, in voxel indices: x 0..12.8 m, y -3.2..3.2 m, z 0..0.8 m. Everything else is unknown.
KNOWN = ((0, -64, 0), (256, 64, 16))


def octree(occupied=(), known=KNOWN):
    """A synthetic octomap: ``known`` is free except the ``occupied`` voxel indices; the rest is unknown."""
    occupied = set(occupied)

    def build(lower, size):
        upper = tuple(value + size for value in lower)
        if any(high <= start or low >= end for low, high, start, end in zip(lower, upper, *known)):
            return 0
        if size == 1:
            return 2 if lower in occupied else 1
        inside = all(low >= start and high <= end for low, high, start, end in zip(lower, upper, *known))
        if inside and not any(all(low <= value < high for low, high, value in zip(lower, upper, voxel))
                              for voxel in occupied):
            return 1
        return tuple(build(tuple(lower[axis] + size // 2 * ((index >> axis) & 1) for axis in range(3)), size // 2)
                     for index in range(8))

    return SearchOctomap(RESOLUTION, build((-KEY_OFFSET,) * 3, 2 * KEY_OFFSET))


def voxel(point):
    return tuple(math.floor(value / RESOLUTION) for value in point)


def pixel(pose, point):
    """Where a pinhole camera at ``pose`` images a map point."""
    forward, right, down = (np.asarray(axis) for axis in optical_axes(pose.yaw, pose.pitch, pose.roll))
    vector = np.asarray(point) - (pose.x, pose.y, pose.z)
    depth = vector @ forward
    return 320 + 500 * (vector @ right) / depth, 240 + 500 * (vector @ down) / depth


def box(pose, point, width=40.0, height=60.0):
    u, v = pixel(pose, point)
    return [u - width / 2, v - height / 2, width, height]


def test_first_hit_is_the_first_occupied_voxel_and_never_looks_through_unknown():
    tree = octree([voxel((2.025, 0.025, 0.325))])
    assert tree.first_hit((0.5, 0.025, 0.325), (1.0, 0.0, 0.0), 6.0) == pytest.approx(1.5)
    # Free all the way to the range limit: nothing was hit.
    assert tree.first_hit((0.5, 1.025, 0.325), (1.0, 0.0, 0.0), 6.0) is None
    assert tree.first_hit((0.5, 0.025, 0.325), (1.0, 0.0, 0.0), 1.0) is None
    # Unknown space on the way may hide a nearer surface, whatever lies behind it.
    assert tree.first_hit((0.5, 1.025, 0.325), (0.0, 1.0, 0.0), 6.0) is None
    assert tree.first_hit((0.5, 0.025, 1.0), (1.0, 0.0, 0.0), 6.0) is None
    behind_unknown = octree([voxel((2.025, 0.025, 0.325))], known=((20, -64, 0), (256, 64, 16)))
    assert behind_unknown.first_hit((0.5, 0.025, 0.325), (1.0, 0.0, 0.0), 6.0) is None
    for start, direction, limit in (((0.5, 0.0, 0.3), (2.0, 0.0, 0.0), 6.0), ((0.5, 0.0, math.nan), (1.0, 0.0, 0.0), 6.0),
                                    ((0.5, 0.0, 0.3), (1.0, 0.0, 0.0), 0.0), ((1e6, 0.0, 0.3), (1.0, 0.0, 0.0), 6.0)):
        assert tree.first_hit(start, direction, limit) is None


def test_ray_through_the_box_centre_lands_on_the_first_surface_with_a_stated_uncertainty():
    target = (2.025, -0.375, 0.125)
    tree = octree([voxel(target)])
    pose = Pose(0.5, 0.025, 0.3, 0.0, 0.0)
    estimate = ray_octomap_hit(box(pose, target), CAMERA, HEIGHT, pose, tree, 6.0)
    assert estimate['method'] == 'ray_octomap_hit' and estimate['qualified'] is False
    assert estimate['calibration_sha256'] is None
    assert any('first surface' in assumption for assumption in estimate['assumptions'])
    assert math.dist(estimate['map_xyz'], target) < RESOLUTION
    distance = math.dist(estimate['map_xyz'], (pose.x, pose.y, pose.z))
    # Voxel size in depth, half the box's larger angular size across.
    assert estimate['sigma_m'] == pytest.approx(math.hypot(RESOLUTION, distance * 30 / 500))
    # A larger box is a less certain centre.
    wide = ray_octomap_hit(box(pose, target, 200.0, 60.0), CAMERA, HEIGHT, pose, tree, 6.0)
    assert wide['sigma_m'] > estimate['sigma_m']
    # No surface on the ray, or unknown space first: no point is invented.
    assert ray_octomap_hit(box(pose, (2.025, 0.5, 0.125)), CAMERA, HEIGHT, pose, tree, 6.0) is None
    assert ray_octomap_hit(box(pose, target), CAMERA, HEIGHT, pose, octree([voxel(target)], ((30, -64, 0), (256, 64, 16))),
                           6.0) is None


def test_bearing_wedge_spans_the_box_and_holds_under_camera_tilt():
    level = Pose(1.0, 2.0, 0.3, 0.5, 0.0)
    wedge = bearing_wedge([300.0, 200.0, 40.0, 60.0], CAMERA, HEIGHT, level, 0.6)
    # A level camera's bearings are the horizontal pixel interval, as coverage_search reads them.
    assert (wedge.x, wedge.y) == (1.0, 2.0)
    assert wedge.low == pytest.approx(0.5 - math.atan(20 / 500)) and wedge.high == pytest.approx(0.5 + math.atan(20 / 500))
    target = (3.0, 2.6, 0.1)
    for tilted in (Pose(1.0, 2.0, 0.6, 0.3, -0.3), Pose(1.0, 2.0, 0.6, 0.3, -0.2, 0.25)):
        wedge = bearing_wedge(box(tilted, target), CAMERA, HEIGHT, tilted, 0.6)
        assert wedge.contains(target) and wedge.half_width < 0.2
        assert not wedge.contains((3.0, 1.2, 0.1)) and wedge.contains((3.0, 1.2, 0.1), margin_m=1.5)
    # Beyond the tilt limit an image box says too little about a horizontal bearing.
    assert bearing_wedge(box(level, target), CAMERA, HEIGHT, Pose(1.0, 2.0, 0.6, 0.3, -0.7), 0.6) is None
    assert bearing_wedge(box(level, target), CAMERA, HEIGHT, Pose(1.0, 2.0, 0.6, 0.3, 0.0, 0.7), 0.6) is None
    assert bearing_only(wedge)['map_xyz'] is None and bearing_only(wedge)['range_m'] == [0.0, None]


def wedge_to(origin, target, half_width=0.03):
    bearing = math.atan2(target[1] - origin[1], target[0] - origin[0])
    return Wedge(origin[0], origin[1], bearing - half_width, bearing + half_width)


def test_triangulation_needs_a_baseline_parallax_and_a_consistent_crossing():
    limits = {'min_baseline_m': 0.3, 'min_parallax_rad': 0.1, 'max_range_m': 6.0}
    target = (2.0, 2.0)
    estimate = triangulated([wedge_to((0.0, 0.0), target), wedge_to((1.0, 0.0), target)], **limits)
    assert estimate['method'] == 'triangulated' and estimate['qualified'] is False
    assert estimate['map_xyz'][:2] == pytest.approx(target) and estimate['map_xyz'][2] is None
    assert 'height unknown' in estimate['assumptions']
    # Sigma grows as the viewpoints close up: the same angular width crosses at a shallower angle.
    narrow = triangulated([wedge_to((0.0, 0.0), target), wedge_to((0.4, 0.0), target)], **limits)
    assert narrow['sigma_m'] > estimate['sigma_m'] > 0
    three = triangulated([wedge_to((0.0, 0.0), target), wedge_to((1.0, 0.0), target), wedge_to((3.0, 0.5), target)],
                         **limits)
    assert three['map_xyz'][:2] == pytest.approx(target)
    refused = [
        [wedge_to((0.0, 0.0), target)],
        [wedge_to((0.0, 0.0), target), wedge_to((0.1, 0.0), target)],  # Baseline too small.
        [wedge_to((0.0, 0.0), target), wedge_to((1.0, 1.0), (3.0, 3.0))],  # One line of sight: no parallax.
        [wedge_to((0.0, 0.0), (2.0, 2.0)), wedge_to((1.0, 0.0), (3.0, 1.0))],  # The lines cross behind a camera.
        [wedge_to((0.0, 0.0), (20.0, 20.0)), wedge_to((1.0, 0.0), (20.0, 20.0))],  # Beyond the range limit.
        [wedge_to((0.0, 0.0), target), wedge_to((1.0, 0.0), target), wedge_to((0.0, 3.0), (2.0, 3.0))],  # Disagree.
    ]
    for wedges in refused:
        assert triangulated(wedges, **limits) is None


def test_only_the_validated_metric_path_is_qualified():
    camera_in_map = Transform.create([1.0, 2.0, 0.3], [-0.5, 0.5, -0.5, 0.5])  # Optical z along map x.
    position = {'mode': 'camera', 'xyz': [0.0, 0.0, 1.5], 'frame_id': 'astra', 'uncertainty_m': 0.03,
                'calibration_sha256': 'c' * 64, 'meaning': 'visible_surface'}
    measured = rgbd_surface({'position': {**position, 'map': {'uncertainty_m': 0.12}}}, 'astra', camera_in_map)
    assert measured['qualified'] is True and measured['sigma_m'] == 0.12 and measured['calibration_sha256'] == 'c' * 64
    assert measured['map_xyz'] == pytest.approx([2.5, 2.0, 0.3])
    # Without a validated map position the measurement is still placed, but nothing vouches for it there.
    unvouched = rgbd_surface({'position': position}, 'astra', camera_in_map)
    assert unvouched['qualified'] is False and unvouched['sigma_m'] == 0.03
    assert rgbd_surface({'position': position}, 'another_frame', camera_in_map) is None
    assert rgbd_surface({'position': None}, 'astra', camera_in_map) is None


def test_estimates_are_ordered_best_first_and_never_touch_the_observation():
    target = (2.025, 0.025, 0.125)
    pose = Pose(0.5, 0.025, 0.3, 0.0, 0.0)
    camera_in_map = Transform.create([0.5, 0.025, 0.3], [-0.5, 0.5, -0.5, 0.5])
    detection = {'label': 'cup', 'score': 0.9, 'bbox': box(pose, target), 'geometry_unavailable': 'no_depth_calibration'}
    before = dict(detection)
    limits = {'max_range_m': 6.0, 'max_tilt_rad': 0.6}
    found = estimates(detection, 'front', CAMERA, HEIGHT, pose, camera_in_map, octree([voxel(target)]), **limits)
    assert [estimate['method'] for estimate in found] == ['ray_octomap_hit', 'bearing_only']
    assert all(estimate['qualified'] is False for estimate in found)
    # Without a 3D map the detection is a bearing; with too much tilt it is nothing at all.
    assert [e['method'] for e in estimates(detection, 'front', CAMERA, HEIGHT, pose, camera_in_map, **limits)] == ['bearing_only']
    assert estimates(detection, 'front', CAMERA, HEIGHT, Pose(0.5, 0.025, 0.3, 0.0, -1.0), camera_in_map,
                     **limits) == []
    assert detection == before and 'position' not in detection
