"""The floor-to-range conversion, checked against a floor we drew ourselves.

Paint a uniform floor with one dark block at a known distance, hand it to the node, and
the beam pointing at the block has to come back with that distance. If the projection
picks up a sign error or the seed patch stops finding floor, this fails.
"""
import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
rclpy = pytest.importorskip("rclpy")

SCRIPT = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "free_space.py"
spec = importlib.util.spec_from_file_location("free_space", SCRIPT)
free_space = importlib.util.module_from_spec(spec)
sys.modules["free_space"] = free_space
spec.loader.exec_module(free_space)

HEIGHT, PITCH = 0.20, 0.30  # m, rad -- the numbers the node is configured with below
K = np.array([[860.9, 0, 360.4], [0, 867.2, 230.8], [0, 0, 1]])


def pixel_of(forward, left):
    """Inverse of the node's projection: where a point on the floor lands in the image."""
    cos_p, sin_p = math.cos(PITCH), math.sin(PITCH)
    # camera-frame ray to that floor point
    z = forward * cos_p + HEIGHT * sin_p
    y = HEIGHT * cos_p - forward * sin_p
    x = -left
    u = K[0, 0] * x / z + K[0, 2]
    v = K[1, 1] * y / z + K[1, 2]
    return int(round(u)), int(round(v))


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = free_space.FreeSpace()
    n.k, n.d = K, np.zeros(5)
    n.set_parameters([
        rclpy.parameter.Parameter("camera_height", value=HEIGHT),
        rclpy.parameter.Parameter("camera_pitch", value=PITCH),
        rclpy.parameter.Parameter("camera_offset_x", value=0.0),
        rclpy.parameter.Parameter("beams", value=41),
    ])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def floor_with_block(distance, half_width=0.15):
    """A grey floor, and a dark block standing on it `distance` metres straight ahead."""
    image = np.full((480, 640, 3), 130, np.uint8)
    image += np.random.default_rng(0).integers(-4, 5, image.shape, dtype=np.int16).astype(np.uint8)
    near_left = pixel_of(distance, half_width)
    near_right = pixel_of(distance, -half_width)
    # everything above the block's base, between its edges, is the block itself
    cv2.rectangle(image, (near_left[0], 0), (near_right[0], near_left[1]), (20, 20, 20), -1)
    return image


@pytest.mark.parametrize("distance", [0.4, 0.8, 1.5])
def test_block_reported_at_its_distance(node, distance):
    scan = node.scan(floor_with_block(distance), stamp=None)
    assert scan is not None
    ahead = scan.ranges[len(scan.ranges) // 2]
    assert ahead == pytest.approx(distance, abs=0.1), f"beam straight ahead said {ahead}"


def test_empty_floor_reports_nothing(node):
    image = np.full((480, 640, 3), 130, np.uint8)
    scan = node.scan(image, stamp=None)
    assert all(math.isinf(r) for r in scan.ranges)
