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
from sensor_msgs.msg import CameraInfo, Image

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


def floor_with_offset_block(forward, left, half_width=0.06):
    image = np.full((480, 640, 3), 130, np.uint8)
    near_left = pixel_of(forward, left + half_width)
    near_right = pixel_of(forward, left - half_width)
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


def test_camera_offset_transforms_bearing_and_range_into_scan_frame(node):
    forward, left, offset = 0.8, 0.25, 0.03
    node.set_parameters([rclpy.parameter.Parameter("camera_offset_x", value=offset)])
    scan = node.scan(floor_with_offset_block(forward, left), stamp=None)
    node.set_parameters([rclpy.parameter.Parameter("camera_offset_x", value=0.0)])

    index = min(range(len(scan.ranges)), key=lambda i: scan.ranges[i])
    angle = scan.angle_min + index * scan.angle_increment
    assert scan.ranges[index] == pytest.approx(math.hypot(forward + offset, left), abs=0.1)
    assert angle == pytest.approx(math.atan2(left, forward + offset), abs=0.1)


def test_distorted_pixels_are_undistorted_before_floor_projection(node):
    forward, left = 1.0, 0.20
    cos_p, sin_p = math.cos(PITCH), math.sin(PITCH)
    point = np.array([[-left, HEIGHT * cos_p - forward * sin_p,
                      forward * cos_p + HEIGHT * sin_p]], dtype=np.float64)
    distortion = np.array([0.16, -0.08, 0.001, -0.001, 0.0])
    pixel, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), K, distortion)
    u, v = pixel.reshape(2)
    node.d = distortion
    measured_fwd, measured_left, visible = node.ground_points(
        np.array([v]), np.array([u])
    )
    node.d = np.zeros(5)

    assert visible[0, 0]
    assert measured_fwd[0, 0] == pytest.approx(forward, abs=1e-6)
    assert measured_left[0, 0] == pytest.approx(left, abs=1e-6)


def test_camera_fault_reports_a_blocked_scan(node):
    scan = node.blocked_scan(stamp=None)
    assert len(scan.ranges) == 41
    assert all(value == pytest.approx(0.11) for value in scan.ranges)


def test_scan_never_exceeds_declared_range_max(node):
    node.set_parameters([
        rclpy.parameter.Parameter("camera_offset_x", value=10.0),
        rclpy.parameter.Parameter("range_max", value=3.0),
    ])
    scan = node.scan(floor_with_block(0.8), stamp=None)
    assert all(math.isinf(value) or scan.range_min <= value <= scan.range_max for value in scan.ranges)
    node.set_parameters([rclpy.parameter.Parameter("camera_offset_x", value=0.0)])


def test_invalid_camera_intrinsics_are_rejected(node):
    info = CameraInfo()
    info.k = [0.0] * 9
    node.on_info(info)
    assert node.k is None
    node.k, node.d = K, np.zeros(5)


def test_no_floor_in_view_does_not_crash(node):
    node.set_parameters([rclpy.parameter.Parameter("camera_pitch", value=-2.0)])
    image = np.full((480, 640, 3), 130, np.uint8)
    assert node.scan(image, stamp=None) is None
    node.set_parameters([rclpy.parameter.Parameter("camera_pitch", value=PITCH)])


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8"])
def test_image_decoder_honors_encoding_and_row_padding(node, encoding):
    bgr = np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
    source = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if encoding == "rgb8" else bgr
    msg = Image()
    msg.height, msg.width, msg.encoding = 1, 2, encoding
    msg.step = 8
    msg.data = source.tobytes() + b"\x00\x00"
    assert np.array_equal(node.bgr_image(msg), bgr)
