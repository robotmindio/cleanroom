"""Checks for ROS image decoding used by the odometry-scale utility."""
import importlib.util
import math
import pathlib
import types

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


SCRIPT = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "odom_scale.py"
spec = importlib.util.spec_from_file_location("odom_scale", SCRIPT)
odom_scale = importlib.util.module_from_spec(spec)
spec.loader.exec_module(odom_scale)


@pytest.mark.parametrize("encoding", ["bgr8", "rgb8"])
def test_image_decoder_honors_color_encoding_and_row_padding(encoding):
    bgr = np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
    source = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if encoding == "rgb8" else bgr
    message = Image()
    message.height, message.width, message.encoding = 1, 2, encoding
    message.step = 8
    message.data = source.tobytes() + b"\x00\x00"

    assert np.array_equal(odom_scale.OdomScale.bgr_image(message), bgr)


def test_image_decoder_expands_mono8_to_bgr():
    message = Image()
    message.height, message.width, message.encoding = 1, 2, "mono8"
    message.step = 3
    message.data = b"\x0a\x14\x00"

    assert np.array_equal(
        odom_scale.OdomScale.bgr_image(message),
        np.array([[[10, 10, 10], [20, 20, 20]]], dtype=np.uint8),
    )


def test_image_decoder_rejects_malformed_data():
    message = Image()
    message.height, message.width, message.encoding = 2, 2, "bgr8"
    message.step = 6
    message.data = b"\x00" * 6

    with pytest.raises(ValueError, match="shorter"):
        odom_scale.OdomScale.bgr_image(message)


def test_bad_frame_clears_previously_captured_image():
    message = Image()
    message.height, message.width, message.encoding = 2, 2, "bgr8"
    message.step = 6
    message.data = b"\x00" * 6
    node = odom_scale.OdomScale.__new__(odom_scale.OdomScale)
    node.frame = np.ones((1, 1, 3), dtype=np.uint8)
    node.get_logger = lambda: type("Logger", (), {"warn": lambda *_: None})()

    node.on_image(message)

    assert node.frame is None


def test_non_finite_odometry_is_not_accepted_as_motion_feedback():
    node = odom_scale.OdomScale.__new__(odom_scale.OdomScale)
    node.odom = (1.0, 2.0, 3.0)
    node.odom_stamp = 1.0
    node.get_logger = lambda: type("Logger", (), {"warn": lambda *_: None})()
    pose = types.SimpleNamespace(
        position=types.SimpleNamespace(x=math.nan, y=0.0, z=0.0),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    vector = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    message = types.SimpleNamespace(
        pose=types.SimpleNamespace(pose=pose),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=vector, angular=vector)
        ),
    )

    node.on_odom(message)

    assert node.odom is None
    assert node.odom_stamp is None


def test_stale_odometry_stops_calibration_before_motion(monkeypatch):
    published = []
    node = odom_scale.OdomScale.__new__(odom_scale.OdomScale)
    node.cmd_pub = types.SimpleNamespace(publish=published.append)
    node.odom_is_fresh = lambda: False
    monkeypatch.setattr(odom_scale.rclpy, "ok", lambda: True)

    with pytest.raises(RuntimeError, match="motion stopped"):
        node.drive_sampling(Twist(), 1.0, [])

    assert len(published) == 1
    assert published[0].linear.x == 0.0


def test_motion_utilities_publish_only_to_manual_source_topic():
    teleop = SCRIPT.parent / "teleop.py"
    assert 'create_publisher(Twist, "/cmd_vel_manual", 10)' in SCRIPT.read_text()
    assert 'create_publisher(Twist, "/cmd_vel_manual", 10)' in teleop.read_text()


def test_launch_calibration_preserves_other_values(monkeypatch, tmp_path):
    path = tmp_path / "launch.conf"
    path.write_text("camera_height=0.200000\n")
    monkeypatch.setenv("LEKIWI_LAUNCH_CALIBRATION", str(path))

    odom_scale.save_launch_calibration("xy_velocity_scale", 1.25)

    assert path.read_text() == "camera_height=0.200000\nxy_velocity_scale=1.250000\n"
