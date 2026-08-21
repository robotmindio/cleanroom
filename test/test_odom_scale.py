"""Checks for ROS image decoding used by the odometry-scale utility."""
import importlib.util
import pathlib

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("rclpy")
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
