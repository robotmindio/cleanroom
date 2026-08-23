"""The relay must actually hear the device machine's compressed stream.

The compressed transport publishes one topic below the raw name; subscribing to
the raw name with the CompressedImage type matches nothing and fails silently --
RTAB-Map just sees black. So publish a JPEG the way v4l2_camera does and require
the canonical topics to come back: a raw image carrying the frame's stamp, and
the last CameraInfo re-stamped to travel with it. The wrist camera carries no
calibration by design, so its info topic must stay silent even though its
images are relayed.
"""
import importlib.util
import pathlib
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
cv2 = pytest.importorskip("cv2")
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

SCRIPT = pathlib.Path(__file__).resolve().parent.parent / "lekiwi_rmf" / "camera_relay.py"
spec = importlib.util.spec_from_file_location("camera_relay", SCRIPT)
camera_relay = importlib.util.module_from_spec(spec)
sys.modules["camera_relay"] = camera_relay
spec.loader.exec_module(camera_relay)

FRAME = np.full((4, 6, 3), 90, np.uint8)
BRIDGE = CvBridge()
SENSOR_QOS = QoSProfile(
    depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
)
RELIABLE_QOS = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
)


class DeviceMachine:
    """Stands in for v4l2_camera on the robot's Pi: one compressed feed per camera."""

    def __init__(self):
        self.node = rclpy.create_node("device_machine")
        jpeg = BRIDGE.cv2_to_compressed_imgmsg(FRAME, "jpg")
        jpeg.header.stamp.sec = 7
        self.front_jpeg = jpeg
        self.front_info = CameraInfo()
        self.front_info.width = 640
        self.front_image_pub = self.node.create_publisher(
            CompressedImage, "/pi/camera/front/image_raw/compressed", SENSOR_QOS
        )
        self.front_info_pub = self.node.create_publisher(
            CameraInfo, "/pi/camera/front/camera_info", SENSOR_QOS
        )
        self.wrist_pub = self.node.create_publisher(
            CompressedImage, "/pi/camera/wrist/image_raw/compressed", SENSOR_QOS
        )

    def publish(self):
        self.front_image_pub.publish(self.front_jpeg)
        self.front_info_pub.publish(self.front_info)
        wrist = CompressedImage()
        wrist.header = self.front_jpeg.header
        wrist.format = self.front_jpeg.format
        wrist.data = self.front_jpeg.data
        self.wrist_pub.publish(wrist)


@pytest.fixture(scope="module")
def graph():
    rclpy.init()
    device = DeviceMachine()
    relay = camera_relay.CameraRelay()
    executor = SingleThreadedExecutor()
    executor.add_node(device.node)
    executor.add_node(relay)
    yield device, relay, executor
    relay.destroy_node()
    device.node.destroy_node()
    rclpy.shutdown()


def pump_until(executor, device, checker, ready, timeout=3.0):
    """Publish and spin until `ready()` or the deadline passes."""
    deadline = time.monotonic() + timeout
    while not ready() and time.monotonic() < deadline:
        device.publish()
        executor.spin_once(timeout_sec=0.05)


def test_front_frame_expands_to_canonical_topics(graph):
    device, _, executor = graph
    seen_images, seen_infos = [], []
    checker = rclpy.create_node("front_checker")
    checker.create_subscription(Image, "/camera/front/image_raw", seen_images.append, SENSOR_QOS)
    # free_space/RTAB-Map request reliable delivery from their scan-and-image
    # sources, so the info topic has to satisfy a reliable subscription too.
    checker.create_subscription(CameraInfo, "/camera/front/camera_info", seen_infos.append, RELIABLE_QOS)
    executor.add_node(checker)
    try:
        pump_until(executor, device, checker, lambda: seen_images and seen_infos)
    finally:
        executor.remove_node(checker)
        checker.destroy_node()

    assert seen_images, "relay published no raw image -- subscription mismatch?"
    assert seen_images[0].header.stamp.sec == 7, "frames must keep their original stamps"
    assert np.array_equal(BRIDGE.imgmsg_to_cv2(seen_images[0], "bgr8"), FRAME)
    assert seen_infos, "relay published no CameraInfo alongside the image"
    assert seen_infos[0].width == 640
    assert seen_infos[0].header.stamp.sec == 7, "info is re-stamped onto each frame"


def test_wrist_frames_are_relayed_but_its_missing_calibration_is_not_invented(graph):
    device, _, executor = graph
    seen_images, seen_infos = [], []
    checker = rclpy.create_node("wrist_checker")
    checker.create_subscription(Image, "/camera/wrist/image_raw", seen_images.append, SENSOR_QOS)
    checker.create_subscription(CameraInfo, "/camera/wrist/camera_info", seen_infos.append, RELIABLE_QOS)
    executor.add_node(checker)
    try:
        pump_until(executor, device, checker, lambda: bool(seen_images))
    finally:
        executor.remove_node(checker)
        checker.destroy_node()

    assert seen_images, "wrist frames should be relayed like any other camera"
    assert not seen_infos, "wrist has no calibration; inventing one would be worse"
