#!/usr/bin/env python3
"""OpenCV Zoo recognition adapted to standard ROS vision_msgs, on CPU."""

import hashlib
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from lekiwi_rmf.opencv_yolox import YoloX
from lekiwi_rmf.object_model import MODEL_NAME, MODEL_SHA256


def detect(detector, image, header):
    """Reuse upstream preprocessing/output; this initial vocabulary is cup only."""
    height, width = image.shape[:2]
    scale = min(640 / width, 640 / height)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (max(1, int(width * scale)), max(1, int(height * scale))))
    padded = np.full((640, 640, 3), 114.0, dtype=np.float32)
    padded[:resized.shape[0], :resized.shape[1]] = resized
    result = Detection2DArray(header=header)
    predictions = detector.infer(padded)
    for x, y, w, h, score, label in predictions:
        if not all(math.isfinite(value) for value in (x, y, w, h, score, label)) or not 0 <= score <= 1:
            raise ValueError('recognizer returned invalid geometry or confidence')
        if int(label) != 41:
            continue
        x0, y0 = max(0.0, float(x / scale)), max(0.0, float(y / scale))
        x1, y1 = min(float(width), float((x + w) / scale)), min(float(height), float((y + h) / scale))
        if x1 <= x0 or y1 <= y0:
            continue
        detection = Detection2D(header=header)
        detection.bbox.center.position.x, detection.bbox.center.position.y = (x0 + x1) / 2, (y0 + y1) / 2
        detection.bbox.size_x, detection.bbox.size_y = x1 - x0, y1 - y0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = 'cup', float(score)
        detection.results = [hypothesis]
        result.detections.append(detection)
    if len(result.detections) > 20:
        raise ValueError('more than 20 cups in one source frame')
    return result


class ObjectDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('object_detector', **kwargs)
        self.declare_parameter('model_path', str(Path.home() / '.ros/lekiwi/models' / MODEL_NAME))
        self.declare_parameter('confidence_threshold', .5)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('max_source_age_sec', 2.0)
        model = Path(self.get_parameter('model_path').value).expanduser()
        with model.open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != MODEL_SHA256:
                raise ValueError('recognizer model does not match the pinned SHA-256')
        confidence = self.get_parameter('confidence_threshold').value
        rate = self.get_parameter('max_rate_hz').value
        self._age = self.get_parameter('max_source_age_sec').value
        if not 0 < confidence <= 1 or not 0 < rate <= 10 or not 0 < self._age <= 10:
            raise ValueError('invalid recognition confidence/rate/freshness limits')
        cv2.setNumThreads(2)
        self._detector = YoloX(str(model), confThreshold=confidence)
        self._bridge = CvBridge()
        self._latest = None
        self._last_stamp = None
        self._publisher = self.create_publisher(Detection2DArray, 'detections', 1)
        # Publish the accepted source with detections after inference, avoiding a history backlog.
        self._source = self.create_publisher(Image, 'source_image', 1)
        self.create_subscription(Image, 'image', self._receive, qos_profile_sensor_data)
        self.create_timer(1 / rate, self._infer, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _receive(self, image):
        self._latest = image

    def _infer(self):
        image, self._latest = self._latest, None
        if image is None:
            return
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        if stamp == self._last_stamp:
            return
        try:
            age = (self.get_clock().now().nanoseconds - stamp) / 10**9
            if not image.header.frame_id or not 0 < stamp or not 0 <= age <= self._age:
                raise ValueError('recognition source is stale, unstamped or in the future')
            if not 1 <= image.width <= 1920 or not 1 <= image.height <= 1080 or len(image.data) > 8 * 1024**2:
                raise ValueError('recognition image exceeds its bounds')
            pixels = self._bridge.imgmsg_to_cv2(image, 'bgr8')
            result = detect(self._detector, pixels, image.header)
            if (self.get_clock().now().nanoseconds - stamp) / 10**9 > self._age:
                raise ValueError('recognition finished after source freshness expired')
            # Canonicalize encoding while preserving the exact capture timestamp.
            source = self._bridge.cv2_to_imgmsg(pixels, 'bgr8')
            source.header = image.header
            self._source.publish(source)
            self._publisher.publish(result)
            self._last_stamp = stamp
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
            self.get_logger().warning(f'recognition failed: {error}', throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
