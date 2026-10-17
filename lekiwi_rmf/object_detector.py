#!/usr/bin/env python3
"""OpenCV Zoo recognition adapted to standard ROS vision_msgs, on CPU."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from collections import deque
import json
import threading
import time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from lekiwi_rmf.object_recognition import Recognizer
from lekiwi_rmf.object_cameras import camera_bindings


def detect(detector, image, header):
    """Reuse upstream preprocessing/output; this initial vocabulary is cup only."""
    height, width = image.shape[:2]
    result = Detection2DArray(header=header)
    predictions = detector.predict(image)
    for x, y, w, h, score, label in predictions:
        if not all(math.isfinite(value) for value in (x, y, w, h, score, label)) or not 0 <= score <= 1:
            raise ValueError('recognizer returned invalid geometry or confidence')
        if int(label) != 41:
            continue
        x0, y0 = max(0.0, float(x)), max(0.0, float(y))
        x1, y1 = min(float(width), float(x + w)), min(float(height), float(y + h))
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
        self.declare_parameter('model_name', 'yolox')
        self.declare_parameter('model_directory', '')
        self.declare_parameter('camera_ids', ['front'])
        self.declare_parameter('camera_profiles_file', '')
        self.declare_parameter('confidence_threshold', .5)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('max_source_age_sec', 2.0)
        confidence = self.get_parameter('confidence_threshold').value
        rate = self.get_parameter('max_rate_hz').value
        self._age = self.get_parameter('max_source_age_sec').value
        if not 0 < confidence <= 1 or not 0 < rate <= 10 or not 0 < self._age <= 10:
            raise ValueError('invalid recognition confidence/rate/freshness limits')
        self._bindings = camera_bindings(self.get_parameter('camera_ids').value,
                                        self.get_parameter('camera_profiles_file').value or None)
        self._detector = Recognizer(self.get_parameter('model_name').value,
                                   self.get_parameter('model_directory').value or None, confidence)
        self._bridge = CvBridge()
        self._frames, self._last_stamp = {}, {}
        self._frame_lock = threading.Lock()
        self._next_camera = 0
        self._generation = 0
        self._clock_jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=self._clock_reset)
        self._stats = {name: {'received': 0, 'published': 0, 'overwritten': 0, 'rejected': 0,
                              'reason': 'waiting for camera', 'inference_sec': 0., 'source_age_sec': 0., 'published_stamp': 0}
                       for name in self._bindings}
        self._durations = {name: deque(maxlen=128) for name in self._bindings}
        self._outputs = {}
        self._receipt_group = MutuallyExclusiveCallbackGroup()
        self._inference_group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, binding in self._bindings.items():
            self._outputs[name] = (
                self.create_publisher(Image, binding['prefix'] + '/source_image', 1),
                self.create_publisher(Detection2DArray, binding['prefix'] + '/detections', 1))
            self.create_subscription(Image, binding['image_topic'],
                lambda message, camera=name: self._receive(message, camera), qos,
                callback_group=self._receipt_group)
        self._diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1., self._health, callback_group=self._receipt_group)
        self.create_timer(1 / rate, self._infer, clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=self._inference_group)

    def _clock_reset(self, _jump):
        with self._frame_lock:
            self._generation += 1
            self._frames.clear()
            self._last_stamp.clear()
            for stats in self._stats.values():
                stats.update(published_stamp=0, reason='clock changed; waiting for fresh captures')

    def _reject(self, camera, reason):
        with self._frame_lock:
            self._stats[camera]['rejected'] += 1
            self._stats[camera]['reason'] = reason
        self.get_logger().warning(f'{camera}: recognition failed: {reason}', throttle_duration_sec=5)

    def _receive(self, image, camera):
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if (image.header.frame_id != self._bindings[camera]['optical_frame'] or
                not 0 < stamp or not 0 <= age <= self._age):
            self._reject(camera, 'camera binding or capture freshness mismatch')
            return
        if (not 1 <= image.width <= 1920 or not 1 <= image.height <= 1080 or
                    image.encoding not in ('rgb8', 'bgr8') or image.is_bigendian or
                    not image.width * 3 <= image.step <= image.width * 3 + 4096 or
                    len(image.data) != image.step * image.height or len(image.data) > 8 * 1024**2):
            self._reject(camera, 'recognition image exceeds encoding/payload bounds')
            return
        with self._frame_lock:
            if stamp <= self._last_stamp.get(camera, 0):
                return
            self._last_stamp[camera] = stamp
            self._stats[camera]['received'] += 1
            self._stats[camera]['overwritten'] += camera in self._frames
            self._frames[camera] = image

    def _infer(self):
        cameras = list(self._bindings)
        with self._frame_lock:
            generation = self._generation
            for offset in range(len(cameras)):
                index = (self._next_camera + offset) % len(cameras)
                camera = cameras[index]
                image = self._frames.pop(camera, None)
                if image is not None:
                    self._next_camera = (index + 1) % len(cameras)
                    break
            else:
                return
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        try:
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not 0 <= age <= self._age:
                raise ValueError('recognition source expired before inference')
            pixels = self._bridge.imgmsg_to_cv2(image, 'bgr8')
            start = time.monotonic()
            result = detect(self._detector, pixels, image.header)
            duration = time.monotonic() - start
            source_age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            with self._frame_lock:
                self._durations[camera].append(duration)
                self._stats[camera].update(inference_sec=duration, source_age_sec=source_age)
            if generation != self._generation:
                raise ValueError('clock changed during inference')
            if not 0 <= source_age <= self._age:
                raise ValueError('recognition finished after source freshness expired')
            source = self._bridge.cv2_to_imgmsg(pixels, 'bgr8')
            source.header = image.header
            image_pub, detection_pub = self._outputs[camera]
            image_pub.publish(source)
            detection_pub.publish(result)
            with self._frame_lock:
                self._stats[camera]['published'] += 1
                self._stats[camera]['published_stamp'] = stamp
                self._stats[camera]['reason'] = ''
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
            self._reject(camera, str(error))

    def _health(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        with self._frame_lock:
            for camera, stats in self._stats.items():
                durations = sorted(self._durations[camera])
                p95 = durations[max(0, math.ceil(.95 * len(durations)) - 1)] if durations else None
                values = {**stats, 'processing_p95_sec': p95, 'model_sha256': self._detector.sha256, 'camera_sha256': self._bindings[camera]['sha256'],
                          'capture_age_sec': (self.get_clock().now().nanoseconds - self._last_stamp.get(camera, 0)) / 1e9,
                          'output_age_sec': (self.get_clock().now().nanoseconds - stats['published_stamp']) / 1e9,
                          'pending_frames': int(camera in self._frames)}
                status = DiagnosticStatus(name='lekiwi/perception/' + camera,
                    hardware_id=camera, level=DiagnosticStatus.WARN if stats['reason'] or not 0 <= values['output_age_sec'] <= self._age else DiagnosticStatus.OK,
                    message=stats['reason'] or ('recognition active' if 0 <= values['output_age_sec'] <= self._age else 'observation output stale'))
                status.values = [KeyValue(key=key, value=json.dumps(value)) for key, value in values.items()]
                message.status.append(status)
        self._diagnostics.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = ObjectDetector()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
