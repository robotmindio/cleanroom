#!/usr/bin/env python3
"""Pinned recognition adapted to standard ROS vision_msgs, locally on CPU or through a remote provider."""

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

from lekiwi_rmf.object_recognition import ProviderUnavailable, Recognizer, reject_parameter_changes
from lekiwi_rmf.object_cameras import camera_bindings


def detect(detector, image, header):
    """Publish every native label; goal-specific class filtering belongs to the consumer."""
    height, width = image.shape[:2]
    result = Detection2DArray(header=header)
    predictions = detector.predict(image)
    for x, y, w, h, score, label in predictions:
        if not all(math.isfinite(value) for value in (x, y, w, h, score)) or not 0 <= score <= 1:
            raise ValueError('recognizer returned invalid geometry or confidence')
        if not isinstance(label, str) or not 1 <= len(label) <= 128:
            raise ValueError('recognizer returned an invalid native label')
        x0, y0 = max(0.0, float(x)), max(0.0, float(y))
        x1, y1 = min(float(width), float(x + w)), min(float(height), float(y + h))
        if x1 <= x0 or y1 <= y0:
            continue
        detection = Detection2D(header=header)
        detection.bbox.center.position.x, detection.bbox.center.position.y = (x0 + x1) / 2, (y0 + y1) / 2
        detection.bbox.size_x, detection.bbox.size_y = x1 - x0, y1 - y0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = label, float(score)
        detection.results = [hypothesis]
        result.detections.append(detection)
    # DEBT: the observation and Fiber contracts cap a capture at 20 detections, so a cluttered frame keeps
    # its 20 highest scores and lower-scored objects are absent; raise the cap with both contracts together.
    result.detections.sort(key=lambda item: item.results[0].hypothesis.score, reverse=True)
    del result.detections[20:]
    return result


# How long frames go straight to the fallback after the primary provider failed, so an unreachable
# provider costs one timeout per window instead of one per frame.
PRIMARY_RETRY_SEC = 10.


class ObjectDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('object_detector', **kwargs)
        self.declare_parameter('model_name', 'yolox')
        self.declare_parameter('fallback_model_name', '')
        self.declare_parameter('model_directory', '')
        self.declare_parameter('camera_ids', ['front'])
        self.declare_parameter('camera_profiles_file', '')
        self.declare_parameter('confidence_threshold', .5)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('max_source_age_sec', 2.0)
        reject_parameter_changes(self, ['model_name', 'fallback_model_name', 'model_directory', 'camera_ids', 'camera_profiles_file',
                                        'confidence_threshold', 'max_rate_hz', 'max_source_age_sec'])
        confidence = self.get_parameter('confidence_threshold').value
        rate = self.get_parameter('max_rate_hz').value
        self._age = self.get_parameter('max_source_age_sec').value
        if not 0 < confidence <= 1 or not 0 < rate <= 10 or not 0 < self._age <= 10:
            raise ValueError('invalid recognition confidence/rate/freshness limits')
        self._bindings = camera_bindings(self.get_parameter('camera_ids').value,
                                        self.get_parameter('camera_profiles_file').value or None)
        self._detector = Recognizer(self.get_parameter('model_name').value,
                                   self.get_parameter('model_directory').value or None, confidence)
        # Frames the fallback serves are published on its own topics, so each observation keeps the
        # identity of the recognizer that actually produced it.
        fallback = self.get_parameter('fallback_model_name').value
        if fallback == self.get_parameter('model_name').value:
            raise ValueError('fallback recognizer must differ from the primary recognizer')
        self._fallback = Recognizer(fallback, None, confidence) if fallback else None
        self._primary_retry_at = 0.
        self._bridge = CvBridge()
        self._frames, self._last_stamp = {}, {}
        self._frame_lock = threading.Lock()
        self._next_camera = 0
        self._generation = 0
        self._clock_jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=self._clock_reset)
        self._stats = {name: {'received': 0, 'published': 0, 'overwritten': 0, 'rejected': 0, 'fallback_published': 0,
                              'reason': 'waiting for camera', 'primary_reason': '', 'served_by': '',
                              'inference_sec': 0., 'source_age_sec': 0., 'published_stamp': 0}
                       for name in self._bindings}
        self._durations = {name: deque(maxlen=128) for name in self._bindings}
        self._outputs = {}
        self._receipt_group = MutuallyExclusiveCallbackGroup()
        self._inference_group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        roles = {'primary': ''} | ({'fallback': '/fallback'} if self._fallback else {})
        for name, binding in self._bindings.items():
            self._outputs[name] = {role: (
                self.create_publisher(Image, binding['prefix'] + suffix + '/source_image', 1),
                self.create_publisher(Detection2DArray, binding['prefix'] + suffix + '/detections', 1))
                for role, suffix in roles.items()}
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
            result, role = self._recognize(camera, pixels, image.header)
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
            image_pub, detection_pub = self._outputs[camera][role]
            image_pub.publish(source)
            detection_pub.publish(result)
            with self._frame_lock:
                self._stats[camera]['published'] += 1
                self._stats[camera]['fallback_published'] += role == 'fallback'
                self._stats[camera]['served_by'] = role
                self._stats[camera]['published_stamp'] = stamp
                self._stats[camera]['reason'] = ''
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
            self._reject(camera, str(error))

    def _recognize(self, camera, pixels, header):
        if self._fallback is None or time.monotonic() >= self._primary_retry_at:
            try:
                result = detect(self._detector, pixels, header)
                with self._frame_lock:
                    self._stats[camera]['primary_reason'] = ''
                return result, 'primary'
            except ProviderUnavailable as error:
                if self._fallback is None:
                    raise
                self._primary_retry_at = time.monotonic() + PRIMARY_RETRY_SEC
                with self._frame_lock:
                    self._stats[camera]['primary_reason'] = str(error)
                self.get_logger().warning(f'{camera}: primary recognizer unavailable, serving fallback: {error}',
                                          throttle_duration_sec=30)
        return detect(self._fallback, pixels, header), 'fallback'

    def _health(self):
        message = DiagnosticArray()
        now = self.get_clock().now()
        message.header.stamp = now.to_msg()
        stamp_now = now.nanoseconds
        with self._frame_lock:
            for camera, stats in self._stats.items():
                durations = sorted(self._durations[camera])
                p95 = durations[max(0, math.ceil(.95 * len(durations)) - 1)] if durations else None
                values = {**stats, 'processing_p95_sec': p95, 'model_sha256': self._detector.sha256,
                          'fallback_model_sha256': self._fallback.sha256 if self._fallback else '',
                          'camera_sha256': self._bindings[camera]['sha256'],
                          'capture_age_sec': (stamp_now - self._last_stamp.get(camera, 0)) / 1e9,
                          'output_age_sec': (stamp_now - stats['published_stamp']) / 1e9,
                          'pending_frames': int(camera in self._frames)}
                fresh = 0 <= values['output_age_sec'] <= self._age
                degraded = stats.get('primary_reason') and stats.get('served_by') == 'fallback'
                status = DiagnosticStatus(name='lekiwi/perception/' + camera,
                    hardware_id=camera, level=DiagnosticStatus.WARN if stats['reason'] or not fresh or degraded else DiagnosticStatus.OK,
                    message=stats['reason'] or ('observation output stale' if not fresh else
                                                f'fallback recognition: {stats["primary_reason"]}' if degraded else 'recognition active'))
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
