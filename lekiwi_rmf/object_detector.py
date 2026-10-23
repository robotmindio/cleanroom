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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from collections import deque
import hashlib
import json
import threading
import time
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from lekiwi_rmf.latency_budget import LatencyBudget
from lekiwi_rmf.object_recognition import ProviderUnavailable, Recognizer, reject_parameter_changes
from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.object_observations import artifact_pixels, capture_artifact
from lekiwi_rmf.perception_concepts import load_concept, reference
from lekiwi_rmf.perception_profiles import make_profile, tracked_profiles
from lekiwi_rmf.perception_query import make_query
from lekiwi_rmf.perception_study import admitted_candidate
from lekiwi_rmf.msg import PerceptionOperatingPoint
from lekiwi_rmf.srv import ApplyPerceptionProfile, SetPerceptionQuery

QUERY_SERVICE = '/robot/perception/set_query'
PROFILE_SERVICE = '/robot/perception/apply_profile'
PROFILE_TOPIC = '/robot/perception/profile'
PROFILE_HISTORY = 16
# The task gateway's exclusive-resource holders; a running search reserves the query.
RESOURCE_TOPIC = '/robot/resources'


def canonical(document):
    return json.dumps(document, sort_keys=True, separators=(',', ':'), ensure_ascii=False)


def detect(detector, image, header, query=None, profile=None, artifact=None):
    """Publish every label the recognizer reports; goal-specific filtering belongs to the consumer.

    Returns the detections, the per-detection evidence aligned with them and the provider run reference.
    """
    height, width = image.shape[:2]
    result = Detection2DArray(header=header)
    predictions = detector.predict(image, query, profile, artifact)
    rows = []
    for x, y, w, h, score, label, *rest in predictions:
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
        rows.append((detection, dict(rest[0]) if rest and rest[0] else {}))
    # The observation and Fiber contracts cap a capture at 20 detections; the profile's max_detections may
    # keep fewer. A cluttered frame keeps its highest scores and lower-scored objects are absent.
    limit = profile.get('max_detections', 20) if profile is not None else 20
    rows.sort(key=lambda item: item[0].results[0].hypothesis.score, reverse=True)
    del rows[min(limit, 20):]
    result.detections = [detection for detection, _ in rows]
    return result, [extras for _, extras in rows], getattr(predictions, 'run', None)


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
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('max_source_age_sec', 2.0)
        reject_parameter_changes(self, ['model_name', 'fallback_model_name', 'model_directory', 'camera_ids', 'camera_profiles_file',
                                        'max_rate_hz', 'max_source_age_sec'])
        rate = self.get_parameter('max_rate_hz').value
        self._age = self.get_parameter('max_source_age_sec').value
        if not 0 < rate <= 10 or not 0 < self._age <= 10:
            raise ValueError('invalid recognition rate/freshness limits')
        self._bindings = camera_bindings(self.get_parameter('camera_ids').value,
                                        self.get_parameter('camera_profiles_file').value or None)
        self._detector = Recognizer(self.get_parameter('model_name').value,
                                   self.get_parameter('model_directory').value or None)
        # Frames the fallback serves are published on its own topics, so each observation keeps the
        # identity of the recognizer that actually produced it.
        fallback = self.get_parameter('fallback_model_name').value
        if fallback == self.get_parameter('model_name').value:
            raise ValueError('fallback recognizer must differ from the primary recognizer')
        self._fallback = Recognizer(fallback, None) if fallback else None
        if self._fallback is not None and self._fallback.default_query is not None:
            raise ValueError('the fallback recognizer must be closed-vocabulary; it serves when the query cannot be')
        self._recognizers = {'primary': self._detector} | ({'fallback': self._fallback} if self._fallback else {})
        # The active query and profiles are runtime state, not parameters: they change per goal or after a
        # calibration study, and every frame carries the snapshot it ran under, so a change mid-inference
        # cannot relabel that frame.
        self._query_lock = threading.Lock()
        self._query = self._detector.default_query
        self._profiles = {role: recognizer.default_profile for role, recognizer in self._recognizers.items()}
        self._history = {role: deque(maxlen=PROFILE_HISTORY) for role in self._recognizers}
        # One inference at a time per recognizer: a profile probe never races a frame on a shared model.
        self._model_locks = {role: threading.Lock() for role in self._recognizers}
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
                              'reason': 'waiting for camera', 'primary_reason': '', 'served_by': '', 'query_revision': '',
                              'profile_revision': '', 'inference_sec': 0., 'source_age_sec': 0., 'published_stamp': 0}
                       for name in self._bindings}
        self._durations = {name: deque(maxlen=128) for name in self._bindings}
        # Stage percentiles across cameras; a detection older than max_source_age_sec never reaches control.
        self._budget = LatencyBudget('perception', self._age)
        self._received_at = {}
        self._outputs = {}
        self._receipt_group = MutuallyExclusiveCallbackGroup()
        self._inference_group = MutuallyExclusiveCallbackGroup()
        # A profile probe may wait for a remote provider; it must stall neither receipt nor inference.
        self._profile_group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        roles = {'primary': ''} | ({'fallback': '/fallback'} if self._fallback else {})
        for name, binding in self._bindings.items():
            # The operating point is stamped with the capture header, so the collector pairs each frame
            # with the exact model, profile and query that produced it.
            self._outputs[name] = {role: (
                self.create_publisher(Image, binding['prefix'] + suffix + '/source_image', 1),
                self.create_publisher(Detection2DArray, binding['prefix'] + suffix + '/detections', 1),
                self.create_publisher(PerceptionOperatingPoint, binding['prefix'] + suffix + '/operating_point', 1))
                for role, suffix in roles.items()}
            self.create_subscription(Image, binding['image_topic'],
                lambda message, camera=name: self._receive(message, camera), qos,
                callback_group=self._receipt_group)
        self._profile = self.create_publisher(String, PROFILE_TOPIC,
                                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publish_profile()
        self._query_holder = None
        self.create_service(SetPerceptionQuery, QUERY_SERVICE, self._set_query, callback_group=self._receipt_group)
        self.create_service(ApplyPerceptionProfile, PROFILE_SERVICE, self._apply_profile, callback_group=self._profile_group)
        self.create_subscription(String, RESOURCE_TOPIC, self._resources_message,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                 callback_group=self._receipt_group)
        self._diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1., self._health, callback_group=self._receipt_group)
        self.create_timer(1 / rate, self._infer, clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=self._inference_group)

    def _publish_profile(self):
        """The operating points in force. Each frame is stored under the model_sha256 and profile revision that
        served it, so a consumer credits a frame only for labels its own recognizer reported under the profile
        it admitted: the primary's labels are the query in force; the fallback's its native catalogue."""
        with self._query_lock:
            query, profiles = self._query, dict(self._profiles)
        self._profile.publish(String(data=json.dumps({
            'schema': 1, 'cameras': {name: binding['sha256'] for name, binding in self._bindings.items()},
            'recognizers': [{
                'role': role, 'model_sha256': recognizer.sha256, 'model_name': recognizer.name,
                'profile': profiles[role].document(), 'open_vocabulary': recognizer.open_vocabulary,
                'concepts_supported': bool(getattr(recognizer, 'supports_concepts', False)),
                'labels': list(query.reportable) if recognizer.open_vocabulary else list(recognizer.labels or []),
                'query': query.document() if recognizer.open_vocabulary else None}
                for role, recognizer in self._recognizers.items()]}, ensure_ascii=False)))

    def _resources_message(self, message):
        try:
            holder = json.loads(message.data)['resources']['perception_query']
            if holder is not None and not isinstance(holder, dict):
                raise TypeError('holder must be an object')
        except (ValueError, KeyError, TypeError) as error:
            # Fail closed: an unreadable reservation is treated as held.
            self.get_logger().error(f'invalid resource state: {error}')
            holder = {'task_id': 'unknown'}
        with self._query_lock:
            self._query_holder = holder

    def _set_query(self, request, response):
        with self._query_lock:
            current = self._query
            if current is not None:
                response.revision, response.labels = current.revision, list(current.labels)
            try:
                if self._query_holder is not None:
                    raise ValueError(f'resource_busy: task {self._query_holder.get("task_id")} reserves the '
                                     'perception query until it stops')
                if current is None:
                    raise ValueError('the active recognizer has a closed vocabulary and cannot take a query')
                if request.expected_revision and request.expected_revision != current.revision:
                    raise ValueError('the active query changed; read its revision and retry')
                concepts = [reference(load_concept(revision)) for revision in request.concept_revisions]
                if concepts and not getattr(self._detector, 'supports_concepts', False):
                    raise ValueError(f'{self._detector.name} cannot match learned concepts')
                if request.labels:
                    query = make_query(list(request.labels), list(request.part_of) or None, concepts)
                elif request.part_of or concepts:
                    raise ValueError('parts and concepts need labels')
                else:
                    query = self._detector.default_query
            except ValueError as error:
                response.success, response.detail = False, str(error)
                self.get_logger().warning(f'perception query rejected: {error}')
                return response
            self._query = query
            response.success, response.detail = True, 'query applies from the next inference'
            response.previous_revision, response.previous_labels = current.revision, list(current.labels)
            response.revision, response.labels = query.revision, list(query.labels)
        self._publish_profile()
        if query != current:
            self.get_logger().info(f'perception query {query.revision[:12]}: {", ".join(query.reportable)}')
        return response

    def _candidate(self, role, recognizer, request):
        """Exactly one admitted source: a tracked profile, an exact earlier revision, or a study's selection."""
        sources = [bool(request.profile_name), bool(request.revert_to_revision), bool(request.parameters_json)]
        if sum(sources) != 1 or bool(request.parameters_json) != bool(request.evidence_id):
            raise ValueError('name exactly one of profile_name, revert_to_revision, or parameters_json with evidence_id')
        if request.profile_name:
            _, profiles = tracked_profiles()
            if request.profile_name not in profiles[recognizer.backend]:
                raise ValueError(f'{request.profile_name} is not a tracked {recognizer.backend} profile')
            return profiles[recognizer.backend][request.profile_name]
        if request.revert_to_revision:
            for profile in [recognizer.default_profile, *self._history[role]]:
                if profile.revision == request.revert_to_revision:
                    return profile
            raise ValueError('that revision is not in this detector\'s profile history')
        if len(request.parameters_json.encode()) > 4096:
            raise ValueError('candidate parameters exceed 4096 bytes')
        try:
            parameters = json.loads(request.parameters_json)
        except (ValueError, RecursionError) as error:
            raise ValueError('candidate parameters are not JSON') from error
        candidate = make_profile(recognizer.backend, parameters)
        # Promotion keeps the existing authority: only a recorded study that selected exactly this
        # candidate for this recognizer on held-out evidence can admit untracked parameters.
        return admitted_candidate(request.evidence_id, recognizer.sha256, candidate)

    def _probe(self, role, recognizer, candidate):
        """Prepare and health-check the candidate before it can serve a frame: one inference on a synthetic
        capture under exactly the candidate. A remote provider must echo the candidate's parameters."""
        artifact = capture_artifact(np.full((48, 64, 3), 127, np.uint8))
        with self._query_lock:
            query = self._query if recognizer.open_vocabulary else None
        with self._model_locks[role]:
            detect(recognizer, artifact_pixels(artifact), Header(frame_id='profile_probe'), query, candidate, artifact)

    def _apply_profile(self, request, response):
        role = request.role or 'primary'
        recognizer = self._recognizers.get(role)
        with self._query_lock:
            current = self._profiles.get(role)
        if current is not None:
            response.revision, response.profile_json = current.revision, canonical(current.document())
        try:
            if recognizer is None:
                raise ValueError(f'no {role} recognizer is configured')
            if not request.expected_revision:
                raise ValueError('expected_revision is required; read the profile in force first')
            if request.expected_revision != current.revision:
                raise ValueError('the active profile changed; read its revision and retry')
            candidate = self._candidate(role, recognizer, request)
            if candidate.revision == current.revision:
                raise ValueError('that profile is already in force')
            self._probe(role, recognizer, candidate)
            with self._query_lock:
                if self._profiles[role].revision != request.expected_revision:
                    raise ValueError('the active profile changed while the candidate was verified; retry')
                self._history[role].append(current)
                self._profiles[role] = candidate
        except (ValueError, RuntimeError, CvBridgeError, cv2.error, OSError) as error:
            # ProviderUnavailable is a RuntimeError: an unconfirmed candidate never replaces the active one.
            response.success, response.detail = False, f'profile not applied: {error}'
            self.get_logger().warning(f'{role} perception profile rejected: {error}')
            return response
        self._publish_profile()
        response.success = True
        response.detail = 'profile applies from the next inference; frames in flight keep their own'
        response.previous_revision = current.revision
        response.revision, response.profile_json = candidate.revision, canonical(candidate.document())
        self.get_logger().info(f'profile_applied role={role} profile={candidate.name} revision={candidate.revision[:12]} '
                               f'previous={current.revision[:12]}')
        return response

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
        if stamp > 0:
            # Stale arrivals count too: under saturation they are the distribution that matters.
            self._budget.record('sensor_age', age)
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
            self._received_at[camera] = time.monotonic()

    def _infer(self):
        cameras = list(self._bindings)
        with self._frame_lock:
            generation = self._generation
            for offset in range(len(cameras)):
                index = (self._next_camera + offset) % len(cameras)
                camera = cameras[index]
                image = self._frames.pop(camera, None)
                received = self._received_at.pop(camera, None)
                if image is not None:
                    self._next_camera = (index + 1) % len(cameras)
                    break
            else:
                return
        with self._query_lock:
            query, profiles = self._query, dict(self._profiles)
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        try:
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not 0 <= age <= self._age:
                raise ValueError('recognition source expired before inference')
            pixels = self._bridge.imgmsg_to_cv2(image, 'bgr8')
            start = time.monotonic()
            # The recognizer consumes the frame's evidence artifact, so its stored copy replays byte-exact.
            artifact = capture_artifact(pixels)
            (result, extras, run), role, degraded = self._recognize(
                camera, artifact_pixels(artifact), artifact, image.header, query, profiles)
            duration = time.monotonic() - start
            source_age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            with self._frame_lock:
                self._durations[camera].append(duration)
                self._stats[camera].update(inference_sec=duration, source_age_sec=source_age)
            self._record_stages(role, None if received is None else start - received, duration)
            if generation != self._generation:
                raise ValueError('clock changed during inference')
            if not self._budget.fresh(source_age):
                raise ValueError('recognition finished after source freshness expired')
            source = self._bridge.cv2_to_imgmsg(pixels, 'bgr8')
            source.header = image.header
            image_pub, detection_pub, operating_pub = self._outputs[camera][role]
            recognizer, frame_query = self._recognizers[role], query if role == 'primary' else None
            operating_pub.publish(PerceptionOperatingPoint(
                header=image.header, model_sha256=recognizer.sha256, profile_json=canonical(profiles[role].document()),
                query_json=canonical(frame_query.document()) if frame_query is not None else '',
                input_sha256=hashlib.sha256(artifact).hexdigest(), detections_json=canonical(extras),
                run_json=canonical(run) if run else '', degraded_reason=degraded))
            image_pub.publish(source)
            detection_pub.publish(result)
            with self._frame_lock:
                self._stats[camera]['published'] += 1
                self._stats[camera]['fallback_published'] += role == 'fallback'
                self._stats[camera]['served_by'] = role
                self._stats[camera]['published_stamp'] = stamp
                self._stats[camera]['query_revision'] = frame_query.revision if frame_query else ''
                self._stats[camera]['profile_revision'] = profiles[role].revision
                self._stats[camera]['reason'] = ''
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
            self._reject(camera, str(error))

    def _record_stages(self, role, queued, duration):
        """One sample per stage per frame. A remote answer splits into the provider's queue (added to the
        robot-side wait), its model time, and the network: the round trip the provider did not account for."""
        recognizer = self._detector if role == 'primary' else self._fallback
        remote = getattr(recognizer, 'last_remote', None)
        timings = remote[1] if remote else None
        if timings is None or timings['total'] > remote[0]:
            if queued is not None:
                self._budget.record('queue', queued)
            self._budget.record('inference', duration)
            return
        if queued is not None:
            self._budget.record('queue', queued + timings['decode'] + timings['queue'])
        self._budget.record('inference', timings['inference'])
        self._budget.record('network', remote[0] - timings['total'])

    def _recognize(self, camera, pixels, artifact, header, query, profiles):
        if self._fallback is None or time.monotonic() >= self._primary_retry_at:
            try:
                with self._model_locks['primary']:
                    result = detect(self._detector, pixels, header, query, profiles['primary'], artifact)
                with self._frame_lock:
                    self._stats[camera]['primary_reason'] = ''
                return result, 'primary', ''
            except ProviderUnavailable as error:
                if self._fallback is None:
                    raise
                self._primary_retry_at = time.monotonic() + PRIMARY_RETRY_SEC
                with self._frame_lock:
                    self._stats[camera]['primary_reason'] = str(error)
                self.get_logger().warning(f'{camera}: primary recognizer unavailable, serving fallback: {error}',
                                          throttle_duration_sec=30)
        with self._frame_lock:
            degraded = self._stats[camera]['primary_reason']
        with self._model_locks['fallback']:
            return detect(self._fallback, pixels, header, None, profiles['fallback'], artifact), 'fallback', degraded

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
        message.status.append(self._budget.status())
        self._diagnostics.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    # Receipt, inference and profile verification each own a thread.
    executor = MultiThreadedExecutor(num_threads=3)
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
