"""Posed captures from the observation store, while TF still holds each capture's time."""

from collections import deque
import json
import math
import threading
import time

from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import TransformException

from lekiwi_rmf.coverage_search import (
    Capture, Mount, camera_from_info, detections_from_record, pose_from_transform,
)
from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.object_observations import valid_id
from lekiwi_rmf.srv import QueryObservations

PAGE = 100


def parse_profile(text):
    """The detector's operating point; anything malformed means no instrument is in force."""
    if len(text.encode()) > 65536:
        raise ValueError('perception profile exceeds 64 KiB')
    value = json.loads(text)
    recognizers = value.get('recognizers') if isinstance(value, dict) else None
    if (not isinstance(value, dict) or value.get('schema') != 1
            or not isinstance(value.get('cameras'), dict) or not value['cameras']
            or any(not valid_id(sha) for sha in value['cameras'].values())
            or not isinstance(recognizers, list) or not 1 <= len(recognizers) <= 4
            or len({item.get('model_sha256') for item in recognizers if isinstance(item, dict)}) != len(recognizers)):
        raise ValueError('invalid perception profile')
    for item in recognizers:
        if (not isinstance(item, dict) or item.get('role') not in ('primary', 'fallback')
                or not valid_id(item.get('model_sha256')) or type(item.get('open_vocabulary')) is not bool
                or not isinstance(item.get('labels'), list) or len(item['labels']) > 4096
                or any(not isinstance(label, str) or not 1 <= len(label) <= 128 for label in item['labels'])):
            raise ValueError('invalid perception recognizer')
    return value


def models(profile):
    return {item['model_sha256'] for item in profile['recognizers']}


class SearchEvidence:
    """Bounded log of captures with their map pose; the log is the search's only memory."""

    def __init__(self, node, transforms, callback_group, capacity):
        self._node, self._tf, self._group = node, transforms, callback_group
        self._lock = threading.Lock()
        self.profile = None
        self.profile_reason = 'waiting for /robot/perception/profile'
        self._bindings = {}
        self._intrinsics = {}
        self._map_context = {}
        self._captures = deque(maxlen=capacity)
        self._serial = 0
        self._cursor = self._epoch = None
        self._future, self._requested = None, 0.0
        self._last_capture = 0.0
        self.rejected = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/robot/perception/profile', self._on_profile, latched,
                                 callback_group=callback_group)
        node.create_subscription(String, '/robot/map_context', self._on_map_context, latched,
                                 callback_group=callback_group)
        self._query = node.create_client(QueryObservations, '/robot/observations/query', callback_group=callback_group)
        node.create_timer(0.5, self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=callback_group)

    def _on_profile(self, message):
        try:
            profile = parse_profile(message.data)
            bindings = camera_bindings(sorted(profile['cameras']))
            if any(bindings[name]['sha256'] != sha for name, sha in profile['cameras'].items()):
                raise ValueError('perception cameras differ from the tracked bindings')
        except (ValueError, TypeError, RecursionError, OSError) as error:
            with self._lock:
                self.profile, self.profile_reason = None, f'perception profile rejected: {error}'
            self._node.get_logger().error(self.profile_reason)
            return
        with self._lock:
            if self.profile is not None and models(self.profile) != models(profile):
                self._captures.clear()  # Evidence under another operating point is not this instrument's.
            self.profile, self.profile_reason = profile, ''
            for name, binding in bindings.items():
                if name not in self._bindings:
                    self._node.create_subscription(
                        CameraInfo, binding['camera_info_topic'],
                        lambda msg, camera=name: self._on_info(camera, msg), qos_profile_sensor_data,
                        callback_group=self._group)
            self._bindings = bindings

    def _on_info(self, camera, message):
        try:
            if message.header.frame_id != self._bindings[camera]['optical_frame']:
                raise ValueError('camera_info frame differs from the tracked binding')
            model = camera_from_info(message.k, message.width, message.height)
        except (ValueError, KeyError):
            self._reject('invalid_camera_info')
            return
        with self._lock:
            self._intrinsics[camera] = model

    def _on_map_context(self, message):
        try:
            value = json.loads(message.data)
            self._map_context = value if isinstance(value, dict) and value.get('ready') is True else {}
        except (ValueError, RecursionError):
            self._map_context = {}

    def _reject(self, reason):
        with self._lock:
            self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def _poll(self):
        now = time.monotonic()
        if self._future is not None:
            if not self._future.done() and now - self._requested < 3.0:
                return
            if not self._future.done():
                self._query.remove_pending_request(self._future)
                self._future.cancel()
            self._future = None
        if self.profile is None or not self._query.service_is_ready():
            return
        request = {'limit': 1} if self._cursor is None else {
            'after_sequence': self._cursor, 'limit': PAGE, 'epoch': self._epoch}
        self._future = self._query.call_async(QueryObservations.Request(request_json=json.dumps(request)))
        self._requested = now
        self._future.add_done_callback(self._on_query)

    def _on_query(self, future):
        if future.cancelled():
            return
        try:
            response = future.result()
            if not response.success:
                if 'epoch changed' in response.detail:
                    self._cursor = self._epoch = None
                raise ValueError(response.detail)
            result = json.loads(response.response_json)
        except (ValueError, RecursionError) as error:
            self._node.get_logger().warning(f'observation query failed: {error}', throttle_duration_sec=10)
            return
        finally:
            self._future = None
        if self._cursor is None:
            # Start at the newest bounded tail; older captures no longer have capture-time TF.
            self._epoch = result['epoch']
            self._cursor = max(0, result['latest_sequence'] - self._captures.maxlen)
            return
        for record in result['observations']:
            self._ingest(record)
        self._cursor = result['next_sequence']

    def _ingest(self, record):
        profile = self.profile
        camera_id = record.get('camera_id')
        if profile is None or record.get('model_sha256') not in models(profile):
            self._reject('other_model')
            return
        with self._lock:
            camera = self._intrinsics.get(camera_id)
            binding = self._bindings.get(camera_id)
        if camera is None or binding is None or record.get('frame_id') != binding['optical_frame']:
            self._reject('no_intrinsics_or_binding')
            return
        stamp = record['stamp']['sec'] + record['stamp']['nanosec'] / 1e9
        pose = None
        try:
            transform = self._tf.lookup_transform('map', record['frame_id'],
                                                  Time(seconds=record['stamp']['sec'],
                                                       nanoseconds=record['stamp']['nanosec']))
            t, q = transform.transform.translation, transform.transform.rotation
            pose = pose_from_transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        except (TransformException, ValueError):
            self._reject('no_capture_time_pose')
        map_id = self._map_context.get('map_id')
        detections = detections_from_record(record, camera, map_id)
        if pose is None and not any(item.map_xyz for item in detections):
            return  # Without a pose a capture proves nothing about any cell.
        query = record.get('query') if record.get('vocabulary') == 'open' else None
        labels = query.get('labels') if isinstance(query, dict) else None
        if record.get('vocabulary') == 'open' and (
                not isinstance(labels, list) or not all(isinstance(label, str) for label in labels)):
            self._reject('invalid_query')
            return
        capture = Capture(record['id'], camera_id, record['model_sha256'], stamp, pose, camera,
                          record.get('stationary') is True, detections, None if labels is None else tuple(labels))
        with self._lock:
            self._serial += 1
            self._captures.append((self._serial, capture))
            self._last_capture = time.monotonic()

    def since(self, serial):
        with self._lock:
            return [item for item in self._captures if item[0] > serial]

    def fresh(self, timeout):
        return time.monotonic() - self._last_capture <= timeout

    def mounts(self):
        """Base-fixed cameras with their current optical pose relative to the base footprint."""
        result = []
        with self._lock:
            cameras = [(name, binding, self._intrinsics.get(name)) for name, binding in self._bindings.items()]
        for name, binding, camera in cameras:
            if binding['mount'] != 'base' or camera is None:
                continue
            try:
                transform = self._tf.lookup_transform('base_footprint', binding['optical_frame'], Time())
            except TransformException:
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            try:
                relative = pose_from_transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            except ValueError:
                continue
            if math.isfinite(relative.z):
                result.append(Mount(name, camera, relative.z, relative.pitch, relative.yaw))
        return result
