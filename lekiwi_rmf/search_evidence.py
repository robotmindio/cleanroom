"""Posed captures from the observation store, kept in the newest revision of the map.

A capture that recorded its capture-time map pose is anchored by spatial memory
and projected into the current map revision; when SLAM revises the map, every
anchored capture in the log is projected again and the log's generation
advances, so coverage built from it moves with the corrected map. A capture
without a recorded map pose falls back to TF while TF still holds its time and
is dropped at the next revision, because nothing can place it there.
"""

from collections import deque
from dataclasses import replace
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
from lekiwi_rmf.spatial_anchors import Transform
from lekiwi_rmf.spatial_anchors import SCHEMA as SPATIAL_SCHEMA
from lekiwi_rmf.srv import QueryObservations, QuerySpatial

PAGE = 100
# Captures per spatial projection request; the service accepts up to 100.
PROJECT_BATCH = 100


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
        # Re-entrant: projection callbacks rebuild captures while holding the log.
        self._lock = threading.RLock()
        self.profile = None
        self.profile_reason = 'waiting for /robot/perception/profile'
        self._bindings = {}
        self._intrinsics = {}
        self._map_context = {}
        self._captures = deque(maxlen=capacity)
        self._anchors = {}
        self._serial = 0
        self._cursor = self._epoch = None
        self._future, self._requested = None, 0.0
        self._last_capture = 0.0
        self.rejected = {}
        # Spatial memory: the revision captures are expressed in, and a log generation per reprojection.
        self.revision = None
        self.generation = 0
        self._target = None
        self._reprojection = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/robot/perception/profile', self._on_profile, latched,
                                 callback_group=callback_group)
        node.create_subscription(String, '/robot/map_context', self._on_map_context, latched,
                                 callback_group=callback_group)
        node.create_subscription(String, '/robot/spatial/revision', self._on_revision, latched,
                                 callback_group=callback_group)
        self._query = node.create_client(QueryObservations, '/robot/observations/query', callback_group=callback_group)
        self._spatial = node.create_client(QuerySpatial, '/robot/spatial/query', callback_group=callback_group)
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

    def _on_revision(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or value.get('schema') != SPATIAL_SCHEMA:
                raise ValueError('invalid spatial revision')
            target = {'map_ref': value['map_ref'], 'revision': value['revision']}
        except (ValueError, KeyError, TypeError, RecursionError):
            self._reject('invalid_spatial_revision')
            return
        with self._lock:
            if target != self._target:
                self._target = target
                # Reproject the whole log under the new revision before more captures are added.
                self._reprojection = None

    def _reject(self, reason):
        with self._lock:
            self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def _call(self, client, request, done):
        self._future = client.call_async(request)
        self._requested = time.monotonic()
        self._future.add_done_callback(done)

    def _poll(self):
        now = time.monotonic()
        if self._future is not None:
            if not self._future.done() and now - self._requested < 3.0:
                return
            if not self._future.done():
                for client in (self._query, self._spatial):
                    client.remove_pending_request(self._future)
                self._future.cancel()
            self._future = None
        with self._lock:
            stale = self._target is not None and self.revision != self._target
            if stale and not self._captures:
                self.revision, stale = self._target, False  # Nothing to move; new captures project into it.
            if stale and self._reprojection is None:
                anchored = [item for item in self._captures if item[1].observation_id in self._anchors]
                self._reprojection = {'target': self._target, 'pending': anchored, 'done': []}
        if stale and self._reprojection is not None:
            if self._spatial.service_is_ready():
                self._reproject_next()
            return
        if self.profile is None or not self._query.service_is_ready():
            return
        request = {'limit': 1} if self._cursor is None else {
            'after_sequence': self._cursor, 'limit': PAGE, 'epoch': self._epoch}
        self._call(self._query, QueryObservations.Request(request_json=json.dumps(request)), self._on_query)

    def _project(self, entries, target, done):
        items = [{'id': record['id'], 'map_pose': record['map_pose'], 'points': points}
                 for record, points in entries]
        request = {'op': 'project', 'items': items}
        if target is not None:
            request['target'] = target
        self._call(self._spatial, QuerySpatial.Request(request_json=json.dumps(request)), done)

    def _reproject_next(self):
        job = self._reprojection
        batch = job['pending'][:PROJECT_BATCH]
        entries = [self._anchors[capture.observation_id] for _, capture in batch]

        def done(future):
            results = self._projections(future)
            self._future = None
            if results is None:
                return  # Retried on the next poll with the same batch.
            with self._lock:
                if self._reprojection is not job:
                    return
                for (serial, capture), result in zip(batch, results):
                    moved = self._capture(*self._anchors[capture.observation_id], result)
                    if moved is not None:
                        job['done'].append((serial, moved))
                job['pending'] = job['pending'][len(batch):]
                if job['pending']:
                    return
                # Unanchored captures cannot be placed in the new revision; they leave the log.
                dropped = len(self._captures) - len(job['done'])
                self.rejected['unanchored_after_revision'] = self.rejected.get('unanchored_after_revision', 0) + dropped
                self._captures.clear()
                self._captures.extend(job['done'])
                self.revision, self._reprojection = job['target'], None
                self.generation += 1
        if not batch:
            with self._lock:
                self._captures.clear()
                self.revision, self._reprojection = job['target'], None
                self.generation += 1
            return
        self._project(entries, job['target'], done)

    def _projections(self, future):
        if future.cancelled():
            return None
        try:
            response = future.result()
            if not response.success:
                raise ValueError(response.detail)
            return json.loads(response.response_json)['items']
        except (ValueError, KeyError, RecursionError) as error:
            self._node.get_logger().warning(f'spatial projection failed: {error}', throttle_duration_sec=10)
            return None

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
            # Start at the newest bounded tail; older captures may predate the current map revision.
            self._epoch = result['epoch']
            self._cursor = max(0, result['latest_sequence'] - self._captures.maxlen)
            return
        entries = [entry for entry in (self._prepare(record) for record in result['observations']) if entry]
        anchored = [entry for entry in entries if entry[0].get('map_pose')]
        if anchored and self._spatial.service_is_ready():
            with self._lock:
                target = self._target

            def done(projected_future):
                results = self._projections(projected_future)
                self._future = None
                if results is None:
                    return  # The cursor did not advance; the page is fetched again.
                by_id = {record['id']: item for (record, _), item in zip(anchored, results)}
                self._admit(entries, by_id)
                self._cursor = result['next_sequence']
            self._project(anchored, target, done)
            return
        if anchored:
            self._reject('spatial_memory_unavailable')
        self._admit(entries, {})
        self._cursor = result['next_sequence']

    def _prepare(self, record):
        """Validate one record; return it with its detections' capture-frame points."""
        profile = self.profile
        if profile is None or record.get('model_sha256') not in models(profile):
            self._reject('other_model')
            return None
        with self._lock:
            binding = self._bindings.get(record.get('camera_id'))
        if binding is None or record.get('frame_id') != binding['optical_frame']:
            self._reject('no_intrinsics_or_binding')
            return None
        points = []
        for item in record.get('detections', []):
            position = item.get('position') or {}
            usable = position.get('mode') == 'camera' and position.get('frame_id') == record['frame_id']
            points.append(position['xyz'] if usable else None)
        return record, points

    def _admit(self, entries, projections):
        for record, points in entries:
            projected = projections.get(record['id'])
            if projected is not None:
                capture = self._capture(record, points, projected)
                if capture is not None:
                    with self._lock:
                        self._anchors[record['id']] = (record, points)
                    self._append(capture)
            else:
                capture = self._tf_capture(record)
                if capture is not None:
                    self._append(capture)

    def _append(self, capture):
        with self._lock:
            self._serial += 1
            self._captures.append((self._serial, capture))
            # Bounded with the log: anchors of captures that left it are not kept.
            if len(self._anchors) > 2 * self._captures.maxlen:
                live = {item.observation_id for _, item in self._captures}
                self._anchors = {key: value for key, value in self._anchors.items() if key in live}
            self._last_capture = time.monotonic()

    def _capture(self, record, points, projected):
        """A capture placed by spatial memory in the projection's revision."""
        with self._lock:
            camera = self._intrinsics.get(record['camera_id'])
        if camera is None or not projected.get('ok'):
            self._reject(projected.get('code', 'no_intrinsics') if camera is not None else 'no_intrinsics_or_binding')
            return None
        pose = projected['camera_pose']
        try:
            camera_pose = pose_from_transform(pose['t'], pose['q'])
            moved = Transform.from_dict(pose)
        except ValueError:
            self._reject('invalid_projection')
            return None
        # Detections with a capture-frame point land where the projected camera puts them.
        detections = tuple(
            replace(detection, map_xyz=moved.apply(point), map_uncertainty_m=float(
                (record['detections'][index].get('position') or {}).get('uncertainty_m', 0.0)))
            if point is not None else replace(detection, map_xyz=None, map_uncertainty_m=0.0)
            for index, (detection, point) in enumerate(zip(detections_from_record(record, camera), points)))
        stamp = record['stamp']['sec'] + record['stamp']['nanosec'] / 1e9
        return Capture(record['id'], record['camera_id'], record['model_sha256'], stamp, camera_pose, camera,
                       record.get('stationary') is True, detections)

    def _tf_capture(self, record):
        """Fallback for a record without a capture-time map pose: TF at its stamp, unanchored."""
        with self._lock:
            camera = self._intrinsics.get(record['camera_id'])
        if camera is None:
            self._reject('no_intrinsics_or_binding')
            return None
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
            return None  # Without a pose a capture proves nothing about any cell.
        self._reject('unanchored_capture')
        return Capture(record['id'], record['camera_id'], record['model_sha256'], stamp, pose, camera,
                       record.get('stationary') is True, detections)

    def snapshot(self):
        """Log generation, its map revision and every capture, read together."""
        with self._lock:
            return self.generation, (self.revision or {}).get('revision'), list(self._captures)

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
