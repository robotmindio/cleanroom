#!/usr/bin/env python3
"""Map revisions from RTAB-Map's optimized graph; anchored captures and the object map.

Each optimized graph RTAB-Map publishes becomes a revision of a persistent map
with one anchor per graph node. Observations keep their capture-time
``T_map_camera`` and revision; this node anchors them to the nearest node and
projects them into any later revision of the same map. It also reads the
observation store behind a persisted cursor and keeps the object map
(lekiwi_rmf/object_map.py) in the same database.
"""

from collections import deque
import json
from pathlib import Path
import sqlite3
import time

from octomap_msgs.msg import Octomap
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import Info, MapGraph
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from lekiwi_rmf.coverage_search import Profile, camera_from_info, normalize_label
from lekiwi_rmf.object_cameras import camera_bindings, config_directory
from lekiwi_rmf.object_map import Absence, Limits, ObjectMap, parse_query
from lekiwi_rmf.object_recognition import reject_parameter_changes
from lekiwi_rmf.search_evidence import parse_profile
from lekiwi_rmf.search_octomap import MAX_MAP_BYTES, SearchOctomap
from lekiwi_rmf.spatial_anchors import SCHEMA, SpatialError, SpatialStore, Transform, rtabmap_database_tag, valid_ref
from lekiwi_rmf.srv import QueryObservations, QuerySpatial

MAX_ITEMS = 100
MAX_POINTS = 20
MAX_ANCHOR_PAGE = 5000
# Observations read per request, and the 3D maps kept for the captures still being read.
PAGE = 100
OCTOMAPS = 8


def absence_criteria(path):
    """The active search's evidence-of-absence criteria (config/exploration.yaml), so both agree on what
    an empty view proves; with them, the 3D map topic and how old a map a capture may be bound to."""
    value = yaml.safe_load(Path(path).read_text())['robot_explorer']['ros__parameters']
    profile = Profile(*(float(value[f'search_{name}']) for name in (
        'min_object_m', 'min_object_px', 'max_range_m', 'min_height_m', 'max_height_m', 'max_age_sec')),
        bool(value['search_require_stationary']))
    return profile, value['search_geometry_topic'], float(value['search_geometry_max_age_sec'])


def parse_request(text):
    if len(text.encode()) > 65536:
        raise ValueError('spatial query exceeds 64 KiB')
    try:
        value = json.loads(text)
    except RecursionError as error:
        raise ValueError('spatial query is nested too deeply') from error
    if not isinstance(value, dict) or value.get('op') not in ('current', 'pose', 'revision', 'anchor', 'project', 'maps'):
        raise ValueError('op must be current, pose, revision, anchor, project or maps')
    allowed = {'current': {'op'}, 'pose': {'op'}, 'maps': {'op'},
               'revision': {'op', 'revision', 'after_anchor', 'limit'},
               'anchor': {'op', 'items'}, 'project': {'op', 'items', 'target'}}[value['op']]
    if set(value) - allowed:
        raise ValueError(f'unknown fields for {value["op"]}')
    if value['op'] == 'revision':
        limit = value.get('limit', 1000)
        if type(limit) is not int or not 1 <= limit <= MAX_ANCHOR_PAGE:
            raise ValueError(f'limit must be an integer in 1..{MAX_ANCHOR_PAGE}')
        if not isinstance(value.get('after_anchor', ''), str) or len(value.get('after_anchor', '')) > 128:
            raise ValueError('after_anchor must be an anchor reference')
    if value['op'] in ('anchor', 'project'):
        items = value.get('items')
        if not isinstance(items, list) or not 1 <= len(items) <= MAX_ITEMS:
            raise ValueError(f'items requires 1..{MAX_ITEMS} captures')
        for item in items:
            if (not isinstance(item, dict) or set(item) - {'id', 'map_pose', 'points', 'uncertainties_m'}
                    or not isinstance(item.get('map_pose'), dict)
                    or not isinstance(item.get('points', []), list) or len(item.get('points', [])) > MAX_POINTS
                    or len(item.get('uncertainties_m', [])) not in (0, len(item.get('points', [])))):
                raise ValueError('each item is {id, map_pose, points[], uncertainties_m[]}')
        target = value.get('target')
        if target is not None and (not isinstance(target, dict) or set(target) != {'map_ref', 'revision'}):
            raise ValueError('target is {map_ref, revision}')
    return value


class SpatialMemory(Node):
    def __init__(self, **kwargs):
        super().__init__('spatial_memory', **kwargs)
        suffix = f'_{self.context.get_domain_id()}'
        if self.get_parameter('use_sim_time').value:
            suffix += '_sim'
        defaults = {'database_path': str(Path.home() / f'.ros/lekiwi/spatial{suffix}.sqlite3'),
                    'rtabmap_database': str(Path.home() / '.ros/lekiwi_rtabmap_floor_v2.db'),
                    'site_ref': '', 'observer_ref': 'lekiwi', 'retention_sec': 7 * 86400.0,
                    'max_revisions': 2000, 'graph_topic': '/mapGraph', 'info_topic': '/info',
                    'base_frame': 'base_footprint', 'pose_max_age_sec': 1.0,
                    'objects_enabled': True, 'objects_tick_sec': 0.5, 'objects_tick_budget_sec': 0.05,
                    'objects_absence_config': '',
                    **{f'objects_{name}': field.default for name, field in Limits.__dataclass_fields__.items()}}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        reject_parameter_changes(self, list(defaults))
        value = {name: self.get_parameter(name).value for name in defaults}
        if (not 0 < value['pose_max_age_sec'] <= 10
                or not 0 < value['objects_tick_budget_sec'] <= value['objects_tick_sec']):
            raise ValueError('pose_max_age_sec must be within (0,10] and the object tick budget within its period')
        if not valid_ref(value['site_ref']):
            raise ValueError('site_ref must name the deployment site (config/spatial_memory.yaml)')
        self._rtabmap_database = value['rtabmap_database']
        self._store = SpatialStore(value['database_path'], site_ref=value['site_ref'],
                                   observer_ref=value['observer_ref'], retention_sec=value['retention_sec'],
                                   max_revisions=value['max_revisions'])
        self._store.start_session(self.get_clock().now().nanoseconds / 1e9)
        self._tag, self._tag_checked = None, 0.0
        self._reason = 'waiting for an optimized RTAB-Map graph'
        self._loop_closure = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._revisions = self.create_publisher(String, '/robot/spatial/revision', latched)
        self.create_subscription(MapGraph, value['graph_topic'], self._graph, 1)
        self.create_subscription(Info, value['info_topic'], self._info, 10)
        self.create_service(QuerySpatial, '/robot/spatial/query', self._query)
        self.create_timer(5.0, self._refresh_tag, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._refresh_tag()
        self._base_frame, self._pose_age = value['base_frame'], value['pose_max_age_sec']
        self._transforms = Buffer()
        self._listener = TransformListener(self._transforms, self)
        self._objects = ObjectMap(self._store, Limits(**{
            name: value[f'objects_{name}'] for name in Limits.__dataclass_fields__}))
        # Graph stamps and the revision each solved, to bind a 3D map to the revision it was built in.
        self._graph_stamps = deque(maxlen=64)
        self.object_counts = {}
        # The same JSON-over-service type as the spatial query; only the request vocabulary differs.
        self.create_service(QuerySpatial, '/robot/objects/query', self._query_objects)
        self._ingesting = value['objects_enabled']
        if self._ingesting:
            self._observe(value)

    # ── object map ───────────────────────────────────────────────────────────

    def _observe(self, value):
        profile, topic, self._octomap_age = absence_criteria(
            value['objects_absence_config'] or config_directory() / 'exploration.yaml')
        self._absence = Absence(profile, {})
        self._budget = value['objects_tick_budget_sec']
        self._bindings, self._intrinsics, self._profile_at = {}, {}, None
        self._octomaps, self._octree = deque(maxlen=OCTOMAPS), (None, None)
        self._cursor = self._objects.cursor()
        self._future, self._requested = None, 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/perception/profile', self._on_profile, latched)
        self.create_subscription(Octomap, topic, self._on_octomap, QoSProfile(depth=1))
        self._observations = self.create_client(QueryObservations, '/robot/observations/query')
        self.create_timer(value['objects_tick_sec'], self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _count(self, reason, amount=1):
        if amount:
            self.object_counts[reason] = self.object_counts.get(reason, 0) + amount

    def _on_profile(self, message):
        """The detector's operating point: which cameras observe and which models may prove absence."""
        try:
            profile = parse_profile(message.data)
            bindings = camera_bindings(sorted(profile['cameras']))
            if any(bindings[name]['sha256'] != sha for name, sha in profile['cameras'].items()):
                raise ValueError('perception cameras differ from the tracked bindings')
        except (ValueError, TypeError, KeyError, RecursionError, OSError) as error:
            self.get_logger().error(f'perception profile rejected: {error}', throttle_duration_sec=10)
            self._absence = Absence(self._absence.profile, {})
            return
        self._absence = Absence(self._absence.profile, {
            item['model_sha256']: (item['profile']['revision'],
                                   frozenset(normalize_label(label) for label in item['labels']))
            for item in profile['recognizers']})
        for name, binding in bindings.items():
            if name not in self._bindings:
                self.create_subscription(CameraInfo, binding['camera_info_topic'],
                                         lambda info, camera=name: self._on_info(camera, info), qos_profile_sensor_data)
        self._bindings = bindings
        if self._profile_at is None:
            self._profile_at = time.monotonic()

    def _on_info(self, camera, message):
        try:
            if message.header.frame_id != self._bindings[camera]['optical_frame']:
                raise ValueError('camera_info frame differs from the tracked binding')
            self._intrinsics[camera] = camera_from_info(message.k, message.width, message.height)
        except (ValueError, KeyError):
            self._count('invalid_camera_info')

    def _on_octomap(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if (message.header.frame_id != 'map' or not message.binary or message.id not in ('OcTree', 'ColorOcTree')
                or stamp <= 0 or len(message.data) > MAX_MAP_BYTES):
            self._count('invalid_3d_geometry')
            return
        # Decoded only when a capture needs it; the callback just keeps the bytes.
        self._octomaps.append((stamp, memoryview(message.data).cast('B').tobytes(), message.resolution))

    def _octomap(self, record):
        """The newest 3D map built in the capture's revision shortly before the capture, or None."""
        stamp = record['stamp']['sec'] + record['stamp']['nanosec'] / 1e9
        revision = (record.get('map_pose') or {}).get('revision')
        solved = [graph_stamp for graph_stamp, name in self._graph_stamps if name == revision]
        for built, payload, resolution in reversed(self._octomaps):
            # RTAB-Map stamps a graph and the 3D map of that solution identically.
            if 0 <= stamp - built <= self._octomap_age and any(abs(built - value) < 1e-6 for value in solved):
                if self._octree[0] != built:
                    try:
                        self._octree = (built, SearchOctomap.from_binary(payload, resolution))
                    except ValueError:
                        self._count('invalid_3d_geometry')
                        return None
                return self._octree[1]
        return None

    def _poll(self):
        now = time.monotonic()
        if self._future is not None:
            if not self._future.done() and now - self._requested < 3.0:
                return
            if not self._future.done():
                self._observations.remove_pending_request(self._future)
                self._future.cancel()
            self._future = None
        if self._profile_at is None or not self._observations.service_is_ready():
            return
        # Wait for the intrinsics of every camera in force, but not forever: a silent camera loses only its own captures.
        if any(name not in self._intrinsics for name in self._bindings) and now - self._profile_at < 10.0:
            return
        request = {'limit': PAGE} if self._cursor is None else {
            'after_sequence': self._cursor[1], 'limit': PAGE, 'epoch': self._cursor[0]}
        self._future = self._observations.call_async(QueryObservations.Request(request_json=json.dumps(request)))
        self._requested = now
        self._future.add_done_callback(self._on_observations)

    def _on_observations(self, future):
        self._future = None
        if future.cancelled():
            return
        try:
            response = future.result()
            if not response.success:
                if 'epoch changed' in response.detail:
                    self._cursor = None  # A new observation store is read from its start.
                raise ValueError(response.detail)
            result = json.loads(response.response_json)
            epoch, evicted = result['epoch'], result.get('evicted_sequence', 0)
            sequence = self._cursor[1] if self._cursor is not None else 0
            deadline = time.monotonic() + self._budget
            for record in result['observations']:
                # Each record's entries and the cursor commit together, so a restart never counts one twice.
                try:
                    counts = self._objects.ingest(record, self._intrinsics.get(record.get('camera_id')),
                                                  self._octomap(record), epoch=epoch, evicted=evicted,
                                                  absence=self._absence)
                except (ValueError, KeyError, TypeError, IndexError):
                    # Its transaction rolled back; one unreadable record must not hold the cursor forever.
                    counts = {'skipped': 'malformed_record'}
                self._count(counts.pop('skipped', None) or 'ingested')
                for reason, amount in counts.items():
                    self._count(reason, amount)
                sequence = record['sequence']
                if time.monotonic() >= deadline:
                    break  # The rest of the page is read again on the next tick.
            if result.get('cursor_gap'):
                self._count('cursor_gap')
            self._cursor = (epoch, sequence, evicted)
            self._objects.set_cursor(*self._cursor)
        except (ValueError, KeyError, TypeError, RecursionError, sqlite3.Error) as error:
            self.get_logger().warning(f'object map ingestion failed: {error}', throttle_duration_sec=10)

    def _query_objects(self, request, response):
        try:
            result = self._objects.query(parse_query(request.request_json))
            cursor = self._objects.cursor()
            # How far the map has read the observation store, so a caller can tell a stale map from an empty one.
            result['ingestion'] = {'enabled': self._ingesting, 'observation_sequence': cursor[1] if cursor else 0,
                                   'counts': dict(self.object_counts)}
            response.response_json = json.dumps(result, allow_nan=False)
            response.success = True
        except (ValueError, TypeError, sqlite3.Error) as error:
            response.detail = str(error)
        return response

    def _pose(self):
        """Where the robot is now, in the newest revision of the map its session is localized in."""
        current = self._current()
        if not current['ready']:
            raise ValueError('localization unavailable: '
                             + (current['reason'] or 'this session has not localized in a known map'))
        try:
            transform = self._transforms.lookup_transform('map', self._base_frame, Time())
        except TransformException as error:
            raise ValueError(f'localization unavailable: no transform from map to {self._base_frame}') from error
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not -0.1 <= age <= self._pose_age:
            raise ValueError(f'localization unavailable: the map transform is {age:.2f} s old')
        t, q = transform.transform.translation, transform.transform.rotation
        pose = Transform.create([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        revision = self._store.revision(current['revision']['revision'])
        return {'schema': SCHEMA, 'site_ref': self._store.site_ref, 'map_ref': revision.map_ref,
                'revision': revision.revision, 'anchor_ref': self._store.nearest_anchor(revision, pose.t)[0],
                'localization_session': current['session']['session'], 'frame_id': 'map',
                'child_frame_id': self._base_frame, 'pose': pose.to_dict(), 'stamp_unix_sec': stamp}

    def _refresh_tag(self, force=False):
        now = time.monotonic()
        if not force and self._tag is not None and now - self._tag_checked < 30:
            return self._tag
        try:
            tag = rtabmap_database_tag(self._rtabmap_database)
        except SpatialError as error:
            # Keep the last identity while the database is briefly locked; refuse to anchor without one.
            if self._tag is None:
                self._reason = str(error)
            return self._tag
        if self._tag is not None and tag != self._tag:
            self.get_logger().warning('RTAB-Map database identity changed; its nodes start a new map')
        self._tag, self._tag_checked = tag, now
        return tag

    def _info(self, message):
        if message.loop_closure_id or message.proximity_detection_id:
            self._loop_closure = True

    def _graph(self, message):
        tag = self._refresh_tag()
        if tag is None:
            self.get_logger().warning(f'graph ignored: {self._reason}', throttle_duration_sec=10)
            return
        if len(message.poses_id) != len(message.poses) or not message.poses_id:
            self._reason = 'graph without node poses'
            return
        try:
            poses = {f'rtabmap:{tag}:{node}': Transform.create(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                for node, pose in zip(message.poses_id, message.poses)}
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            created = stamp if stamp > 0 else self.get_clock().now().nanoseconds / 1e9
            revision = self._store.ingest(poses, created_at=created,
                                          cause='loop_closure' if self._loop_closure else 'optimization')
        except (ValueError, sqlite3.Error) as error:
            self._reason = f'graph rejected: {error}'
            self.get_logger().error(self._reason, throttle_duration_sec=10)
            return
        self._loop_closure = False
        self._reason = ''
        self._graph_stamps.append((created, revision.revision))
        self._revisions.publish(String(data=json.dumps(
            {**revision.to_dict(), 'session': self._store.session_state(),
             'graph_stamp_unix_sec': created}, allow_nan=False)))

    def _current(self):
        latest = self._store.latest()
        session = self._store.session_state()
        current = None
        if latest is not None and session['map_ref'] is not None:
            current = self._store.latest(self._store.surviving_map(session['map_ref']))
        return {'schema': SCHEMA, 'ready': current is not None, 'reason': self._reason if current is None else '',
                'site_ref': self._store.site_ref, 'observer_ref': self._store.observer_ref, 'session': session,
                'revision': current.to_dict() if current else None}

    def _items(self, request, method):
        results = []
        for item in request['items']:
            try:
                results.append({'id': item.get('id'), 'ok': True, **method(item)})
            except SpatialError as error:
                results.append({'id': item.get('id'), 'ok': False, 'code': error.code, 'detail': str(error)})
            except (ValueError, TypeError) as error:
                results.append({'id': item.get('id'), 'ok': False, 'code': 'schema_incompatible', 'detail': str(error)})
        return results

    def _query(self, request, response):
        try:
            value = parse_request(request.request_json)
            if value['op'] == 'current':
                result = self._current()
            elif value['op'] == 'pose':
                result = self._pose()
            elif value['op'] == 'maps':
                result = {'schema': SCHEMA, 'site_ref': self._store.site_ref, 'maps': self._store.maps(),
                          'sessions': self._store.sessions(), 'alignments': self._store.alignments()}
            elif value['op'] == 'revision':
                revision = (self._store.revision(value['revision']) if 'revision' in value
                            else self._store.latest())
                if revision is None:
                    raise ValueError('revision unknown or pruned')
                anchors, truncated = self._store.anchors(revision, after=value.get('after_anchor', ''),
                                                         limit=value.get('limit', 1000))
                result = {**revision.to_dict(), 'anchors': anchors, 'truncated': truncated,
                          'next_anchor': anchors[-1]['anchor_ref'] if truncated else None}
            elif value['op'] == 'anchor':
                result = {'schema': SCHEMA, 'items': self._items(value, lambda item: self._store.anchor(
                    item['map_pose'], item.get('points', []), item.get('uncertainties_m', [0.0] * len(item.get('points', [])))))}
            else:
                result = {'schema': SCHEMA, 'items': self._items(value, lambda item: self._store.project(
                    item['map_pose'], item.get('points', []), value.get('target')))}
            response.response_json = json.dumps(result, allow_nan=False)
            response.success = True
        except (ValueError, TypeError, sqlite3.Error) as error:
            response.detail = str(error)
        return response

    def destroy_node(self):
        self._store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpatialMemory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
