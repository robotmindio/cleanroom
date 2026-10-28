#!/usr/bin/env python3
"""Map revisions from RTAB-Map's optimized graph; anchoring and reprojection of captures.

Each optimized graph RTAB-Map publishes becomes a revision of a persistent map
with one anchor per graph node. Observations keep their capture-time
``T_map_camera`` and revision; this node anchors them to the nearest node and
projects them into any later revision of the same map.
"""

import json
from pathlib import Path
import sqlite3
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rtabmap_msgs.msg import Info, MapGraph
from std_msgs.msg import String

from lekiwi_rmf.object_recognition import reject_parameter_changes
from lekiwi_rmf.spatial_anchors import SCHEMA, SpatialError, SpatialStore, Transform, rtabmap_database_tag, valid_ref
from lekiwi_rmf.srv import QuerySpatial

MAX_ITEMS = 100
MAX_POINTS = 20
MAX_ANCHOR_PAGE = 5000


def parse_request(text):
    if len(text.encode()) > 65536:
        raise ValueError('spatial query exceeds 64 KiB')
    try:
        value = json.loads(text)
    except RecursionError as error:
        raise ValueError('spatial query is nested too deeply') from error
    if not isinstance(value, dict) or value.get('op') not in ('current', 'revision', 'anchor', 'project', 'maps'):
        raise ValueError('op must be current, revision, anchor, project or maps')
    allowed = {'current': {'op'}, 'maps': {'op'},
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
                    'max_revisions': 2000, 'graph_topic': '/mapGraph', 'info_topic': '/info'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        reject_parameter_changes(self, list(defaults))
        value = {name: self.get_parameter(name).value for name in defaults}
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
        self._revisions.publish(String(data=json.dumps(
            {**revision.to_dict(), 'session': self._store.session_state()}, allow_nan=False)))

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
