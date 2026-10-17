#!/usr/bin/env python3
"""Collect any standard ROS 2 detector's exactly synchronized image observations."""

import base64
import json
import math
from pathlib import Path
import sqlite3
import time

import message_filters
from cv_bridge import CvBridgeError
import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from lekiwi_rmf.object_observations import ObservationStore, observation, valid_id
from lekiwi_rmf.object_geometry import ObjectGeometry
from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.observation_motion import ObservationMotion
from lekiwi_rmf.object_recognition import recognition_identity
from lekiwi_rmf.srv import GetObservationEvidence, QueryObservations, PinObservationEvidence


class ObjectObservations(Node):
    def __init__(self, **kwargs):
        super().__init__('object_observations', **kwargs)
        suffix = f'_{self.context.get_domain_id()}'
        if self.get_parameter('use_sim_time').value:
            suffix += '_sim'
        self.declare_parameter('database_path', str(Path.home() / f'.ros/lekiwi/observations{suffix}.sqlite3'))
        for name, value in {'camera_ids': ['front'], 'camera_profiles_file': '', 'model_sha256': '', 'max_source_age_sec': 2.0,
                            'max_records': 100000, 'max_bytes': 2 * 1024**3, 'retention_sec': 7 * 86400.0}.items():
            self.declare_parameter(name, value)
        self._bindings = camera_bindings(self.get_parameter('camera_ids').value,
                                        self.get_parameter('camera_profiles_file').value or None)
        self.declare_parameter('model_name', 'yolox')
        self.declare_parameter('confidence_threshold', .5)
        self._model = self.get_parameter('model_sha256').value or recognition_identity(
            self.get_parameter('model_name').value, self.get_parameter('confidence_threshold').value)[0]
        self._transforms = Buffer()
        self._listener = TransformListener(self._transforms, self)
        self._map_context = {'ready': False}
        self.create_subscription(String, '/robot/map_context', self._map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._motion = ObservationMotion()
        self.create_subscription(Odometry, '/wheel/odometry', self._motion.odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._motion.joint, qos_profile_sensor_data)
        self._age = self.get_parameter('max_source_age_sec').value
        if not 0 < self._age <= 10:
            raise ValueError('max_source_age_sec must be within (0,10]')
        # Retain the tracked 30-Hz depth stream throughout the inference freshness window.
        cache_size = math.ceil(self._age * 30) + 4
        self._streams = {}
        for name, binding in self._bindings.items():
            geometry = ObjectGeometry(binding['calibration_file'])
            if geometry.calibration and (geometry.calibration['camera_id'] != name or
                                         geometry.calibration['optical_frame'] != binding['optical_frame']):
                raise ValueError('calibration identity differs from tracked camera binding')
            if (geometry.sha256 or '') != binding['calibration_sha256']:
                raise ValueError('camera calibration changed during startup')
            stream = {'last': 0., 'stamp': 0, 'reason': 'waiting for synchronized image/detections',
                      'geometry': geometry, 'depth': None, 'info': None}
            if binding['depth_topic']:
                for key, kind, topic, size in [('depth', Image, binding['depth_topic'], cache_size),
                                               ('info', CameraInfo, binding['camera_info_topic'], cache_size)]:
                    subscriber = message_filters.Subscriber(self, kind, topic, qos_profile=qos_profile_sensor_data)
                    stream[key] = message_filters.Cache(subscriber, size)
            self._streams[name] = stream
        if not valid_id(self._model):
            raise ValueError('configured model SHA-256 and camera identity are required')
        self._store = ObservationStore(self.get_parameter('database_path').value,
            max_records=self.get_parameter('max_records').value, max_bytes=self.get_parameter('max_bytes').value,
            retention_sec=self.get_parameter('retention_sec').value)
        self._events = self.create_publisher(String, '/robot/observations/events', 10)
        self._synchronizers = []
        for name, binding in self._bindings.items():
            image = message_filters.Subscriber(self, Image, binding['prefix'] + '/source_image',
                                               qos_profile=qos_profile_sensor_data)
            detections = message_filters.Subscriber(self, Detection2DArray, binding['prefix'] + '/detections',
                                                    qos_profile=qos_profile_sensor_data)
            sync = message_filters.TimeSynchronizer([image, detections], 2)
            sync.registerCallback(lambda image, found, camera=name: self._capture(image, found, camera))
            self._synchronizers.append(sync)
        self.create_service(QueryObservations, '/robot/observations/query', self._query)
        self.create_service(GetObservationEvidence, '/robot/observations/evidence', self._evidence)
        self.create_service(PinObservationEvidence, '/robot/observations/pin', self._pin)
        self.create_service(Trigger, '/robot/observations/status', self._status)

    def _capture(self, image, detections, camera):
        stream = self._streams[camera]
        started = time.monotonic()
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 10**9
        try:
            if not 0 <= age <= self._age:
                raise ValueError('capture timestamp is stale or in the future')
            if image.header.frame_id != self._bindings[camera]['optical_frame']:
                raise ValueError('capture frame differs from tracked camera binding')
            record, jpeg = observation(image, detections, self._model, camera)
            record['camera_sha256'] = self._bindings[camera]['sha256']
            record['stationary'] = self._motion.stationary(stamp / 1e9, self._bindings[camera]['mount'] == 'arm')
            record['clock_domain'] = 'ros_sim_time' if self.get_parameter('use_sim_time').value else 'ros_system_time'
            record['ros_domain_id'] = self.context.get_domain_id()
            capture_time = Time.from_msg(image.header.stamp)
            interval = Duration(seconds=.05)
            def nearest(cache):
                if cache is None:
                    return None
                messages = cache.getInterval(capture_time-interval, capture_time+interval)
                return min(messages, key=lambda msg: abs((Time.from_msg(msg.header.stamp)-capture_time).nanoseconds)) if messages else None
            stream['geometry'].ground(record, image, nearest(stream['depth']), nearest(stream['info']),
                                  self._transforms, self._map_context)
            source_age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not 0 <= source_age <= self._age:
                raise ValueError('observation expired during ingestion')
            sequence = self._store.put(record, jpeg)
            stream.update(ingestion_sec=time.monotonic() - started, source_age_at_ingest_sec=source_age)
        except (ValueError, RuntimeError, OSError, sqlite3.Error, CvBridgeError, cv2.error) as error:
            stream['reason'] = str(error)
            self.get_logger().warning(f'observation rejected: {error}', throttle_duration_sec=5)
            return
        stream.update(last=time.monotonic(), stamp=stamp, reason='')
        self._events.publish(String(data=json.dumps({'schema': 1, 'observation_id': record['id'], 'sequence': sequence})))

    def _readiness(self):
        now, stamp_now = time.monotonic(), self.get_clock().now().nanoseconds
        result = {}
        for name, stream in self._streams.items():
            age = (stamp_now - stream['stamp']) / 1e9
            ready = bool(stream['last'] and now - stream['last'] <= self._age and
                         0 <= age <= self._age and not stream['reason'])
            result[name] = {'ready': ready, 'reason': stream['reason'] or ('' if ready else 'observation stream stale'),
                            'ingestion_sec': stream.get('ingestion_sec'),
                            'source_age_at_ingest_sec': stream.get('source_age_at_ingest_sec'), 'source_age_sec': age}
        return result

    def _status(self, _request, response):
        streams = self._readiness()
        response.success = all(value['ready'] for value in streams.values())
        response.message = json.dumps({'streams': streams}, allow_nan=False)
        return response

    def _map(self, message):
        try:
            if len(message.data.encode()) > 4096:
                raise ValueError('map context exceeds 4096 bytes')
            value = json.loads(message.data)
            if not isinstance(value, dict) or type(value.get('ready')) is not bool:
                raise ValueError('invalid map context')
            if value['ready'] and (not valid_id(value.get('map_id')) or not isinstance(value.get('epoch'), str) or len(value['epoch']) != 32 or any(c not in '0123456789abcdef' for c in value['epoch'])):
                raise ValueError('invalid approved map identity')
            if value['ready'] and (type(value.get('started_at')) not in (int, float) or not math.isfinite(value['started_at']) or value['started_at'] <= 0):
                raise ValueError('map context requires its activation timestamp')
            self._map_context = value
        except (ValueError, TypeError, RecursionError):
            self._map_context = {'ready': False, 'reason': 'invalid map context'}

    def _query(self, request, response):
        try:
            result = self._store.query(request.request_json)
            result['streams'] = self._readiness()
            result['stream_ready'] = all(value['ready'] for value in result['streams'].values())
            result['map_context'] = self._map_context
            response.response_json = json.dumps(result, allow_nan=False)
            response.success = True
        except (ValueError, RuntimeError, OSError, sqlite3.Error) as error:
            response.detail = str(error)
        return response

    def _evidence(self, request, response):
        try:
            data, response.sha256 = self._store.evidence(request.observation_id)
            response.mime_type = 'image/jpeg'
            response.data_base64 = base64.b64encode(data).decode('ascii')
            response.success = True
        except (ValueError, RuntimeError, OSError, sqlite3.Error) as error:
            response.detail = str(error)
        return response

    def _pin(self, request, response):
        try:
            self._store.pin(request.request_json)
            response.success = True
        except (ValueError, RuntimeError, OSError, sqlite3.Error) as error:
            response.detail = str(error)
        return response

    def destroy_node(self):
        self._store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectObservations()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
