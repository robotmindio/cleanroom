#!/usr/bin/env python3
"""Collect any standard ROS 2 detector's exactly synchronized image observations."""

import base64
import json
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
from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from lekiwi_rmf.object_observations import ObservationStore, observation, valid_id
from lekiwi_rmf.object_geometry import ObjectGeometry
from lekiwi_rmf.srv import GetObservationEvidence, QueryObservations, PinObservationEvidence


class ObjectObservations(Node):
    def __init__(self, **kwargs):
        super().__init__('object_observations', **kwargs)
        suffix = f'_{self.context.get_domain_id()}'
        if self.get_parameter('use_sim_time').value:
            suffix += '_sim'
        self.declare_parameter('database_path', str(Path.home() / f'.ros/lekiwi/observations{suffix}.sqlite3'))
        for name, value in {'camera_id': 'astra_color', 'model_sha256': '', 'max_source_age_sec': 2.0,
                            'max_records': 100000, 'max_bytes': 2 * 1024**3, 'retention_sec': 7 * 86400.0}.items():
            self.declare_parameter(name, value)
        self._camera = self.get_parameter('camera_id').value
        self._model = self.get_parameter('model_sha256').value
        self.declare_parameter('calibration_file', '')
        self._geometry = ObjectGeometry(self.get_parameter('calibration_file').value)
        self._transforms = Buffer()
        self._listener = TransformListener(self._transforms, self)
        self._map_context = {'ready': False}
        self.create_subscription(String, '/robot/map_context', self._map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._depth_source = message_filters.Subscriber(self, Image, 'depth', qos_profile=qos_profile_sensor_data)
        self._info_source = message_filters.Subscriber(self, CameraInfo, 'camera_info', qos_profile=qos_profile_sensor_data)
        self._depth_cache = message_filters.Cache(self._depth_source, 16)
        self._info_cache = message_filters.Cache(self._info_source, 64)
        self._age = self.get_parameter('max_source_age_sec').value
        if not valid_id(self._model) or not self._camera or len(self._camera) > 128:
            raise ValueError('configured model SHA-256 and camera identity are required')
        if not 0 < self._age <= 10:
            raise ValueError('max_source_age_sec must be within (0,10]')
        self._store = ObservationStore(self.get_parameter('database_path').value,
            max_records=self.get_parameter('max_records').value, max_bytes=self.get_parameter('max_bytes').value,
            retention_sec=self.get_parameter('retention_sec').value)
        self._last = 0.0
        self._stamp = 0
        self._reason = 'waiting for synchronized image/detections'
        self._events = self.create_publisher(String, '/robot/observations/events', 10)
        self._image = message_filters.Subscriber(self, Image, 'image', qos_profile=qos_profile_sensor_data)
        self._detections = message_filters.Subscriber(self, Detection2DArray, 'detections', qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.TimeSynchronizer([self._image, self._detections], 4)
        self._sync.registerCallback(self._capture)
        self.create_service(QueryObservations, '/robot/observations/query', self._query)
        self.create_service(GetObservationEvidence, '/robot/observations/evidence', self._evidence)
        self.create_service(PinObservationEvidence, '/robot/observations/pin', self._pin)
        self.create_service(Trigger, '/robot/observations/status', self._status)

    def _capture(self, image, detections):
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 10**9
        try:
            if not 0 <= age <= self._age:
                raise ValueError('capture timestamp is stale or in the future')
            record, jpeg = observation(image, detections, self._model, self._camera)
            record['clock_domain'] = 'ros_sim_time' if self.get_parameter('use_sim_time').value else 'ros_system_time'
            record['ros_domain_id'] = self.context.get_domain_id()
            capture_time = Time.from_msg(image.header.stamp)
            interval = Duration(seconds=.05)
            def nearest(cache):
                messages = cache.getInterval(capture_time-interval, capture_time+interval)
                return min(messages, key=lambda msg: abs((Time.from_msg(msg.header.stamp)-capture_time).nanoseconds)) if messages else None
            self._geometry.ground(record, image, nearest(self._depth_cache), nearest(self._info_cache),
                                  self._transforms, self._map_context)
            sequence = self._store.put(record, jpeg)
        except (ValueError, RuntimeError, OSError, sqlite3.Error, CvBridgeError, cv2.error) as error:
            self._reason = str(error)
            self.get_logger().warning(f'observation rejected: {error}', throttle_duration_sec=5)
            return
        self._last, self._stamp, self._reason = time.monotonic(), stamp, ''
        self._events.publish(String(data=json.dumps({'schema': 1, 'observation_id': record['id'], 'sequence': sequence})))

    def _status(self, _request, response):
        source_age = (self.get_clock().now().nanoseconds - self._stamp) / 10**9
        response.success = bool(self._last and time.monotonic() - self._last <= self._age and
                                0 <= source_age <= self._age and not self._reason)
        response.message = self._reason or ('fresh image observations' if response.success else 'observation stream stale')
        return response

    def _map(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or type(value.get('ready')) is not bool:
                raise ValueError('invalid map context')
            if value['ready'] and (not valid_id(value.get('map_id')) or not isinstance(value.get('epoch'), str)):
                raise ValueError('invalid approved map identity')
            self._map_context = value
        except (ValueError, TypeError):
            self._map_context = {'ready': False, 'reason': 'invalid map context'}

    def _query(self, request, response):
        try:
            result = self._store.query(request.request_json)
            result['stream_ready'] = self._status(None, Trigger.Response()).success
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
