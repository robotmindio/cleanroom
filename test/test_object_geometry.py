"""Measured calibration gates and read-only travel accounting."""

import json

from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from lekiwi_rmf.navigation_distance import NavigationDistance
from lekiwi_rmf.object_geometry import ObjectGeometry
from test_object_observations import capture, document


def calibration(tmp_path):
    values = {'schema': 1, 'validated': True, 'depth_registered': True, 'camera_id': 'astra_color',
              'optical_frame': 'camera_optical', 'rgb_size': [60, 40], 'depth_size': [60, 40],
              'rgb_k': [100., 0., 30., 0., 100., 20., 0., 0., 1.],
              'rgb_p': [100., 0., 30., 0., 0., 100., 20., 0., 0., 0., 1., 0.],
              'rgb_r': [1., 0., 0., 0., 1., 0., 0., 0., 1.], 'rgb_d': [0.] * 5,
              'rgb_to_depth': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
              'camera_uncertainty_m': .03, 'map_uncertainty_m': .05, 'clock_uncertainty_sec': .01}
    path = tmp_path / 'calibration.json'
    path.write_text(json.dumps(values))
    return values, path


def test_metric_requires_validated_calibration(tmp_path):
    value, path = calibration(tmp_path)
    value['validated'] = False
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='validated'):
        ObjectGeometry(str(path))
    assert ObjectGeometry('').calibration is None


def test_camera_surface_uses_depth_and_rejects_unsynchronized_frames(tmp_path):
    value, path = calibration(tmp_path)
    geometry = ObjectGeometry(str(path))
    image, _ = capture()
    record, _ = document()
    depth = CvBridge().cv2_to_imgmsg(np.full((40, 60), 1000, np.uint16), '16UC1')
    depth.header = image.header
    info = CameraInfo(header=image.header, width=60, height=40, k=value['rgb_k'], p=value['rgb_p'], r=value['rgb_r'])
    info.distortion_model = 'plumb_bob'
    info.d = [0.] * 5
    geometry.ground(record, image, depth, info, None, {'ready': False})
    position = record['detections'][0]['position']
    assert position['xyz'][2] == 1 and position['uncertainty_m'] == .03
    assert position['meaning'] == 'visible_surface' and 'map' not in position
    other, _ = document()
    depth.header = type(image.header)(stamp=Time(sec=2), frame_id=image.header.frame_id)
    geometry.ground(other, image, depth, info, None, {'ready': False})
    assert 'position' not in other['detections'][0]


def test_odometry_epochs_gaps_and_invalid_readings():
    meter = NavigationDistance()
    message = Odometry()
    message.header.frame_id = 'odom'
    message.header.stamp = Time(sec=1)
    meter.update(message)
    assert meter.status()['ready']
    message.header.stamp = Time(sec=1, nanosec=500000000)
    message.pose.pose.position.x = .1
    meter.update(message)
    assert meter.status()['meters'] == .1
    epoch = meter.epoch
    message.header.stamp = Time(sec=4)
    meter.update(message)
    assert meter.epoch != epoch and not meter.status()['ready']
    message.header.stamp = Time(sec=4, nanosec=100000000)
    meter.update(message)
    assert meter.status()['ready']
    message.pose.pose.position.x = float('nan')
    meter.update(message)
    assert not meter.status()['ready']


def test_map_grounding_requires_current_epoch_and_capture_time_transform(tmp_path):
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformException

    class Transforms:
        def __init__(self):
            self.calls = []

        def lookup_transform(self, target, source, stamp):
            self.calls.append((target, source, stamp.nanoseconds))
            result = TransformStamped()
            result.transform.rotation.w = 1.
            result.transform.translation.x = 2.
            return result

    value, path = calibration(tmp_path)
    geometry = ObjectGeometry(str(path))
    image, _ = capture(stamp=2)
    depth = CvBridge().cv2_to_imgmsg(np.full((40, 60), 1000, np.uint16), '16UC1')
    depth.header = image.header
    info = CameraInfo(header=image.header, width=60, height=40, k=value['rgb_k'], p=value['rgb_p'], r=value['rgb_r'])
    info.distortion_model, info.d = 'plumb_bob', value['rgb_d']
    transforms = Transforms()
    context = {'ready': True, 'map_id': 'a'*64, 'epoch': 'b'*32, 'started_at': 1.}
    record, _ = document(stamp=2)
    geometry.ground(record, image, depth, info, transforms, context)
    mapped = record['detections'][0]['position']['map']
    assert mapped['epoch'] == context['epoch'] and mapped['xyz'][0] > 1.
    assert transforms.calls == [('map', image.header.frame_id, 2*10**9)]
    old, _ = document(stamp=2)
    geometry.ground(old, image, depth, info, transforms, {**context, 'started_at': 3.})
    assert 'map' not in old['detections'][0]['position']

    class Missing:
        def lookup_transform(self, *args):
            raise TransformException('no historical transform')

    missing, _ = document(stamp=2)
    geometry.ground(missing, image, depth, info, Missing(), context)
    assert missing['detections'][0]['position']['map_unavailable_reason'] == 'historical_transform_unavailable'
