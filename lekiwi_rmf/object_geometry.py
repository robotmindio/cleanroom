"""Calibrated visible-surface estimates using ROS camera geometry and capture-time TF."""

import hashlib
import math
from pathlib import Path

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
import numpy as np
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
import yaml


class ObjectGeometry:
    def __init__(self, calibration_file):
        self.calibration = None
        self.sha256 = None
        self.bridge = CvBridge()
        if calibration_file:
            content = Path(calibration_file).expanduser().read_bytes()
            value = yaml.safe_load(content)
            if not isinstance(value, dict) or value.get('schema') != 1 or value.get('validated') is not True:
                raise ValueError('metric calibration must be explicitly validated')
            if value.get('depth_registered') is not True:
                raise ValueError('metric grounding requires depth registered by the ROS camera driver')
            for name in ('camera_uncertainty_m', 'map_uncertainty_m', 'clock_uncertainty_sec'):
                number = value.get(name)
                if type(number) not in (int, float) or not math.isfinite(number) or number < 0:
                    raise ValueError(f'invalid calibration {name}')
            if value['camera_uncertainty_m'] > .1 or value['map_uncertainty_m'] > .2 or value['clock_uncertainty_sec'] > .02:
                raise ValueError('metric calibration exceeds the proposed uncertainty gates')
            matrix = np.asarray(value.get('rgb_to_depth'), dtype=float)
            intrinsics = np.asarray(value.get('rgb_k'), dtype=float)
            projection = np.asarray(value.get('rgb_p'), dtype=float)
            if (matrix.shape != (3, 3) or intrinsics.shape != (9,) or
                    projection.shape != (12,) or not np.isfinite(projection).all() or projection[0] <= 0 or projection[5] <= 0 or
                    not np.isfinite(matrix).all() or not np.isfinite(intrinsics).all() or
                    abs(np.linalg.det(matrix)) < 1e-12 or np.abs(matrix).max() > 1e6 or intrinsics[0] <= 0 or intrinsics[4] <= 0):
                raise ValueError('calibration requires measured RGB intrinsics and raster mapping')
            rotation = np.asarray(value.get('rgb_r'), dtype=float)
            distortion = np.asarray(value.get('rgb_d'), dtype=float)
            if (rotation.shape != (9,) or not np.isfinite(rotation).all() or
                    abs(np.linalg.det(rotation.reshape(3, 3))) < 1e-12 or
                    distortion.ndim != 1 or not 4 <= len(distortion) <= 14 or not np.isfinite(distortion).all()):
                raise ValueError('calibration requires measured distortion and rectification')
            for name in ('rgb_size', 'depth_size'):
                size = value.get(name)
                if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or not 0 < v <= 1920 for v in size):
                    raise ValueError(f'invalid calibration {name}')
            if not value.get('optical_frame') or not value.get('camera_id'):
                raise ValueError('calibration requires camera/frame identity')
            if value.get('sampling', 'box_interior') != 'box_interior':
                raise ValueError('geometry samples the box interior of any object; color sampling was removed')
            limits = value.get('depth_range_m')
            if (not isinstance(limits, list) or len(limits) != 2 or any(type(v) not in (int, float) for v in limits)
                    or not 0 < limits[0] < limits[1] <= 20):
                raise ValueError('calibration requires the measured valid depth_range_m [min, max]')
            self.calibration, self.sha256 = value, hashlib.sha256(content).hexdigest()

    def ground(self, record, image, depth, info, transforms, map_context):
        value = self.calibration
        if value is None or depth is None or info is None:
            return
        def stamp(msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec / 10**9
        if (record['camera_id'] != value['camera_id'] or
                any(msg.header.frame_id != value['optical_frame'] for msg in (image, depth, info)) or
                abs(stamp(image)-stamp(depth)) > .05 or abs(stamp(image)-stamp(info)) > .05 or
                [image.width, image.height] != value['rgb_size'] or [depth.width, depth.height] != value['depth_size'] or
                list(info.k) != value['rgb_k'] or list(info.p) != value['rgb_p'] or
                list(info.r) != value['rgb_r'] or list(info.d) != value['rgb_d'] or [info.width, info.height] != value['rgb_size'] or
                depth.encoding != '16UC1' or depth.step < depth.width * 2 or len(depth.data) != depth.step * depth.height):
            return
        depths = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
        camera = PinholeCameraModel()
        camera.fromCameraInfo(info)
        mapping = np.asarray(value['rgb_to_depth'], dtype=float)
        for item in record['detections']:
            x, y, w, h = item['bbox']
            left, top = int(x+w*.2), int(y+h*.2)
            right, bottom = max(left+1,int(x+w*.8)), max(top+1,int(y+h*.8))
            # The box interior limits background without assuming any object color.
            rows, columns = (axis.ravel() for axis in np.indices((bottom - top, right - left)))
            if len(rows) < 10:
                continue
            step = max(1, math.ceil(len(rows)/1000))
            rgb = np.vstack((columns[::step]+left, rows[::step]+top, np.ones(len(rows[::step]))))
            mapped = mapping @ rgb
            valid = np.abs(mapped[2]) > 1e-12
            if not valid.any():
                continue
            mapped = mapped[:,valid]
            u, v = np.rint(mapped[0]/mapped[2]).astype(int), np.rint(mapped[1]/mapped[2]).astype(int)
            valid = (u>=0)&(v>=0)&(u<depth.width)&(v<depth.height)
            samples = depths[v[valid], u[valid]].astype(float)*.001
            samples = samples[(samples >= value['depth_range_m'][0])&(samples <= value['depth_range_m'][1])]
            if len(samples) < 10:
                continue
            median = float(np.median(samples))
            spread = float(np.median(np.abs(samples-median)))*1.4826
            uncertainty = value['camera_uncertainty_m'] + 3*spread
            coherent = np.abs(samples - median) <= max(.01, 3 * spread)
            if uncertainty > .1 or np.count_nonzero(coherent) < max(10, math.ceil(.6 * len(samples))):
                continue
            pixel = camera.rectifyPoint((float(np.median(rgb[0])), float(np.median(rgb[1]))))
            ray = camera.projectPixelTo3dRay(pixel)
            if not all(math.isfinite(coordinate) for coordinate in ray) or abs(ray[2]) < 1e-12:
                continue
            xyz = [float(coordinate*median/ray[2]) for coordinate in ray]
            position = {'mode': 'camera', 'xyz': xyz, 'frame_id': image.header.frame_id,
                        'uncertainty_m': uncertainty, 'calibration_sha256': self.sha256,
                        'meaning': 'visible_surface', 'depth_samples': len(samples)}
            if map_context.get('ready') and stamp(image) >= map_context.get('started_at', math.inf):
                try:
                    transform = transforms.lookup_transform('map', image.header.frame_id, Time.from_msg(image.header.stamp))
                    point = PointStamped(header=image.header)
                    point.point.x, point.point.y, point.point.z = xyz
                    world = do_transform_point(point, transform).point
                    world_uncertainty = uncertainty + value['map_uncertainty_m']
                    if world_uncertainty <= .2 and all(math.isfinite(v) for v in (world.x,world.y,world.z)):
                        position['map'] = {'mode': 'map', 'xyz': [world.x,world.y,world.z], 'frame_id': 'map',
                            'map_id': map_context['map_id'], 'epoch': map_context['epoch'],
                            'uncertainty_m': world_uncertainty, 'calibration_sha256': self.sha256}
                except TransformException:
                    # Camera position remains valid when historical localization is unavailable.
                    position['map_unavailable_reason'] = 'historical_transform_unavailable'
            item['position'] = position
        if any(item.get('position') for item in record['detections']):
            record['grounding'] = 'camera_position'
