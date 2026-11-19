"""ROS-free map-position estimates for image detections.

Each estimate names its method, its uncertainty and the assumptions it rests on.
Only the validated RGB-D path (lekiwi_rmf/object_geometry.py) measures; every
other method infers a place from an image box and stays ``qualified: false``
until it is physically calibrated. None of them is ever written to an
observation's ``position``.
"""

from dataclasses import dataclass
import math

import numpy as np

from lekiwi_rmf.coverage_search import optical_axes

# Best first: a capture's detection is placed by the first method that applies.
METHODS = ('rgbd_surface', 'ray_octomap_hit', 'triangulated', 'bearing_only')
PINHOLE = 'pinhole intrinsics from camera_info; lens distortion ignored'


@dataclass(frozen=True)
class Wedge:
    """A horizontal sector of map bearings from an optical centre: the object is somewhere along it."""
    x: float
    y: float
    low: float
    high: float

    @property
    def bearing(self):
        return (self.low + self.high) / 2

    @property
    def half_width(self):
        return (self.high - self.low) / 2

    def contains(self, point, margin_m=0.0):
        """Whether a disk of ``margin_m`` around a map point reaches into the sector."""
        distance = math.hypot(point[0] - self.x, point[1] - self.y)
        if distance <= margin_m or distance < 1e-9:
            return True
        offset = abs(math.remainder(math.atan2(point[1] - self.y, point[0] - self.x) - self.bearing, 2 * math.pi))
        return offset <= self.half_width + math.asin(margin_m / distance)

    def to_dict(self):
        return {'origin': [round(self.x, 4), round(self.y, 4)],
                'bearing_rad': [round(self.low, 5), round(self.high, 5)]}


def _vertical(camera, height):
    """Image rows per unit of vertical tangent, from the half-angles a Camera keeps."""
    return (math.tan(camera.up) + math.tan(camera.down)) / height


def ray(camera, height, pose, u, v):
    """Unit map direction of the ray through pixel (u, v) of a camera at ``pose``."""
    forward, right, down = optical_axes(pose.yaw, pose.pitch, pose.roll)
    across, below = (u - camera.cx) / camera.fx, v * _vertical(camera, height) - math.tan(camera.up)
    direction = np.asarray(forward) + across * np.asarray(right) + below * np.asarray(down)
    return tuple(float(value) for value in direction / np.linalg.norm(direction))


def bearing_wedge(box, camera, height, pose, max_tilt_rad):
    """The map bearings an image box spans, from the rays through its outline.

    The outline rays hold under camera tilt, unlike the box's horizontal pixel
    interval. Beyond ``max_tilt_rad`` of roll or pitch a box says too little
    about a horizontal bearing and there is no wedge.
    """
    if abs(pose.roll) > max_tilt_rad or abs(pose.pitch) > max_tilt_rad:
        return None
    x, y, w, h = box
    centre = ray(camera, height, pose, x + w / 2, y + h / 2)
    outline = [ray(camera, height, pose, x + w * i / 2, y + h * j / 2)
               for i in range(3) for j in range(3) if (i, j) != (1, 1)]
    if any(math.hypot(direction[0], direction[1]) < 0.2 for direction in (centre, *outline)):
        return None  # A ray this close to vertical has no usable horizontal bearing.
    bearing = math.atan2(centre[1], centre[0])
    offsets = [math.remainder(math.atan2(direction[1], direction[0]) - bearing, 2 * math.pi) for direction in outline]
    return Wedge(pose.x, pose.y, bearing + min(offsets), bearing + max(offsets))


def rgbd_surface(detection, frame_id, camera_in_map):
    """The validated metric path's visible surface, placed by the capture-time camera pose.

    Qualified only when that path also validated the map position of this
    capture; its camera-frame measurement alone carries no map uncertainty.
    """
    position = detection.get('position') or {}
    if position.get('mode') != 'camera' or position.get('frame_id') != frame_id:
        return None
    on_map = position.get('map')
    return {'method': 'rgbd_surface', 'map_xyz': list(camera_in_map.apply(position['xyz'])),
            'sigma_m': float((on_map or position)['uncertainty_m']), 'wedge': None,
            'assumptions': ['visible surface, not the object centre']
            + ([] if on_map else ['the map uncertainty of this capture was not validated']),
            'calibration_sha256': position.get('calibration_sha256'), 'qualified': on_map is not None}


def ray_octomap_hit(box, camera, height, pose, octomap, max_range_m):
    """Where the ray through the box centre first meets an occupied voxel of the capture-time octomap."""
    x, y, w, h = box
    direction = ray(camera, height, pose, x + w / 2, y + h / 2)
    distance = octomap.first_hit((pose.x, pose.y, pose.z), direction, max_range_m)
    if distance is None or distance <= octomap.resolution:
        return None
    # The object centre may be anywhere in the box; the voxel bounds the depth of its surface.
    half_angle = max(w / 2 / camera.fx, h / 2 * _vertical(camera, height))
    return {'method': 'ray_octomap_hit',
            'map_xyz': [origin + distance * step for origin, step in zip((pose.x, pose.y, pose.z), direction)],
            'sigma_m': math.hypot(octomap.resolution, distance * half_angle), 'wedge': None,
            'assumptions': ['the object is the first surface on the ray through the box centre',
                            'the capture-time octomap contains the object or the surface it rests against',
                            PINHOLE],
            'calibration_sha256': None, 'qualified': False}


def bearing_only(wedge):
    """The wedge itself: a direction without a range, so no point."""
    return {'method': 'bearing_only', 'map_xyz': None, 'sigma_m': None, 'wedge': wedge, 'range_m': [0.0, None],
            'assumptions': ['range unknown: the object is somewhere along the bearing', PINHOLE],
            'calibration_sha256': None, 'qualified': False}


def triangulated(wedges, *, min_baseline_m, min_parallax_rad, max_range_m):
    """Least-squares crossing of the bearings of one label seen from different places; x and y only.

    Refused when the viewpoints are closer than ``min_baseline_m``, the bearings
    are nearly parallel, or the crossing is behind a camera, beyond
    ``max_range_m`` or outside one of the wedges.
    """
    if len(wedges) < 2:
        return None
    origins = np.array([(wedge.x, wedge.y) for wedge in wedges])
    directions = np.array([(math.cos(wedge.bearing), math.sin(wedge.bearing)) for wedge in wedges])
    baseline = max(float(np.linalg.norm(a - b)) for a in origins for b in origins)
    parallax = max(abs(float(a[0] * b[1] - a[1] * b[0])) for a in directions for b in directions)
    if baseline < min_baseline_m or parallax < math.sin(min_parallax_rad):
        return None
    # Each bearing constrains the point to its line: minimise the squared distances to all of them.
    normals = [np.eye(2) - np.outer(direction, direction) for direction in directions]
    point = np.linalg.solve(sum(normals), sum(normal @ origin for normal, origin in zip(normals, origins)))
    ranges = np.sum((point - origins) * directions, axis=1)
    if (ranges <= 0).any() or (ranges > max_range_m).any() or not all(wedge.contains(point) for wedge in wedges):
        return None
    lateral = max(float(distance) * math.tan(wedge.half_width) for distance, wedge in zip(ranges, wedges))
    return {'method': 'triangulated', 'map_xyz': [float(point[0]), float(point[1]), None],
            'sigma_m': lateral / parallax, 'wedge': None,
            'assumptions': ['every bearing is the same object', 'the object did not move between the captures',
                            'height unknown', PINHOLE],
            'calibration_sha256': None, 'qualified': False}


def estimates(detection, frame_id, camera, height, pose, camera_in_map, octomap=None, *, max_range_m, max_tilt_rad):
    """Every estimate one capture supports for one detection, best first; possibly none.

    ``pose`` and ``camera_in_map`` are the same capture-time camera pose, in the
    map revision ``octomap`` was built in. Triangulation needs other captures and
    is done by the object map from the wedges returned here.
    """
    result = [rgbd_surface(detection, frame_id, camera_in_map)]
    if octomap is not None:
        result.append(ray_octomap_hit(detection['bbox'], camera, height, pose, octomap, max_range_m))
    wedge = bearing_wedge(detection['bbox'], camera, height, pose, max_tilt_rad)
    if wedge is not None:
        result.append(bearing_only(wedge))
    return [estimate for estimate in result if estimate is not None]
