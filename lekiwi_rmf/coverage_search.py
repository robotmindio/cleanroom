"""ROS-free per-query coverage, negative evidence and viewpoint utility for active search.

A cell counts as searched only when a capture from the perception profile in force saw it
unoccluded, inside the camera frustum, close enough to resolve the configured minimum object
size, across the whole configured height band, and without a matching detection. Anything
else stays unobserved: a visit, a failed provider or a missing pose proves nothing.
"""

from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np

from lekiwi_rmf.exploration import cell_to_world, disk, footprint_is_free, known_safe_cells, world_to_cell

# An occupancy above this blocks a camera ray; the blocking cell's visible surface is observed.
OCCUPIED = 50
OUTCOMES = ('found', 'coverage_reached', 'inaccessible', 'budget_exhausted', 'incapable', 'canceled', 'failed')
# Bounded report lists; counts stay exact.
REPORT_ITEMS = 20


def normalize_label(text):
    return ' '.join(str(text).lower().split())


@dataclass(frozen=True)
class Camera:
    """Pinhole half-angles; azimuth is positive to the left of the optical axis."""
    fx: float
    cx: float
    left: float
    right: float
    up: float
    down: float


def camera_from_info(k, width, height):
    fx, cx, fy, cy = float(k[0]), float(k[2]), float(k[4]), float(k[5])
    if (not all(math.isfinite(v) for v in (fx, cx, fy, cy)) or fx <= 0 or fy <= 0
            or not 0 < cx < width or not 0 < cy < height):
        raise ValueError('camera intrinsics are not a valid pinhole model')
    return Camera(fx=fx, cx=cx, left=math.atan(cx / fx), right=math.atan((width - cx) / fx),
                  up=math.atan(cy / fy), down=math.atan((height - cy) / fy))


@dataclass(frozen=True)
class Profile:
    """Quality criteria that make an unoccluded view count as evidence of absence."""
    min_object_m: float
    min_object_px: float
    max_range_m: float
    min_height_m: float
    max_height_m: float
    max_age_sec: float
    require_stationary: bool = True

    def range_limit(self, camera):
        return min(self.max_range_m, camera.fx * self.min_object_m / self.min_object_px)

    def criteria(self):
        return {'min_object_m': self.min_object_m, 'min_object_px': self.min_object_px,
                'max_range_m': self.max_range_m, 'height_band_m': [self.min_height_m, self.max_height_m],
                'max_age_sec': self.max_age_sec, 'require_stationary': self.require_stationary,
                'occlusion': 'known_free_octomap_3d'}


@dataclass(frozen=True)
class Pose:
    """Optical centre and orientation in map; pitch is positive upwards."""
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float = 0.0


def optical_axes(yaw, pitch, roll=0.):
    """Optical forward/right/down axes in map, including camera roll."""
    cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch),
                             math.cos(roll), math.sin(roll))
    forward, right, down = (cp * cy, cp * sy, sp), (sy, -cy, 0.), (sp * cy, sp * sy, -cp)
    return (forward, tuple(cr * a + sr * b for a, b in zip(right, down)),
            tuple(-sr * a + cr * b for a, b in zip(right, down)))


def inside_frustum(points, pose, camera, limit):
    vector = points - np.asarray((pose.x, pose.y, pose.z))
    forward, right, down = (np.sum(vector * axis, axis=-1) for axis in optical_axes(pose.yaw, pose.pitch, pose.roll))
    return ((forward > 0) & (np.sum(vector * vector, axis=-1) <= limit * limit)
            & (right >= -math.tan(camera.left) * forward) & (right <= math.tan(camera.right) * forward)
            & (down >= -math.tan(camera.up) * forward) & (down <= math.tan(camera.down) * forward))


def pose_from_transform(translation, rotation):
    x, y, z, w = rotation
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not all(math.isfinite(v) for v in (*translation, x, y, z, w)) or abs(norm - 1) > 1e-3:
        raise ValueError('camera transform is not finite and normalized')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # Third column of the rotation matrix: the optical +Z axis expressed in map.
    forward = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    yaw = math.atan2(forward[1], forward[0])
    right = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))
    pitch = math.atan2(forward[2], math.hypot(forward[0], forward[1]))
    _, unrolled_right, unrolled_down = optical_axes(yaw, pitch)
    roll = math.atan2(sum(a * b for a, b in zip(right, unrolled_down)),
                      sum(a * b for a, b in zip(right, unrolled_right)))
    return Pose(float(translation[0]), float(translation[1]), float(translation[2]),
                yaw, pitch, roll)


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    left: float
    right: float
    map_xyz: tuple | None = None
    map_uncertainty_m: float = 0.0


@dataclass(frozen=True)
class Capture:
    observation_id: str
    camera_id: str
    model_sha256: str
    stamp: float
    pose: Pose
    camera: Camera
    stationary: bool
    detections: tuple
    # The labels an open-vocabulary inference was asked for; None for a closed native catalogue.
    query_labels: tuple | None = None
    # The proposal profile revision the frame ran under; only the admitted revision is evidence.
    profile_revision: str | None = None


def detections_from_record(record, camera, map_id=None):
    """Bearings from image boxes; a map position is used only for the approved map."""
    result = []
    for item in record.get('detections', []):
        x, _, w, _ = item['bbox']
        position = (item.get('position') or {}).get('map')
        xyz, uncertainty = None, 0.0
        if position and position.get('frame_id') == 'map' and map_id and position.get('map_id') == map_id:
            xyz, uncertainty = tuple(float(v) for v in position['xyz']), float(position['uncertainty_m'])
        result.append(Detection(normalize_label(item['label']), float(item['score']),
                                math.atan((camera.cx - x) / camera.fx), math.atan((camera.cx - x - w) / camera.fx),
                                xyz, uncertainty))
    return tuple(result)


def _cells(xs, ys, shape, resolution, origin):
    c, s = math.cos(origin[2]), math.sin(origin[2])
    dx, dy = xs - origin[0], ys - origin[1]
    cx = np.floor((c * dx + s * dy) / resolution).astype(np.int64)
    cy = np.floor((-s * dx + c * dy) / resolution).astype(np.int64)
    inside = (cx >= 0) & (cy >= 0) & (cx < shape[1]) & (cy < shape[0])
    return np.where(inside, cy * shape[1] + cx, 0), inside


def _band_visible(distances, pose, camera, profile):
    low = np.arctan2(profile.min_height_m - pose.z, distances)
    high = np.arctan2(profile.max_height_m - pose.z, distances)
    return (low >= pose.pitch - camera.down) & (high <= pose.pitch + camera.up)


def _fan(grid, resolution, origin, pose, camera, profile, offsets, step):
    """Sample rays at relative azimuths; return flat cells, visibility and sample distances."""
    # Conservative 2D candidates only; evidence of absence also requires known-free 3D rays.
    limit = profile.range_limit(camera)
    distances = np.arange(step, limit + 1e-9, step)
    angles = pose.yaw + offsets
    xs = pose.x + np.outer(np.cos(angles), distances)
    ys = pose.y + np.outer(np.sin(angles), distances)
    flat, inside = _cells(xs, ys, grid.shape, resolution, origin)
    values = np.where(inside, grid.reshape(-1)[flat], -1)
    blocking = (values < 0) | (values > OCCUPIED)
    first = np.where(blocking.any(axis=1), blocking.argmax(axis=1), len(distances))
    index = np.arange(len(distances))[None, :]
    open_ray = index < first[:, None]
    surface = (index == first[:, None]) & (values > OCCUPIED)
    tilted = abs(pose.roll) > 1e-9 or abs(math.cos(pose.pitch)) < 1e-6
    banded = inside if tilted else _band_visible(distances, pose, camera, profile)[None, :] & inside
    seen = (open_ray | surface) & banded
    occluded = ~open_ray & ~surface & banded & (values >= 0)
    return flat, seen, occluded, distances


def observed_cells(grid, resolution, origin, pose, camera, profile, geometry=None):
    """Candidate cells, filtered by known-free 3D geometry when supplied.

    The 2D-only form is a viewpoint heuristic; QueryCoverage requires geometry
    before it can turn these candidates into evidence of absence.
    """
    limit = profile.range_limit(camera)
    if limit <= resolution:
        return np.empty(0, np.int64), np.empty(0), 0
    step = resolution / 2
    # Ray spacing at the range limit is half a cell, so no resolved cell is skipped.
    tilted = abs(pose.roll) > 1e-9 or abs(math.cos(pose.pitch)) < 1e-6
    low, high = (-math.pi, math.pi) if tilted else (-camera.right, camera.left)
    offsets = np.arange(low, high + 1e-12, step / limit)
    flat, seen, occluded, _ = _fan(grid, resolution, origin, pose, camera, profile, offsets, step)
    if tilted:
        ys, xs = np.divmod(flat, grid.shape[1])
        wx, wy = cell_to_world(xs, ys, resolution, origin)
        points = np.empty((*flat.shape, 2, 3))
        points[..., 0], points[..., 1] = wx[..., None], wy[..., None]
        points[..., 2] = (profile.min_height_m, profile.max_height_m)
        seen &= inside_frustum(points, pose, camera, limit).all(axis=-1)
    if geometry is not None:
        visible = geometry.visible_columns(np.unique(flat[seen]), grid.shape, resolution, origin, pose, camera, profile)
        blocked = seen & ~np.isin(flat, visible)
        seen &= ~blocked
    azimuths = np.broadcast_to(offsets[:, None], flat.shape)
    return flat[seen], azimuths[seen], int(np.setdiff1d(np.unique(flat[occluded]), flat[seen]).size)


def searchable_mask(grid, resolution, origin, center, radius):
    """Mapped open cells and obstacle surfaces inside a disk; unknown space is reported separately."""
    yy, xx = np.indices(grid.shape)
    wx, wy = cell_to_world(xx, yy, resolution, origin)
    region = (wx - center[0]) ** 2 + (wy - center[1]) ** 2 <= radius ** 2
    open_cells = (grid >= 0) & (grid <= OCCUPIED)
    cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)
    surface = (grid > OCCUPIED) & (cv2.dilate(open_cells.astype(np.uint8), cross) != 0)
    return region & (open_cells | surface), region & (grid < 0)


class QueryCoverage:
    """Per-label negative evidence from the models able to report it; memory is bounded by the map grid.

    `models` maps each admitted model_sha256 to the profile revision admitted with it. A frame under any
    other profile (for example after a profile change mid-search) is silent here, not negative evidence:
    its thresholds are not the ones this session's coverage claims."""

    def __init__(self, label, models, profile, shape, resolution, origin):
        if not isinstance(models, dict):
            raise ValueError('admitted models map each model_sha256 to its profile revision')
        self.label, self.models, self.profile = normalize_label(label), dict(models), profile
        self.resolution, self.origin = resolution, origin
        self.negative = np.full(shape, -np.inf)
        self.sighting = np.full(shape, -np.inf)
        self.contributors = deque(maxlen=REPORT_ITEMS)
        self.viewpoints = deque(maxlen=REPORT_ITEMS)
        self.cameras = set()
        self.counts = {'captures': 0, 'negative_captures': 0, 'other_model': 0, 'other_profile': 0, 'other_query': 0,
                       'moving': 0, 'occluded_cells': 0, 'no_geometry': 0}
        self.by_model = {}
        self.span = [math.inf, -math.inf]

    def rebase(self, shape, resolution, origin):
        """Keep evidence across map growth; a different raster discards it rather than misplacing it."""
        if shape == self.negative.shape and resolution == self.resolution and np.allclose(origin, self.origin, atol=1e-6):
            return False
        shift = ((origin[0] - self.origin[0]) / resolution, (origin[1] - self.origin[1]) / resolution)
        aligned = (abs(resolution - self.resolution) < 1e-9 and abs(origin[2] - self.origin[2]) < 1e-9
                   and abs(origin[2]) < 1e-9 and all(abs(v - round(v)) < 1e-3 for v in shift))
        arrays = []
        for old in (self.negative, self.sighting):
            new = np.full(shape, -np.inf)
            if aligned:
                ox, oy = round(shift[0]), round(shift[1])
                # New cell (y, x) is old cell (y + oy, x + ox).
                ys = slice(max(0, -oy), min(shape[0], old.shape[0] - oy))
                xs = slice(max(0, -ox), min(shape[1], old.shape[1] - ox))
                if ys.start < ys.stop and xs.start < xs.stop:
                    new[ys, xs] = old[ys.start + oy:ys.stop + oy, xs.start + ox:xs.stop + ox]
            arrays.append(new)
        self.negative, self.sighting = arrays
        self.resolution, self.origin = resolution, origin
        return not aligned

    def add(self, capture, grid, geometry=None):
        """Integrate one capture; return its detections of this label."""
        self.counts['captures'] += 1
        if capture.model_sha256 not in self.models:
            # A frame from a recognizer whose catalogue lacks the label is silent about it, not negative.
            self.counts['other_model'] += 1
            return []
        if capture.profile_revision != self.models[capture.model_sha256]:
            self.counts['other_profile'] += 1
            return []
        if capture.query_labels is not None and self.label not in capture.query_labels:
            # An open inference that was not asked for this label is equally silent about it.
            self.counts['other_query'] += 1
            return []
        if geometry is None:
            self.counts['no_geometry'] += 1
        if capture.pose is None or geometry is None:
            flat, azimuths, occluded = np.empty(0, np.int64), np.empty(0), 0
        else:
            flat, azimuths, occluded = observed_cells(grid, self.resolution, self.origin, capture.pose,
                                                      capture.camera, self.profile, geometry)
        matches = [d for d in capture.detections if d.label == self.label]
        bearing = np.zeros(flat.shape, dtype=bool)
        margin = self.resolution / max(self.profile.range_limit(capture.camera), self.resolution)
        for detection in matches:
            if detection.map_xyz is None:
                # Without depth an image box is a bearing: every cell along it may hold the object.
                if capture.pose is not None and (abs(capture.pose.roll) > 1e-9 or abs(capture.pose.pitch) > 1e-9):
                    # A horizontal image interval alone cannot locate a box in a tilted frustum.
                    bearing[:] = True
                else:
                    bearing |= (azimuths <= detection.left + margin) & (azimuths >= detection.right - margin)
        sighted = np.union1d(np.unique(flat[bearing]), self._point_cells([d for d in matches if d.map_xyz is not None]))
        self.sighting.reshape(-1)[sighted] = np.maximum(self.sighting.reshape(-1)[sighted], capture.stamp)
        if self.profile.require_stationary and not capture.stationary:
            self.counts['moving'] += 1
            return matches
        negative = np.setdiff1d(np.unique(flat), sighted)
        if negative.size:
            self.negative.reshape(-1)[negative] = np.maximum(self.negative.reshape(-1)[negative], capture.stamp)
            self.counts['negative_captures'] += 1
            self.by_model[capture.model_sha256] = self.by_model.get(capture.model_sha256, 0) + 1
            self.counts['occluded_cells'] += occluded
            self.contributors.append(capture.observation_id)
            self.cameras.add(capture.camera_id)
            self.viewpoints.append({'camera_id': capture.camera_id, 'x': round(capture.pose.x, 3),
                                    'y': round(capture.pose.y, 3), 'yaw': round(capture.pose.yaw, 3),
                                    'stamp': capture.stamp, 'observation_id': capture.observation_id})
            self.span = [min(self.span[0], capture.stamp), max(self.span[1], capture.stamp)]
        return matches

    def _point_cells(self, detections):
        cells = []
        for detection in detections:
            x, y = world_to_cell(detection.map_xyz[:2], self.resolution, self.origin)
            radius = max(1, math.ceil(detection.map_uncertainty_m / self.resolution))
            ys, xs = np.nonzero(disk(radius * self.resolution, self.resolution))
            ys, xs = ys - radius + y, xs - radius + x
            inside = (ys >= 0) & (xs >= 0) & (ys < self.negative.shape[0]) & (xs < self.negative.shape[1])
            cells.append(ys[inside] * self.negative.shape[1] + xs[inside])
        return np.concatenate(cells) if cells else np.empty(0, np.int64)

    def covered(self, now):
        """Fresh evidence of absence: seen after the last sighting and within the persistence horizon."""
        return (self.negative >= now - self.profile.max_age_sec) & (self.negative > self.sighting)

    def weights(self, searchable, now, prior_weight):
        """Heuristic search mass: unobserved or stale cells, raised where the label was last seen."""
        weights = (searchable & ~self.covered(now)).astype(np.float64)
        weights[searchable & (self.sighting > self.negative)] += prior_weight
        return weights


@dataclass(frozen=True)
class Mount:
    """A base-fixed camera: optical height, pitch and axis yaw relative to the base."""
    camera_id: str
    camera: Camera
    z: float
    pitch: float
    yaw_offset: float
    roll: float = 0.0


def heading_gains(weights, grid, resolution, origin, position, mount, profile, headings):
    """Expected newly resolved mass for each base heading; an area-weighted estimate, not a probability."""
    limit = profile.range_limit(mount.camera)
    if limit <= resolution:
        return np.zeros(headings)
    rays = max(8, math.ceil(2 * math.pi * limit / resolution))
    offsets = np.arange(rays) * (2 * math.pi / rays)
    pose = Pose(position[0], position[1], mount.z, 0.0, mount.pitch, mount.roll)
    flat, seen, _, distances = _fan(grid, resolution, origin, pose, mount.camera, profile, offsets, resolution)
    # Each sample represents d·Δθ·step of area, so near cells sampled by many rays are not overcounted.
    area = distances[None, :] * (2 * math.pi / rays) * resolution / resolution ** 2
    if abs(mount.roll) > 1e-9 or abs(math.cos(mount.pitch)) < 1e-6:
        points = np.empty((*flat.shape, 2, 3))
        points[..., 0] = (position[0] + np.cos(offsets)[:, None] * distances)[..., None]
        points[..., 1] = (position[1] + np.sin(offsets)[:, None] * distances)[..., None]
        points[..., 2] = (profile.min_height_m, profile.max_height_m)
        mass = weights.reshape(-1)[flat] * area
        result = []
        for index in range(headings):
            heading = Pose(position[0], position[1], mount.z, index * 2 * math.pi / headings + mount.yaw_offset,
                           mount.pitch, mount.roll)
            visible = seen & inside_frustum(points, heading, mount.camera, limit).all(axis=-1)
            result.append(float(np.where(visible, mass, 0.).sum()))
        return np.asarray(result)
    per_ray = np.where(seen, weights.reshape(-1)[flat] * area, 0.0).sum(axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(np.concatenate((per_ray, per_ray)))))
    result = np.empty(headings)
    for index in range(headings):
        axis = index * 2 * math.pi / headings + mount.yaw_offset
        low = math.ceil((axis - mount.camera.right) / (2 * math.pi / rays)) % rays
        count = int((mount.camera.left + mount.camera.right) / (2 * math.pi / rays)) + 1
        result[index] = cumulative[low + count] - cumulative[low]
    return result


@dataclass(frozen=True)
class Viewpoint:
    x: float
    y: float
    yaw: float
    camera_id: str
    gain: float
    travel_m: float
    utility: float


def viewpoint_gain(viewpoint, weights, grid, resolution, origin, mount, profile):
    pose = Pose(viewpoint.x, viewpoint.y, mount.z, viewpoint.yaw + mount.yaw_offset, mount.pitch, mount.roll)
    flat, _, _ = observed_cells(grid, resolution, origin, pose, mount.camera, profile)
    return float(weights.reshape(-1)[np.unique(flat)].sum()) if flat.size else 0.0


def select_viewpoint(weights, grid, resolution, origin, position, center, radius, mounts, profile, *,
                     clearance, footprint, region_margin, free_threshold, spacing, headings, max_candidates,
                     overhead_m, min_gain, excluded=()):
    """Pick the reachable base pose and camera with the best gain per metre of travel.

    ponytail: straight-line travel is the cost estimate; Nav2 plans and validates the real route.
    """
    if not mounts or not weights.any():
        return None
    yy, xx = np.indices(grid.shape)
    wx, wy = cell_to_world(xx, yy, resolution, origin)
    motion = (wx - center[0]) ** 2 + (wy - center[1]) ** 2 < (radius - region_margin) ** 2
    safe = known_safe_cells(grid, resolution, clearance, free_threshold) & motion.astype(np.uint8)
    rx, ry = world_to_cell(position, resolution, origin)
    if not (0 <= ry < grid.shape[0] and 0 <= rx < grid.shape[1] and safe[ry, rx]):
        raise ValueError('robot pose is outside known space with the configured clearance')
    connected = safe.copy()
    cv2.floodFill(connected, None, (rx, ry), 2, flags=4)
    reachable = connected == 2
    step = max(1, round(spacing / resolution))
    lattice = reachable & (xx % step == 0) & (yy % step == 0)
    limit = max(profile.range_limit(mount.camera) for mount in mounts)
    near_mass = cv2.dilate((weights > 0).astype(np.uint8), disk(limit, resolution)) != 0
    lattice &= near_mass
    for point in excluded:
        lattice &= (wx - point[0]) ** 2 + (wy - point[1]) ** 2 >= spacing ** 2
    candidates = [(float(wx[y, x]), float(wy[y, x])) for y, x in zip(*np.nonzero(lattice))]
    if all(math.dist(position, point) >= spacing for point in excluded):
        candidates.append(tuple(position))  # Turning in place is a viewpoint too.
    candidates.sort(key=lambda point: math.dist(point, position))
    scored = []
    for point in candidates[:max_candidates]:
        travel = math.dist(point, position)
        for mount in mounts:
            gains = heading_gains(weights, grid, resolution, origin, point, mount, profile, headings)
            for index in np.argsort(gains)[::-1][:3]:
                if gains[index] >= min_gain:
                    scored.append(Viewpoint(point[0], point[1], math.remainder(index * 2 * math.pi / headings, 2 * math.pi),
                                            mount.camera_id, float(gains[index]), travel,
                                            float(gains[index]) / (travel + overhead_m)))
    for viewpoint in sorted(scored, key=lambda item: item.utility, reverse=True):
        if footprint_is_free(grid, resolution, origin, (viewpoint.x, viewpoint.y), viewpoint.yaw,
                             footprint, free_threshold):
            return viewpoint
    return None


def unobserved_regions(mask, resolution, origin, limit=REPORT_ITEMS):
    """Largest connected unobserved areas as map-frame boxes, for an honest report."""
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    order = sorted(range(1, count), key=lambda index: stats[index, cv2.CC_STAT_AREA], reverse=True)
    regions = []
    for index in order[:limit]:
        x, y, w, h, area = stats[index]
        low = cell_to_world(np.float64(x), np.float64(y), resolution, origin)
        high = cell_to_world(np.float64(x + w - 1), np.float64(y + h - 1), resolution, origin)
        centre = cell_to_world(np.float64(centroids[index][0]), np.float64(centroids[index][1]), resolution, origin)
        regions.append({'area_m2': round(float(area) * resolution ** 2, 3),
                        'centroid': [round(float(v), 3) for v in centre],
                        'bounds': [round(float(min(low[0], high[0])), 3), round(float(min(low[1], high[1])), 3),
                                   round(float(max(low[0], high[0])), 3), round(float(max(low[1], high[1])), 3)]})
    return regions, max(0, count - 1)


def statement(outcome, label, fraction, found_ids):
    if outcome == 'found':
        return f'found "{label}" in {len(found_ids)} observation(s); see evidence_ids'
    covered = f'{fraction:.0%} of the searchable mapped cells in the region have fresh evidence of absence'
    if outcome == 'incapable':
        return f'cannot search for "{label}": no observation instrument in force can detect it; nothing is claimed'
    return (f'"{label}" was not found in the inspected zones: {covered}. '
            'Unobserved, unmapped and stale space was not searched; this is not evidence of absence there')


class SearchSession:
    """One query's coverage, sightings, plan revisions and terminal report."""

    def __init__(self, query, perception, profile, geometry, *, started, target, region):
        self.label = normalize_label(query)
        self.perception = perception or {}
        self.recognizers = [{**item, 'label_supported': self.label in {normalize_label(label) for label in item['labels']}}
                            for item in self.perception.get('recognizers', [])]
        self.supported = any(item['label_supported'] for item in self.recognizers)
        grid, resolution, origin = geometry
        self.coverage = QueryCoverage(self.label, {item['model_sha256']: item['profile']['revision'] for item in self.recognizers
                                                   if item['label_supported']}, profile, grid.shape, resolution, origin)
        self.started, self.target, self.region = started, target, region
        self.found = []
        self.prior_sightings = deque(maxlen=REPORT_ITEMS)
        self.prior_count = 0
        self.revisions = deque(maxlen=REPORT_ITEMS)
        self.revision_count = 0
        self.discarded_rebases = 0

    def rebase(self, geometry):
        grid, resolution, origin = geometry
        if self.coverage.rebase(grid.shape, resolution, origin):
            self.discarded_rebases += 1
            self.revise('map raster changed; earlier coverage discarded', [])

    def reproject(self, captures, grid, revision, geometry_for=None):
        """Rebuild coverage from the capture log after spatial memory moved it into a new map revision.

        Sightings during the search keep their record; coverage and prior
        sightings are recomputed, so evidence follows the corrected map instead
        of staying at coordinates the optimization moved. Return True when a
        reprojected capture found the label.
        """
        coverage = self.coverage
        self.coverage = QueryCoverage(self.label, coverage.models, coverage.profile, coverage.negative.shape,
                                      coverage.resolution, coverage.origin)
        self.prior_sightings.clear()
        self.prior_count = 0
        known = {record['observation_id'] for record in self.found}
        self.found = []
        for capture in captures:
            self.integrate(capture, grid, geometry_for(capture) if geometry_for is not None else None)
        self.revise(f'map revision {revision}: coverage reprojected from {len(captures)} anchored captures', [])
        return any(record['observation_id'] not in known for record in self.found)

    def integrate(self, capture, grid, geometry=None):
        """Return True when this capture found the label during the search."""
        matches = self.coverage.add(capture, grid, geometry)
        if not matches:
            return False
        record = {'observation_id': capture.observation_id, 'camera_id': capture.camera_id, 'stamp': capture.stamp,
                  'detections': [{'score': round(d.score, 4), 'bearing_rad': None if capture.pose is None or
                                  abs(capture.pose.roll) > 1e-9 or abs(capture.pose.pitch) > 1e-9 else
                                  [round(capture.pose.yaw + d.right, 4), round(capture.pose.yaw + d.left, 4)],
                                  'map_xyz': d.map_xyz, 'map_uncertainty_m': d.map_uncertainty_m or None}
                                 for d in matches]}
        if capture.stamp >= self.started:
            if len(self.found) < REPORT_ITEMS:
                self.found.append(record)
            return True
        self.prior_count += 1
        self.prior_sightings.append(record)
        return False

    def revise(self, reason, evidence, viewpoint=None):
        self.revision_count += 1
        entry = {'revision': self.revision_count, 'reason': reason, 'evidence_ids': list(evidence)[-REPORT_ITEMS:]}
        if viewpoint is not None:
            entry['viewpoint'] = {'x': round(viewpoint.x, 3), 'y': round(viewpoint.y, 3), 'yaw': round(viewpoint.yaw, 3),
                                  'camera_id': viewpoint.camera_id, 'expected_gain_cells': round(viewpoint.gain, 1)}
        self.revisions.append(entry)

    def masks(self, grid):
        return searchable_mask(grid, self.coverage.resolution, self.coverage.origin, *self.region)

    def fraction(self, searchable, now):
        total = int(np.count_nonzero(searchable))
        return float(np.count_nonzero(searchable & self.coverage.covered(now))) / total if total else 0.0

    def outcome(self, searchable, now):
        """Resolved outcomes only; budgets, cancellation and faults are decided by the executor."""
        if not self.supported:
            return 'incapable'
        if self.found:
            return 'found'
        if np.count_nonzero(searchable) and self.fraction(searchable, now) >= self.target:
            return 'coverage_reached'
        return None

    def evidence_ids(self):
        return [record['observation_id'] for record in self.found]

    def report(self, outcome, grid, now, progress):
        searchable, unmapped = self.masks(grid)
        covered = searchable & self.coverage.covered(now)
        stale = searchable & ~covered & (self.coverage.negative > -np.inf) & (self.coverage.negative > self.coverage.sighting)
        unobserved = searchable & ~covered
        area = self.coverage.resolution ** 2
        regions, region_count = unobserved_regions(unobserved, self.coverage.resolution, self.coverage.origin)
        fraction = self.fraction(searchable, now)
        span = self.coverage.span if self.coverage.counts['negative_captures'] else [None, None]
        return {
            'schema': 1, 'outcome': outcome, 'statement': statement(outcome, self.label, fraction, self.evidence_ids()),
            'query': {'label': self.label, 'region': {'x': self.region[0][0], 'y': self.region[0][1],
                                                      'radius_m': self.region[1]}},
            'perception': {'label_supported': self.supported, 'cameras': self.perception.get('cameras', {}),
                           'recognizers': [{key: item.get(key) for key in (
                               'role', 'model_sha256', 'model_name', 'open_vocabulary', 'label_supported')}
                              | {'profile_revision': item['profile']['revision']} for item in self.recognizers]},
            'coverage': {'fraction': round(fraction, 4), 'target': self.target,
                         'searchable_cells': int(np.count_nonzero(searchable)), 'covered_cells': int(np.count_nonzero(covered)),
                         'resolution_m': self.coverage.resolution,
                         'covered_area_m2': round(float(np.count_nonzero(covered)) * area, 3),
                         'unobserved_area_m2': round(float(np.count_nonzero(unobserved)) * area, 3),
                         'stale_area_m2': round(float(np.count_nonzero(stale)) * area, 3),
                         'unmapped_area_m2': round(float(np.count_nonzero(unmapped)) * area, 3),
                         'unobserved_regions': regions, 'unobserved_region_count': region_count,
                         'map_raster_resets': self.discarded_rebases},
            'negative_evidence': {'label': self.label, 'model_sha256s': sorted(self.coverage.models),
                                  'observations_by_model': dict(self.coverage.by_model),
                                  'criteria': self.coverage.profile.criteria(), 'cameras': sorted(self.coverage.cameras),
                                  'observation_count': self.coverage.counts['negative_captures'],
                                  'recent_observation_ids': list(self.coverage.contributors),
                                  'recent_viewpoints': list(self.coverage.viewpoints),
                                  'first_stamp': span[0], 'last_stamp': span[1],
                                  'captures_considered': self.coverage.counts['captures'],
                                  'ignored_unable_model': self.coverage.counts['other_model'],
                                  'ignored_other_profile': self.coverage.counts['other_profile'],
                                  'ignored_other_query': self.coverage.counts['other_query'],
                                  'rejected_moving': self.coverage.counts['moving'],
                                  'missing_3d_geometry': self.coverage.counts['no_geometry'],
                                  'occluded_cells_observed_behind_obstacles': self.coverage.counts['occluded_cells']},
            'found': {'evidence_ids': self.evidence_ids(), 'observations': self.found},
            'prior_sightings': {'count': self.prior_count, 'recent': list(self.prior_sightings)},
            'plan': {'revision_count': self.revision_count, 'recent_revisions': list(self.revisions),
                     'utility': 'heuristic: expected newly resolved cells per metre of straight-line travel; '
                                'uncalibrated, not a probability'},
            'progress': progress,
        }
