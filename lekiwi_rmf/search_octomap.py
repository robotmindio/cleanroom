"""Known-free 3D visibility from OctoMap's binary occupancy format.

Unknown voxels block visibility. The binary encoding follows
octomap/OccupancyOcTreeBase.hxx: 00 unknown, 01 free, 10 occupied, 11 children.
"""

import math
from dataclasses import dataclass
from itertools import product

import numpy as np

from lekiwi_rmf.coverage_search import inside_frustum, optical_axes

MAX_MAP_BYTES = 2 * 1024 * 1024
# shortcut: global voxels assume static occluders; use per-frame depth before admitting dynamic scenes.
TREE_DEPTH = 16
KEY_OFFSET = 1 << (TREE_DEPTH - 1)
MAX_RAY_NODES = 4096


@dataclass(frozen=True)
class SearchOctomap:
    resolution: float
    root: tuple
    reference_pose: tuple[float, float, float, float, float, float] | None = None

    @classmethod
    def from_binary(cls, data, resolution):
        if type(resolution) not in (int, float) or not math.isfinite(resolution) or not 0 < resolution <= 1:
            raise ValueError('octomap resolution must be finite and in (0, 1] metres')
        try:
            raw = memoryview(data).cast('B')
        except (TypeError, ValueError) as error:
            raise ValueError('octomap data must be an int8 byte buffer') from error
        if not 2 <= len(raw) <= MAX_MAP_BYTES:
            raise ValueError('octomap payload must contain 2 bytes to 2 MiB')
        cursor = 0

        def read(depth):
            nonlocal cursor
            if depth >= TREE_DEPTH or cursor + 2 > len(raw):
                raise ValueError('truncated or excessively deep binary octomap')
            bits = int(raw[cursor]) | int(raw[cursor + 1]) << 8
            cursor += 2
            children = [(bits >> (2 * index)) & 3 for index in range(8)]
            for index, state in enumerate(children):
                if state == 3:
                    children[index] = read(depth + 1)
            return tuple(children)

        root = read(0)
        if cursor != len(raw):
            raise ValueError('trailing binary octomap data')
        return cls(float(resolution), root)

    def ray_free(self, start, end):
        """Traverse intersected octree boxes; unknown and even grazing contacts block."""
        if len(start) != 3 or len(end) != 3 or not all(math.isfinite(v) for v in (*start, *end)):
            return False
        extent = KEY_OFFSET * self.resolution
        if any(value < -extent or value >= extent for value in (*start, *end)):
            return False
        delta = tuple(b - a for a, b in zip(start, end))

        def intersects(lower, size):
            near, far = 0., 1.
            for axis, direction in enumerate(delta):
                low, high = lower[axis], lower[axis] + size
                if direction == 0:
                    if start[axis] < low - 1e-10 or start[axis] > high + 1e-10:
                        return False
                else:
                    first, last = (low - start[axis]) / direction, (high - start[axis]) / direction
                    near, far = max(near, min(first, last)), min(far, max(first, last))
                    if near > far + 1e-10:
                        return False
            return True

        return self._free(intersects)

    def volume_free(self, start, low, high):
        """Conservatively test the whole camera-to-box volume, including its interior."""
        extent = KEY_OFFSET * self.resolution
        if (not all(math.isfinite(value) for value in (*start, *low, *high))
                or any(a > b for a, b in zip(low, high))
                or any(value < -extent or value >= extent for value in (*start, *low, *high))):
            return False
        if all(child == 1 for child in self.root):
            return True
        corners = np.asarray(list(product(*zip(low, high))))
        planes = []
        for axis in range(3):
            for sign, bound in ((-1., min(start[axis], low[axis])), (1., max(start[axis], high[axis]))):
                planes.append((tuple(sign if index == axis else 0. for index in range(3)), sign * bound))
            for sign, bound in ((-1., low[axis]), (1., high[axis])):
                normal = tuple(sign if index == axis else 0. for index in range(3))
                if sign * start[axis] <= sign * bound:
                    planes.append((normal, sign * bound))
            # The other hull faces pass through the camera and an edge of the box.
            others = [index for index in range(3) if index != axis]
            for sides in product((0, 1), repeat=2):
                edge = np.asarray(low, dtype=float)
                for index, side in zip(others, sides):
                    edge[index] = (low, high)[side][index]
                direction = np.zeros(3)
                direction[axis] = 1.
                normal = np.cross(direction, edge - start)
                length = np.linalg.norm(normal)
                if length < 1e-12:
                    continue
                normal /= length
                offset = float(normal @ start)
                distances = corners @ normal - offset
                if distances.max() <= 1e-10:
                    planes.append((tuple(normal), offset))
                elif distances.min() >= -1e-10:
                    planes.append((tuple(-normal), -offset))

        def intersects(lower, size):
            # A box wholly outside any hull plane is disjoint; unresolved overlap blocks.
            return all(sum(n * (value if n >= 0 else value + size)
                           for n, value in zip(normal, lower)) <= offset + 1e-10
                       for normal, offset in planes)

        return self._free(intersects)

    def _free(self, intersects):
        extent = KEY_OFFSET * self.resolution
        pending = [(self.root, (-extent, -extent, -extent), 2 * extent)]
        visited = 0
        while pending:
            node, lower, size = pending.pop()
            visited += 1
            if visited > MAX_RAY_NODES:
                return False
            half = size / 2
            for index, child in enumerate(node):
                if child == 1:
                    continue
                corner = tuple(lower[axis] + half * ((index >> axis) & 1) for axis in range(3))
                if not intersects(corner, half):
                    continue
                if isinstance(child, int):
                    return False
                pending.append((child, corner, half))
        return True

    def visible_columns(self, cells, shape, resolution, origin, pose, camera, profile):
        """Require the entire cell/height volume and its camera rays to be known free."""
        if self.resolution > profile.min_object_m:
            return np.empty(0, np.int64)
        c, s = math.cos(origin[2]), math.sin(origin[2])
        forward, right, down = optical_axes(pose.yaw, pose.pitch, pose.roll)
        start, visible = (pose.x, pose.y, pose.z), []
        source_start, rotation = start, None
        if self.reference_pose is not None:
            source_start = self.reference_pose[:3]
            old_forward, old_right, old_down = optical_axes(*self.reference_pose[3:])
            rotation = np.asarray((old_right, old_down, old_forward)).T @ np.asarray((right, down, forward))

        limit = profile.range_limit(camera)
        for flat in cells:
            row, column = divmod(int(flat), shape[1])
            corners = [(origin[0] + resolution * (c * x - s * y),
                        origin[1] + resolution * (s * x + c * y))
                       for x, y in product((column, column + 1), (row, row + 1))]
            corners = np.asarray([(x, y, z) for x, y in corners
                                  for z in (profile.min_height_m, profile.max_height_m)])
            if not inside_frustum(corners, pose, camera, limit).all():
                continue
            if rotation is not None:
                corners = (corners - start) @ rotation.T + source_start
            if self.volume_free(source_start, corners.min(axis=0), corners.max(axis=0)):
                visible.append(flat)
        return np.asarray(visible, dtype=np.int64)
