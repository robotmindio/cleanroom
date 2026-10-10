"""ROS-free map target selection and the shared mapping storage quota."""

from pathlib import Path
import json
import math
import os
import tempfile
import time

import cv2
import numpy as np
import yaml


# Accepted stopping bound; separate from the folded body's map clearance.
STOPPING_MARGIN_M = 0.77


def task_time():
    """Task budgets include suspend time and survive process restarts."""
    return time.clock_gettime(time.CLOCK_BOOTTIME)


def save_task_checkpoint(path, state):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as output:
            temporary = Path(output.name)
            json.dump(state, output, allow_nan=False)
            output.flush()
            os.fsync(output.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def load_task_checkpoint(path, identifier, *, maximum_duration, maximum_radius, region_margin, maximum_points):
    """Reject lost, expired or altered bounds instead of starting a fresh task."""
    if len(identifier) != 32 or any(c not in "0123456789abcdef" for c in identifier):
        raise ValueError("invalid exploration resume task ID")
    if path.stat().st_size > 1024 * 1024:
        raise ValueError("exploration checkpoint is too large")
    state = json.loads(path.read_text())
    if (not isinstance(state, dict) or type(state.get("schema")) is not int or state["schema"] != 1
            or state.get("id") != identifier
            or state.get("boot_id") != Path("/proc/sys/kernel/random/boot_id").read_text().strip()):
        raise ValueError("exploration checkpoint does not match this task and host boot")
    for name in ("deadline", "radius", "mapped_area_m2"):
        value = state.get(name)
        if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
            raise ValueError(f"invalid checkpoint {name}")
    if not 0 < state["deadline"] - task_time() <= maximum_duration:
        raise ValueError("saved exploration duration reached or exceeds current limits")
    if not region_margin < state["radius"] <= maximum_radius:
        raise ValueError("saved exploration radius exceeds current limits")
    for name in ("revisit_known", "retrying", "previous_mapping"):
        if type(state.get(name)) is not bool:
            raise ValueError(f"invalid checkpoint {name}")
    center = state.get("center")
    groups = [state.get("visited"), state.get("blocked")]
    if (not isinstance(center, list) or len(center) != 2
            or any(not isinstance(points, list) for points in groups)
            or sum(map(len, groups)) > maximum_points):
        raise ValueError("invalid exploration checkpoint progress")
    for point in [center, *groups[0], *groups[1]]:
        if (not isinstance(point, list) or len(point) != 2
                or any(type(v) not in (float, int) or not math.isfinite(v) for v in point)):
            raise ValueError("invalid checkpoint position")
        if math.dist(point, center) >= state["radius"] - region_margin:
            raise ValueError("checkpoint progress exceeds its original region")
    return state


def load_navigation_footprint(path):
    """Use the same unpadded convex body as both Nav2 costmaps."""
    params = yaml.safe_load(path.read_text())
    footprints = []
    for name in ("local_costmap", "global_costmap"):
        config = params[name][name]["ros__parameters"]
        vertices = np.asarray(yaml.safe_load(config["footprint"]), dtype=np.float64)
        if (vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3
                or not np.all(np.isfinite(vertices)) or config.get("footprint_padding", 0.0) != 0.0
                or not cv2.isContourConvex(vertices.astype(np.float32))):
            raise ValueError(f"{name} needs a finite convex footprint with zero padding")
        footprints.append(vertices)
    if not np.array_equal(*footprints):
        raise ValueError("exploration requires matching local/global Nav2 footprints")
    footprint = footprints[0]
    inscribed = cv2.pointPolygonTest(footprint.astype(np.float32), (0, 0), True)
    if inscribed <= 0:
        raise ValueError("Nav2 footprint must enclose the robot origin")
    boundary_margin = math.ceil(float(np.linalg.norm(footprint, axis=1).max()) * 100) / 100 + STOPPING_MARGIN_M
    return footprint, inscribed, boundary_margin


def _body_cells(grid, resolution, origin, position, yaw, footprint, selected):
    """Yield grid cells (y, x) among ``selected`` whose area the oriented body overlaps.

    Returns None when the body is not entirely inside the grid.
    """
    dx, dy = position[0] - origin[0], position[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    center = np.array((c * dx + s * dy, -s * dx + c * dy))
    c, s = math.cos(yaw - origin[2]), math.sin(yaw - origin[2])
    body = (footprint @ np.array(((c, s), (-s, c)))).astype(np.float32)
    low = np.floor((body.min(axis=0) + center) / resolution).astype(int)
    high = np.floor((body.max(axis=0) + center) / resolution).astype(int)
    if np.any(low < 0) or high[0] >= grid.shape[1] or high[1] >= grid.shape[0]:
        return None
    ys, xs = np.nonzero(selected(grid[low[1]:high[1] + 1, low[0]:high[0] + 1]))
    square = np.array(((0, 0), (resolution, 0), (resolution, resolution), (0, resolution)), dtype=np.float32)
    overlapped = []
    for x, y in zip(xs + low[0], ys + low[1], strict=True):
        cell = (square + np.array((x * resolution, y * resolution)) - center).astype(np.float32)
        area, _ = cv2.intersectConvexConvex(body, cell)
        if area > 0:
            overlapped.append((y, x))
    return overlapped


def footprint_is_free(grid, resolution, origin, position, yaw, footprint, free_threshold):
    """Check the whole oriented body against occupied/unknown cell areas."""
    if not all(math.isfinite(v) for v in (*position, yaw)):
        return False
    blocked = _body_cells(grid, resolution, origin, position, yaw, footprint,
                          lambda cells: (cells < 0) | (cells > free_threshold))
    return blocked == []


def with_body_free(grid, resolution, origin, position, yaw, footprint):
    """A copy of the grid with every cell under the robot's current body marked free.

    The robot occupies its own footprint, so a mapped obstacle or unknown cell
    there is stale or self-observed. A stationary robot adds no map node to
    clear it. Nearby real obstacles remain the live collision monitor's job.
    """
    grid = grid.copy()
    if all(math.isfinite(v) for v in (*position, yaw)):
        for y, x in _body_cells(grid, resolution, origin, position, yaw, footprint,
                                lambda cells: np.ones(cells.shape, dtype=bool)) or ():
            grid[y, x] = 0
    return grid


def database_size(database: Path) -> int:
    """Count the active SQLite database and its transient sidecars."""
    total = 0
    for suffix in ("", "-wal", "-shm", "-journal"):
        try:
            total += Path(f"{database}{suffix}").stat().st_size
        except FileNotFoundError:
            continue  # SQLite can remove a sidecar between samples.
    return total


def task_limits(duration, radius, maximum_duration, maximum_radius):
    limits = []
    for name, value, maximum in (
        ("max_duration_sec", duration, maximum_duration),
        ("max_radius_m", radius, maximum_radius),
    ):
        if not math.isfinite(value) or value < 0 or value > maximum:
            raise ValueError(f"{name} must be zero or within (0, {maximum}]")
        limits.append(value or maximum)
    return tuple(limits)


def map_geometry(message, maximum_cells):
    info = message.info
    if (message.header.frame_id != "map" or info.width < 1 or info.height < 1
            or info.width * info.height > maximum_cells
            or len(message.data) != info.width * info.height
            or not math.isfinite(info.resolution) or not 0.01 <= info.resolution <= 0.2):
        raise ValueError("expected a bounded metric occupancy grid in frame map (1–20 cm cells)")
    p, q = info.origin.position, info.origin.orientation
    if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
            or abs(q.z * q.z + q.w * q.w - 1) > 1e-3):
        raise ValueError("occupancy grid origin must have a finite planar unit rotation")
    grid = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
    if np.any((grid < -1) | (grid > 100)):
        raise ValueError("occupancy grid cells must be -1 or 0–100")
    return grid, info.resolution, (p.x, p.y, 2 * math.atan2(q.z, q.w))


def world_to_cell(xy, resolution, origin):
    dx, dy = xy[0] - origin[0], xy[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return math.floor((c * dx + s * dy) / resolution), math.floor((-s * dx + c * dy) / resolution)


def cell_to_world(xs, ys, resolution, origin):
    x, y = (xs + 0.5) * resolution, (ys + 0.5) * resolution
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return origin[0] + c * x - s * y, origin[1] + s * x + c * y


def disk(radius, resolution):
    cells = math.ceil(radius / resolution)
    y, x = np.ogrid[-cells:cells + 1, -cells:cells + 1]
    # Keep the metric radius, allowing only floating-point roundoff.
    return ((x * x + y * y) * resolution ** 2 <= radius ** 2 + 1e-12).astype(np.uint8)


def known_safe_cells(grid, resolution, clearance, free_threshold):
    free = ((grid >= 0) & (grid <= free_threshold)).astype(np.uint8)
    return cv2.erode(free, disk(clearance, resolution), borderType=cv2.BORDER_CONSTANT, borderValue=0)


def select_target(grid, resolution, origin, position, center, radius, visited, blocked,
                  *, clearance, footprint, region_margin, observation_distance, spacing, revisit_spacing,
                  free_threshold, revisit):
    """Choose a reachable view, staying clear of unknown cells and obstacles.

    ponytail: nearest view first, not an optimal tour; optimize only if measured
    exploration time warrants it. OpenCV handles the grid operations.
    """
    yy, xx = np.indices(grid.shape)
    wx, wy = cell_to_world(xx, yy, resolution, origin)
    region = (wx - center[0]) ** 2 + (wy - center[1]) ** 2 <= (radius - region_margin) ** 2
    free = ((grid >= 0) & (grid <= free_threshold)).astype(np.uint8)
    safe = known_safe_cells(grid, resolution, clearance, free_threshold)
    safe &= region.astype(np.uint8)
    x, y = world_to_cell(position, resolution, origin)
    if not (0 <= y < grid.shape[0] and 0 <= x < grid.shape[1] and safe[y, x]):
        raise ValueError("robot pose is outside known space with the configured clearance")
    connected = safe.copy()
    cv2.floodFill(connected, None, (x, y), 2, flags=4)
    reachable = connected == 2
    unknown_neighbor = cv2.dilate((grid == -1).astype(np.uint8), np.array(
        [[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8))
    connected_free = free.copy()
    cv2.floodFill(connected_free, None, (x, y), 2, flags=4)
    frontier = (connected_free == 2).astype(np.uint8) & unknown_neighbor
    candidates = reachable & (cv2.dilate(frontier, disk(observation_distance, resolution)) != 0)
    available = reachable & ((wx - position[0]) ** 2 + (wy - position[1]) ** 2 >= spacing ** 2)
    for vx, vy in (*visited, *blocked):
        available &= (wx - vx) ** 2 + (wy - vy) ** 2 >= spacing ** 2
    candidates &= available
    stages = [("exploring_frontiers", candidates)]
    if revisit:
        step = max(1, math.ceil(revisit_spacing / resolution))
        stages.append(("revisiting_known_space", available & (xx % step == 0) & (yy % step == 0)))
    mapped_area = float(np.count_nonzero(region & (grid >= 0))) * resolution ** 2
    for stage, candidates in stages:
        ys, xs = np.nonzero(candidates)
        for closest in np.argsort((wx[ys, xs] - position[0]) ** 2 + (wy[ys, xs] - position[1]) ** 2):
            target = (float(wx[ys[closest], xs[closest]]), float(wy[ys[closest], xs[closest]]))
            yaw = math.atan2(target[1] - position[1], target[0] - position[0])
            if footprint_is_free(grid, resolution, origin, target, yaw, footprint, free_threshold):
                return target, stage, mapped_area
    return None, stage, mapped_area
