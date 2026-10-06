"""ROS-free map target selection and the shared mapping storage quota."""

from pathlib import Path
import math

import cv2
import numpy as np


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
    return (x * x + y * y <= cells * cells).astype(np.uint8)


def known_safe_cells(grid, resolution, clearance, free_threshold):
    free = ((grid >= 0) & (grid <= free_threshold)).astype(np.uint8)
    return cv2.erode(free, disk(clearance, resolution), borderType=cv2.BORDER_CONSTANT, borderValue=0)


def select_target(grid, resolution, origin, position, center, radius, visited, blocked,
                  *, clearance, observation_distance, spacing, revisit_spacing,
                  free_threshold, revisit):
    """Choose a reachable view, staying clear of unknown cells and obstacles.

    ponytail: nearest view first, not an optimal tour; optimize only if measured
    exploration time warrants it. OpenCV handles the grid operations.
    """
    yy, xx = np.indices(grid.shape)
    wx, wy = cell_to_world(xx, yy, resolution, origin)
    region = (wx - center[0]) ** 2 + (wy - center[1]) ** 2 <= (radius - clearance) ** 2
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
    stage = "exploring_frontiers"
    if not np.any(candidates) and revisit:
        step = max(1, math.ceil(revisit_spacing / resolution))
        candidates = available & (xx % step == 0) & (yy % step == 0)
        stage = "revisiting_known_space"
    mapped_area = float(np.count_nonzero(region & (grid >= 0))) * resolution ** 2
    ys, xs = np.nonzero(candidates)
    if not len(xs):
        return None, stage, mapped_area
    closest = np.argmin((wx[ys, xs] - position[0]) ** 2 + (wy[ys, xs] - position[1]) ** 2)
    return (float(wx[ys[closest], xs[closest]]), float(wy[ys[closest], xs[closest]])), stage, mapped_area
