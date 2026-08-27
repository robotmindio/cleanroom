"""Validation for immutable navigation/RMF deployment map bundles."""

from __future__ import annotations

import hashlib
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class MapBundleError(ValueError):
    """A bundle is incomplete, inconsistent, or not deployment-approved."""


@dataclass(frozen=True)
class ValidatedMapBundle:
    map_id: str
    occupancy_yaml: Path
    occupancy_image: Path
    navigation_graph: Path
    fleet_config: Path
    footprint_radius: float


@dataclass(frozen=True)
class _OccupancyMap:
    """Small dependency-free view of a ROS ``trinary`` PGM map."""

    width: int
    height: int
    pixels: tuple[int, ...]
    maximum: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    occupied_threshold: float
    free_threshold: float
    negate: bool

    @property
    def size_x(self) -> float:
        return self.width * self.resolution

    @property
    def size_y(self) -> float:
        return self.height * self.resolution

    def local(self, x: float, y: float) -> tuple[float, float]:
        dx, dy = x - self.origin_x, y - self.origin_y
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def cell_free(self, column: int, row: int) -> bool:
        if not 0 <= column < self.width or not 0 <= row < self.height:
            return False
        # OccupancyGrid row zero is at the YAML origin (bottom-left), while a
        # PGM raster starts at the top-left. Mirror the raster row exactly as
        # nav2_map_server does or graph clearance is checked against a
        # vertically reflected map.
        image_row = self.height - 1 - row
        value = self.pixels[image_row * self.width + column] / self.maximum
        occupancy = value if self.negate else 1.0 - value
        # Unknown cells are deliberately not traversable for a deployment
        # graph.  A graph validated on unknown space is not a safe graph.
        return occupancy <= self.free_threshold and occupancy <= self.occupied_threshold


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise MapBundleError(f"cannot read YAML {path}: {error}") from error
    if not isinstance(value, dict):
        raise MapBundleError(f"{path} must contain a YAML mapping")
    return value


def _finite_float(value: Any, label: str, *, positive: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise MapBundleError(f"{label} must be a finite number") from error
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "positive and finite" if positive else "finite"
        raise MapBundleError(f"{label} must be {qualifier}")
    return result


def _pgm_token(data: bytes, offset: int) -> tuple[bytes, int]:
    length = len(data)
    while offset < length:
        if data[offset] in b" \t\r\n":
            offset += 1
        elif data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            offset = length if newline < 0 else newline + 1
        else:
            break
    start = offset
    while offset < length and data[offset] not in b" \t\r\n#":
        offset += 1
    if start == offset:
        raise MapBundleError("occupancy image has an incomplete PGM header")
    return data[start:offset], offset


def _read_pgm(path: Path) -> tuple[int, int, int, tuple[int, ...]]:
    try:
        data = path.read_bytes()
    except OSError as error:
        raise MapBundleError(f"cannot read occupancy image {path}: {error}") from error
    try:
        magic, offset = _pgm_token(data, 0)
        width_token, offset = _pgm_token(data, offset)
        height_token, offset = _pgm_token(data, offset)
        maximum_token, offset = _pgm_token(data, offset)
        width, height, maximum = int(width_token), int(height_token), int(maximum_token)
    except (ValueError, MapBundleError) as error:
        raise MapBundleError(f"occupancy image {path} has an invalid PGM header") from error
    if magic not in {b"P2", b"P5"} or width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise MapBundleError(f"occupancy image {path} must be a valid P2/P5 PGM")
    count = width * height
    if magic == b"P2":
        values: list[int] = []
        while len(values) < count:
            try:
                token, offset = _pgm_token(data, offset)
            except MapBundleError:
                break
            try:
                values.append(int(token))
            except ValueError as error:
                raise MapBundleError(f"occupancy image {path} has a non-numeric pixel") from error
        if len(values) != count or any(value < 0 or value > maximum for value in values):
            raise MapBundleError(f"occupancy image {path} has the wrong number of pixels")
        return width, height, maximum, tuple(values)

    # P5 stores samples big-endian when maxval is greater than 255.  Consume
    # the one header/raster separator (and a CRLF pair), but do not skip all
    # whitespace: a raster's first pixel is allowed to be byte 0x20.
    if offset >= len(data) or data[offset] not in b" \t\r\n":
        raise MapBundleError(f"occupancy image {path} has no PGM raster separator")
    if data[offset] == ord("\r") and offset + 1 < len(data) and data[offset + 1] == ord("\n"):
        offset += 2
    else:
        offset += 1
    sample_bytes = 1 if maximum < 256 else 2
    expected = count * sample_bytes
    raster = data[offset:offset + expected]
    if len(raster) != expected:
        raise MapBundleError(f"occupancy image {path} has a truncated raster")
    if sample_bytes == 1:
        values = tuple(raster)
    else:
        values = tuple(int.from_bytes(raster[index:index + 2], "big") for index in range(0, expected, 2))
    return width, height, maximum, values


def _occupancy_map(occupancy: dict[str, Any], image: Path) -> _OccupancyMap:
    try:
        resolution = float(occupancy["resolution"])
        origin = occupancy["origin"]
        origin_x, origin_y, origin_yaw = (float(value) for value in origin)
        occupied_threshold = float(occupancy.get("occupied_thresh", 0.65))
        free_threshold = float(occupancy.get("free_thresh", 0.25))
        negate = bool(int(occupancy.get("negate", 0)))
    except (KeyError, TypeError, ValueError) as error:
        raise MapBundleError("occupancy YAML has invalid resolution, origin, or thresholds") from error
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise MapBundleError("occupancy resolution must be finite and positive")
    if not math.isfinite(origin_x) or not math.isfinite(origin_y) or not math.isfinite(origin_yaw):
        raise MapBundleError("occupancy origin must be finite")
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise MapBundleError("occupancy thresholds must satisfy 0 <= free < occupied <= 1")
    width, height, maximum, pixels = _read_pgm(image)
    return _OccupancyMap(width, height, pixels, maximum, resolution, origin_x, origin_y,
                         origin_yaw, occupied_threshold, free_threshold, negate)


def _point_rectangle_distance(x: float, y: float, left: float, bottom: float,
                              right: float, top: float) -> float:
    dx = max(left - x, 0.0, x - right)
    dy = max(bottom - y, 0.0, y - top)
    return math.hypot(dx, dy)


def _segment_intersects_rectangle(ax: float, ay: float, bx: float, by: float,
                                  left: float, bottom: float, right: float, top: float) -> bool:
    dx, dy = bx - ax, by - ay
    t_min, t_max = 0.0, 1.0
    for start, delta, low, high in ((ax, dx, left, right), (ay, dy, bottom, top)):
        if abs(delta) < 1e-12:
            if start < low or start > high:
                return False
            continue
        near, far = (low - start) / delta, (high - start) / delta
        if near > far:
            near, far = far, near
        t_min, t_max = max(t_min, near), min(t_max, far)
        if t_min > t_max:
            return False
    return True


def _segment_rectangle_distance(ax: float, ay: float, bx: float, by: float,
                                left: float, bottom: float, right: float, top: float) -> float:
    if _segment_intersects_rectangle(ax, ay, bx, by, left, bottom, right, top):
        return 0.0
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    def point_segment_distance(px: float, py: float) -> float:
        if length_squared <= 1e-20:
            return math.hypot(px - ax, py - ay)
        parameter = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
        return math.hypot(px - (ax + parameter * dx), py - (ay + parameter * dy))
    return min(point_segment_distance(px, py) for px, py in (
        (left, bottom), (left, top), (right, bottom), (right, top)
    ))


def _check_disk_free(grid: _OccupancyMap, x: float, y: float, radius: float,
                     label: str) -> None:
    local_x, local_y = grid.local(x, y)
    epsilon = grid.resolution * 1e-7
    if (local_x - radius < -epsilon or local_y - radius < -epsilon or
            local_x + radius > grid.size_x + epsilon or local_y + radius > grid.size_y + epsilon):
        raise MapBundleError(f"{label} footprint leaves occupancy map bounds")
    first_col = max(0, int(math.floor((local_x - radius) / grid.resolution)))
    last_col = min(grid.width - 1, int(math.floor((local_x + radius) / grid.resolution)))
    first_row = max(0, int(math.floor((local_y - radius) / grid.resolution)))
    last_row = min(grid.height - 1, int(math.floor((local_y + radius) / grid.resolution)))
    for row in range(first_row, last_row + 1):
        for column in range(first_col, last_col + 1):
            left, bottom = column * grid.resolution, row * grid.resolution
            if not grid.cell_free(column, row) and _point_rectangle_distance(
                    local_x, local_y, left, bottom, left + grid.resolution, bottom + grid.resolution
            ) <= radius + epsilon:
                raise MapBundleError(f"{label} footprint intersects occupied/unknown map cell ({column}, {row})")


def _check_segment_free(grid: _OccupancyMap, start: tuple[float, float], end: tuple[float, float],
                        radius: float, label: str) -> None:
    ax, ay = grid.local(*start)
    bx, by = grid.local(*end)
    epsilon = grid.resolution * 1e-7
    if any(value < radius - epsilon for value in (ax, ay, grid.size_x - ax, grid.size_y - ay,
                                                   bx, by, grid.size_x - bx, grid.size_y - by)):
        raise MapBundleError(f"{label} footprint leaves occupancy map bounds")
    first_col = max(0, int(math.floor((min(ax, bx) - radius) / grid.resolution)))
    last_col = min(grid.width - 1, int(math.floor((max(ax, bx) + radius) / grid.resolution)))
    first_row = max(0, int(math.floor((min(ay, by) - radius) / grid.resolution)))
    last_row = min(grid.height - 1, int(math.floor((max(ay, by) + radius) / grid.resolution)))
    for row in range(first_row, last_row + 1):
        for column in range(first_col, last_col + 1):
            left, bottom = column * grid.resolution, row * grid.resolution
            if not grid.cell_free(column, row) and _segment_rectangle_distance(
                    ax, ay, bx, by, left, bottom, left + grid.resolution, bottom + grid.resolution
            ) <= radius + epsilon:
                raise MapBundleError(f"{label} footprint intersects occupied/unknown map cell ({column}, {row})")


def _artifact(bundle_dir: Path, entry: Any, label: str) -> Path:
    if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
        raise MapBundleError(f"artifact {label} needs path and sha256")
    digest = entry.get("sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise MapBundleError(f"artifact {label} has an invalid sha256")
    path = (bundle_dir / entry["path"]).resolve()
    if not path.is_file():
        raise MapBundleError(f"artifact {label} does not exist: {path}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != digest.lower():
        raise MapBundleError(f"artifact {label} checksum mismatch: {path}")
    return path


def _reference_residual(fleet: dict[str, Any]) -> float:
    references = fleet.get("reference_coordinates")
    if not isinstance(references, dict) or not references:
        raise MapBundleError("fleet config has no reference_coordinates")
    worst = 0.0
    for level, coordinates in references.items():
        if not isinstance(coordinates, dict):
            raise MapBundleError(f"reference coordinates for {level} are malformed")
        rmf = coordinates.get("rmf")
        robot = coordinates.get("robot")
        if not isinstance(rmf, list) or not isinstance(robot, list) or len(rmf) != len(robot) or len(rmf) < 3:
            raise MapBundleError(f"reference coordinates for {level} need at least three pairs")
        try:
            rmf_points = [(float(a[0]), float(a[1])) for a in rmf]
            points = [(float(a[0]), float(a[1])) for a in robot]
        except (IndexError, TypeError, ValueError) as error:
            raise MapBundleError(f"reference coordinates for {level} contain invalid points") from error
        if any(not math.isfinite(value) for point in (*rmf_points, *points) for value in point):
            raise MapBundleError(f"reference coordinates for {level} contain non-finite points")
        def maximum_triangle_area(coordinates: list[tuple[float, float]]) -> float:
            return max(
                abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
                for index, a in enumerate(coordinates)
                for j, b in enumerate(coordinates[index + 1 :], start=index + 1)
                for c in coordinates[j + 1 :]
            )
        if maximum_triangle_area(points) <= 1e-6 or maximum_triangle_area(rmf_points) <= 1e-6:
            raise MapBundleError(f"reference coordinates for {level} are collinear")
        # RMF and robot coordinates may have different origins and axes. Fit
        # the best SE(2) transform (translation + rotation) before measuring
        # residual; direct point subtraction incorrectly rejects valid maps.
        rmf_centre = tuple(sum(point[index] for point in rmf_points) / len(rmf_points) for index in (0, 1))
        robot_centre = tuple(sum(point[index] for point in points) / len(points) for index in (0, 1))
        cross = sum(
            (robot_point[0] - robot_centre[0]) * (rmf_point[1] - rmf_centre[1])
            - (robot_point[1] - robot_centre[1]) * (rmf_point[0] - rmf_centre[0])
            for robot_point, rmf_point in zip(points, rmf_points)
        )
        dot = sum(
            (robot_point[0] - robot_centre[0]) * (rmf_point[0] - rmf_centre[0])
            + (robot_point[1] - robot_centre[1]) * (rmf_point[1] - rmf_centre[1])
            for robot_point, rmf_point in zip(points, rmf_points)
        )
        angle = math.atan2(cross, dot)
        cosine, sine = math.cos(angle), math.sin(angle)
        translation = (
            rmf_centre[0] - cosine * robot_centre[0] + sine * robot_centre[1],
            rmf_centre[1] - sine * robot_centre[0] - cosine * robot_centre[1],
        )
        residual = max(
            math.hypot(
                cosine * robot_point[0] - sine * robot_point[1] + translation[0] - rmf_point[0],
                sine * robot_point[0] + cosine * robot_point[1] + translation[1] - rmf_point[1],
            )
            for robot_point, rmf_point in zip(points, rmf_points)
        )
        worst = max(worst, residual)
    return worst


def _validate_graph(graph: dict[str, Any]) -> dict[str, list[tuple[float, float]]]:
    levels = graph.get("levels")
    if not isinstance(levels, dict) or not levels:
        raise MapBundleError("navigation graph has no levels")
    graph_points: dict[str, list[tuple[float, float]]] = {}
    for level, description in levels.items():
        if not isinstance(description, dict):
            raise MapBundleError(f"navigation graph level {level} is malformed")
        vertices = description.get("vertices")
        lanes = description.get("lanes")
        if not isinstance(vertices, list) or not vertices:
            raise MapBundleError(f"navigation graph level {level} has no vertices")
        if not isinstance(lanes, list):
            raise MapBundleError(f"navigation graph level {level} has no lanes")
        names: set[str] = set()
        adjacency = [set() for _ in vertices]
        reverse_adjacency = [set() for _ in vertices]
        points: list[tuple[float, float]] = []
        for index, vertex in enumerate(vertices):
            if not isinstance(vertex, list) or len(vertex) < 3:
                raise MapBundleError(f"vertex {index} on {level} is malformed")
            try:
                x, y = float(vertex[0]), float(vertex[1])
            except (IndexError, TypeError, ValueError) as error:
                raise MapBundleError(f"vertex {index} on {level} is malformed") from error
            if not math.isfinite(x) or not math.isfinite(y):
                raise MapBundleError(f"vertex {index} on {level} is non-finite")
            points.append((x, y))
            attributes = vertex[2]
            if isinstance(attributes, dict) and attributes.get("name"):
                name = str(attributes["name"])
                if name in names:
                    raise MapBundleError(f"duplicate vertex name {name!r} on {level}")
                names.add(name)
        for lane_index, lane in enumerate(lanes):
            if not isinstance(lane, list) or len(lane) < 2:
                raise MapBundleError(f"lane {lane_index} on {level} is malformed")
            if isinstance(lane[0], bool) or isinstance(lane[1], bool):
                raise MapBundleError(f"lane {lane_index} on {level} has invalid endpoints")
            try:
                start, finish = int(lane[0]), int(lane[1])
            except (TypeError, ValueError) as error:
                raise MapBundleError(f"lane {lane_index} on {level} has invalid endpoints") from error
            if start != lane[0] or finish != lane[1]:
                raise MapBundleError(f"lane {lane_index} on {level} has non-integer endpoints")
            if start == finish or not 0 <= start < len(vertices) or not 0 <= finish < len(vertices):
                raise MapBundleError(f"lane {lane_index} on {level} has invalid endpoints")
            adjacency[start].add(finish)
            reverse_adjacency[finish].add(start)

        def reachable(edges: list[set[int]]) -> set[int]:
            visited = {0}
            queue = deque([0])
            while queue:
                current = queue.popleft()
                for neighbor in edges[current] - visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
            return visited

        # RMF lanes are directed. Weak connectivity can strand a robot after a
        # one-way dispatch, so every deployment level must be strongly
        # connected (a route out and a route back).
        if (
            len(reachable(adjacency)) != len(vertices)
            or len(reachable(reverse_adjacency)) != len(vertices)
        ):
            raise MapBundleError(
                f"navigation graph level {level} is not strongly connected"
            )
        graph_points[str(level)] = points
    return graph_points


def _validate_fleet_footprint(fleet: dict[str, Any], radius: float, tolerance: float) -> None:
    rmf_fleet = fleet.get("rmf_fleet")
    profile = rmf_fleet.get("profile") if isinstance(rmf_fleet, dict) else None
    if not isinstance(profile, dict):
        raise MapBundleError("fleet config has no rmf_fleet.profile")
    try:
        footprint = float(profile["footprint"])
    except (KeyError, TypeError, ValueError) as error:
        raise MapBundleError("fleet profile needs a finite footprint radius") from error
    if not math.isfinite(footprint) or footprint <= 0.0:
        raise MapBundleError("fleet profile footprint must be finite and positive")
    if abs(footprint - radius) > tolerance:
        raise MapBundleError(
            f"fleet footprint {footprint:.4f} m does not match bundle footprint {radius:.4f} m"
        )


def _validate_optional_navigation_footprint(path: Path, radius: float, tolerance: float) -> None:
    """Validate an optional Nav2 footprint artifact when a bundle supplies one."""
    params = _load_yaml(path)
    found = False
    for key, value in _walk_mappings(params):
        if key != "footprint":
            continue
        if isinstance(value, str):
            try:
                value = yaml.safe_load(value)
            except yaml.YAMLError as error:
                raise MapBundleError(f"navigation footprint in {path} is invalid") from error
        if not isinstance(value, list) or not value:
            raise MapBundleError(f"navigation footprint in {path} is malformed")
        try:
            vertices = [(float(point[0]), float(point[1])) for point in value]
        except (IndexError, TypeError, ValueError) as error:
            raise MapBundleError(f"navigation footprint in {path} is malformed") from error
        if any(not math.isfinite(x) or not math.isfinite(y) for x, y in vertices):
            raise MapBundleError(f"navigation footprint in {path} is non-finite")
        enclosing_radius = max(math.hypot(x, y) for x, y in vertices)
        if enclosing_radius > radius + tolerance:
            raise MapBundleError(
                f"bundle footprint {radius:.4f} m does not enclose navigation "
                f"footprint radius {enclosing_radius:.4f} m"
            )
        found = True
    if not found:
        raise MapBundleError(f"navigation config {path} has no footprint")


def _walk_mappings(value: Any):
    if isinstance(value, dict):
        for key, child in value.items():
            yield key, child
            yield from _walk_mappings(child)


def validate_map_bundle(path: str | Path, require_approved: bool = True) -> ValidatedMapBundle:
    manifest_path = Path(path).expanduser().resolve()
    manifest = _load_yaml(manifest_path)
    if manifest.get("schema_version") != 1:
        raise MapBundleError("unsupported map bundle schema_version")
    map_id = manifest.get("map_id")
    if not isinstance(map_id, str) or not map_id.strip():
        raise MapBundleError("map bundle needs a non-empty map_id")
    if require_approved and manifest.get("validated") is not True:
        raise MapBundleError(f"map bundle {map_id!r} has not passed deployment acceptance")
    radius = _finite_float(manifest.get("robot_footprint_radius"), "robot_footprint_radius", positive=True)

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise MapBundleError("map bundle has no artifacts")
    occupancy_yaml = _artifact(manifest_path.parent, artifacts.get("occupancy_yaml"), "occupancy_yaml")
    occupancy_image = _artifact(manifest_path.parent, artifacts.get("occupancy_image"), "occupancy_image")
    navigation_graph = _artifact(manifest_path.parent, artifacts.get("navigation_graph"), "navigation_graph")
    fleet_config = _artifact(manifest_path.parent, artifacts.get("fleet_config"), "fleet_config")

    occupancy = _load_yaml(occupancy_yaml)
    resolution = _finite_float(occupancy.get("resolution"), "occupancy resolution", positive=True)
    validation = manifest.get("validation", {})
    if not isinstance(validation, dict):
        raise MapBundleError("map bundle validation must be a mapping")
    maximum_resolution = _finite_float(validation.get("maximum_resolution", 0.05), "maximum_resolution", positive=True)
    if resolution > maximum_resolution:
        raise MapBundleError(
            f"occupancy resolution {resolution} exceeds bundle limit {maximum_resolution}"
        )
    image_name = occupancy.get("image")
    if not isinstance(image_name, str) or not image_name:
        raise MapBundleError("occupancy YAML needs a non-empty image path")
    expected_image = (occupancy_yaml.parent / image_name).resolve()
    if expected_image != occupancy_image:
        raise MapBundleError("occupancy YAML image does not match bundled occupancy_image")

    fleet = _load_yaml(fleet_config)
    footprint_tolerance = _finite_float(
        validation.get("footprint_tolerance", 1e-3), "footprint_tolerance", positive=True
    )
    _validate_fleet_footprint(fleet, radius, footprint_tolerance)
    graph_points = _validate_graph(_load_yaml(navigation_graph))
    grid = _occupancy_map(occupancy, occupancy_image)
    for level, points in graph_points.items():
        for index, point in enumerate(points):
            _check_disk_free(grid, *point, radius, f"vertex {index} on {level}")
    graph = _load_yaml(navigation_graph)
    for level, description in graph["levels"].items():
        points = graph_points[str(level)]
        for lane_index, lane in enumerate(description["lanes"]):
            _check_segment_free(
                grid, points[int(lane[0])], points[int(lane[1])], radius,
                f"lane {lane_index} on {level}",
            )
    residual = _reference_residual(fleet)
    maximum_residual = _finite_float(
        validation.get("maximum_reference_residual", 0.02), "maximum_reference_residual", positive=True
    )
    if residual > maximum_residual:
        raise MapBundleError(
            f"reference-coordinate residual {residual:.4f} exceeds {maximum_residual:.4f} m"
        )
    if require_approved:
        # The approval evidence is part of the immutable bundle too. Merely
        # checking that an unhashed report path exists lets it be replaced
        # after approval while all deployment checks continue to pass.
        _artifact(
            manifest_path.parent,
            artifacts.get("validation_report"),
            "validation_report",
        )

    navigation_params = artifacts.get("nav2_params")
    if navigation_params is not None:
        navigation_path = _artifact(manifest_path.parent, navigation_params, "nav2_params")
        _validate_optional_navigation_footprint(
            navigation_path, radius, footprint_tolerance
        )

    return ValidatedMapBundle(
        map_id=map_id,
        occupancy_yaml=occupancy_yaml,
        occupancy_image=occupancy_image,
        navigation_graph=navigation_graph,
        fleet_config=fleet_config,
        footprint_radius=radius,
    )
