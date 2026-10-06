"""Planar polygon and segment geometry shared by safety and map validation."""

from __future__ import annotations

import math

import yaml


def polygon(value, description: str) -> tuple[tuple[float, float], ...]:
    """Parse one simple finite polygon from a YAML value or ROS string value."""
    if isinstance(value, str):
        try:
            value = yaml.safe_load(value)
        except yaml.YAMLError as error:
            raise ValueError(f"{description} is not valid YAML") from error
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError(f"{description} must contain at least three points")
    points = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"{description} points must be [x, y] pairs")
        if any(isinstance(coordinate, bool) for coordinate in point):
            raise ValueError(f"{description} coordinates must be numeric")
        try:
            parsed = tuple(float(coordinate) for coordinate in point)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{description} coordinates must be numeric") from error
        if not all(math.isfinite(coordinate) for coordinate in parsed):
            raise ValueError(f"{description} coordinates must be finite")
        points.append(parsed)
    if len(set(points)) != len(points):
        raise ValueError(f"{description} contains duplicate points")
    twice_area = sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )
    if abs(twice_area) <= 1e-12:
        raise ValueError(f"{description} has zero area")
    return tuple(points)


def same_polygon(first, second, tolerance: float = 1e-9) -> bool:
    """Compare polygon vertices independent of start vertex and winding."""
    if len(first) != len(second):
        return False
    for candidate in (second, tuple(reversed(second))):
        for offset in range(len(candidate)):
            rotated = candidate[offset:] + candidate[:offset]
            if all(
                math.dist(left, right) <= tolerance
                for left, right in zip(first, rotated)
            ):
                return True
    return False


def point_in_polygon(point, vertices) -> bool:
    """Return true for points inside or on the boundary of a simple polygon."""
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(vertices, vertices[1:] + vertices[:1]):
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) <= 1e-12 and min(x1, x2) - 1e-12 <= x <= max(x1, x2) + 1e-12 \
                and min(y1, y2) - 1e-12 <= y <= max(y1, y2) + 1e-12:
            return True
        if (y1 > y) != (y2 > y):
            boundary_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < boundary_x:
                inside = not inside
    return inside


def point_segment_distance(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 0.0:
        return math.dist(point, start)
    projection = max(0.0, min(1.0, (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / denominator))
    return math.dist(point, (start[0] + projection * dx, start[1] + projection * dy))


def segment_distance(first_start, first_end, second_start, second_end) -> float:
    # Proper or endpoint intersection has zero clearance. Collinear cases are
    # also caught by the endpoint-to-segment distances below.
    def orientation(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    orientations = (
        orientation(first_start, first_end, second_start),
        orientation(first_start, first_end, second_end),
        orientation(second_start, second_end, first_start),
        orientation(second_start, second_end, first_end),
    )
    if orientations[0] * orientations[1] < 0.0 and orientations[2] * orientations[3] < 0.0:
        return 0.0
    return min(
        point_segment_distance(first_start, second_start, second_end),
        point_segment_distance(first_end, second_start, second_end),
        point_segment_distance(second_start, first_start, first_end),
        point_segment_distance(second_end, first_start, first_end),
    )


def polygon_boundary_distance(first, second) -> float:
    return min(
        segment_distance(first_start, first_end, second_start, second_end)
        for first_start, first_end in zip(first, first[1:] + first[:1])
        for second_start, second_end in zip(second, second[1:] + second[:1])
    )
