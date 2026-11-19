"""Navigation targets that are not a base pose yet; pure and ROS-free.

A map point (an object's location, a spot on a table) becomes the base pose from which
the robot faces it at a requested distance. The pose is admitted with the explorer's own
checks on the occupancy map: the clearance prefilter and the whole oriented Nav2 footprint
on known free cells. Nav2 still plans and validates the route.
"""

import math

from lekiwi_rmf.exploration import footprint_is_free, known_safe_cells, world_to_cell

# The body reaches 0.274 m ahead of its origin; closer than that the point would be under it.
MIN_STANDOFF_M, DEFAULT_STANDOFF_M, MAX_STANDOFF_M = 0.3, 0.6, 3.0
MAX_BEARINGS = 360


class NoStandoff(ValueError):
    """No admissible pose on the standoff circle; ``unmapped`` when part of it is unknown space."""

    def __init__(self, unmapped):
        super().__init__('part of the standoff circle is unmapped' if unmapped
                         else 'every pose on the standoff circle is blocked or cannot see the point')
        self.unmapped = unmapped


def _sees(free, resolution, origin, start, point):
    """Whether the straight line from ``start`` reaches the point or the body that contains it.

    Walking back from the point, the occupied or unknown cells it lies in are its own body
    (a table, a shelf); every cell before them must be free, so a wall in between hides it.
    """
    steps = max(1, math.ceil(math.dist(start, point) / (resolution / 2)))
    in_body = True
    for step in range(steps + 1):
        fraction = step / steps
        x, y = world_to_cell((point[0] + fraction * (start[0] - point[0]), point[1] + fraction * (start[1] - point[1])),
                             resolution, origin)
        clear = 0 <= y < free.shape[0] and 0 <= x < free.shape[1] and bool(free[y, x])
        if clear:
            in_body = False
        elif not in_body:
            return False
    return True


def approach_pose(grid, resolution, origin, point, standoff, robot, *, clearance, footprint, free_threshold):
    """The pose (x, y, yaw) ``standoff`` metres from ``point`` that faces it, nearest to ``robot``.

    Raises NoStandoff when no pose on that circle is known, clear and in sight of the point.
    """
    if not all(math.isfinite(value) for value in (*point, *robot, standoff)) or standoff <= 0:
        raise ValueError('point, robot position and standoff must be finite, with a positive standoff')
    safe = known_safe_cells(grid, resolution, clearance, free_threshold)
    free = (grid >= 0) & (grid <= free_threshold)
    count = min(MAX_BEARINGS, max(16, math.ceil(2 * math.pi * standoff / resolution)))
    candidates = []
    for index in range(count):
        bearing = 2 * math.pi * index / count
        position = (point[0] + standoff * math.cos(bearing), point[1] + standoff * math.sin(bearing))
        candidates.append((math.dist(position, robot), index, position, math.remainder(bearing + math.pi, 2 * math.pi)))
    unmapped = False
    for _, _, position, yaw in sorted(candidates):
        x, y = world_to_cell(position, resolution, origin)
        if not (0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]) or grid[y, x] < 0:
            unmapped = True
        elif (safe[y, x] and _sees(free, resolution, origin, position, point)
              and footprint_is_free(grid, resolution, origin, position, yaw, footprint, free_threshold)):
            return position[0], position[1], yaw
    raise NoStandoff(unmapped)
