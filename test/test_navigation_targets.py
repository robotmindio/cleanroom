"""A map point becomes a base pose that faces it from a clear, known place; no ROS."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from lekiwi_rmf.exploration import footprint_is_free, known_safe_cells, load_navigation_footprint, world_to_cell
from lekiwi_rmf.navigation_targets import MIN_STANDOFF_M, NoStandoff, approach_pose

CONFIG = Path(__file__).parents[1] / 'config'
FOOTPRINT = load_navigation_footprint(CONFIG / 'nav2_params.yaml')[0]
EXPLORER = yaml.safe_load((CONFIG / 'exploration.yaml').read_text())['robot_explorer']['ros__parameters']
CLEARANCE, FREE = EXPLORER['clearance_m'], EXPLORER['free_threshold']
RESOLUTION, ORIGIN = 0.05, (-3.0, -3.0, 0.0)


def room(value=0, size=120):
    return np.full((size, size), value, dtype=np.int16)


def fill(grid, low, high, value, origin=ORIGIN):
    """Set the cells of the map-frame box low..high."""
    (x0, y0), (x1, y1) = world_to_cell(low, RESOLUTION, origin), world_to_cell(high, RESOLUTION, origin)
    grid[y0:y1 + 1, x0:x1 + 1] = value


def approach(grid, point, standoff, robot, origin=ORIGIN):
    return approach_pose(grid, RESOLUTION, origin, point, standoff, robot,
                         clearance=CLEARANCE, footprint=FOOTPRINT, free_threshold=FREE)


def admissible(grid, pose, point, standoff, origin=ORIGIN):
    """What every answer must satisfy: at the standoff, facing the point, on the explorer's own terms."""
    x, y, yaw = pose
    cell = world_to_cell((x, y), RESOLUTION, origin)
    return (math.dist((x, y), point) == pytest.approx(standoff)
            and math.remainder(yaw - math.atan2(point[1] - y, point[0] - x), 2 * math.pi) == pytest.approx(0, abs=1e-9)
            and bool(known_safe_cells(grid, RESOLUTION, CLEARANCE, FREE)[cell[1], cell[0]])
            and footprint_is_free(grid, RESOLUTION, origin, (x, y), yaw, FOOTPRINT, FREE))


@pytest.mark.parametrize('robot,expected', [
    ((-2.0, 0.0), (-0.6, 0.0, 0.0)), ((2.0, 0.0), (0.6, 0.0, math.pi)),
    ((0.0, 2.5), (0.0, 0.6, -math.pi / 2)), ((0.0, -0.1), (0.0, -0.6, math.pi / 2))])
def test_open_space_gives_the_pose_nearest_the_robot_facing_the_point(robot, expected):
    grid = room()
    pose = approach(grid, (0.0, 0.0), 0.6, robot)
    assert pose[:2] == pytest.approx(expected[:2], abs=1e-9)
    assert math.remainder(pose[2] - expected[2], 2 * math.pi) == pytest.approx(0, abs=1e-9)
    assert admissible(grid, pose, (0.0, 0.0), 0.6)


def test_a_rotated_map_origin_is_honoured():
    origin = (-1.0, -4.0, 0.4)
    grid = room(size=160)
    pose = approach(grid, (1.0, 0.5), 0.8, (-0.5, 0.5), origin)
    # Within one step along the circle of the pose straight between the robot and the point.
    assert math.dist(pose[:2], (0.2, 0.5)) < RESOLUTION and admissible(grid, pose, (1.0, 0.5), 0.8, origin)


def test_a_blocked_circle_is_refused_as_blocked():
    with pytest.raises(NoStandoff) as refusal:
        approach(room(100), (0.0, 0.0), 0.6, (-2.0, 0.0))
    assert refusal.value.unmapped is False
    # Free only where the point is: nowhere on the circle has room for the body.
    grid = room(100)
    fill(grid, (-0.2, -0.2), (0.2, 0.2), 0)
    with pytest.raises(NoStandoff) as refusal:
        approach(grid, (0.0, 0.0), 0.6, (-2.0, 0.0))
    assert refusal.value.unmapped is False


def test_unknown_space_is_never_a_standoff_and_is_reported_as_unmapped():
    with pytest.raises(NoStandoff) as refusal:
        approach(room(-1), (0.0, 0.0), 0.6, (-2.0, 0.0))
    assert refusal.value.unmapped is True
    # The robot's side of the circle is unmapped: the answer is the nearest pose that is known.
    grid = room()
    fill(grid, (-3.0, -3.0), (0.0, 2.95), -1)
    pose = approach(grid, (0.0, 0.0), 0.6, (-2.0, 0.0))
    assert pose[0] > 0.2 and admissible(grid, pose, (0.0, 0.0), 0.6)
    # A point beyond the map's edge has part of its circle outside the grid.
    grid = room()
    fill(grid, (2.0, -3.0), (2.95, 2.95), 100)
    with pytest.raises(NoStandoff) as refusal:
        approach(grid, (2.9, 0.0), 0.6, (0.0, 0.0))
    assert refusal.value.unmapped is True


def test_a_point_inside_an_obstacle_is_approached_from_outside_its_body():
    grid = room()
    fill(grid, (-0.2, -0.2), (0.2, 0.2), 100)  # A table; the point is on it.
    pose = approach(grid, (0.0, 0.0), 0.7, (-2.0, 0.3))
    assert admissible(grid, pose, (0.0, 0.0), 0.7) and pose[0] < -0.6
    # Too close for the body to clear the table: refused, never squeezed in.
    with pytest.raises(NoStandoff) as refusal:
        approach(grid, (0.0, 0.0), MIN_STANDOFF_M, (-2.0, 0.3))
    assert refusal.value.unmapped is False


def test_a_wall_between_hides_the_point_even_from_the_nearest_pose():
    grid = room()
    fill(grid, (-0.45, -3.0), (-0.4, 2.95), 100)  # A wall west of the point, between it and the robot.
    pose = approach(grid, (0.0, 0.0), 0.9, (-2.0, 0.0))
    # West of the wall is nearer and clear, but the point cannot be seen from there.
    assert pose[0] > -0.4 and admissible(grid, pose, (0.0, 0.0), 0.9)
    assert footprint_is_free(grid, RESOLUTION, ORIGIN, (-0.9, 0.0), 0.0, FOOTPRINT, FREE)


def test_an_obstacle_under_the_body_moves_the_answer_along_the_circle():
    grid = room()
    fill(grid, (-0.75, -0.02), (-0.72, 0.02), 100)  # Behind the nearest pose, under the body's rear.
    pose = approach(grid, (0.0, 0.0), 0.6, (-2.0, 0.0))
    assert pose[:2] != pytest.approx((-0.6, 0.0), abs=1e-3) and admissible(grid, pose, (0.0, 0.0), 0.6)
    assert not footprint_is_free(grid, RESOLUTION, ORIGIN, (-0.6, 0.0), 0.0, FOOTPRINT, FREE)


def test_every_answer_on_cluttered_maps_passes_the_explorers_own_checks():
    random = np.random.default_rng(7)
    answered = 0
    for _ in range(40):
        grid = room()
        for _ in range(12):
            x, y = random.uniform(-2.5, 2.3, size=2)
            fill(grid, (x, y), (x + random.uniform(0.05, 0.5), y + random.uniform(0.05, 0.5)),
                 int(random.choice([100, -1])))
        point = tuple(random.uniform(-1.5, 1.5, size=2))
        standoff = float(random.uniform(MIN_STANDOFF_M, 1.2))
        try:
            pose = approach(grid, point, standoff, tuple(random.uniform(-2.5, 2.5, size=2)))
        except NoStandoff:
            continue
        answered += 1
        assert admissible(grid, pose, point, standoff)
    assert answered >= 10


@pytest.mark.parametrize('point,standoff,robot', [
    ((math.nan, 0.0), 0.6, (0.0, 0.0)), ((0.0, 0.0), 0.0, (0.0, 0.0)), ((0.0, 0.0), -1.0, (0.0, 0.0)),
    ((0.0, 0.0), 0.6, (math.inf, 0.0)), ((0.0, 0.0), math.nan, (0.0, 0.0))])
def test_non_finite_inputs_are_errors_not_poses(point, standoff, robot):
    with pytest.raises(ValueError) as error:
        approach(room(), point, standoff, robot)
    assert not isinstance(error.value, NoStandoff)
