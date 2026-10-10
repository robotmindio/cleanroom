"""Reachability and trust-boundary checks for explicit exploration."""

from types import SimpleNamespace
import math
from pathlib import Path

import numpy as np
import pytest

from lekiwi_rmf.exploration import (
    cell_to_world, footprint_is_free, known_safe_cells, load_navigation_footprint,
    map_geometry, select_target, task_limits, with_body_free, world_to_cell,
)

FOOTPRINT, INSCRIBED_RADIUS, REGION_MARGIN = load_navigation_footprint(
    Path(__file__).parents[1] / "config/nav2_params.yaml")


def choose(grid, *, revisit=False, visited=(), blocked=(), position=(0, 0), origin=(-2, -2, 0)):
    return select_target(grid, 0.05, origin, position, (0, 0), 2.5, visited, blocked,
                         clearance=0.22, footprint=FOOTPRINT, region_margin=REGION_MARGIN,
                         observation_distance=0.8, spacing=0.5,
                         revisit_spacing=1.0, free_threshold=20, revisit=revisit)


def test_frontier_approach_is_known_clear_and_connected():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[:, 60:] = -1
    target, stage, area = choose(grid)
    assert target is not None and stage == "exploring_frontiers" and area > 0
    assert 0.2 < target[0] < 0.8  # The actual body remains outside unknown cells.
    assert math.hypot(*target) < 2.5 - REGION_MARGIN
    # A separating wall makes those frontiers unreachable.
    grid[:, 45] = 100
    assert choose(grid, position=(-0.5, 0))[0] is None


def test_known_map_can_be_revisited_without_frontiers():
    grid = np.zeros((80, 80), dtype=np.int16)
    assert choose(grid)[0] is None
    target, stage, _ = choose(grid, revisit=True)
    assert target is not None and stage == "revisiting_known_space"
    following = choose(grid, revisit=True, visited=[target])[0]
    assert following != target
    assert math.dist(target, following) >= 0.5


def test_close_wall_uses_actual_footprint_and_blocks_corner_turns():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[[34, 46], :] = 100  # 55 cm corridor: ~5.5 cm beside the 44 cm body.
    position = (0.025, 0.025)
    assert INSCRIBED_RADIUS == pytest.approx(0.22)
    assert REGION_MARGIN == pytest.approx(1.10)
    assert not known_safe_cells(grid, 0.05, 0.33, 20)[40, 40]
    assert known_safe_cells(grid, 0.05, 0.22, 20)[40, 40]
    assert footprint_is_free(grid, 0.05, (-2, -2, 0), position, 0, FOOTPRINT, 20)
    assert not footprint_is_free(grid, 0.05, (-2, -2, 0), position, math.pi / 4, FOOTPRINT, 20)
    # Map rotation changes the world heading, not collision geometry.
    rotated_origin = (-2, -2, math.pi / 2)
    rotated_position = cell_to_world(40, 40, 0.05, rotated_origin)
    assert footprint_is_free(grid, 0.05, rotated_origin, rotated_position, math.pi / 2, FOOTPRINT, 20)
    assert not footprint_is_free(grid, 0.05, rotated_origin, rotated_position, 3 * math.pi / 4, FOOTPRINT, 20)
    target, _, area = choose(grid, revisit=True, position=position)
    assert target is not None
    assert math.hypot(*target) < 2.5 - REGION_MARGIN
    assert area > 0
    grid[40, 40] = -1  # Unknown under the body is blocked, including its interior.
    assert not footprint_is_free(grid, 0.05, (-2, -2, 0), position, 0, FOOTPRINT, 20)
    grid[40, 40] = 100
    assert not footprint_is_free(grid, 0.05, (-2, -2, 0), position, 0, FOOTPRINT, 20)
    assert not footprint_is_free(grid, 0.05, (-2, -2, 0), (-1.9, 0), 0, FOOTPRINT, 20)
    # These frontier centres pass the coarse disk, but their bodies hit unknown.
    grid[:] = 0
    grid[[33, 47], :] = 100
    grid[40, 55] = -1
    assert known_safe_cells(grid, 0.05, 0.22, 20)[39, 50]
    assert choose(grid, position=position)[0] is None
    target, stage, _ = choose(grid, position=position, revisit=True)
    assert target is not None and stage == "revisiting_known_space"


def test_cells_under_the_current_body_count_as_free_and_nothing_beyond_it():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[44, 44] = 100  # A stale obstacle under the body's front-left corner.
    grid[45, 49] = 100  # A real obstacle beyond the front edge.
    position = (0.025, 0.025)
    assert not footprint_is_free(grid, 0.05, (-2, -2, 0), position, 0, FOOTPRINT, 20)
    cleared = with_body_free(grid, 0.05, (-2, -2, 0), position, 0, FOOTPRINT)
    assert footprint_is_free(cleared, 0.05, (-2, -2, 0), position, 0, FOOTPRINT, 20)
    assert cleared[44, 44] == 0 and cleared[45, 49] == 100 and grid[44, 44] == 100
    assert choose(cleared, revisit=True, position=position)[0] is not None


def test_body_fitting_a_45_cm_corridor_needs_no_rounded_extra_clearance():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[[35, 45], :] = 100
    position = (0.025, 0.025)
    assert footprint_is_free(grid, 0.05, (-2, -2, 0), position, 0, FOOTPRINT, 20)
    assert choose(grid, revisit=True, position=position)[0] is not None


@pytest.mark.parametrize("yaw", [0.0, 0.65, math.pi / 4])
def test_rotated_current_body_is_not_rejected_by_grid_clearance_rounding(yaw):
    grid = np.full((80, 80), -1, dtype=np.int16)
    position = (0.026, 0.026)
    grid = with_body_free(grid, 0.05, (-2, -2, 0), position, yaw, FOOTPRINT)
    assert footprint_is_free(grid, 0.05, (-2, -2, 0), position, yaw, FOOTPRINT, 20)
    assert choose(grid, revisit=True, position=position)[0] is None


def test_unknown_or_unsafe_start_and_blocked_regions_are_not_traversed():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[:, 60:] = -1
    first = choose(grid)[0]
    second = choose(grid, blocked=[first])[0]
    assert second is None or math.dist(first, second) >= 0.5
    grid[40, 40] = 100
    with pytest.raises(ValueError, match="clearance"):
        choose(grid)
    grid[:] = -1
    with pytest.raises(ValueError, match="known space"):
        choose(grid)


def test_rotated_map_origin_round_trip():
    origin = (1, -1, math.pi / 2)
    for cell in ((0, 0), (7, 18)):
        xy = cell_to_world(*cell, 0.05, origin)
        assert world_to_cell(xy, 0.05, origin) == cell


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), 901])
def test_goal_limits_cannot_bypass_configuration(value):
    with pytest.raises(ValueError):
        task_limits(value, 0, 900, 5)
    assert task_limits(0, 0, 900, 5) == (900, 5)
    assert task_limits(30, 1, 900, 5) == (30, 1)


def test_map_metadata_is_validated_before_allocation():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"), data=[0] * 100,
        info=SimpleNamespace(width=10, height=10, resolution=0.05,
                             origin=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=0),
                                                    orientation=SimpleNamespace(x=0, y=0, z=0, w=1))),
    )
    assert map_geometry(message, 100)[0].shape == (10, 10)
    with pytest.raises(ValueError, match="bounded"):
        map_geometry(message, 99)
    message.data[0] = 101
    with pytest.raises(ValueError, match="cells"):
        map_geometry(message, 100)
