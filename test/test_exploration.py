"""Reachability and trust-boundary checks for explicit exploration."""

from types import SimpleNamespace
import math

import numpy as np
import pytest

from lekiwi_rmf.exploration import (
    STOPPING_MARGIN_M, cell_to_world, known_safe_cells, map_geometry, select_target, task_limits, world_to_cell,
)


def choose(grid, *, revisit=False, visited=(), blocked=(), position=(0, 0), origin=(-2, -2, 0)):
    return select_target(grid, 0.05, origin, position, (0, 0), 1.8, visited, blocked,
                         clearance=0.33, observation_distance=0.8, spacing=0.5,
                         revisit_spacing=1.0, free_threshold=20, revisit=revisit)


def test_frontier_approach_is_known_clear_and_connected():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[:, 60:] = -1
    target, stage, area = choose(grid)
    assert target is not None and stage == "exploring_frontiers" and area > 0
    assert 0.2 < target[0] < 0.65  # Remains a full footprint away from unknown.
    assert math.hypot(*target) < 1.8 - 0.38
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


def test_folded_body_clearance_retains_the_region_stopping_margin():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[40, 48] = 100  # 40 cm from the pose cell's centre.
    assert not known_safe_cells(grid, 0.05, 0.38, 20)[40, 40]
    assert known_safe_cells(grid, 0.05, 0.33, 20)[40, 40]
    target, _, area = choose(grid, revisit=True)
    assert target is not None
    assert math.hypot(*target) < 1.8 - 0.33 - STOPPING_MARGIN_M
    region_cells = sum(((-2 + (x + 0.5) * 0.05) ** 2
                        + (-2 + (y + 0.5) * 0.05) ** 2 <= (1.8 - 0.38) ** 2)
                       for y in range(80) for x in range(80))
    assert area == pytest.approx(region_cells * 0.05 ** 2)
    grid[40, 46] = 100  # A cell inside the body's envelope still blocks it.
    with pytest.raises(ValueError, match="clearance"):
        choose(grid, revisit=True)


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
