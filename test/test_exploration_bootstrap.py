"""Exploration must acquire observations when a first map offers no global target."""

import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
import numpy as np
import pytest

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration import laser_free_grid, needs_observation, sweep_is_free
from lekiwi_rmf.exploration_node import Paused
from test_exploration_ros import graph as graph, response, wait
from test_exploration_nav2 import run_nav2_case


ROOT = Path(__file__).parents[1]
BODY = np.array([[.24, .22], [.24, -.22], [-.22, -.22], [-.22, .22]])


def test_empty_candidates_require_observation_at_unknown_boundaries_and_cropped_map_edges():
    grid = np.zeros((40, 40), dtype=np.int16)
    args = (.05, (-1., -1., 0.), (0., 0.), (0., 0.), 1.8, 1.1, 20)
    assert not needs_observation(grid, *args)
    grid[:, 24:] = -1
    assert needs_observation(grid, *args)
    assert needs_observation(np.zeros((4, 4), dtype=np.int16), .05, (-.1, -.1, 0.), *args[2:])
    # A fully observed enclosing wall really can finish without entering space behind it.
    grid[:, 23] = 100
    assert not needs_observation(grid, *args)


def test_laser_rays_do_not_clear_missing_returns_or_cells_at_and_behind_obstacles():
    grid = laser_free_grid((40, 40), .05, (-1., -1., 0.), (0., 0., 0.),
                           [float('nan'), .5, float('inf')], -.1, .1, .05, 4.)
    assert grid[20, 25] == 0
    assert grid[20, 30] == -1 and grid[20, 35] == -1
    assert grid[19, 25] == -1 and grid[21, 25] == -1


def test_swept_body_checks_intermediate_rotation_and_translation_and_rejects_unknown():
    grid = np.zeros((80, 80), dtype=np.int16)
    # A narrow corridor admits either endpoint but not the intermediate body rotation.
    grid[45:, :] = 100
    assert not sweep_is_free(grid, .05, (-2., -2., 0.), (0., 0., 0.), (0., 0., math.pi / 2), BODY, 20)
    grid[:] = 0
    grid[40, 47] = -1
    assert not sweep_is_free(grid, .05, (-2., -2., 0.), (0., 0., 0.), (.15, 0., 0.), BODY, 20)


def test_tracked_behavior_distances_match_checked_sweeps_and_keep_collision_checks():
    for filename, tag, port, value in (
        ('explore_observe.xml', 'Spin', 'spin_dist', math.pi / 4),
        ('explore_observe_reverse.xml', 'Spin', 'spin_dist', -math.pi / 4),
        ('explore_bootstrap.xml', 'DriveOnHeading', 'dist_to_travel', .15),
        ('explore_bootstrap_reverse.xml', 'BackUp', 'backup_dist', .15),
    ):
        behavior = ET.parse(ROOT / 'config' / filename).find(f'.//{tag}')
        assert float(behavior.get(port)) == value
        assert behavior.get('disable_collision_checks', 'false') == 'false'


def first_map(peers, explorer, *, cropped=False):
    if cropped:
        peers.grid.info.origin.position.x = .5
    peers.grid.data = [-1] * 6400
    wait(lambda: explorer._map[0][40, 40] == -1)


@pytest.mark.parametrize('cropped', [False, True])
def test_first_map_observes_then_finds_and_visits_a_global_frontier(graph, cropped):
    peers, explorer, client = graph
    first_map(peers, explorer, cropped=cropped)
    peers.nav_mode = 'success'
    peers.observation_reveal_after = 1
    if cropped:
        # New post-turn observations expand the original grid around the robot.
        original = peers.navigate

        def navigate(goal):
            result = original(goal)
            peers.grid.info.origin.position.x = -2.
            return result

        peers.navigator._execute_callback = navigate
    feedback = []
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5), feedback_callback=feedback.append))
    result = response(handle.get_result_async(), timeout=15)
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.complete
    assert result.result.visited_targets == 1
    assert peers.observation_goals == ['explore_observe.xml']
    assert peers.planner_ids == ['ExploreKnown']
    assert any(item.feedback.stage == 'observing_without_frontier' for item in feedback)


def test_zero_frontiers_does_not_report_success_or_dispatch_a_global_route_and_cancel_stops_it(graph):
    peers, explorer, client = graph
    first_map(peers, explorer)
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    wait(lambda: peers.nav_active)
    assert peers.observation_goals == ['explore_observe.xml']
    assert peers.planner_ids == [] and not handle.get_result_async().done()
    response(handle.cancel_goal_async())
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not result.result.complete and peers.nav_canceled == 1 and not peers.nav_active


@pytest.mark.parametrize('backward', [False, True])
def test_full_scan_without_frontiers_advances_only_through_measured_local_space(graph, backward):
    peers, explorer, client = graph
    first_map(peers, explorer)
    peers.nav_mode = 'success'
    movement = 'explore_bootstrap_reverse.xml' if backward else 'explore_bootstrap.xml'
    peers.observation_reveal_on = movement
    if backward:
        # The front is unobserved by this scan; the rear remains measured free.
        original_publish = peers.scan_pub.publish

        def scan(message):
            message.ranges = [value if math.cos(message.angle_min + index * message.angle_increment) < 0
                              else float('inf') for index, value in enumerate(message.ranges)]
            original_publish(message)

        peers.scan_pub.publish = scan
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    result = response(handle.get_result_async(), timeout=20)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert peers.observation_goals[-1] == movement
    assert len(peers.observation_goals) >= 9
    assert set(peers.observation_goals[:-1]) == {'explore_observe.xml'}
    assert result.result.visited_targets == 1 and peers.planner_ids == ['ExploreKnown']


@pytest.mark.parametrize('cell', [-1, 100])
def test_blocked_local_sweep_finishes_incomplete_instead_of_claiming_region_complete(graph, cell):
    peers, explorer, client = graph
    first_map(peers, explorer)
    peers.local_grid.data = [cell] * 6400
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED and not result.result.complete
    assert 'no collision-free' in result.result.message
    assert peers.nav_count == 0 and peers.planner_ids == []


def test_continuing_observation_retains_the_original_duration_bound(graph):
    peers, explorer, client = graph
    first_map(peers, explorer)
    peers.nav_mode = 'success'
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5, max_duration_sec=2.0)))
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED and not result.result.complete
    assert result.result.message == 'exploration duration reached'
    assert peers.observation_goals and not peers.planner_ids and not peers.nav_active


def test_persistent_occupied_global_start_observes_and_translates_before_retrying_the_target(graph):
    from nav2_msgs.action import ComputePathToPose

    peers, _, client = graph
    peers.plan_error = ComputePathToPose.Result.START_OCCUPIED
    peers.nav_mode = 'success'
    navigate = peers.navigate

    def recover_start(goal):
        if goal.request.behavior_tree.endswith('explore_bootstrap.xml'):
            peers.plan_error = 0
        return navigate(goal)

    peers.navigator._execute_callback = recover_start
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    result = response(handle.get_result_async(), timeout=20)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert peers.observation_goals == ['explore_observe.xml'] * 8 + ['explore_bootstrap.xml']
    assert result.result.visited_targets == 1 and result.result.unreachable_targets == 0


@pytest.mark.parametrize('fault', ['scan', 'obstacle', 'permission', 'collision'])
def test_bootstrap_cancels_its_owned_motion_when_live_guards_fail(graph, fault):
    from nav2_msgs.msg import CollisionMonitorState

    peers, explorer, client = graph
    first_map(peers, explorer)
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    wait(lambda: peers.nav_active)
    if fault == 'scan':
        peers.publish_scan = False
    elif fault == 'obstacle':
        peers.local_grid.data = [100] * 6400
    elif fault == 'permission':
        peers.permitted = False
    else:
        peers.collision = CollisionMonitorState(action_type=CollisionMonitorState.STOP, polygon_name='StopZone')
    # The fake action acknowledges cancellation before its execute callback
    # clears nav_active. Wait for the stop, not the intermediate acknowledgement.
    wait(lambda: peers.nav_canceled == 1 and not peers.nav_active)
    assert not peers.nav_active
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED


def test_stale_scan_and_local_costmap_frames_are_rejected_before_motion(graph):
    peers, explorer, _ = graph
    wait(lambda: explorer._scan is not None)
    peers.publish_scan = False
    time.sleep(1.1)
    with pytest.raises(Paused, match='scan is stale'):
        explorer._local_observation_grid(lambda: None)
    peers.publish_scan = True
    wait(lambda: time.monotonic() - explorer._scan[1] < .1)
    peers.local_grid.header.frame_id = 'map'
    with pytest.raises(Paused, match='frame odom'):
        explorer._local_observation_grid(lambda: None)


@pytest.mark.parametrize('behavior', ['bootstrap', 'bootstrap_advance', 'cancel_bootstrap'])
def test_native_navigator_observes_without_a_global_path_and_cancels_its_child(tmp_path, behavior):
    run_nav2_case(tmp_path, behavior)
