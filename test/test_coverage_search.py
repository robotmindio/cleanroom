"""Coverage geometry, information-gain selection, termination and replanning without ROS."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from lekiwi_rmf.coverage_search import (
    Capture, Detection, Mount, Pose, Profile, QueryCoverage, SearchSession, Viewpoint, camera_from_info,
    detections_from_record, heading_gains, observed_cells, optical_axes, pose_from_transform, searchable_mask, select_viewpoint,
    viewpoint_gain,
)
from lekiwi_rmf.exploration import load_navigation_footprint, world_to_cell
from lekiwi_rmf.search_octomap import SearchOctomap

FREE_SPACE = SearchOctomap.from_binary(bytes((0x55, 0x55)), .05)
RESOLUTION = 0.05
ORIGIN = (-3.0, -3.0, 0.0)
# 640x480 with a 60-degree horizontal field of view.
CAMERA = camera_from_info([554.3, 0, 320, 0, 554.3, 240, 0, 0, 1], 640, 480)
PROFILE_REVISION = 'e' * 64  # The proposal profile admitted with each model.
PROFILE = Profile(min_object_m=0.08, min_object_px=24, max_range_m=3.0, min_height_m=0.02,
                  max_height_m=0.3, max_age_sec=600.0)
FOOTPRINT, _, MARGIN = load_navigation_footprint(Path(__file__).parents[1] / 'config/nav2_params.yaml')


def free_grid(size=120):
    return np.zeros((size, size), dtype=np.int16)


def capture(identifier='a' * 64, pose=Pose(0.0, 0.0, 0.2, 0.0, 0.0), detections=(), stamp=100.0,
            model='m' * 64, stationary=True, query=None, profile=PROFILE_REVISION):
    return Capture(identifier, 'front', model, stamp, pose, CAMERA, stationary, tuple(detections), query, profile)


def cell_xy(flat, grid):
    ys, xs = np.unravel_index(flat, grid.shape)
    return ORIGIN[0] + (xs + 0.5) * RESOLUTION, ORIGIN[1] + (ys + 0.5) * RESOLUTION


def test_camera_model_and_resolution_limited_range():
    assert CAMERA.left == pytest.approx(math.radians(30), abs=1e-3)
    assert CAMERA.right == pytest.approx(CAMERA.left)
    # A 0.08 m object spans 24 pixels out to f·size/px.
    assert PROFILE.range_limit(CAMERA) == pytest.approx(554.3 * 0.08 / 24)
    with pytest.raises(ValueError):
        camera_from_info([0, 0, 320, 0, 554.3, 240, 0, 0, 1], 640, 480)


def test_optical_axis_from_transform():
    # Optical +Z along map +X, optical +X along map -Y (the ROS optical convention on a forward camera).
    pose = pose_from_transform((1.0, 2.0, 0.2), (-0.5, 0.5, -0.5, 0.5))
    assert (pose.x, pose.y, pose.z) == (1.0, 2.0, 0.2)
    assert pose.yaw == pytest.approx(0.0, abs=1e-9) and pose.pitch == pytest.approx(0.0, abs=1e-9)
    with pytest.raises(ValueError):
        pose_from_transform((0, 0, 0), (0, 0, 0, 2))


@pytest.mark.parametrize('roll', [math.pi / 2, -math.pi / 2, math.pi])
def test_camera_roll_preserves_the_full_optical_orientation(roll):
    # Rotate the usual forward optical frame around its optical +Z axis.
    c, s = math.cos(roll / 2), math.sin(roll / 2)
    q = ((s - c) / 2, (c + s) / 2, (s - c) / 2, (c + s) / 2)
    pose = pose_from_transform((0, 0, .2), q)
    assert pose.roll == pytest.approx(roll)
    forward, right, down = optical_axes(pose.yaw, pose.pitch, pose.roll)
    assert forward == pytest.approx((1., 0., 0.))
    assert right == pytest.approx((0., -math.cos(roll), -math.sin(roll)))
    assert down == pytest.approx((0., math.sin(roll), -math.cos(roll)))


def test_vertical_axis_and_unrolled_pitch_have_consistent_optical_frames():
    vertical = pose_from_transform((0, 0, 1.), (0, 0, 0, 1))
    assert vertical.pitch == pytest.approx(math.pi / 2)
    assert optical_axes(vertical.yaw, vertical.pitch, vertical.roll)[1] == pytest.approx((1., 0., 0.))
    pitch = .3
    c, s = math.cos(pitch / 2), math.sin(pitch / 2)
    q = ((s - c) / 2, (c - s) / 2, -(c + s) / 2, (c + s) / 2)
    assert pose_from_transform((0, 0, .2), q).pitch == pytest.approx(pitch)


def test_frustum_resolution_band_and_occlusion():
    grid = free_grid()
    wall_x = 1.0
    wx, _ = world_to_cell((wall_x, 0.0), RESOLUTION, ORIGIN)
    grid[:, wx] = 100
    grid[:40, :] = -1  # Unknown space below y=-1 m.
    pose = Pose(0.0, 0.0, 0.2, 0.0, 0.0)
    flat, _, occluded = observed_cells(grid, RESOLUTION, ORIGIN, pose, CAMERA, PROFILE)
    xs, ys = cell_xy(np.unique(flat), grid)
    limit = PROFILE.range_limit(CAMERA)
    distance = np.hypot(xs, ys)
    assert xs.max() <= wall_x + RESOLUTION  # Nothing behind the wall.
    assert np.any(np.isclose(xs, wall_x + RESOLUTION / 2))  # The wall's surface is observed.
    assert np.all(distance <= limit + RESOLUTION)
    assert np.all(np.abs(np.arctan2(ys, xs)) <= CAMERA.left + 0.05)
    assert np.all(ys >= -1.0)  # Unknown cells are never credited.
    assert occluded > 0
    # Near the camera the 0.02–0.3 m band is below the vertical field of view.
    near = (0.2 - 0.02) / math.tan(CAMERA.down)
    assert distance.min() >= near - RESOLUTION


def test_negative_evidence_respects_model_motion_bearing_and_age():
    grid = free_grid()
    coverage = QueryCoverage('Cup', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    assert coverage.add(capture(model='o' * 64), grid, geometry=FREE_SPACE) == []
    assert not coverage.covered(100).any()
    coverage.add(capture(stationary=False), grid, geometry=FREE_SPACE)
    assert not coverage.covered(100).any()
    # A cup at the left edge: its bearing is a possible location, the rest is evidence of absence.
    cup = Detection('cup', 0.9, left=0.5, right=0.35)
    other = Detection('chair', 0.9, left=0.1, right=-0.1)
    assert coverage.add(capture(detections=[cup, other]), grid, geometry=FREE_SPACE) == [cup]
    covered = coverage.covered(100)
    xs, ys = cell_xy(np.flatnonzero(covered), grid)
    bearings = np.arctan2(ys, xs)
    assert covered.any() and not np.any((bearings > 0.37) & (bearings < 0.48))
    assert np.any(np.abs(bearings) < 0.05)  # The chair's bearing still proves no cup there.
    assert (coverage.sighting > coverage.negative).any()
    assert coverage.counts == {'captures': 3, 'negative_captures': 1, 'other_model': 1, 'other_profile': 0, 'other_query': 0,
                               'moving': 1, 'occluded_cells': 0, 'no_geometry': 0}
    assert not coverage.covered(100 + PROFILE.max_age_sec + 1).any()
    # A later look without the cup refutes the earlier sighting there.
    coverage.add(capture(stamp=200.0), grid, geometry=FREE_SPACE)
    assert not (coverage.sighting > coverage.negative).any()


def test_open_inferences_count_only_for_labels_their_query_asked_for():
    grid = free_grid()
    coverage = QueryCoverage('screwdriver', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    # A frame that localized only cups says nothing about a screwdriver, though its model could have.
    assert coverage.add(capture(query=('a cup', 'a mug')), grid, geometry=FREE_SPACE) == []
    assert not coverage.covered(100).any() and coverage.counts['other_query'] == 1
    coverage.add(capture(query=('screwdriver',)), grid, geometry=FREE_SPACE)
    assert coverage.covered(100).any() and coverage.counts['negative_captures'] == 1


def test_map_growth_keeps_world_aligned_coverage_and_raster_change_discards_it():
    grid = free_grid()
    coverage = QueryCoverage('cup', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    coverage.add(capture(), grid, geometry=FREE_SPACE)
    before = {tuple(np.round(v, 6)) for v in zip(*cell_xy(np.flatnonzero(coverage.covered(100)), grid))}
    grown = (-4.0, -3.5, 0.0)
    assert not coverage.rebase((150, 150), RESOLUTION, grown)
    ys, xs = np.nonzero(coverage.covered(100))
    after = {(round(grown[0] + (x + 0.5) * RESOLUTION, 6), round(grown[1] + (y + 0.5) * RESOLUTION, 6))
             for y, x in zip(ys, xs)}
    assert after == before
    assert coverage.rebase((150, 150), 0.1, grown)
    assert not coverage.covered(100).any()


def test_detection_bearings_and_map_positions_from_observation_records():
    record = {'detections': [
        {'label': ' Cup ', 'score': 0.8, 'bbox': [320, 0, 40, 10]},
        {'label': 'cup', 'score': 0.7, 'bbox': [0, 0, 10, 10],
         'position': {'map': {'frame_id': 'map', 'map_id': 'x' * 64, 'xyz': [1, 2, 0.1], 'uncertainty_m': 0.1}}}]}
    first, second = detections_from_record(record, CAMERA, 'x' * 64)
    assert first.label == 'cup' and first.left == pytest.approx(0) and first.right < 0
    assert second.map_xyz == (1.0, 2.0, 0.1)
    assert detections_from_record(record, CAMERA, 'y' * 64)[1].map_xyz is None


def mount():
    return Mount('front', CAMERA, 0.2, 0.0, 0.0)


def select(weights, grid, position=(0.0, 0.0), **overrides):
    options = dict(clearance=0.22, footprint=FOOTPRINT, region_margin=MARGIN, free_threshold=20, spacing=0.5,
                   headings=12, max_candidates=200, overhead_m=1.0, min_gain=5.0)
    options.update(overrides)
    return select_viewpoint(weights, grid, RESOLUTION, ORIGIN, position, (0.0, 0.0), 6.0, [mount()], PROFILE,
                            **options)


def test_information_gain_turns_toward_unobserved_space_before_driving():
    grid = free_grid()
    weights = np.zeros(grid.shape)
    _, ys = cell_xy(np.arange(grid.size), grid)
    weights.reshape(-1)[ys > 0.3] = 1.0  # Only the space to the robot's left is unobserved.
    gains = heading_gains(weights, grid, RESOLUTION, ORIGIN, (0.0, 0.0), mount(), PROFILE, 12)
    assert np.argmax(gains) == 3  # +90 degrees.
    viewpoint = select(weights, grid)
    assert viewpoint is not None and viewpoint.gain >= 5
    assert math.cos(viewpoint.yaw - math.pi / 2) > 0.8


def test_prior_sightings_and_travel_cost_shape_the_choice():
    grid = free_grid()
    weights = np.zeros(grid.shape)
    xs, ys = cell_xy(np.arange(grid.size), grid)
    near = (np.abs(xs - 0.6) < 0.25) & (np.abs(ys) < 0.25)
    far = (np.abs(xs + 1.8) < 0.25) & (np.abs(ys) < 0.25)
    weights.reshape(-1)[near | far] = 1.0
    nearer = select(weights, grid)
    assert nearer is not None and math.cos(nearer.yaw) > 0.8  # Equal mass: the closer region wins.
    weights.reshape(-1)[far] += 20.0  # The label was last seen over there.
    prior = select(weights, grid)
    assert prior is not None and (math.cos(prior.yaw) < -0.8 or prior.x < -0.5)


def test_unreachable_or_fully_observed_space_has_no_viewpoint():
    grid = free_grid()
    assert select(np.zeros(grid.shape), grid) is None
    walled = grid.copy()
    walled[:, 80:83] = 100  # Mass hidden behind a wall beyond camera range and reach.
    weights = np.zeros(grid.shape)
    weights[:, 100:] = 1.0
    assert select(weights, walled) is None
    with pytest.raises(ValueError):
        select(weights, np.full(grid.shape, -1, dtype=np.int16))


def session(labels=('cup', 'chair'), target=0.5):
    grid = free_grid(60)
    perception = {'cameras': {'front': 'c' * 64}, 'recognizers': [
        {'role': 'primary', 'model_sha256': 'm' * 64, 'model_name': 'test', 'profile': {'revision': PROFILE_REVISION},
         'labels': list(labels), 'open_vocabulary': False}]}
    origin = (-1.5, -1.5, 0.0)
    result = SearchSession('CUP', perception, PROFILE, (grid, RESOLUTION, origin), started=50.0, target=target,
                           region=((0.0, 0.0), 1.5))
    return result, grid, origin


def test_termination_found_coverage_and_incapable_with_honest_report():
    unsupported, grid, _ = session(labels=('chair',))
    assert unsupported.outcome(unsupported.masks(grid)[0], 100) == 'incapable'
    report = unsupported.report('incapable', grid, 100, {})
    assert 'nothing is claimed' in report['statement'] and not report['perception']['label_supported']

    search, grid, origin = session()
    searchable, _ = search.masks(grid)
    assert search.outcome(searchable, 100) is None
    # A stored sighting from before the search is a prior, never a result.
    assert not search.integrate(capture(detections=[Detection('cup', 0.9, 0.1, -0.1)], stamp=10.0), grid, geometry=FREE_SPACE)
    assert search.prior_count == 1 and search.outcome(searchable, 100) is None
    for yaw in np.arange(0, 2 * math.pi, 0.5):
        search.integrate(capture(identifier=f'{int(yaw * 10):064d}', pose=Pose(0, 0, 0.2, yaw, 0)), grid, geometry=FREE_SPACE)
    fraction = search.fraction(searchable, 100)
    assert 0.5 <= fraction < 1.0
    assert search.outcome(searchable, 100) == 'coverage_reached'
    report = search.report('coverage_reached', grid, 100, {'viewpoints_visited': 0})
    assert report['coverage']['fraction'] == pytest.approx(fraction, abs=1e-4)
    assert report['coverage']['unobserved_area_m2'] > 0 and report['coverage']['unobserved_regions']
    assert 'not evidence of absence there' in report['statement']
    # The prior sighting's capture also proved absence everywhere off the cup's bearing.
    assert report['negative_evidence']['observation_count'] == 14
    assert len(report['negative_evidence']['recent_observation_ids']) == 14
    assert report['negative_evidence']['model_sha256s'] == ['m' * 64]
    assert report['negative_evidence']['observations_by_model'] == {'m' * 64: 14}

    found = search.integrate(capture(identifier='f' * 64, detections=[Detection('cup', 0.8, 0.1, 0.0)],
                                     stamp=120.0), grid, geometry=FREE_SPACE)
    assert found and search.outcome(searchable, 130) == 'found'
    assert search.evidence_ids() == ['f' * 64]


def test_new_coverage_en_route_refutes_the_planned_viewpoint():
    grid = free_grid()
    coverage = QueryCoverage('cup', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    searchable, _ = searchable_mask(grid, RESOLUTION, ORIGIN, (0.0, 0.0), 2.5)
    plan = Viewpoint(0.5, 0.0, 0.0, 'front', 0.0, 0.5, 0.0)
    planned = viewpoint_gain(plan, coverage.weights(searchable, 100, 20.0), grid, RESOLUTION, ORIGIN, mount(), PROFILE)
    assert planned > 50
    coverage.add(capture(pose=Pose(0.2, 0.0, 0.2, 0.0, 0.0)), grid, geometry=FREE_SPACE)
    remaining = viewpoint_gain(plan, coverage.weights(searchable, 100, 20.0), grid, RESOLUTION, ORIGIN, mount(), PROFILE)
    assert remaining < 0.5 * planned


def profile_text(**recognizer):
    import json
    item = {'role': 'primary', 'model_sha256': 'a' * 64, 'labels': ['cup'], 'open_vocabulary': False,
            'profile': {'revision': PROFILE_REVISION}, **recognizer}
    return json.dumps({'schema': 1, 'cameras': {'front': 'c' * 64}, 'recognizers': [item]})


@pytest.mark.parametrize('text', [
    'invalid', '[]', '{}', '{"schema":2}', profile_text(model_sha256='x'), profile_text(labels='cup'),
    profile_text(labels=['']), profile_text(role='backup'), profile_text(open_vocabulary=1),
    profile_text(profile=None), profile_text(profile={'revision': 'short'}),
    '{"schema":1,"cameras":{},"recognizers":[]}', ' ' * 65537,
])
def test_malformed_perception_profile_means_no_instrument(text):
    from lekiwi_rmf.search_evidence import parse_profile
    with pytest.raises((ValueError, RecursionError)):
        parse_profile(text)
    parse_profile(profile_text())


def test_fallback_frames_never_cover_labels_only_the_primary_can_report():
    """An open-vocabulary primary and a closed-set fallback: each frame speaks only for its own catalogue."""
    grid = free_grid(60)
    origin = (-1.5, -1.5, 0.0)
    perception = {'cameras': {'front': 'c' * 64}, 'recognizers': [
        {'role': 'primary', 'model_sha256': 'p' * 64, 'labels': ['a cup', 'a mug'], 'open_vocabulary': True,
         'profile': {'revision': PROFILE_REVISION}},
        {'role': 'fallback', 'model_sha256': 'f' * 64, 'labels': ['chair', 'book'], 'open_vocabulary': False,
         'profile': {'revision': PROFILE_REVISION}}]}

    def search(query):
        return SearchSession(query, perception, PROFILE, (grid, RESOLUTION, origin), started=50.0, target=0.5,
                             region=((0.0, 0.0), 1.5))

    primary_only = search('a mug')
    assert primary_only.supported and primary_only.coverage.models == {'p' * 64: PROFILE_REVISION}
    primary_only.integrate(capture(model='f' * 64), grid, geometry=FREE_SPACE)
    searchable, _ = primary_only.masks(grid)
    assert primary_only.fraction(searchable, 100) == 0.0
    assert primary_only.coverage.counts['other_model'] == 1
    primary_only.integrate(capture(model='p' * 64), grid, geometry=FREE_SPACE)
    assert primary_only.fraction(searchable, 100) > 0
    report = primary_only.report(None, grid, 100, {})
    assert [(r['role'], r['label_supported']) for r in report['perception']['recognizers']] == [
        ('primary', True), ('fallback', False)]
    assert report['negative_evidence']['observations_by_model'] == {'p' * 64: 1}
    fallback_only = search('book')
    fallback_only.integrate(capture(model='p' * 64), grid, geometry=FREE_SPACE)
    assert fallback_only.fraction(searchable, 100) == 0.0
    assert not search('unicorn').supported


def test_reprojection_moves_coverage_with_the_corrected_map():
    """#90 gap: a SLAM correction moves earlier evidence instead of leaving it at stale coordinates."""
    search, grid, _ = session()
    before = capture(identifier='1' * 64, pose=Pose(0.0, 0.0, 0.2, 0.0, 0.0), stamp=10.0,
                     detections=[Detection('cup', 0.9, 0.3, 0.2)])
    search.integrate(before, grid, geometry=FREE_SPACE)
    covered_before = search.coverage.negative > -np.inf
    assert covered_before.any() and search.prior_count == 1
    # Spatial memory reprojects the same capture 0.5 m further along y in the corrected revision.
    after = capture(identifier='1' * 64, pose=Pose(0.0, 0.5, 0.2, 0.0, 0.0), stamp=10.0,
                    detections=[Detection('cup', 0.9, 0.3, 0.2)])
    assert not search.reproject([after], grid, 'r2', geometry_for=lambda _: FREE_SPACE)
    covered_after = search.coverage.negative > -np.inf
    shift = round(0.5 / RESOLUTION)
    assert np.array_equal(covered_after[shift:], covered_before[:-shift])
    assert search.prior_count == 1  # Rebuilt, not counted twice.
    assert search.revisions[-1]['reason'] == 'map revision r2: coverage reprojected from 1 anchored captures'
    # A capture found during the search stays found after the rebuild.
    found = capture(identifier='2' * 64, detections=[Detection('cup', 0.9, 0.1, -0.1)], stamp=200.0)
    assert search.integrate(found, grid, geometry=FREE_SPACE)
    assert search.reproject([after, found], grid, 'r3', geometry_for=lambda _: FREE_SPACE) is False and search.evidence_ids()[-1] == '2' * 64
    # A frame whose own query did not ask for the label gains it no coverage after reprojection either.
    unasked = Capture('3' * 64, 'front', 'm' * 64, 300.0, Pose(0.0, 0.0, 0.2, 0.0, 0.0), CAMERA, True, (), ('chair',))
    search.reproject([unasked], grid, 'r4', geometry_for=lambda _: FREE_SPACE)
    assert not (search.coverage.negative > -np.inf).any()


def test_a_frame_under_another_profile_is_silent_not_negative_evidence():
    """A profile change mid-search must not let frames under other thresholds claim this session's coverage."""
    grid = free_grid()
    coverage = QueryCoverage('cup', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    coverage.add(capture(profile='d' * 64), grid, geometry=FREE_SPACE)
    assert coverage.counts['other_profile'] == 1 and coverage.counts['negative_captures'] == 0
    coverage.add(capture(), grid, geometry=FREE_SPACE)
    assert coverage.counts['negative_captures'] == 1
    with pytest.raises(ValueError, match='profile revision'):
        QueryCoverage('cup', ['m' * 64], PROFILE, grid.shape, RESOLUTION, ORIGIN)


def native_octomap():
    # Fixture generated by native OctoMap, not by the decoder under test.
    return SearchOctomap.from_binary((Path(__file__).parent / 'data/search_octomap_3d.bin').read_bytes(), .05)


def test_native_octomap_known_free_occupied_and_unknown_rays():
    geometry = native_octomap()
    assert geometry.ray_free((.025, .025, .525), (1.525, .025, .525))
    assert not geometry.ray_free((.025, .025, .225), (1.525, .025, .225))
    assert not geometry.ray_free((.025, .025, .525), (4., .025, .525))
    assert not geometry.ray_free((math.nan, 0, 0), (1, 0, 0))
    assert not geometry.ray_free((0, 0, 0), (math.inf, 0, 0))


def test_ray_on_an_occupied_voxel_face_does_not_slip_through():
    geometry = native_octomap()
    # z=.35 touches the top face of the overhang's occupied z=.30..35 voxel.
    assert not geometry.ray_free((.025, .025, .35), (1.525, .025, .35))


@pytest.mark.parametrize('payload', [b'', b'\x55', b'\xff\xff', b'\x55\x55\x00', b'\x03\x00' * 17])
def test_octomap_rejects_truncated_trailing_or_overdeep_data(payload):
    with pytest.raises(ValueError):
        SearchOctomap.from_binary(payload, .05)


@pytest.mark.parametrize('resolution', [0., -1., math.nan, math.inf, 2., True, '.05'])
def test_octomap_rejects_invalid_resolution(resolution):
    with pytest.raises(ValueError):
        SearchOctomap.from_binary(b'\x55\x55', resolution)


def test_missing_or_unknown_3d_geometry_never_proves_absence_but_preserves_a_sighting():
    grid = free_grid()
    coverage = QueryCoverage('cup', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    coverage.add(capture(), grid)
    assert not coverage.covered(100).any() and coverage.counts['no_geometry'] == 1
    coverage.add(capture(), grid, SearchOctomap.from_binary(b'\x00\x00', .05))
    assert not coverage.covered(100).any()
    cup = Detection('cup', .9, .1, -.1, map_xyz=(1., 0., .1))
    assert coverage.add(capture(detections=[cup]), grid) == [cup]
    assert (coverage.sighting > -np.inf).any()


def test_3d_overhang_blocks_negative_evidence_even_with_an_empty_2d_map():
    grid = free_grid()
    pose = Pose(.025, .025, .225, 0., 0.)
    plain, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, pose, CAMERA, PROFILE, FREE_SPACE)
    blocked, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, pose, CAMERA, PROFILE, native_octomap())
    x, y = world_to_cell((1.1, .025), RESOLUTION, ORIGIN)
    target = y * grid.shape[1] + x
    assert target in plain
    assert target not in blocked
    assert len(blocked) < len(plain)


def test_capture_time_3d_visibility_follows_the_corrected_camera_anchor():
    grid = free_grid()
    original = Pose(.025, .025, .225, 0., 0.)
    geometry = native_octomap()
    before, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, original, CAMERA, PROFILE, geometry)
    moved = Pose(.025, .525, .225, 0., 0.)
    anchored = replace(geometry, reference_pose=(original.x, original.y, original.z, original.yaw, original.pitch, original.roll))
    after, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, moved, CAMERA, PROFILE, anchored)
    before_mask, after_mask = np.zeros(grid.shape, bool), np.zeros(grid.shape, bool)
    before_mask.reshape(-1)[before], after_mask.reshape(-1)[after] = True, True
    shift = round(.5 / RESOLUTION)
    assert before_mask.any() and np.array_equal(after_mask[shift:], before_mask[:-shift])
    # A rotated corrected anchor must rotate the original overhang, too.
    rotated = Pose(.025, .525, .225, math.pi / 2, 0.)
    flat, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, rotated, CAMERA, PROFILE, anchored)
    x, y = world_to_cell((.025, 1.6), RESOLUTION, ORIGIN)
    assert y * grid.shape[1] + x not in flat


def test_rolled_frustum_swaps_horizontal_and_vertical_limits():
    grid = free_grid()
    pose = Pose(0., 0., .2, 0., 0.)
    upright, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, pose, CAMERA, PROFILE, FREE_SPACE)
    rolled, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, replace(pose, roll=math.pi / 2), CAMERA, PROFILE, FREE_SPACE)
    x, y = world_to_cell((1.2, .58), RESOLUTION, ORIGIN)
    target = y * grid.shape[1] + x
    assert target in upright and target not in rolled
    assert len(rolled) > 0


def test_vertical_camera_only_credits_the_ground_when_looking_down():
    grid = free_grid()
    down = pose_from_transform((0, 0, 1.), (1, 0, 0, 0))
    up = pose_from_transform((0, 0, 1.), (0, 0, 0, 1))
    seen, _, _ = observed_cells(grid, RESOLUTION, ORIGIN, down, CAMERA, PROFILE, FREE_SPACE)
    assert len(seen) > 0
    assert len(observed_cells(grid, RESOLUTION, ORIGIN, up, CAMERA, PROFILE, FREE_SPACE)[0]) == 0


@pytest.mark.parametrize('pose', [Pose(0., 0., .2, 0., 0., math.pi / 2), Pose(0., 0., .2, 0., -.1)])
def test_tilted_image_only_sighting_cannot_clear_an_ambiguous_bearing(pose):
    grid = free_grid()
    coverage = QueryCoverage('cup', {'m' * 64: PROFILE_REVISION}, PROFILE, grid.shape, RESOLUTION, ORIGIN)
    cup = Detection('cup', .9, .1, -.1)
    assert coverage.add(capture(pose=pose, detections=[cup]), grid, FREE_SPACE) == [cup]
    assert not coverage.covered(100).any()


def test_volume_visibility_checks_between_the_corner_rays():
    geometry = native_octomap()
    start, low, high = (.025, .025, .225), (1.4, -.9, .025), (1.5, .9, .9)
    # Individual rays can clear the overhang while the volume between them intersects it.
    assert geometry.ray_free(start, (1.5, .9, .9))
    assert not geometry.volume_free(start, low, high)
    assert geometry.volume_free((.025, .025, .525), (1.4, -.1, .5), (1.5, .1, .6))
