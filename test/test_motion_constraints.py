"""Constraint parsing, the uncertainty-aware check and the kinematic search, on a toy 5-joint arm."""

import math
import time

import numpy as np
import pytest

from lekiwi_rmf.motion_constraints import (
    KinematicSearch, judge, parse_request, residuals, resolve, solvable,
)


def _rotation(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def toy_fk(joints):
    """Pan about z, three pitches about y and a roll about the tool's x: an SO-101-like 5-DOF chain."""
    pan, lift, elbow, wrist, roll = joints
    rotation = _rotation((0, 0, 1), pan)
    position = np.array([0.0, 0.0, 0.1])
    for angle, length in ((lift, 0.12), (elbow, 0.12), (wrist, 0.06)):
        rotation = rotation @ _rotation((0, 1, 0), angle)
        position = position + rotation @ np.array([length, 0.0, 0.0])
    return position, rotation @ _rotation((1, 0, 0), roll)


LOWER, UPPER = [-2.0, -1.8, -1.7, -1.7, -2.8], [2.0, 1.8, 1.7, 1.7, 2.8]


def sphere(xyz, radius, **extra):
    return {'kind': 'position', 'target': {'point': {'frame': 'base_link', 'xyz': xyz}},
            'shape': 'sphere', 'radius_m': radius, **extra}


def resolved(request, uncertainty=0.0):
    return [resolve(item, item['target']['point']['xyz'] if item['kind'] == 'position' else None, uncertainty)
            for item in parse_request(request)['constraints']]


@pytest.mark.parametrize('request_value', [
    {}, {'op': 'move'}, {'op': 'solve'}, {'op': 'solve', 'constraints': []},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.05)] * 5},
    {'op': 'solve', 'constraints': [sphere([0, 0, math.nan], 0.05)]},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.0)]},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.6)]},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.05, half_extents_m=[0.1, 0.1, 0.1])]},
    {'op': 'solve', 'constraints': [{'kind': 'position', 'target': {'observation': {'id': 'x', 'detection': 0}},
                                     'radius_m': 0.05}]},
    {'op': 'solve', 'constraints': [{'kind': 'position', 'target': {'point': {'frame': 'a b', 'xyz': [0, 0, 0]}},
                                     'radius_m': 0.05}]},
    {'op': 'solve', 'constraints': [{'kind': 'orientation', 'axis': [0, 0, 0], 'direction': [0, 0, 1],
                                     'tolerance_rad': 0.1}]},
    {'op': 'solve', 'constraints': [{'kind': 'orientation', 'axis': [1, 0, 0], 'direction': [0, 0, 1],
                                     'tolerance_rad': 4}]},
    {'op': 'solve', 'constraints': [{'kind': 'joint'}]},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.05)], 'plan': 'yes'},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.05)], 'max_time_sec': 60},
    {'op': 'solve', 'constraints': [sphere([0, 0, 0], 0.05)], 'execute': True},
])
def test_invalid_requests_are_refused(request_value):
    with pytest.raises(ValueError):
        parse_request(request_value)


def test_judge_keeps_target_uncertainty_visible():
    constraint = resolved({'op': 'verify', 'constraints': [sphere([0.3, 0, 0.1], 0.05)]}, uncertainty=0.02)
    identity = np.eye(3)
    assert judge(constraint, np.array([0.3, 0, 0.1]), identity)[0] == 'holds'
    # 4 cm from the centre: inside the nominal sphere, but not by more than the 2 cm uncertainty.
    assert judge(constraint, np.array([0.34, 0, 0.1]), identity)[0] == 'uncertain'
    assert judge(constraint, np.array([0.38, 0, 0.1]), identity)[0] == 'violated'


def test_orientation_is_one_axis_within_a_cone():
    constraint = resolved({'op': 'verify', 'constraints': [
        {'kind': 'orientation', 'axis': [1, 0, 0], 'direction': [0, 0, -1], 'tolerance_rad': 0.2}]})
    down = _rotation((0, 1, 0), math.pi / 2)
    assert judge(constraint, np.zeros(3), down)[0] == 'holds'
    # Rotation about the aligned axis is free.
    assert judge(constraint, np.zeros(3), down @ _rotation((1, 0, 0), 2.0))[0] == 'holds'
    assert judge(constraint, np.zeros(3), _rotation((0, 1, 0), math.pi / 2 - 0.3))[0] == 'violated'


def test_uncertainty_larger_than_the_region_cannot_be_solved():
    constraint = resolved({'op': 'solve', 'constraints': [sphere([0.3, 0, 0.1], 0.03)]}, uncertainty=0.025)
    assert 'uncertainty' in solvable(constraint)


def test_search_reaches_a_region_with_an_aligned_tool():
    constraint = resolved({'op': 'solve', 'constraints': [
        sphere([0.18, 0.08, 0.05], 0.03),
        {'kind': 'orientation', 'axis': [1, 0, 0], 'direction': [0, 0, -1], 'tolerance_rad': 0.15}]})
    search = KinematicSearch(toy_fk, LOWER, UPPER)
    found, used, stopped = search.solve(constraint, np.zeros(5), time.monotonic() + 10)
    assert found and used <= search.max_evaluations and stopped in (None, 'budget')
    for joints in found:
        assert judge(constraint, *toy_fk(joints))[0] == 'holds'
        assert np.all(joints >= LOWER) and np.all(joints <= UPPER)
        assert not np.any(residuals(constraint, *toy_fk(joints)))


def test_search_reports_nothing_for_an_unreachable_region_within_its_budget():
    constraint = resolved({'op': 'solve', 'constraints': [sphere([0.0, 0.0, 0.48], 0.05)]})
    search = KinematicSearch(toy_fk, LOWER, UPPER, max_evaluations=600)
    found, used, stopped = search.solve(constraint, np.zeros(5), time.monotonic() + 10)
    assert found == [] and used <= 600 and stopped in (None, 'budget')


def test_search_respects_its_deadline():
    constraint = resolved({'op': 'solve', 'constraints': [sphere([0.0, 0.0, 0.48], 0.05)]})
    search = KinematicSearch(toy_fk, LOWER, UPPER, samples=16)
    started = time.monotonic()
    found, _, stopped = search.solve(constraint, np.zeros(5), started + 0.05)
    assert found == [] and stopped == 'deadline' and time.monotonic() - started < 1.0


def test_current_joints_that_already_satisfy_are_preferred():
    position, _ = toy_fk(np.zeros(5))
    constraint = resolved({'op': 'solve', 'constraints': [sphere(position.tolist(), 0.05)]})
    found, _, _ = KinematicSearch(toy_fk, LOWER, UPPER).solve(constraint, np.zeros(5), time.monotonic() + 10)
    assert np.allclose(found[0], np.zeros(5))
