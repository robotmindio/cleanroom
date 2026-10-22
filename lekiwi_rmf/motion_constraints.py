"""Geometric constraints on the arm's tool, their numeric check and a kinematic search over them.

Pure and independent of ROS: forward kinematics is injected, so the same search runs on
MoveIt's model in the robot (``motion_solver``) and on a toy chain in tests. Nothing here
names a task. A request describes where the tool must be (a region around a point or an
observed object) and how it must point (one tool axis within a cone around a direction);
the solver finds joint values that satisfy every constraint, and ``judge`` decides whether
a measured pose satisfies them, with the target's measurement uncertainty kept visible.
"""

from dataclasses import dataclass
import math
import re
import time

import numpy as np
from scipy.optimize import least_squares

SCHEMA = 1
OPERATIONS = ('solve', 'verify')
MAX_CONSTRAINTS = 4
MAX_REGION_M = 0.5
MAX_TIME_SEC = 10.0
OBSERVATION_ID = re.compile(r'^[0-9a-f]{64}$')
FRAME = re.compile(r'^[A-Za-z0-9_/.-]{1,128}$')
# The solver keeps this much inside each constraint, so the executed motion (MoveIt's goal
# joint tolerance) still satisfies the unmargined check afterwards.
POSITION_MARGIN_M = 0.01
ORIENTATION_MARGIN_RAD = 0.03


def _vector(value, name, *, unit=False):
    if (not isinstance(value, list) or len(value) != 3
            or any(type(item) not in (int, float) or not math.isfinite(item) for item in value)):
        raise ValueError(f'{name} must be three finite numbers')
    vector = np.asarray(value, dtype=float)
    if unit:
        norm = float(np.linalg.norm(vector))
        if norm < 1e-6:
            raise ValueError(f'{name} must be a nonzero direction')
        vector = vector / norm
    return vector


def _number(value, name, lower, upper):
    if type(value) not in (int, float) or not math.isfinite(value) or not lower < value <= upper:
        raise ValueError(f'{name} must be in ({lower}, {upper}]')
    return float(value)


def _target(value):
    if not isinstance(value, dict) or len(value) != 1:
        raise ValueError('target is exactly one of point {frame, xyz} or observation {id, detection}')
    if 'point' in value:
        point = value['point']
        if not isinstance(point, dict) or set(point) != {'frame', 'xyz'}:
            raise ValueError('point requires frame and xyz')
        if not isinstance(point['frame'], str) or not FRAME.match(point['frame']):
            raise ValueError('point frame must be a TF frame name')
        return {'point': {'frame': point['frame'], 'xyz': _vector(point['xyz'], 'point xyz').tolist()}}
    if 'observation' in value:
        reference = value['observation']
        if (not isinstance(reference, dict) or set(reference) != {'id', 'detection'}
                or not isinstance(reference['id'], str) or not OBSERVATION_ID.match(reference['id'])
                or type(reference['detection']) is not int or not 0 <= reference['detection'] < 20):
            raise ValueError('observation requires a 64-hex id and a detection index 0..19')
        return {'observation': dict(reference)}
    raise ValueError('target is exactly one of point or observation')


def _constraint(value):
    if not isinstance(value, dict):
        raise ValueError('each constraint must be an object')
    kind = value.get('kind')
    if kind == 'position':
        allowed = {'kind', 'target', 'offset_m', 'shape', 'radius_m', 'half_extents_m'}
        if set(value) - allowed or 'target' not in value:
            raise ValueError('a position constraint takes target, offset_m and a sphere or box shape')
        shape = value.get('shape', 'sphere')
        result = {'kind': 'position', 'target': _target(value['target']),
                  'offset_m': _vector(value.get('offset_m', [0, 0, 0]), 'offset_m').tolist(), 'shape': shape}
        if np.linalg.norm(result['offset_m']) > MAX_REGION_M:
            raise ValueError(f'offset_m must stay within {MAX_REGION_M} m of the target')
        if shape == 'sphere':
            if 'half_extents_m' in value:
                raise ValueError('a sphere takes radius_m')
            result['radius_m'] = _number(value.get('radius_m'), 'radius_m', 0, MAX_REGION_M)
        elif shape == 'box':
            if 'radius_m' in value:
                raise ValueError('a box takes half_extents_m')
            extents = _vector(value.get('half_extents_m'), 'half_extents_m')
            if any(not 0 < item <= MAX_REGION_M for item in extents):
                raise ValueError(f'half_extents_m must be in (0, {MAX_REGION_M}]')
            result['half_extents_m'] = extents.tolist()
        else:
            raise ValueError('shape must be sphere or box')
        return result
    if kind == 'orientation':
        if set(value) != {'kind', 'axis', 'direction', 'tolerance_rad'}:
            raise ValueError('an orientation constraint takes axis, direction and tolerance_rad')
        return {'kind': 'orientation', 'axis': _vector(value['axis'], 'axis', unit=True).tolist(),
                'direction': _vector(value['direction'], 'direction', unit=True).tolist(),
                'tolerance_rad': _number(value['tolerance_rad'], 'tolerance_rad', 0, math.pi)}
    raise ValueError('constraint kind must be position or orientation')


def parse_request(values):
    """Validate a ``/robot/motion/solve`` request; frames and units are the caller's contract."""
    if not isinstance(values, dict) or values.get('op') not in OPERATIONS:
        raise ValueError(f'op must be one of {OPERATIONS}')
    allowed = {'op', 'constraints', 'plan', 'max_time_sec'}
    if set(values) - allowed:
        raise ValueError(f'unknown fields {sorted(set(values) - allowed)}')
    constraints = values.get('constraints')
    if not isinstance(constraints, list) or not 1 <= len(constraints) <= MAX_CONSTRAINTS:
        raise ValueError(f'constraints must hold 1..{MAX_CONSTRAINTS} constraints')
    plan = values.get('plan', True)
    if type(plan) is not bool:
        raise ValueError('plan must be boolean')
    return {'op': values['op'], 'constraints': [_constraint(item) for item in constraints], 'plan': plan,
            'max_time_sec': _number(values.get('max_time_sec', 5.0), 'max_time_sec', 0, MAX_TIME_SEC)}


@dataclass(frozen=True)
class Resolved:
    """A constraint with its target in the planning frame and the target's measurement uncertainty."""

    kind: str
    center: np.ndarray = None
    shape: str = 'sphere'
    size: np.ndarray = None
    uncertainty_m: float = 0.0
    axis: np.ndarray = None
    direction: np.ndarray = None
    tolerance_rad: float = 0.0


def resolve(constraint, center=None, uncertainty_m=0.0):
    """Bind a parsed constraint to a planning-frame target position (``None`` for orientation)."""
    if constraint['kind'] == 'orientation':
        return Resolved('orientation', axis=np.asarray(constraint['axis']),
                        direction=np.asarray(constraint['direction']), tolerance_rad=constraint['tolerance_rad'])
    size = np.asarray([constraint['radius_m']] if constraint['shape'] == 'sphere' else constraint['half_extents_m'])
    return Resolved('position', center=np.asarray(center, dtype=float) + np.asarray(constraint['offset_m']),
                    shape=constraint['shape'], size=size, uncertainty_m=float(uncertainty_m))


def _excess(item, position, slack):
    """How far ``position`` lies outside the region shrunk by ``slack`` (negative inside)."""
    delta = np.abs(position - item.center)
    if item.shape == 'sphere':
        return np.atleast_1d(np.linalg.norm(delta) - (item.size[0] - slack))
    return delta - (item.size - slack)


def _angle(item, rotation):
    cosine = float(np.clip(np.dot(rotation @ item.axis, item.direction), -1.0, 1.0))
    return math.acos(cosine)


def residuals(resolved, position, rotation):
    """Nonnegative violation of every constraint, with the solver's margins and uncertainty."""
    parts = []
    for item in resolved:
        if item.kind == 'position':
            parts.append(np.maximum(_excess(item, position, item.uncertainty_m + POSITION_MARGIN_M), 0.0))
        else:
            # Radians and metres share one vector; a radian here weighs like 0.1 m of position.
            parts.append(np.atleast_1d(max(0.0, _angle(item, rotation) - (item.tolerance_rad - ORIENTATION_MARGIN_RAD)) * .1))
    return np.concatenate(parts)


def solvable(resolved):
    """Why no margin is left to solve inside, or ``None``."""
    for item in resolved:
        if item.kind == 'position' and np.min(item.size) <= item.uncertainty_m + POSITION_MARGIN_M:
            return (f'the target uncertainty {item.uncertainty_m:.3f} m leaves no room inside a '
                    f'{np.min(item.size):.3f} m region')
        if item.kind == 'orientation' and item.tolerance_rad <= ORIENTATION_MARGIN_RAD:
            return f'tolerance_rad must exceed {ORIENTATION_MARGIN_RAD}'
    return None


def judge(resolved, position, rotation):
    """Whether a measured tool pose satisfies every constraint: ``holds``, ``violated`` or ``uncertain``.

    A position counts as inside only when it is inside by more than the target's uncertainty,
    and outside only when it is outside by more than it; in between nobody can tell.
    """
    details, verdicts = [], []
    for item in resolved:
        if item.kind == 'position':
            inside = float(np.max(_excess(item, position, item.uncertainty_m)))
            outside = float(np.max(_excess(item, position, -item.uncertainty_m)))
            verdict = 'holds' if inside <= 0 else 'violated' if outside > 0 else 'uncertain'
            details.append({'kind': 'position', 'verdict': verdict,
                            'distance_m': float(np.linalg.norm(position - item.center)),
                            'uncertainty_m': item.uncertainty_m})
        else:
            angle = _angle(item, rotation)
            verdict = 'holds' if angle <= item.tolerance_rad else 'violated'
            details.append({'kind': 'orientation', 'verdict': verdict, 'angle_rad': angle,
                            'tolerance_rad': item.tolerance_rad})
        verdicts.append(verdict)
    overall = 'violated' if 'violated' in verdicts else 'uncertain' if 'uncertain' in verdicts else 'holds'
    return overall, details


def halton(count, dimensions):
    """Deterministic low-discrepancy samples in [0, 1)^dimensions."""
    primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29][:dimensions]
    samples = np.zeros((count, dimensions))
    for column, base in enumerate(primes):
        for row in range(count):
            value, fraction, index = 0.0, 1.0 / base, row + 1
            while index:
                value += fraction * (index % base)
                index //= base
                fraction /= base
            samples[row, column] = value
    return samples


class Exhausted(Exception):
    """The forward-kinematics budget or deadline ran out."""


class KinematicSearch:
    """Joint values whose tool pose satisfies the constraints, found on an injected FK model.

    A deterministic low-discrepancy table of joint samples (built once, through the same FK)
    seeds bounded least-squares refinement from the closest samples. The search is not
    complete: finding nothing within its budget is reported as such, never as impossibility.
    """

    def __init__(self, fk, lower, upper, samples=256, seeds=6, max_evaluations=1500):
        self._fk, self._samples, self._seeds = fk, samples, seeds
        self.lower, self.upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        self.max_evaluations = max_evaluations
        self._table = None

    def _poses(self, budget):
        if self._table is None:
            joints = self.lower + halton(self._samples, len(self.lower)) * (self.upper - self.lower)
            # The table costs one FK per sample once; it is reused by every later request.
            self._table = [(row, *self._fk(row)) for row in joints]
            budget['used'] += len(joints)
        return self._table

    def solve(self, resolved, current, deadline):
        """Candidate joint vectors that satisfy the constraints, closest to ``current`` first."""
        budget = {'used': 0}
        current = np.clip(np.asarray(current, dtype=float), self.lower, self.upper)

        def cost(position, rotation):
            return float(np.sum(residuals(resolved, position, rotation) ** 2))

        def evaluate(joints):
            if budget['used'] >= self.max_evaluations or time.monotonic() >= deadline:
                raise Exhausted()
            budget['used'] += 1
            return residuals(resolved, *self._fk(joints))

        table = self._poses(budget)
        start = evaluate(current)
        seeds = [(float(np.sum(start ** 2)), current)]
        seeds += sorted(((cost(position, rotation), joints) for joints, position, rotation in table),
                        key=lambda item: item[0])[:self._seeds]
        found = []
        for score, seed in seeds:
            if score == 0.0:
                found.append(seed)
                continue
            try:
                fit = least_squares(evaluate, seed, bounds=(self.lower, self.upper), method='trf',
                                    diff_step=1e-3, xtol=1e-9, ftol=1e-12, gtol=1e-12, max_nfev=40)
            except Exhausted:
                break
            if np.max(fit.fun) <= 1e-9:
                found.append(np.clip(fit.x, self.lower, self.upper))
        unique = []
        for joints in sorted(found, key=lambda item: float(np.linalg.norm(item - current))):
            if all(np.max(np.abs(joints - other)) > 1e-3 for other in unique):
                unique.append(joints)
        return unique, budget['used']
