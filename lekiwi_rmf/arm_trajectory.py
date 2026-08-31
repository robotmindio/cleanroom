import json
import math
from dataclasses import dataclass
from pathlib import Path


ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
JOINT_LIMITS = {
    "arm_shoulder_pan": (-1.92, 1.92),
    "arm_shoulder_lift": (-1.75, 1.75),
    "arm_elbow_flex": (-1.75, 1.75),
    "arm_wrist_flex": (-1.75, 1.75),
    # Bounded consistently with the real revolute URDF joint and its wiring.
    "arm_wrist_roll": (-math.pi, math.pi),
    "arm_gripper": (0.0, 1.57),
}
JOINT_VELOCITY_LIMITS = {
    "arm_shoulder_pan": 2.0,
    "arm_shoulder_lift": 2.0,
    "arm_elbow_flex": 2.0,
    "arm_wrist_flex": 3.0,
    "arm_wrist_roll": 3.0,
    "arm_gripper": 2.0,
}
JOINT_ACCELERATION_LIMITS = {
    "arm_shoulder_pan": 3.0,
    "arm_shoulder_lift": 3.0,
    "arm_elbow_flex": 3.0,
    "arm_wrist_flex": 5.0,
    "arm_wrist_roll": 5.0,
    "arm_gripper": 3.0,
}


@dataclass(frozen=True)
class TrajectoryPoint:
    """A validated trajectory point in ROS units.

    ``None`` means that a derivative was not supplied.  Keeping that distinction
    is important: position-only segments are linear, position/velocity segments
    are cubic, and fully specified segments are quintic, matching the interpolation
    contract used by ros2_control's joint trajectory controller.
    """

    time: float
    positions: dict
    velocities: dict | None
    accelerations: dict | None


def load_calibration(path):
    zero_positions = dict.fromkeys(ARM_JOINTS, 0.0)
    directions = dict.fromkeys(ARM_JOINTS, 1.0)
    calibration = Path(path).expanduser()
    if not calibration.exists():
        return zero_positions, directions
    try:
        data = json.loads(calibration.read_text())
        zero_positions = {name: float(data["zero_positions"][name]) for name in ARM_JOINTS}
        directions = {name: float(data.get("directions", {})[name]) for name in ARM_JOINTS}
    except (KeyError, OSError, TypeError, ValueError) as error:
        raise ValueError(f"invalid arm calibration {calibration}: {error}") from error
    if not all(math.isfinite(value) for value in zero_positions.values()):
        raise ValueError(f"invalid arm calibration {calibration}: non-finite zero position")
    if any(value not in (-1.0, 1.0) for value in directions.values()):
        raise ValueError(f"invalid arm calibration {calibration}: directions must be -1 or 1")
    return zero_positions, directions


def joint_positions(observation, zero_positions, directions):
    raw = {
        name: float(observation.get(f"{name}.pos", 0.0)) / 100.0 * math.pi / 2
        if name == "arm_gripper"
        else math.radians(float(observation.get(f"{name}.pos", 0.0)))
        for name in ARM_JOINTS
    }
    return {
        name: directions[name] * (raw[name] - zero_positions[name]) for name in ARM_JOINTS
    }


def action_positions(names, positions, zero_positions=None, directions=None):
    zero_positions = zero_positions or dict.fromkeys(ARM_JOINTS, 0.0)
    directions = directions or dict.fromkeys(ARM_JOINTS, 1.0)
    positions = tuple(positions)
    if len(names) != len(positions) or not names or len(set(names)) != len(names):
        raise ValueError("trajectory joints and positions must be non-empty and one-to-one")
    if unknown := set(names) - set(ARM_JOINTS):
        raise ValueError(f"unsupported arm joints: {', '.join(sorted(unknown))}")
    if not all(math.isfinite(position) for position in positions):
        raise ValueError("trajectory positions must be finite")
    for name, position in zip(names, positions):
        lower, upper = JOINT_LIMITS.get(name, (-math.inf, math.inf))
        if not lower <= position <= upper:
            raise ValueError("trajectory position exceeds joint limits")
    raw_positions = {
        name: zero_positions[name] + directions[name] * position
        for name, position in zip(names, positions)
    }
    return {
        name: position / (math.pi / 2) * 100 if name == "arm_gripper" else math.degrees(position)
        for name, position in raw_positions.items()
    }


def _power_to_bernstein(coefficients):
    """Return Bernstein coefficients for a polynomial on the unit interval."""
    degree = len(coefficients) - 1
    return tuple(
        sum(
            math.comb(index, power) / math.comb(degree, power) * coefficients[power]
            for power in range(index + 1)
        )
        for index in range(degree + 1)
    )


def _derivative(coefficients):
    return tuple(index * value for index, value in enumerate(coefficients))[1:]


def _split_bernstein(coefficients):
    levels = [tuple(coefficients)]
    while len(levels[-1]) > 1:
        previous = levels[-1]
        levels.append(tuple(
            (previous[index] + previous[index + 1]) * 0.5
            for index in range(len(previous) - 1)
        ))
    left = tuple(level[0] for level in levels)
    right = tuple(level[-1] for level in reversed(levels))
    return left, right


def _polynomial_within(coefficients, lower, upper, depth=16):
    """Prove a polynomial remains bounded, refining loose Bernstein hulls."""
    bernstein = _power_to_bernstein(coefficients)

    def bounded(control, remaining):
        if min(control) >= lower - 1e-9 and max(control) <= upper + 1e-9:
            return True
        if remaining == 0:
            return False
        left, right = _split_bernstein(control)
        return bounded(left, remaining - 1) and bounded(right, remaining - 1)

    return bounded(bernstein, depth)


def _segment_coefficients(position0, position1, velocity0, velocity1,
                          acceleration0, acceleration1, duration):
    """Polynomial coefficients in normalized segment time ``s`` (0 through 1)."""
    delta = position1 - position0
    if velocity0 is None or velocity1 is None:
        return (position0, delta)
    velocity0 *= duration
    velocity1 *= duration
    if acceleration0 is None or acceleration1 is None:
        return (
            position0,
            velocity0,
            3.0 * delta - 2.0 * velocity0 - velocity1,
            -2.0 * delta + velocity0 + velocity1,
        )
    acceleration0 *= duration * duration
    acceleration1 *= duration * duration
    c0 = delta - velocity0 - 0.5 * acceleration0
    c1 = velocity1 - velocity0 - acceleration0
    c2 = acceleration1 - acceleration0
    return (
        position0,
        velocity0,
        0.5 * acceleration0,
        10.0 * c0 - 4.0 * c1 + 0.5 * c2,
        -15.0 * c0 + 7.0 * c1 - c2,
        6.0 * c0 - 3.0 * c1 + 0.5 * c2,
    )


def _evaluate(coefficients, value):
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * value + coefficient
    return result


def _check_segment_limits(name, coefficients, duration):
    """Conservatively bound an entire polynomial using its Bernstein hull."""
    lower, upper = JOINT_LIMITS[name]
    if not _polynomial_within(coefficients, lower, upper):
        raise ValueError(f"trajectory interpolation exceeds {name} position limits")

    velocity = tuple(value / duration for value in _derivative(coefficients))
    velocity_limit = JOINT_VELOCITY_LIMITS[name]
    if not _polynomial_within(velocity, -velocity_limit, velocity_limit):
        raise ValueError(f"trajectory interpolation exceeds {name} velocity limits")

    acceleration = _derivative(_derivative(coefficients))
    if acceleration:
        acceleration = tuple(value / (duration * duration) for value in acceleration)
        acceleration_limit = JOINT_ACCELERATION_LIMITS[name]
        if not _polynomial_within(acceleration, -acceleration_limit, acceleration_limit):
            raise ValueError(f"trajectory interpolation exceeds {name} acceleration limits")


def duration_seconds(duration):
    if duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000:
        raise ValueError("duration is malformed")
    value = float(duration.sec) + float(duration.nanosec) / 1e9
    if not math.isfinite(value):
        raise ValueError("duration must be finite")
    return value


def stamp_nanoseconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("trajectory header timestamp is malformed")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def trajectory_rows(trajectory):
    return [
        (
            duration_seconds(point.time_from_start),
            tuple(point.positions),
            tuple(point.velocities),
            tuple(point.accelerations),
            tuple(point.effort),
        )
        for point in trajectory.points
    ]


def prepare_trajectory(names, points, start_positions):
    """Validate and normalize raw FollowJointTrajectory points.

    ``points`` contains ``(time_from_start, positions, velocities)`` tuples, optionally
    followed by accelerations and effort. Empty derivative tuples are valid (MoveIt
    commonly omits them), but supplied arrays must be complete and finite. Supplied
    velocities also stay within the limits used for time feasibility.
    """
    names = tuple(names)
    if not points:
        raise ValueError("trajectory must contain a point")
    if not names or len(set(names)) != len(names) or set(names) - set(ARM_JOINTS):
        # Reuse action_positions for its precise malformed/unknown-joint messages.
        action_positions(names, (0.0,) * len(names))
    try:
        previous = {name: float(start_positions[name]) for name in names}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("current arm positions are incomplete or non-finite") from error
    if not all(math.isfinite(value) for value in previous.values()):
        raise ValueError("current arm positions are incomplete or non-finite")

    normalized = []
    previous_time = 0.0
    previous_point = TrajectoryPoint(
        0.0, previous, dict.fromkeys(names, 0.0), dict.fromkeys(names, 0.0)
    )
    for point_index, point in enumerate(points):
        if len(point) not in (3, 5):
            raise ValueError("trajectory point has an invalid shape")
        point_time, positions, velocities = point[:3]
        accelerations, effort = point[3:] if len(point) == 5 else ((), ())
        positions = tuple(positions)
        action_positions(names, positions)
        if not math.isfinite(point_time) or point_time < 0.0:
            raise ValueError("trajectory times must be finite and non-negative")
        if point_index and point_time <= previous_time:
            raise ValueError("trajectory times must be strictly increasing")
        velocities = tuple(velocities)
        if velocities:
            if len(velocities) != len(names):
                raise ValueError("trajectory velocities must be empty or match the joint list")
            if not all(math.isfinite(velocity) for velocity in velocities):
                raise ValueError("trajectory velocities must be finite")
            if any(
                abs(velocity) > JOINT_VELOCITY_LIMITS[name]
                for name, velocity in zip(names, velocities)
            ):
                raise ValueError("trajectory velocity exceeds joint limits")
        for label, values in (("accelerations", accelerations), ("effort", effort)):
            values = tuple(values)
            if values and len(values) != len(names):
                raise ValueError(f"trajectory {label} must be empty or match the joint list")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"trajectory {label} must be finite")
        if accelerations and any(
            abs(acceleration) > JOINT_ACCELERATION_LIMITS[name]
            for name, acceleration in zip(names, accelerations)
        ):
            raise ValueError("trajectory acceleration exceeds joint limits")
        if accelerations and not velocities:
            raise ValueError("trajectory accelerations require velocities")
        if effort:
            raise ValueError("trajectory effort is unsupported by the position controller")

        current = dict(zip(names, positions))
        duration = point_time - previous_time
        distances = {name: abs(current[name] - previous[name]) for name in names}
        if duration <= 0.0:
            if any(distance > 1e-9 for distance in distances.values()):
                raise ValueError("trajectory requests motion with no time to execute it")
        current_point = TrajectoryPoint(
            point_time,
            current,
            dict(zip(names, velocities)) if velocities else None,
            dict(zip(names, accelerations)) if accelerations else None,
        )
        if duration > 0.0:
            for name in names:
                coefficients = _segment_coefficients(
                    previous_point.positions[name], current[name],
                    None if previous_point.velocities is None else previous_point.velocities[name],
                    None if current_point.velocities is None else current_point.velocities[name],
                    None if previous_point.accelerations is None else previous_point.accelerations[name],
                    None if current_point.accelerations is None else current_point.accelerations[name],
                    duration,
                )
                _check_segment_limits(name, coefficients, duration)
        normalized.append(current_point)
        previous_time, previous, previous_point = point_time, current, current_point
    if len({point.velocities is None for point in normalized}) > 1:
        raise ValueError("trajectory velocities must be supplied consistently at every point")
    if len({point.accelerations is None for point in normalized}) > 1:
        raise ValueError("trajectory accelerations must be supplied consistently at every point")
    final = normalized[-1]
    if final.velocities is not None and any(
        abs(value) > 1e-6 for value in final.velocities.values()
    ):
        raise ValueError("trajectory must end at zero velocity")
    # MoveIt time-parameterization may preserve a bounded acceleration at the
    # final waypoint.  It remains subject to the finite and per-joint bounds
    # above; only non-zero terminal velocity is unsafe for this position
    # controller's stop contract.
    return normalized


def sample_trajectory(names, start_positions, points, elapsed):
    """Sample desired position, velocity, and acceleration from prepared points."""
    names = tuple(names)
    previous = TrajectoryPoint(
        0.0, dict(start_positions), dict.fromkeys(names, 0.0), dict.fromkeys(names, 0.0)
    )
    for point in points:
        if elapsed <= point.time:
            duration = point.time - previous.time
            if duration <= 0.0:
                return point.positions.copy(), dict.fromkeys(names, 0.0), dict.fromkeys(names, 0.0)
            segment_elapsed = max(0.0, elapsed - previous.time)
            normalized_time = segment_elapsed / duration
            positions, velocities, accelerations = {}, {}, {}
            for name in names:
                coefficients = _segment_coefficients(
                    previous.positions[name], point.positions[name],
                    None if previous.velocities is None else previous.velocities[name],
                    None if point.velocities is None else point.velocities[name],
                    None if previous.accelerations is None else previous.accelerations[name],
                    None if point.accelerations is None else point.accelerations[name],
                    duration,
                )
                positions[name] = _evaluate(coefficients, normalized_time)
                velocities[name] = _evaluate(_derivative(coefficients), normalized_time) / duration
                second = _derivative(_derivative(coefficients))
                accelerations[name] = (
                    _evaluate(second, normalized_time) / (duration * duration) if second else 0.0
                )
            return positions, velocities, accelerations
        previous = point
    final = points[-1]
    return (
        final.positions.copy(),
        final.velocities.copy() if final.velocities is not None else dict.fromkeys(names, 0.0),
        final.accelerations.copy() if final.accelerations is not None else dict.fromkeys(names, 0.0),
    )


def position_tolerances(names, entries, defaults=None):
    """Resolve ROS JointTolerance entries, rejecting fields we cannot measure."""
    result = dict(defaults or {})
    seen = set()
    for entry in entries:
        if entry.name not in names:
            raise ValueError(f"tolerance names unsupported joint {entry.name}")
        if entry.name in seen:
            raise ValueError(f"duplicate tolerance for {entry.name}")
        seen.add(entry.name)
        if entry.velocity not in (0.0, -1.0) or entry.acceleration not in (0.0, -1.0):
            raise ValueError("velocity and acceleration tolerances require measured derivatives")
        if (
            not math.isfinite(entry.position)
            or (entry.position < 0.0 and entry.position != -1.0)
        ):
            raise ValueError("position tolerance must be finite, non-negative, or -1")
        if entry.position == -1.0:
            result.pop(entry.name, None)
        elif entry.position > 0.0:
            result[entry.name] = entry.position
    return result
