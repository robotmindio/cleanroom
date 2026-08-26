import json
import math
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
    # The URDF models this joint as continuous, but the physical servo and its
    # wiring are not safe targets for arbitrarily large multi-turn commands.
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


def validate_trajectory(names, points, start_positions):
    """Validate finite, bounded points and the velocity implied by their timing.

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

    previous_time = 0.0
    for point in points:
        if len(point) not in (3, 5):
            raise ValueError("trajectory point has an invalid shape")
        point_time, positions, velocities = point[:3]
        accelerations, effort = point[3:] if len(point) == 5 else ((), ())
        positions = tuple(positions)
        action_positions(names, positions)
        if not math.isfinite(point_time) or point_time < previous_time:
            raise ValueError("trajectory times must be finite and ordered")
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

        current = dict(zip(names, positions))
        duration = point_time - previous_time
        distances = {name: abs(current[name] - previous[name]) for name in names}
        if duration <= 0.0:
            if any(distance > 1e-9 for distance in distances.values()):
                raise ValueError("trajectory requests motion with no time to execute it")
        elif any(
            distance > JOINT_VELOCITY_LIMITS[name] * duration + 1e-9
            for name, distance in distances.items()
        ):
            raise ValueError("trajectory timing exceeds joint velocity limits")
        previous_time, previous = point_time, current


def interpolate_positions(start, points, elapsed):
    previous_time, previous = 0.0, start
    for point_time, point in points:
        if elapsed <= point_time:
            if point_time == previous_time:
                return point.copy()
            fraction = (elapsed - previous_time) / (point_time - previous_time)
            return {name: previous[name] + fraction * (point[name] - previous[name]) for name in point}
        previous_time, previous = point_time, point
    return points[-1][1].copy()
