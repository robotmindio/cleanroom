"""Checks shared by the nodes that gate motion on time-limited inputs.

Kept free of ROS imports: every helper takes plain values or duck-typed
messages, so the nodes and their unit tests use the same code.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional, Protocol

import yaml


def load_base_speed_limits(path: str | Path) -> tuple[float, float]:
    """Use the tracked MPPI limits for both acceptance and manual commands."""
    try:
        data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
        controller = data["controller_server"]["ros__parameters"]["FollowPath"]
        values = [controller[key] for key in ("vx_max", "vx_min", "vy_max", "wz_max")]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values):
            raise ValueError("speed limits must be finite numbers")
        vx, reverse, vy, angular = values
        if vx <= 0 or reverse >= 0 or vy <= 0 or angular <= 0:
            raise ValueError("expected positive maxima and a negative vx_min")
        return float(max(vx, -reverse, vy)), float(angular)
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid Nav2 speed limits in {path}: {error}") from error


class _Vector(Protocol):
    x: float
    y: float
    z: float


class _Twist(Protocol):
    linear: _Vector
    angular: _Vector


class _Stamp(Protocol):
    sec: int
    nanosec: int


def lease_is_fresh(
    received_at_ns: Optional[int], timeout_ns: int, now_ns: Optional[int] = None
) -> bool:
    """Return whether a receive-time lease (monotonic ns) was refreshed in time.

    Bool permissions carry no source stamp, so the receive time is the lease.
    A latched sample replayed after a restart is only as good as its age.
    """
    if received_at_ns is None:
        return False
    current = time.monotonic_ns() if now_ns is None else now_ns
    age = current - received_at_ns
    return 0 <= age <= timeout_ns


def twist_is_finite(message: _Twist) -> bool:
    """Return whether every Twist field is finite.

    Checking all six fields is deliberate: a malformed client can populate
    axes a consumer does not use.
    """
    return all(math.isfinite(value) for value in (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z,
    ))


def inside_base_test_boundary(pose, center) -> bool:
    """Stop at 20 cm, reserving 10 cm of the authorized radius for stopping.

    Wheel odometry can slip; use short routes and independently check the
    physical position. This is a commissioning bound, not certified geofencing.
    """
    return all(math.isfinite(v) for v in (*pose[:2], *center)) and (
        math.hypot(pose[0] - center[0], pose[1] - center[1]) < 0.20
    )


def positive_seconds_ns(value: float, name: str) -> int:
    """Convert a finite, positive duration in seconds to integer nanoseconds."""
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return int(value * 1_000_000_000)


def stamp_ns(stamp: _Stamp) -> int:
    """Return a builtin_interfaces/Time as integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
