"""Checks shared by the nodes that gate motion on time-limited inputs.

Kept free of ROS imports: every helper takes plain values or duck-typed
messages, so the nodes and their unit tests use the same code.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Protocol

import yaml

# Pi capture timestamps were measured a few milliseconds ahead of compute.
# This bounds source-clock skew; monotonic receive-time leases remain strict.
FUTURE_STAMP_TOLERANCE_NS = 50_000_000


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


def bounded_test_speed_limits(linear, angular, production):
    """Allow attended speed trials only within the configured production caps."""
    requested = (linear, angular)
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or v <= 0 for v in (*requested, *production)):
        raise ValueError("base test speed limits must be finite and positive")
    if any(value > maximum for value, maximum in zip(requested, production)):
        raise ValueError("base test speed limits exceed configured production limits")
    return requested


def load_base_test_profile(nav2_file, stage):
    """Read an explicit attended stage; never change production speed limits."""
    directory = Path(nav2_file).parent
    try:
        data = yaml.safe_load((directory / 'base_speed_qualification.yaml').read_text())
        profile = {**data['bounds'], **data['stages'][stage]}
        names = ('linear_speed_m_s', 'angular_speed_rad_s', 'maximum_stopping_distance_m',
                 'maximum_center_radius_m', 'independent_center_radius_m',
                 'driver_center_radius_m', 'required_clearance_m', 'nominal_command_duration_s',
                 'return_linear_speed_m_s', 'return_angular_speed_rad_s')
        if set(profile) != set(names) or not all(
            isinstance(profile[k], (int, float)) and not isinstance(profile[k], bool)
            and math.isfinite(profile[k]) and profile[k] > 0 for k in names
        ):
            raise ValueError('stage values must be finite and positive')
        if profile['linear_speed_m_s'] > .4 or profile['angular_speed_rad_s'] > math.pi / 2:
            raise ValueError('stage exceeds documented hardware speed limits')
        runner, independent, driver, clearance = (profile[k] for k in names[3:7])
        if not runner < independent < driver < clearance:
            raise ValueError('stage boundaries must increase from runner to clear area')
        braking = yaml.safe_load((directory / 'onboard_braking.yaml').read_text())
        point_speed = 1.15 * max(profile['linear_speed_m_s'], braking['body_radius_m'] * profile['angular_speed_rad_s'],
                                 profile['return_linear_speed_m_s'] + braking['body_radius_m'] * profile['return_angular_speed_rad_s'])
        if clearance - driver < point_speed * braking['maximum_stop_time_s'] + braking['measurement_uncertainty_m']:
            raise ValueError('stage clear area lacks the braking reserve')
        if runner <= profile['nominal_command_duration_s'] * profile['linear_speed_m_s'] / 2 + braking['measurement_uncertainty_m']:
            raise ValueError('stage cannot contain a nominal trial')
        profile['point_speed_bound_m_s'] = point_speed
        return profile
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f'invalid base qualification stage {stage}: {error}') from error


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


@dataclass
class Lease:
    """A supervisor permission whose authority lasts ``timeout_ns`` from its receipt."""

    timeout_ns: int
    value: bool = False
    received_at_ns: Optional[int] = None

    def grant(self, value, received_at_ns: int) -> None:
        self.value = bool(value)
        self.received_at_ns = received_at_ns

    def current(self, now_ns: Optional[int] = None) -> bool:
        return self.value and lease_is_fresh(self.received_at_ns, self.timeout_ns, now_ns)


def twist_is_finite(message: _Twist) -> bool:
    """Return whether every Twist field is finite.

    Checking all six fields is deliberate: a malformed client can populate
    axes a consumer does not use.
    """
    return all(math.isfinite(value) for value in (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z,
    ))


def inside_base_test_boundary(pose, center, radius=0.20) -> bool:
    """Stop at the configured radius (20 cm for ordinary bounded tests).

    Wheel odometry can slip; use short routes and independently check the
    physical position. This is a commissioning bound, not certified geofencing.
    """
    return math.isfinite(radius) and radius > 0 and all(math.isfinite(v) for v in (*pose[:2], *center)) and (
        math.hypot(pose[0] - center[0], pose[1] - center[1]) < radius
    )


def positive_seconds_ns(value: float, name: str) -> int:
    """Convert a finite, positive duration in seconds to integer nanoseconds."""
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return int(value * 1_000_000_000)


def stamp_ns(stamp: _Stamp) -> int:
    """Return a builtin_interfaces/Time as integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
