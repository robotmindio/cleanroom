"""Checks shared by the nodes that gate motion on time-limited inputs.

Kept free of ROS imports: every helper takes plain values or duck-typed
messages, so the nodes and their unit tests use the same code.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Protocol


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


def positive_seconds_ns(value: float, name: str) -> int:
    """Convert a finite, positive duration in seconds to integer nanoseconds."""
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return int(value * 1_000_000_000)


def stamp_ns(stamp: _Stamp) -> int:
    """Return a builtin_interfaces/Time as integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
