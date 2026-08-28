"""Versioned, transport-safe motor-health telemetry helpers.

The motor host is the only process permitted to access the servo serial bus.
It sends the resulting read-only snapshot alongside its normal observation;
the ROS driver validates and republishes it as ``DiagnosticArray``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


MOTOR_HEALTH_KEY = "_lekiwi_motor_health"
MOTOR_HEALTH_VERSION = 1
OK, WARN, ERROR, STALE = range(4)


@dataclass(frozen=True)
class MotorHealthStatus:
    name: str
    level: int
    message: str
    values: tuple[tuple[str, str], ...]


def make_status(name: str, level: int, message: str, **values) -> dict:
    """Create a JSON-safe status used only by the motor-host boundary."""
    return {
        "level": int(level),
        "message": str(message),
        "values": {str(key): str(value) for key, value in values.items()},
    }


def healthy_snapshot(
    motors, torque_enabled: bool, *, detail: Mapping[str, Mapping] | None = None,
    warnings: Mapping[str, str] | None = None,
) -> dict:
    """Build an all-OK snapshot for a complete successful grouped bus read."""
    entries = {
        "motor_bus": make_status("motor_bus", OK, "all configured servos responded",
                                 torque_enabled=torque_enabled),
    }
    for motor in motors:
        values = {"torque_enabled": torque_enabled}
        if detail and motor in detail:
            values.update(detail[motor])
        warning = warnings.get(motor) if warnings else None
        entries[f"servo/{motor}"] = make_status(
            f"servo/{motor}", WARN if warning else OK,
            warning or "servo responded", **values
        )
    return {"version": MOTOR_HEALTH_VERSION, "statuses": entries}


def fault_snapshot(motors, message: str, *, failed_motor: str | None = None) -> dict:
    """Build a fail-closed snapshot after a bus/readback failure."""
    entries = {
        "motor_bus": make_status("motor_bus", ERROR, message),
    }
    for motor in motors:
        detail = message if failed_motor in (None, motor) else "bus health unavailable"
        entries[f"servo/{motor}"] = make_status(f"servo/{motor}", ERROR, detail)
    return {"version": MOTOR_HEALTH_VERSION, "statuses": entries}


def parse_motor_health(payload) -> tuple[MotorHealthStatus, ...]:
    """Strictly validate motor-health payload before it can imply an OK state."""
    if not isinstance(payload, dict):
        raise ValueError("motor-health payload must be an object")
    if payload.get("version") != MOTOR_HEALTH_VERSION:
        raise ValueError("unsupported motor-health telemetry version")
    statuses = payload.get("statuses")
    if not isinstance(statuses, dict) or not statuses:
        raise ValueError("motor-health statuses must be a non-empty object")
    parsed = []
    for name, entry in statuses.items():
        if not isinstance(name, str) or not name or len(name) > 128:
            raise ValueError("motor-health status name is invalid")
        if not isinstance(entry, dict):
            raise ValueError("motor-health status must be an object")
        level, message, values = entry.get("level"), entry.get("message"), entry.get("values")
        if isinstance(level, bool) or not isinstance(level, int) or level not in (OK, WARN, ERROR, STALE):
            raise ValueError("motor-health diagnostic level is invalid")
        if not isinstance(message, str) or not message or len(message) > 512:
            raise ValueError("motor-health diagnostic message is invalid")
        if not isinstance(values, dict):
            raise ValueError("motor-health diagnostic values must be an object")
        converted = []
        for key, value in values.items():
            if not isinstance(key, str) or not key or len(key) > 128:
                raise ValueError("motor-health diagnostic key is invalid")
            if isinstance(value, bool):
                text = "true" if value else "false"
            elif isinstance(value, (str, int, float)):
                if isinstance(value, float) and not math.isfinite(value):
                    raise ValueError("motor-health diagnostic value is non-finite")
                text = str(value)
            else:
                raise ValueError("motor-health diagnostic value is invalid")
            converted.append((key, text))
        parsed.append(MotorHealthStatus(name, level, message, tuple(sorted(converted))))
    if "motor_bus" not in statuses or not any(name.startswith("servo/") for name in statuses):
        raise ValueError("motor-health payload lacks bus or servo status")
    return tuple(sorted(parsed, key=lambda status: status.name))
