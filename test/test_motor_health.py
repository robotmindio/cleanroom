"""Contract tests for versioned motor-health telemetry."""

import pytest

from lekiwi_rmf.motor_health import (
    ERROR, OK, fault_snapshot, healthy_snapshot, parse_motor_health,
)


MOTORS = ("wheel_left", "wheel_right")


def test_complete_healthy_snapshot_has_bus_and_per_servo_statuses():
    statuses = parse_motor_health(healthy_snapshot(MOTORS, True, detail={
        "wheel_left": {"present_position": 12.5},
    }))
    assert [status.name for status in statuses] == [
        "motor_bus", "servo/wheel_left", "servo/wheel_right",
    ]
    assert all(status.level == OK for status in statuses)
    assert dict(statuses[1].values)["present_position"] == "12.5"


@pytest.mark.parametrize("payload", [
    None,
    {"version": 99, "statuses": {}},
    {"version": 1, "statuses": {"motor_bus": {"level": 0, "message": "OK", "values": {}}}},
    {"version": 1, "statuses": {"motor_bus": {"level": 0, "message": "OK", "values": {}},
                                "servo/x": {"level": 7, "message": "bad", "values": {}}}},
])
def test_missing_or_malformed_health_never_validates(payload):
    with pytest.raises(ValueError):
        parse_motor_health(payload)


def test_bus_fault_marks_every_configured_servo_unsafe():
    statuses = parse_motor_health(fault_snapshot(MOTORS, "bus disconnected"))
    assert all(status.level == ERROR for status in statuses)
    assert {status.name for status in statuses} == {
        "motor_bus", "servo/wheel_left", "servo/wheel_right",
    }


def test_advisory_electrical_limit_is_a_warning_not_a_motion_fault():
    statuses = parse_motor_health(healthy_snapshot(
        MOTORS, True,
        detail={"wheel_left": {"present_voltage_v": 7.4, "present_current_ma": 650.0}},
        warnings={"wheel_left": "supply voltage is at or beyond the configured servo limit"},
    ))
    left = next(status for status in statuses if status.name == "servo/wheel_left")
    assert left.level == 1
    assert dict(left.values)["present_current_ma"] == "650.0"
