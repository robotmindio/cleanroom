import math

import pytest

from lekiwi_rmf.odometry import (
    OdometrySampleClock, TELEMETRY_MONOTONIC_NS_KEY, TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_PROTOCOL_VERSION, TELEMETRY_SEQUENCE_KEY, TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
    TelemetrySequenceTracker, accept_validated_telemetry, integrate_pose,
    parse_telemetry_metadata,
)


def test_integrates_body_velocity_at_heading():
    x, y, yaw = integrate_pose((1.0, 2.0, math.pi / 2), (1.0, 0.5, 0.2), 2.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 4.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2 + 0.4)


def metadata(session="boot-a", sequence=0, sample_ns=1_000_000_000, torque=False):
    return {
        TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
        TELEMETRY_SESSION_KEY: session,
        TELEMETRY_SEQUENCE_KEY: sequence,
        TELEMETRY_MONOTONIC_NS_KEY: sample_ns,
        TELEMETRY_TORQUE_ENABLED_KEY: torque,
    }


def test_telemetry_sequence_advances_only_for_valid_ordered_metadata():
    tracker = TelemetrySequenceTracker()
    first = tracker.accept(metadata())
    second = tracker.accept(metadata(sequence=1, sample_ns=1_100_000_000))

    assert first.token == ("host", "boot-a", 0)
    assert second.sample_monotonic_ns == 1_100_000_000
    assert second.session_changed is False
    with pytest.raises(ValueError, match="duplicate or backward"):
        tracker.accept(metadata(sequence=1, sample_ns=1_200_000_000))
    with pytest.raises(ValueError, match="non-monotonic"):
        tracker.accept(metadata(sequence=2, sample_ns=1_050_000_000))
    restarted = tracker.accept(metadata(session="boot-b", sequence=0, sample_ns=10))
    assert restarted.session_changed is True
    with pytest.raises(ValueError, match="retired telemetry session"):
        tracker.accept(metadata(session="boot-a", sequence=2, sample_ns=1_200_000_000))


def test_authenticated_telemetry_carries_boolean_physical_torque_state():
    tracker = TelemetrySequenceTracker()
    assert tracker.accept(metadata(torque=True)).torque_enabled is True
    with pytest.raises(ValueError, match="torque state"):
        tracker.accept(metadata(session="invalid", torque=1))


def test_missing_partial_or_malformed_metadata_is_rejected():
    with pytest.raises(ValueError, match="incomplete"):
        parse_telemetry_metadata({TELEMETRY_SESSION_KEY: "boot-a"})
    with pytest.raises(ValueError, match="sequence"):
        parse_telemetry_metadata(metadata(sequence=True))
    with pytest.raises(ValueError, match="incomplete"):
        parse_telemetry_metadata({"x.vel": 0.0})


def test_invalid_state_does_not_consume_a_freshness_sequence():
    tracker = TelemetrySequenceTracker()
    packet = {**metadata(), "x.vel": float("nan")}
    with pytest.raises(ValueError, match="invalid"):
        accept_validated_telemetry(tracker, packet, ("x.vel",))

    packet["x.vel"] = 0.0
    accepted = accept_validated_telemetry(tracker, packet, ("x.vel",))
    assert accepted.token == ("host", "boot-a", 0)


def test_odometry_uses_accepted_sample_time_not_timer_frequency():
    clock = OdometrySampleClock()
    assert clock.accept(("host", "a", 1), 10_000_000_000, 1_000_000_000) is None
    # ROS timer/arrival jitter does not affect a host-timestamped sample.
    assert clock.accept(
        ("host", "a", 2), 10_173_000_000, 1_100_000_000
    ) == pytest.approx(0.1)


def test_odometry_does_not_bridge_restart_link_loss_or_large_gap():
    clock = OdometrySampleClock(max_interval=0.2)
    assert clock.accept(("host", "a", 1), 0, 1_000_000_000) is None
    assert clock.accept(("host", "a", 2), 100_000_000, 1_100_000_000) == pytest.approx(0.1)
    assert clock.accept(("host", "b", 0), 200_000_000, 10_000_000) is None
    assert clock.discontinuity == "telemetry source session changed"
    clock.reset()
    assert clock.accept(("host", "b", 1), 5_000_000_000, 110_000_000) is None
    assert clock.accept(("host", "b", 2), 5_500_000_000, 610_000_000) is None
    assert "exceeds" in clock.discontinuity
    assert clock.accept(("host", "b", 3), 5_600_000_000, 710_000_000) == pytest.approx(0.1)
