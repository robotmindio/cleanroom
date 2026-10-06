import math

import pytest

from lekiwi_rmf.host_protocol import (
    TELEMETRY_MONOTONIC_NS_KEY, TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_PROTOCOL_VERSION, TELEMETRY_SEQUENCE_KEY, TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)
from lekiwi_rmf.odometry import (
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
    assert second.token == ("host", "boot-a", 1)
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


def test_host_odometry_keeps_motion_across_lost_packets_and_reanchors_restart(tmp_path):
    from lekiwi_rmf.host_protocol import HOST_ODOMETRY_KEY
    from lekiwi_rmf.odometry import HostOdometry, HostPoseTracker, load_base_scales, parse_host_odometry

    assert load_base_scales(tmp_path / "missing.conf") == (1.0, 0.976)
    assert HostOdometry().scales == (1.0, 0.976)
    calibration = tmp_path / "calibration.conf"
    calibration.write_text("camera_pitch=0.02\nxy_velocity_scale=2\nyaw_velocity_scale=0.9\n")
    assert load_base_scales(calibration) == (2.0, 0.9)
    host, tracker = HostOdometry(1, 1), HostPoseTracker()
    sample = host.update((0.1, 0, 0), 1_000_000_000, 1_000_000_000)
    assert parse_host_odometry({HOST_ODOMETRY_KEY: sample}) == sample
    pose = tracker.update("a", sample["pose"], (1, 2, math.pi / 2))
    for i in range(1, 11):  # Only the last packet reaches compute.
        sample = host.update((0.1, 0, 0), 1_000_000_000 + i * 100_000_000, 1_000_000_000)
    pose = tracker.update("a", sample["pose"], pose)
    assert pose == pytest.approx((1, 2.1, math.pi / 2))
    assert tracker.update("b", (0, 0, 0), pose) == pytest.approx(pose)
    with pytest.raises(ValueError):
        parse_host_odometry({HOST_ODOMETRY_KEY: {**sample, "pose": [float("nan"), 0, 0]}})
    with pytest.raises(ValueError):
        parse_host_odometry({})  # every deployed host sends its integrated pose
