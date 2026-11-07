"""The torque-cut gate accepts only the SRDF travel_stow pose."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location("check_travel_stow", ROOT / "scripts/check-travel-stow.py")
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)


def test_gate_uses_the_tracked_arm_stow_and_its_tolerance():
    stow = check.travel_stow()
    assert set(stow) == {"arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll"}
    names = [*stow, "arm_gripper"]
    assert check.stow_errors(stow, names, [*(value + 0.019 for value in stow.values()), 1.0]) == {}
    off = [*stow.values(), 0.0]
    off[1] += 0.03
    assert set(check.stow_errors(stow, names, off)) == {"arm_shoulder_lift"}
    assert check.stow_errors(stow, names[1:], off[1:])["arm_shoulder_pan"] is None


@pytest.mark.parametrize("position", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_arm_position_never_authorizes_torque_cut(position):
    stow = check.travel_stow()
    values = list(stow.values())
    values[0] = position
    assert set(check.stow_errors(stow, list(stow), values)) == {"arm_shoulder_pan"}


@pytest.mark.parametrize("fault", ["duplicate", "extra_position", "missing_position"])
def test_malformed_snapshot_never_authorizes_torque_cut(fault):
    stow = check.travel_stow()
    names, values = list(stow), list(stow.values())
    if fault == "duplicate":
        names.append(names[0])
        values.append(values[0])
    elif fault == "extra_position":
        values.append(0.)
    else:
        values.pop()
    assert check.stow_errors(stow, names, values)


@pytest.mark.parametrize("stamp_ns,expected", [
    (0, False), (8_999_999_999, False), (9_000_000_000, True),
    (10_000_000_000, True), (10_050_000_000, True), (10_050_000_001, False),
])
def test_source_stamp_must_be_fresh_with_only_the_shared_clock_skew_allowance(stamp_ns, expected):
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))
    assert check.fresh_joint_stamp(message, 10_000_000_000, 1_000_000_000) is expected


@pytest.mark.parametrize("option", ["--max-age", "--timeout"])
@pytest.mark.parametrize("value", ["nan", "inf", "0", "-1"])
def test_invalid_limits_fail_before_ros_initialization(monkeypatch, option, value):
    monkeypatch.setattr("sys.argv", ["check-travel-stow.py", option, value])
    monkeypatch.setattr(check.rclpy, "init", lambda: pytest.fail("invalid limits reached ROS initialization"))
    with pytest.raises(SystemExit) as error:
        check.main()
    assert error.value.code == 2
