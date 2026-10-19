"""The torque-cut gate accepts only the SRDF travel_stow pose."""

import importlib.util
from pathlib import Path

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
