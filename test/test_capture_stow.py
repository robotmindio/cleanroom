"""The stow capture writes one pose into both safety files, exactly matching."""

import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location("capture_stow", ROOT / "scripts/capture_stow.py")
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


def test_both_files_receive_the_same_pose_and_keep_their_comments():
    stow = dict(zip(capture.STOW_JOINTS, (0.1, -1.5708, 1.4, 0.95, -0.02, 0.3)))
    production, acceptance = capture.write_stow(
        (ROOT / "config/safety_production.yaml").read_text(encoding="utf-8"),
        (ROOT / "config/safety_acceptance.yaml").read_text(encoding="utf-8"),
        stow,
    )
    params = yaml.safe_load(production)["safety_supervisor"]["ros__parameters"]
    configured = dict(zip(params["stow_joint_names"], params["stow_joint_positions"]))
    accepted = yaml.safe_load(acceptance)["accepted_stow_joint_positions"]
    assert configured == accepted == stow
    assert "measured, collision-checked" in production
    assert yaml.safe_load(acceptance)["validated"] is False


def test_a_moving_or_incomplete_arm_is_refused():
    still = {joint: 0.5 for joint in capture.STOW_JOINTS}
    assert capture.stow_from_samples([still, dict(still, arm_gripper=0.505)])["arm_gripper"] == 0.5025
    with pytest.raises(ValueError, match="moved"):
        capture.stow_from_samples([still, dict(still, arm_elbow_flex=0.6)])
    with pytest.raises(ValueError, match="missing"):
        capture.stow_from_samples([{"arm_shoulder_pan": 0.0}])
