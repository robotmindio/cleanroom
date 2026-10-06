"""The stow capture writes one pose into both safety files, exactly matching."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location("capture_stow", ROOT / "scripts/capture_stow.py")
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


def test_both_files_receive_the_same_pose_and_keep_their_comments():
    stow = dict(zip(capture.STOW_JOINTS, (0.1, -1.5708, 1.4, 0.95, -0.02, 0.3)))
    original = (ROOT / "config/safety_production.yaml").read_text(encoding="utf-8")
    production, acceptance = capture.write_stow(
        original,
        (ROOT / "config/safety_acceptance.yaml").read_text(encoding="utf-8")
        .replace("validated: false", "validated: true")
        .replace("validated_at: null", 'validated_at: "2026-09-30"'),
        stow,
    )
    params = yaml.safe_load(production)["safety_supervisor"]["ros__parameters"]
    configured = dict(zip(params["stow_joint_names"], params["stow_joint_positions"]))
    accepted = yaml.safe_load(acceptance)["accepted_stow_joint_positions"]
    assert configured == accepted == stow
    assert [line for line in original.splitlines() if line.lstrip().startswith("#")] == [
        line for line in production.splitlines() if line.lstrip().startswith("#")
    ]
    assert yaml.safe_load(acceptance)["validated"] is False
    assert yaml.safe_load(acceptance)["validated_at"] is None


def test_a_moving_or_incomplete_arm_is_refused():
    still = {joint: 0.5 for joint in capture.STOW_JOINTS}
    assert capture.stow_from_samples([still, dict(still, arm_gripper=0.505)])["arm_gripper"] == 0.5025
    with pytest.raises(ValueError, match="moved"):
        capture.stow_from_samples([still, dict(still, arm_elbow_flex=0.6)])
    with pytest.raises(ValueError, match="missing"):
        capture.stow_from_samples([{"arm_shoulder_pan": 0.0}])


def test_named_travel_pose_matches_safety_and_preserves_home():
    original = (ROOT / "config/lekiwi.srdf").read_text()
    production = yaml.safe_load((ROOT / "config/safety_production.yaml").read_text())["safety_supervisor"]["ros__parameters"]
    stow = dict(zip(production["stow_joint_names"], production["stow_joint_positions"]))
    changed = capture.write_named_stow(original, stow)
    assert capture.write_named_stow(changed, stow) == changed
    semantic = ET.fromstring(changed)
    assert ET.tostring(semantic.find("group_state[@name='home']")) == ET.tostring(ET.fromstring(original).find("group_state[@name='home']"))
    recorded = {joint.get("name"): float(joint.get("value"))
                for state in semantic.findall("group_state[@name='travel_stow']") for joint in state}
    assert recorded == stow
    assert dict(zip(production["stow_joint_names"], production["stow_joint_positions"])) == recorded
    stored = {joint.get("name"): float(joint.get("value"))
              for state in ET.fromstring(original).findall("group_state[@name='travel_stow']") for joint in state}
    assert stored == stow
    for fault in (float("nan"), float("inf"), -2.0):
        with pytest.raises(ValueError):
            capture.stow_from_samples([dict(stow, arm_shoulder_lift=fault)])
