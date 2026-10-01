import json
import xml.etree.ElementTree as ET

import pytest

from lekiwi_rmf.moveit_config import apply_gripper_calibration, moveit_config_builder


def test_moveit_execution_budget_allows_real_servo_timing_margin():
    execution = moveit_config_builder("false").to_moveit_configs().trajectory_execution
    assert execution["trajectory_execution"] == {
        "allowed_execution_duration_scaling": 1.5,
        "allowed_goal_duration_margin": 1.0,
    }


def parameters():
    return {
        "robot_description_planning": {
            "joint_limits": {
                "arm_gripper": {"min_position": -0.174533, "max_position": 1.74533}
            }
        },
        "robot_description_semantic": (
            '<robot name="lekiwi">'
            '<group_state name="closed" group="gripper"><joint name="arm_gripper" value="-0.174533"/></group_state>'
            '<group_state name="open" group="gripper"><joint name="arm_gripper" value="1.74533"/></group_state>'
            '</robot>'
        ),
    }


def calibration_file(tmp_path, zero, direction=1.0):
    path = tmp_path / "arm-calibration.json"
    joints = ("arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll", "arm_gripper")
    path.write_text(json.dumps({
        "zero_positions": {**dict.fromkeys(joints, 0.0), "arm_gripper": zero},
        "directions": {**dict.fromkeys(joints, 1.0), "arm_gripper": direction},
    }))
    return path


def test_moveit_close_state_and_limits_follow_the_reachable_servo_range(tmp_path):
    config = parameters()
    path = calibration_file(tmp_path, -0.13763789773507207)

    apply_gripper_calibration(config, path)

    limits = config["robot_description_planning"]["joint_limits"]["arm_gripper"]
    states = ET.fromstring(config["robot_description_semantic"])
    closed = states.find("group_state[@name='closed']/joint").get("value")
    opened = states.find("group_state[@name='open']/joint").get("value")
    assert float(limits["min_position"]) == pytest.approx(-0.03689510226492793)
    assert float(closed) == pytest.approx(0.0)
    assert float(opened) == pytest.approx(limits["max_position"])


def test_moveit_leaves_simulation_range_unchanged_without_a_calibration(tmp_path):
    config = parameters()
    original = config["robot_description_semantic"]

    apply_gripper_calibration(config, tmp_path / "missing.json")

    assert config["robot_description_planning"]["joint_limits"]["arm_gripper"] == {
        "min_position": -0.174533, "max_position": 1.74533,
    }
    assert config["robot_description_semantic"] == original


def test_moveit_rejects_calibration_that_has_no_intersection_with_urdf_limits(tmp_path):
    config = parameters()
    path = calibration_file(tmp_path, 20.0)

    with pytest.raises(ValueError, match="no reachable MoveIt range"):
        apply_gripper_calibration(config, path)


def test_moveit_rejects_calibration_with_unreachable_physical_closed_position(tmp_path):
    config = parameters()
    path = calibration_file(tmp_path, -0.2)

    with pytest.raises(ValueError, match="calibrated closed position is outside"):
        apply_gripper_calibration(config, path)
