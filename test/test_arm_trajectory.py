import json
import math

import pytest

from lekiwi_rmf.arm_trajectory import action_positions, interpolate_positions, joint_positions, load_calibration


def test_action_positions_converts_ros_radians_to_lerobot_units():
    assert action_positions(("arm_shoulder_pan", "arm_gripper"), (math.pi / 2, math.pi / 4)) == {
        "arm_shoulder_pan": 90.0,
        "arm_gripper": 50.0,
    }


def test_action_positions_rejects_invalid_joint_lists():
    with pytest.raises(ValueError):
        action_positions(("arm_shoulder_pan", "unknown"), (0.0, 0.0))
    with pytest.raises(ValueError):
        action_positions(("arm_shoulder_pan",), (2.0,))


def test_interpolate_positions_reaches_waypoints_on_time():
    points = [(1.0, {"arm_shoulder_pan": 1.0}), (2.0, {"arm_shoulder_pan": 0.0})]
    assert interpolate_positions({"arm_shoulder_pan": 0.0}, points, 0.5) == {"arm_shoulder_pan": 0.5}
    assert interpolate_positions({"arm_shoulder_pan": 0.0}, points, 1.5) == {"arm_shoulder_pan": 0.5}


def test_calibration_maps_urdf_zero_to_the_lerobot_position(tmp_path):
    calibration = tmp_path / "arm.json"
    zeros = {
        "arm_shoulder_pan": 0.5,
        "arm_shoulder_lift": 0,
        "arm_elbow_flex": 0,
        "arm_wrist_flex": 0,
        "arm_wrist_roll": 0,
        "arm_gripper": 0,
    }
    calibration.write_text(json.dumps({"zero_positions": zeros, "directions": dict.fromkeys(zeros, 1)}))
    zero_positions, directions = load_calibration(calibration)
    positions = joint_positions(
        {"arm_shoulder_pan.pos": math.degrees(0.5)}, zero_positions, directions
    )
    assert positions["arm_shoulder_pan"] == 0
    assert action_positions(("arm_shoulder_pan",), (0.0,), zero_positions, directions) == {
        "arm_shoulder_pan": math.degrees(0.5)
    }


@pytest.mark.parametrize("contents", ["[]", '"not an object"'])
def test_calibration_with_wrong_json_shape_is_reported_as_invalid(tmp_path, contents):
    calibration = tmp_path / "arm.json"
    calibration.write_text(contents)

    with pytest.raises(ValueError, match="invalid arm calibration"):
        load_calibration(calibration)
