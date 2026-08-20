import math

import pytest

from lekiwi_rmf.arm_trajectory import action_positions


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
