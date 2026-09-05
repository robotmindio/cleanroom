"""Pose capture accepts only fresh, complete raw observations."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, raw_joint_positions

SPEC = importlib.util.spec_from_file_location(
    "arm_calibration", Path(__file__).parents[1] / "scripts/arm_calibration.py"
)
CALIBRATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CALIBRATION)


@pytest.mark.parametrize("fault", [None, "stale", "future", "nan", "missing", "duplicate"])
def test_pose_capture_checks_sample_boundary(fault):
    node = SimpleNamespace(positions=None, get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)
    ))
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        name=list(ARM_JOINTS), position=[0.2] * len(ARM_JOINTS),
    )
    if fault == "stale":
        message.header.stamp.sec = 8
    elif fault == "future":
        message.header.stamp.sec = 11
    elif fault == "nan":
        message.position[0] = float("nan")
    elif fault == "missing":
        message.position.pop()
    elif fault == "duplicate":
        message.name[-1] = message.name[0]
    CALIBRATION.ArmCalibration.on_joint_state(node, message)
    assert node.positions == (dict.fromkeys(ARM_JOINTS, 0.2) if fault is None else None)


def test_raw_pose_uses_ros_units_without_old_pose_offsets():
    observation = {f"{name}.pos": 90.0 for name in ARM_JOINTS}
    observation["arm_gripper.pos"] = 100.0
    raw = raw_joint_positions(observation)
    assert raw["arm_shoulder_pan"] == pytest.approx(1.57079632679)
    assert raw["arm_gripper"] == pytest.approx(1.74533)
