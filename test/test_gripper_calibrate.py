"""Check the gripper contact command against measured travel."""

from pathlib import Path
import runpy

import pytest


calibration_limits = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "scripts/gripper-calibrate.py")
)["calibration_limits"]


def test_verified_contact_goal_sets_closed_limit():
    assert calibration_limits(3491, 2040, 2014) == (2014, 3471, 0)
    assert calibration_limits(3491, 2040) == (2060, 3471, 0)
    assert calibration_limits(1000, 3000, 3020) == (1020, 3020, 1)
    with pytest.raises(RuntimeError):
        calibration_limits(3491, 2040, 1950)
