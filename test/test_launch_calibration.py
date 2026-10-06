import os
import subprocess
import sys

import pytest

from lekiwi_rmf.launch_calibration import load_launch_calibration, save_launch_calibration
from lekiwi_rmf.odometry import load_base_scales


def test_shared_calibration_preserves_signed_pitch_and_explicit_overrides(tmp_path, monkeypatch):
    path = tmp_path / "launch.conf"
    path.write_text("# measured geometry\n camera_pitch = -0.031000\n")
    monkeypatch.setenv("LEKIWI_LAUNCH_CALIBRATION", str(path))
    save_launch_calibration(xy_velocity_scale=1.25, yaw_velocity_scale=0.976)
    assert load_launch_calibration() == {
        "camera_pitch": -0.031, "xy_velocity_scale": 1.25, "yaw_velocity_scale": 0.976,
    }
    assert load_base_scales(path) == (1.25, 0.976)
    result = subprocess.run(
        [sys.executable, "-m", "lekiwi_rmf.launch_calibration", "xy_velocity_scale:=2"],
        capture_output=True, text=True, check=True,
    )
    assert result.stdout.splitlines() == ["camera_pitch:=-0.031000", "yaw_velocity_scale:=0.976000"]


@pytest.mark.parametrize("contents", [
    "camera_pitch=nan", "camera_pitch=inf", "camera_height=0", "xy_velocity_scale=-1",
    "yaw_velocity_scale=0", "unknown=1", "camera_pitch=0\ncamera_pitch=1",
    "camera_pitch=$(touch unsafe)", "camera_pitch", "camera_pitch=",
])
def test_invalid_calibration_fails_all_readers_and_cli(tmp_path, contents):
    path = tmp_path / "launch.conf"
    path.write_text(contents)
    for reader in (load_launch_calibration, load_base_scales):
        with pytest.raises(ValueError, match="launch.conf"):
            reader(path)
    result = subprocess.run(
        [sys.executable, "-m", "lekiwi_rmf.launch_calibration"],
        env={**os.environ, "LEKIWI_LAUNCH_CALIBRATION": str(path)},
        capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert str(path) in result.stderr


def test_missing_calibration_and_invalid_writer_values(tmp_path, monkeypatch):
    path = tmp_path / "missing.conf"
    monkeypatch.setenv("LEKIWI_LAUNCH_CALIBRATION", str(path))
    assert load_launch_calibration() == {}
    assert load_base_scales(path) == (1.0, 0.976)
    for values in ({"camera_pitch": float("nan")}, {"xy_velocity_scale": 0}, {"camera_height": 1e-9}, {"unknown": 1}):
        with pytest.raises(ValueError):
            save_launch_calibration(**values)
    assert not path.exists()
