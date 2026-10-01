"""Unit checks for scripts that qualify a remote simulation host."""

import importlib.util
import os
import pathlib
import shutil
import subprocess
import sys
import time

import pytest


ROOT = pathlib.Path(__file__).parents[1]


def _load(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_renderer_version_check_requires_gl_33():
    renderer = _load("sim_renderer_check", "scripts/sim-renderer-check.py")
    assert renderer._version_from_string("4.6 Mesa") == (4, 6)
    assert renderer._at_least((3, 3))
    assert not renderer._at_least((3, 1))
    assert not renderer._at_least(None)


def test_scan_check_rejects_the_all_minimum_renderer_failure():
    scan = _load("sim_scan_check", "scripts/sim-scan-check.py")
    bad = scan.assess_scan([0.08] * 360, 0.08)
    assert not bad.usable
    assert "all 360 ranges" in bad.message

    good = scan.assess_scan([0.08, 1.2, float("inf")] * 120, 0.08)
    assert good.usable
    assert "360 ranges" in good.message


def test_gripper_calibration_requires_an_explicit_apply_flag(monkeypatch):
    calibration = _load("gripper_calibrate", "scripts/gripper-calibrate.py")
    monkeypatch.setattr(calibration.sys, "argv", ["gripper-calibrate.py"])

    with pytest.raises(RuntimeError, match="without --apply"):
        calibration.main()


def test_gripper_calibration_keeps_goals_clear_of_measured_stops():
    calibration = _load("gripper_calibrate_limits", "scripts/gripper-calibrate.py")
    assert calibration.calibration_limits(3491, 2054) == (2074, 3471, 0)
    with pytest.raises(RuntimeError, match="too close"):
        calibration.calibration_limits(2054, 2038)


def test_sim_up_records_one_launch_session_and_passes_arguments_through(tmp_path):
    checkout = tmp_path / "checkout"
    (checkout / "scripts").mkdir(parents=True)
    shutil.copy2(ROOT / "scripts" / "sim-up.sh", checkout / "scripts" / "sim-up.sh")
    (checkout / "scripts" / "setup.bash").write_text("export FROM_SETUP=1\n")
    check = checkout / "scripts" / "sim-renderer-check.py"
    check.write_text("#!/bin/sh\nexit 0\n")
    check.chmod(0o755)
    fakes = tmp_path / "bin"
    fakes.mkdir()
    record = tmp_path / "ros2.args"
    ros2 = fakes / "ros2"
    ros2.write_text(f'#!/bin/sh\necho "$FROM_SETUP $$ $*" > "{record}"\n')
    ros2.chmod(0o755)
    logs = tmp_path / "logs"
    environment = {**os.environ, "PATH": f"{fakes}:{os.environ['PATH']}", "LEKIWI_LOGS": str(logs)}
    environment.pop("LEKIWI_RUNTIME_DIR", None)

    result = subprocess.run(
        ["bash", str(checkout / "scripts" / "sim-up.sh"), "slam_mode:=localization", "gui:=false"],
        env=environment, capture_output=True, text=True, timeout=30,
    )

    assert result.returncode == 0, result.stderr
    deadline = time.monotonic() + 10
    while not record.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    setup, pid, *command = record.read_text().split()
    assert setup == "1"
    assert command == ["launch", "lekiwi_rmf", "bringup.launch.py", "mode:=sim", "slam_mode:=localization", "gui:=false"]
    # The recorded stack PID is the process that became ros2 launch.
    assert (logs / "runtime" / "stack.pid").read_text().strip() == pid
