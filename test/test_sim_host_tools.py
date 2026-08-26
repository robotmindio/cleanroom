"""Unit checks for scripts that qualify a remote simulation host."""

import importlib.util
import pathlib
import sys

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
