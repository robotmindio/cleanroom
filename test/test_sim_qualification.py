"""Focused tests for the source-checkout simulation qualification runner."""

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).parents[1]


def _load():
    spec = importlib.util.spec_from_file_location(
        "sim_qualification", ROOT / "scripts" / "sim-qualification.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_ctest_list_parser_handles_ament_names():
    qualification = _load()
    output = """\
  Test  #1: test_odometry
  Test #28: test_test_moveit_driver_e2e_launch.py
Total Tests: 2
"""
    assert qualification.ctest_names(output) == {
        "test_odometry", "test_test_moveit_driver_e2e_launch.py"
    }


def test_configured_python_uses_build_cache(tmp_path):
    qualification = _load()
    executable = tmp_path / "python3"
    executable.touch()
    (tmp_path / "CMakeCache.txt").write_text(
        f"Python3_EXECUTABLE:FILEPATH={executable}\n", encoding="utf-8"
    )
    assert qualification.configured_python(tmp_path) == str(executable)


def test_build_provenance_rejects_a_build_from_another_checkout(tmp_path):
    qualification = _load()
    evidence = tmp_path / "evidence"
    (evidence / "commands").mkdir(parents=True)
    install = tmp_path / "install" / "lekiwi_rmf"
    (install / "share" / "lekiwi_rmf").mkdir(parents=True)
    (install / "share" / "lekiwi_rmf" / "package.xml").touch()
    build = tmp_path / "build"
    build.mkdir()
    (build / "CMakeCache.txt").write_text(
        "CMAKE_HOME_DIRECTORY:INTERNAL=/different/checkout\n"
        f"CMAKE_INSTALL_PREFIX:PATH={install}\n",
        encoding="utf-8",
    )

    result, detected_install = qualification.build_provenance_result(
        evidence, build
    )

    assert not result.passed
    assert detected_install == install


def test_moveit_shutdown_artifact_requires_a_clean_current_probe():
    qualification = _load()
    artifact = {
        "schema_version": 1,
        "revision": "abc123",
        "clean_shutdown": True,
        "move_group_exit_code": 0,
        "package_versions": {"ros-jazzy-moveit-core": "2.12.4"},
        "lekiwi_rmf_package_prefix": "/selected/install/lekiwi_rmf",
    }
    install = Path("/selected/install/lekiwi_rmf")
    assert qualification.valid_moveit_shutdown_artifact(
        artifact, "abc123", install, True
    )
    assert not qualification.valid_moveit_shutdown_artifact(
        {**artifact, "move_group_exit_code": -11}, "abc123", install, False
    )
    assert not qualification.valid_moveit_shutdown_artifact(
        artifact, "different", install, True
    )
    assert not qualification.valid_moveit_shutdown_artifact(
        artifact, "abc123", Path("/another/install"), True
    )


def test_runner_rejects_evidence_inside_the_source_checkout(monkeypatch):
    qualification = _load()
    monkeypatch.setattr(
        qualification.sys, "argv",
        ["sim-qualification.py", "--output-dir", str(ROOT / "qualification-evidence")],
    )
    with pytest.raises(SystemExit) as error:
        qualification.main()
    assert error.value.code == 2
