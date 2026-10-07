"""Focused tests for the source-checkout simulation qualification runner."""

import importlib.util
from pathlib import Path
import re
import sys

import pytest
import yaml


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


def test_timed_out_command_preserves_partial_output(tmp_path):
    qualification = _load()
    (tmp_path / "commands").mkdir()
    result = qualification.run_command(
        tmp_path, "timeout", [sys.executable, "-u", "-c",
                              "import time; print('partial output'); time.sleep(10)"],
        timeout=0.5,
    )
    assert not result.passed and result.returncode is None
    assert "partial output" in (tmp_path / result.log).read_text()
    assert "command timed out" in (tmp_path / result.log).read_text()


@pytest.mark.parametrize("physical_accepted", [False, True])
def test_simulation_guards_are_independent_of_physical_acceptance(tmp_path, monkeypatch, physical_accepted):
    qualification = _load()
    monkeypatch.setattr(qualification, "ROOT", tmp_path)
    (tmp_path / "config").mkdir()
    (tmp_path / "commands").mkdir()
    (tmp_path / "config/safety_acceptance.yaml").write_text(f"validated: {physical_accepted}\n")
    (tmp_path / "config/safety_production.yaml").write_text(
        (ROOT / "config/safety_production.yaml").read_text())
    profile = yaml.safe_load((ROOT / "config/safety_simulation.yaml").read_text())
    path = tmp_path / "config/safety_simulation.yaml"
    path.write_text(yaml.safe_dump(profile))
    assert qualification.simulation_safety_result(tmp_path).passed
    profile["safety_supervisor"]["ros__parameters"]["require_depth"] = False
    path.write_text(yaml.safe_dump(profile))
    assert not qualification.simulation_safety_result(tmp_path).passed
    path.write_text("[invalid YAML")
    assert not qualification.simulation_safety_result(tmp_path).passed


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


def test_expected_ctests_match_every_test_the_package_registers():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    registered = set(re.findall(r"add_lekiwi_pytest\((\w+)", cmake))
    registered |= set(re.findall(r"add_test\(NAME\s+(\w+)", cmake))
    registered |= set(re.findall(r"ament_add_test\((\w+)", cmake))
    registered |= {f"test_{name}.py" for name in re.findall(r"add_lekiwi_launch_test\((\w+) \d+\)", cmake)}
    assert registered == set(_load().EXPECTED_CTESTS)
    unit_tests = {path.stem for path in (ROOT / "test").glob("test_*.py") if not path.stem.endswith("_launch")}
    assert unit_tests <= registered


def test_every_ctest_dds_test_gets_an_isolated_domain_below_ephemeral_ports():
    from domain_coordinator.impl import default_selector

    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert not re.search(r"ROS_DOMAIN_ID=\d", cmake)
    # Tests register only through the isolated helpers; the C++ contact test
    # opens no DDS participant.
    registrations = re.findall(
        r"^\s*(add_test|add_launch_test|ament_add_test|ament_add_pytest_test|"
        r"ament_add_ros_isolated_pytest_test)\(", cmake, flags=re.MULTILINE)
    assert sorted(registrations) == [
        "add_launch_test", "add_test", "ament_add_ros_isolated_pytest_test", "ament_add_test"]
    assert cmake.count('RUNNER "${test_isolated_runner}"') == 2
    assert "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:" in cmake
    # The coordinator hands out domains 1-100; Cyclone's highest discovery
    # port for domain 100 stays below Linux's ephemeral range.
    selector = default_selector()
    assert max(selector() for _ in range(200)) == 100
    assert 7400 + 250 * 100 + 11 + 2 * 119 < 32768
