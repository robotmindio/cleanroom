"""Unit checks for the fail-closed MoveIt shutdown evidence probe."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


PROBE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "moveit-shutdown-probe.py"
SPEC = importlib.util.spec_from_file_location("moveit_shutdown_probe", PROBE_PATH)
assert SPEC is not None and SPEC.loader is not None
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def test_stack_excerpt_starts_at_backtrace_and_honors_limit():
    output = "preamble\nStack trace (most recent call last):\n#0 callback\nsegfault\n"

    assert probe._stack_excerpt(output, limit=2) == [
        "Stack trace (most recent call last):",
        "#0 callback",
    ]


def test_stack_excerpt_falls_back_to_tail_without_backtrace():
    assert probe._stack_excerpt("one\ntwo\nthree\n", limit=2) == ["two", "three"]


@pytest.mark.parametrize(
    ("ready", "timed_out", "returncode", "launch_error", "expected"),
    [
        (True, False, 0, None, True),
        (False, False, 0, None, False),
        (True, True, 0, None, False),
        (True, False, -11, None, False),
        (True, False, None, None, False),
        (True, False, 0, "launch failed", False),
    ],
)
def test_shutdown_result_is_fail_closed(
    ready, timed_out, returncode, launch_error, expected
):
    assert probe._is_clean_shutdown(
        ready=ready,
        timed_out=timed_out,
        returncode=returncode,
        launch_error=launch_error,
    ) is expected


def test_command_output_reports_success_and_failure(tmp_path):
    assert probe._command_output([sys.executable, "-c", "print('2.12.4')"]) == "2.12.4"
    assert probe._command_output([str(tmp_path / "not-an-executable")]) is None


def test_package_versions_records_each_required_binary_package(monkeypatch):
    commands = []

    def fake_command_output(arguments, **_kwargs):
        commands.append(tuple(arguments))
        return "test-version"

    monkeypatch.setattr(probe, "_command_output", fake_command_output)

    versions = probe._package_versions()

    assert versions == {
        "ros-jazzy-moveit-ros-move-group": "test-version",
        "ros-jazzy-moveit-core": "test-version",
        "ros-jazzy-rclcpp": "test-version",
    }
    assert commands == [
        ("dpkg-query", "-W", "-f=${Version}", package)
        for package in versions
    ]


def test_package_prefix_is_recorded_from_the_active_ament_index(monkeypatch):
    import ament_index_python.packages

    monkeypatch.setattr(
        ament_index_python.packages,
        "get_package_prefix",
        lambda package: f"/selected/install/{package}",
    )

    assert probe._lekiwi_package_prefix() == "/selected/install/lekiwi_rmf"
