#!/usr/bin/env python3
"""Qualify orderly shutdown of the installed MoveIt ``move_group`` binary.

This probe intentionally does not launch the LeKiwi driver or send a trajectory.
It waits until MoveIt reports that its services are ready, asks the ROS launch
service to shut down normally (SIGINT), and records the actual process result.
An upstream shutdown crash must remain a failed qualification, even if planning
and trajectory execution succeeded before shutdown.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
from collections.abc import Sequence
from datetime import datetime, timezone
from pathlib import Path


READY_MARKER = "You can start planning now!"
STACK_MARKER = "Stack trace"


def _command_output(arguments: Sequence[str], *, cwd: Path | None = None) -> str | None:
    try:
        result = subprocess.run(
            arguments,
            cwd=cwd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def _package_versions() -> dict[str, str | None]:
    packages = (
        "ros-jazzy-moveit-ros-move-group",
        "ros-jazzy-moveit-core",
        "ros-jazzy-rclcpp",
    )
    return {
        package: _command_output(("dpkg-query", "-W", "-f=${Version}", package))
        for package in packages
    }


def _lekiwi_package_prefix() -> str | None:
    try:
        from ament_index_python.packages import get_package_prefix

        return get_package_prefix("lekiwi_rmf")
    except (ImportError, LookupError):
        return None


def _stack_excerpt(output: str, limit: int = 40) -> list[str]:
    lines = output.splitlines()
    start = next(
        (index for index, line in enumerate(lines) if STACK_MARKER in line),
        max(0, len(lines) - limit),
    )
    return lines[start : start + limit]


def _is_clean_shutdown(
    *,
    ready: bool,
    timed_out: bool,
    returncode: int | None,
    launch_error: str | None,
) -> bool:
    """Classify only a ready process with an actual zero exit as clean."""
    return launch_error is None and ready and not timed_out and returncode == 0


def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="absolute path for the machine-readable JSON result",
    )
    parser.add_argument(
        "--timeout",
        default=30.0,
        type=float,
        help="seconds allowed for move_group to become ready (default: 30)",
    )
    arguments = parser.parse_args()
    if not arguments.output.is_absolute():
        parser.error("--output must be an absolute path")
    if arguments.timeout <= 0:
        parser.error("--timeout must be positive")
    return arguments


def main() -> int:
    arguments = _parse_arguments()
    repository = Path(__file__).resolve().parents[1]
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    log_directory = arguments.output.parent / f"{arguments.output.stem}.logs"
    log_directory.mkdir(parents=True, exist_ok=True)

    # launch resolves this when its logging configuration is first accessed.
    # A probe-owned path keeps the evidence deterministic and self-contained.
    os.environ["ROS_LOG_DIR"] = str(log_directory)
    state: dict[str, object] = {
        "ready": False,
        "timed_out": False,
        "returncode": None,
    }
    output_chunks: list[str] = []
    readiness_buffer = ""
    launch_error: str | None = None

    try:
        import launch
        from launch.actions import EmitEvent, TimerAction
        from launch.event_handlers import OnProcessExit, OnProcessIO
        from launch.events import Shutdown
        import launch_ros.actions
        from moveit_configs_utils import MoveItConfigsBuilder

        moveit_config = (
            MoveItConfigsBuilder("lekiwi", package_name="lekiwi_rmf")
            .robot_description(
                file_path="urdf/lekiwi.urdf.xacro", mappings={"sim": "false"}
            )
            .robot_description_semantic(file_path="config/lekiwi.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )
        parameters = moveit_config.to_dict()
        parameters["octomap_resolution"] = 0.1

        move_group = launch_ros.actions.Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group_shutdown_probe",
            output="log",
            parameters=[parameters],
            respawn=False,
        )

        def on_output(event):
            nonlocal readiness_buffer
            text = event.text.decode(errors="replace")
            output_chunks.append(text)
            readiness_buffer = (readiness_buffer + text)[-512:]
            if not state["ready"] and READY_MARKER in readiness_buffer:
                state["ready"] = True
                return [EmitEvent(event=Shutdown(reason="move_group readiness observed"))]
            return None

        def on_exit(event, _context):
            state["returncode"] = event.returncode

        def on_timeout(_context):
            state["timed_out"] = not bool(state["ready"])
            return [EmitEvent(event=Shutdown(reason="move_group readiness timeout"))]

        description = launch.LaunchDescription(
            [
                move_group,
                launch.actions.RegisterEventHandler(
                    OnProcessIO(
                        target_action=move_group,
                        on_stdout=on_output,
                        on_stderr=on_output,
                    )
                ),
                launch.actions.RegisterEventHandler(
                    OnProcessExit(target_action=move_group, on_exit=on_exit)
                ),
                TimerAction(
                    period=arguments.timeout,
                    actions=[launch.actions.OpaqueFunction(function=on_timeout)],
                ),
            ]
        )
        launch_service = launch.LaunchService(argv=[])
        launch_service.include_launch_description(description)
        launch_service.run(shutdown_when_idle=False)
    except Exception as error:  # Preserve evidence for dependency/config failures.
        launch_error = f"{type(error).__name__}: {error}"

    combined_output = "".join(output_chunks)
    returncode = state["returncode"]
    clean_shutdown = _is_clean_shutdown(
        ready=bool(state["ready"]),
        timed_out=bool(state["timed_out"]),
        returncode=returncode if isinstance(returncode, int) else None,
        launch_error=launch_error,
    )
    revision = _command_output(["git", "rev-parse", "HEAD"], cwd=repository)
    dirty_output = _command_output(["git", "status", "--porcelain"], cwd=repository)
    artifact = {
        "schema_version": 1,
        "probe": "moveit_shutdown",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "revision": revision,
        "working_tree_dirty": bool(dirty_output),
        "clean_shutdown": clean_shutdown,
        "move_group_exit_code": returncode,
        "move_group_ready": bool(state["ready"]),
        "timed_out": bool(state["timed_out"]),
        "launch_error": launch_error,
        "package_versions": _package_versions(),
        "lekiwi_rmf_package_prefix": _lekiwi_package_prefix(),
        "platform": {
            "machine": platform.machine(),
            "release": platform.release(),
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        },
        "log_directory": str(log_directory),
        "output_excerpt": _stack_excerpt(combined_output),
    }
    arguments.output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n")

    if clean_shutdown:
        print(f"move_group shut down cleanly; evidence: {arguments.output}")
        return 0
    print(
        "move_group shutdown qualification failed "
        f"(ready={state['ready']}, exit={returncode}, timeout={state['timed_out']}); "
        f"evidence: {arguments.output}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
