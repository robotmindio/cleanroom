#!/usr/bin/env python3
"""Collect fail-closed evidence for qualified LeKiwi simulation acceptance.

This is deliberately an *evidence* runner, not an approval switch.  It can
prove repository checks and record observations from an already managed
simulation, but it cannot replace the administrator's GPU, runtime, fault
injection, or RViz/MoveIt review.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from typing import Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_CTESTS = (
    "test_odometry", "test_odom_scale", "test_free_space", "test_camera_relay",
    "test_arm_trajectory", "test_collision_model", "test_driver_link",
    "test_cmd_vel_mux", "test_teleop", "test_readiness_gate", "test_sim_host_tools",
    "test_rtabmap_db_maintenance", "test_torque_control", "test_lidar_detection",
    "test_safety_supervisor", "test_arm_workspace_monitor", "test_map_bundle",
    "test_launch_validation", "test_rtabmap_session_guard", "test_fake_host",
    "test_zmq_security", "test_simulation_model", "test_service_installation",
    "test_rmf_owner_guard", "test_moveit_shutdown_probe",
    "test_sim_qualification",
    "test_test_cmd_vel_mux_launch.py", "test_test_driver_fake_host_launch.py",
    "test_test_simulation_physics_launch.py", "test_test_sim_native_failsafe_launch.py",
    "test_test_safety_supervisor_launch.py", "test_test_arm_workspace_monitor_launch.py",
    "test_test_rmf_owner_guard_launch.py",
    "test_test_moveit_driver_e2e_launch.py",
)


@dataclass
class Result:
    name: str
    command: list[str]
    returncode: int | None
    required: bool
    passed: bool
    log: str
    note: str = ""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def run_command(
    evidence: Path, name: str, command: Sequence[str], *, required: bool = True,
    cwd: Path = ROOT, timeout: float | None = None, note: str = "",
    environment: Mapping[str, str] | None = None,
) -> Result:
    """Run one bounded check and persist combined stdout/stderr."""
    safe_name = re.sub(r"[^a-zA-Z0-9_.-]+", "_", name)
    log = evidence / "commands" / f"{safe_name}.log"
    try:
        completed = subprocess.run(
            list(command), cwd=cwd, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=timeout, check=False,
            env=environment,
        )
        output = completed.stdout
        returncode = completed.returncode
    except FileNotFoundError as error:
        output = f"command unavailable: {error}\n"
        returncode = None
    except subprocess.TimeoutExpired as error:
        output = (error.stdout or "") + f"\ncommand timed out after {timeout:g} s\n"
        returncode = None
    log.write_text(
        "$ " + " ".join(command) + "\n\n" + output,
        encoding="utf-8", errors="replace",
    )
    return Result(
        name=name,
        command=list(command),
        returncode=returncode,
        required=required,
        passed=returncode == 0,
        log=str(log.relative_to(evidence)),
        note=note,
    )


def ctest_names(output: str) -> set[str]:
    return set(re.findall(r"^\s*Test\s+#?\d+:\s+(.+?)\s*$", output, flags=re.MULTILINE))


def cmake_cache_value(build_dir: Path, key: str) -> str | None:
    cache = build_dir / "CMakeCache.txt"
    if cache.is_file():
        match = re.search(
            rf"^{re.escape(key)}(?::[^=]+)?=(.+)$",
            cache.read_text(encoding="utf-8", errors="replace"),
            re.MULTILINE,
        )
        if match:
            return match.group(1)
    return None


def configured_python(build_dir: Path) -> str:
    configured = cmake_cache_value(build_dir, "Python3_EXECUTABLE")
    if configured and Path(configured).is_file():
        return configured
    return sys.executable


def build_provenance_result(evidence: Path, build_dir: Path) -> tuple[Result, Path | None]:
    """Bind test and probe evidence to a build configured from this checkout."""
    source_value = cmake_cache_value(build_dir, "CMAKE_HOME_DIRECTORY")
    install_value = cmake_cache_value(build_dir, "CMAKE_INSTALL_PREFIX")
    source = Path(source_value).expanduser().resolve() if source_value else None
    install = Path(install_value).expanduser().resolve() if install_value else None
    installed_manifest = install / "share" / "lekiwi_rmf" / "package.xml" if install else None
    valid = source == ROOT and installed_manifest is not None and installed_manifest.is_file()
    log = evidence / "commands" / "build-provenance.json"
    log.write_text(json.dumps({
        "build_dir": str(build_dir.expanduser().resolve()),
        "configured_source": str(source) if source else None,
        "expected_source": str(ROOT),
        "install_prefix": str(install) if install else None,
        "installed_manifest": str(installed_manifest) if installed_manifest else None,
        "valid": valid,
    }, indent=2) + "\n", encoding="utf-8")
    return Result(
        name="build-provenance", command=[], returncode=0 if valid else 1,
        required=True, passed=valid, log=str(log.relative_to(evidence)),
        note="The selected CMake build and installed package must come from this checkout.",
    ), install


def environment_for_install(install_prefix: Path | None) -> dict[str, str]:
    """Put the selected build's package ahead of any previously sourced overlay."""
    environment = os.environ.copy()
    if install_prefix is None:
        return environment

    def prepend(name: str, values: Sequence[Path]) -> None:
        current = environment.get(name, "")
        entries = [str(value) for value in values]
        if current:
            entries.append(current)
        environment[name] = os.pathsep.join(entries)

    prepend("AMENT_PREFIX_PATH", [install_prefix])
    prepend("CMAKE_PREFIX_PATH", [install_prefix])
    python_paths = sorted(install_prefix.glob("lib/python*/site-packages"))
    prepend("PYTHONPATH", [ROOT, *python_paths])
    prepend("PATH", [install_prefix / "lib" / "lekiwi_rmf"])
    return environment


def write_runtime_checklist(evidence: Path) -> None:
    (evidence / "runtime-checklist.md").write_text(
        """# Required simulation acceptance observations

This runner never marks the simulation qualified.  The administrator must
complete and attach the following to this evidence directory after reviewing
the final revision on the qualified server:

- [ ] Confirm the renderer evidence came from the service account and intended GPU.
- [ ] With the clear simulated room, record manual-over-Nav2 mux priority and
  zero output after the publishers stop (`/cmd_vel_muxed` and `/cmd_vel_safe`).
- [ ] Record collision-monitor active state, the `/cmd_vel_safe` publisher and
  subscriber topology, and a live delayed/noisy `/camera/depth/points` sample.
- [ ] Launch MoveIt, place an arm-workspace obstacle, and record the fresh
  `move_group` octomap, `/safety/arm_workspace_clear=false`, and interruption
  of the guarded trajectory.
- [ ] Open a newly launched RViz MotionPlanning panel and record its values as
  well as the corresponding `move_group` values.  A `move_group` parameter
  alone is not RViz evidence.
- [ ] Fault-inject ROS omni-controller/bridge and arm-adapter heartbeat loss;
  retain native failsafe output proving wheel demand zero and arm measured hold.
- [ ] Stop only the recorded process group with `scripts/ros-stop.sh`, then
  attach the final 30 lines of `~/.ros/lekiwi/sim-stack.log` (collected when
  available by `--collect-runtime`).

Do not set `config/safety_acceptance.yaml` to `validated: true`: this is
simulation acceptance evidence only and cannot satisfy physical acceptance.
""",
        encoding="utf-8",
    )


def collect_log_tails(evidence: Path, logs_dir: Path) -> dict[str, str]:
    logs = evidence / "logs"
    logs.mkdir(exist_ok=True)
    collected: dict[str, str] = {}
    candidates = {"sim-stack.log": logs_dir / "sim-stack.log"}
    latest = Path.home() / ".ros" / "log" / "latest"
    if latest.exists():
        candidates["ros-launch.log"] = latest.resolve() / "launch.log"
    for name, source in candidates.items():
        if source.is_file():
            target = logs / name
            lines = source.read_text(encoding="utf-8", errors="replace").splitlines()
            target.write_text("\n".join(lines[-30:]) + ("\n" if lines else ""), encoding="utf-8")
            collected[name] = str(target.relative_to(evidence))
    return collected


def valid_moveit_shutdown_artifact(
    payload: object, revision: str, install_prefix: Path | None, probe_passed: bool,
) -> bool:
    """Return whether direct probe output proves an orderly current-revision exit."""
    required_keys = (
        "schema_version", "revision", "clean_shutdown", "move_group_exit_code",
        "package_versions", "lekiwi_rmf_package_prefix",
    )
    return (
        isinstance(payload, dict)
        and all(key in payload for key in required_keys)
        and payload["schema_version"] == 1
        and payload["revision"] == revision
        and install_prefix is not None
        and isinstance(payload["lekiwi_rmf_package_prefix"], str)
        and Path(payload["lekiwi_rmf_package_prefix"]).resolve() == install_prefix
        and payload["clean_shutdown"] is True
        and payload["move_group_exit_code"] == 0
        and isinstance(payload["package_versions"], dict)
        and bool(payload["package_versions"])
        and probe_passed
    )


def moveit_shutdown_results(
    evidence: Path, probe: Path, install_prefix: Path | None,
) -> tuple[Result, Result]:
    """Require separately collected, revision-bound clean MoveIt shutdown proof.

    The current E2E launch test intentionally ignores move_group's shutdown
    exit status, so its passing CTest result cannot be substituted here.
    """
    output = evidence / "moveit-clean-shutdown.json"
    probe_run = run_command(
        evidence, "moveit-clean-shutdown-probe", [sys.executable, str(probe), "--output", str(output)],
        timeout=180.0,
        note="The probe is invoked directly; a pre-existing artifact is never trusted.",
        environment=environment_for_install(install_prefix),
    )
    log = evidence / "commands" / "moveit-clean-shutdown-evidence.log"
    try:
        payload = json.loads(output.read_text(encoding="utf-8"))
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False).stdout.strip()
        valid = valid_moveit_shutdown_artifact(
            payload, revision, install_prefix, probe_run.passed
        )
        message = json.dumps(payload, indent=2) + "\n"
        message += f"expected revision: {revision}\n"
        if not valid:
            message += (
                "invalid: require schema_version=1, current revision and selected "
                "install prefix, package_versions, clean_shutdown=true, "
                "move_group_exit_code=0, and a successful probe\n"
            )
        log.write_text(message, encoding="utf-8")
        return probe_run, Result(
            "moveit-clean-shutdown-evidence", [str(output)], 0 if valid else 1, True, valid,
            str(log.relative_to(evidence)),
            "Required because the E2E test does not assert move_group's shutdown exit code.",
        )
    except (OSError, json.JSONDecodeError) as error:
        log.write_text(f"could not read direct-probe output {output}: {error}\n", encoding="utf-8")
        return probe_run, Result(
            "moveit-clean-shutdown-evidence", [str(output)], None, True, False,
            str(log.relative_to(evidence)),
            "Required because the E2E test does not assert move_group's shutdown exit code.",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build" / "lekiwi_rmf",
                        help="configured CMake build directory")
    parser.add_argument("--output-dir", type=Path,
                        help="new evidence directory outside the source checkout (default: ~/.ros/lekiwi/qualification-evidence/<UTC timestamp>)")
    parser.add_argument("--collect-runtime", action="store_true",
                        help="collect bounded observations from an already-running managed simulation")
    parser.add_argument("--runtime-timeout", type=float, default=30.0,
                        help="seconds to wait for each runtime ROS observation (default: 30)")
    parser.add_argument("--logs-dir", type=Path, default=Path.home() / ".ros" / "lekiwi",
                        help="managed LeKiwi log directory")
    parser.add_argument("--moveit-shutdown-probe", type=Path,
                        default=ROOT / "scripts" / "moveit-shutdown-probe.py",
                        help="repository MoveIt clean-shutdown probe (default: scripts/moveit-shutdown-probe.py)")
    args = parser.parse_args()
    if args.runtime_timeout <= 0:
        parser.error("--runtime-timeout must be positive")

    evidence = args.output_dir or Path.home() / ".ros" / "lekiwi" / "qualification-evidence" / utc_now()
    evidence = evidence.expanduser().resolve()
    try:
        evidence.relative_to(ROOT)
    except ValueError:
        pass
    else:
        parser.error("--output-dir must be outside the source checkout so evidence cannot dirty it")
    if evidence.exists():
        parser.error(f"refusing to overwrite existing evidence directory: {evidence}")
    (evidence / "commands").mkdir(parents=True)
    write_runtime_checklist(evidence)
    results: list[Result] = []
    build_result, install_prefix = build_provenance_result(evidence, args.build_dir)

    # Record provenance first so partial/failing collections remain auditable.
    results.append(run_command(evidence, "git-revision", ["git", "rev-parse", "HEAD"]))
    results.append(build_result)
    status_result = run_command(evidence, "git-status", ["git", "status", "--porcelain=v1"], required=False,
                                note="The exact dirty state is retained in this command log.")
    results.append(status_result)
    status_output = (evidence / status_result.log).read_text(encoding="utf-8", errors="replace").partition("\n\n")[2]
    worktree_clean = status_result.passed and not status_output.strip()
    results.append(Result(
        name="git-working-tree-clean", command=[], returncode=0 if worktree_clean else 1,
        required=True, passed=worktree_clean, log=status_result.log,
        note="Qualification requires an exact committed revision; inspect git-status.log for any dirty paths.",
    ))
    results.append(run_command(evidence, "git-diff-check", ["git", "diff", "--check"]))
    results.append(run_command(evidence, "safety-default-deny", [
        sys.executable, "-c",
        "import pathlib,re,sys; s=pathlib.Path('config/safety_acceptance.yaml').read_text(); "
        "sys.exit(0 if re.search(r'^\\s*validated:\\s*false\\s*(?:#.*)?$',s,re.M) else 1)",
    ], note="The source acceptance switch must remain false."))

    for directory in (ROOT / "lekiwi_rmf", ROOT / "scripts", ROOT / "test"):
        files = sorted(str(path) for path in directory.rglob("*.py"))
        results.append(run_command(evidence, f"python-syntax-{directory.name}", [
            sys.executable, "-c",
            "import ast, pathlib, sys; [ast.parse(pathlib.Path(p).read_text(encoding='utf-8'), filename=p) for p in sys.argv[1:]]",
            *files,
        ]))

    results.append(run_command(evidence, "shellcheck", [
        "shellcheck", "--severity=warning", *sorted(str(path) for path in (ROOT / "scripts").glob("*.sh")),
    ], note="ShellCheck is a required deployment-image dependency."))

    python_for_tests = configured_python(args.build_dir)
    results.append(run_command(evidence, "pyzmq-dependency", [python_for_tests, "-c", "import zmq; print(zmq.__version__)"],
                               note="Must be available to CMake's selected test interpreter."))
    listed = run_command(evidence, "ctest-list", ["ctest", "--test-dir", str(args.build_dir), "-N"])
    results.append(listed)
    listed_output = (evidence / listed.log).read_text(encoding="utf-8", errors="replace")
    names = ctest_names(listed_output)
    missing = sorted(set(EXPECTED_CTESTS) - names)
    unexpected = sorted(names - set(EXPECTED_CTESTS))
    expected_log = evidence / "expected-ctest-tests.json"
    expected_log.write_text(json.dumps({"expected": EXPECTED_CTESTS, "registered": sorted(names), "missing": missing,
                                        "unexpected": unexpected}, indent=2) + "\n", encoding="utf-8")
    results.append(Result(
        name="required-ctest-names", command=[], returncode=0 if not missing else 1,
        required=True, passed=not missing, log=str(expected_log.relative_to(evidence)),
        note="The three pyzmq-dependent tests are required: test_fake_host, test_test_driver_fake_host_launch.py, test_test_moveit_driver_e2e_launch.py.",
    ))
    results.append(run_command(evidence, "ctest", ["ctest", "--test-dir", str(args.build_dir), "--output-on-failure"], timeout=600.0))
    results.extend(moveit_shutdown_results(
        evidence,
        args.moveit_shutdown_probe.expanduser().resolve(),
        install_prefix,
    ))
    results.append(run_command(evidence, "renderer-preflight", [str(ROOT / "scripts" / "sim-renderer-check.py")],
                               note="Pass requires EGL/OpenGL 3.3+ under this service account."))

    runtime: dict[str, object] = {"requested": args.collect_runtime, "collected_logs": {}}
    if args.collect_runtime:
        runtime_commands = (
            ("runtime-scan", [str(ROOT / "scripts" / "sim-scan-check.py"), "--timeout", str(args.runtime_timeout)]),
            ("runtime-collision-monitor", ["ros2", "lifecycle", "get", "/collision_monitor"]),
            ("runtime-cmd-vel-safe-topology", ["ros2", "topic", "info", "-v", "/cmd_vel_safe"]),
            ("runtime-depth-topology", ["ros2", "topic", "info", "-v", "/camera/depth/points"]),
            ("runtime-depth-sample", ["timeout", str(args.runtime_timeout), "ros2", "topic", "echo", "--once", "/camera/depth/points"]),
            ("runtime-nodes", ["ros2", "node", "list"]),
            ("runtime-move-group-parameters", ["ros2", "param", "list", "/move_group"]),
        )
        for name, command in runtime_commands:
            results.append(run_command(evidence, name, command, timeout=args.runtime_timeout + 5.0,
                                       note="Runtime output is evidence for administrator review; it is not approval."))
        runtime["collected_logs"] = collect_log_tails(evidence, args.logs_dir.expanduser())
    else:
        runtime["reason"] = "Runtime collection was not requested; launch the managed stack separately, then rerun with --collect-runtime."

    required_failures = [result.name for result in results if result.required and not result.passed]
    status = {
        "schema": 1,
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "repository": str(ROOT),
        "build_dir": str(args.build_dir.expanduser().resolve()),
        "results": [asdict(result) for result in results],
        "runtime": runtime,
        "qualification": {
            "status": "NOT_QUALIFIED",
            "required_check_failures": required_failures,
            "reason": "This tool records simulation evidence only. Physical safety acceptance, administrator GPU/service-account review, runtime fault injection, and RViz MotionPlanning-panel observations remain mandatory.",
        },
    }
    (evidence / "summary.json").write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
    print(f"evidence: {evidence}")
    print("qualification: NOT_QUALIFIED (evidence runner never grants approval)")
    if required_failures:
        print("failed required checks: " + ", ".join(required_failures))
        return 1
    print("repository checks passed; complete runtime-checklist.md before administrator review")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
