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
    bad = scan.assess_scan([0.08] * 360, 0.08, 12.0)
    assert not bad.usable
    assert "all 360 ranges" in bad.message

    good = scan.assess_scan([0.08, 1.2, float("inf")] * 120, 0.08, 12.0)
    assert good.usable
    assert "360 ranges" in good.message


@pytest.mark.parametrize('invalid', [float('-inf'), float('nan'), 0.01, 13.0])
def test_scan_check_rejects_a_single_invalid_ray_even_when_most_ranges_are_good(invalid):
    scan = _load('sim_scan_check_invalid', 'scripts/sim-scan-check.py')
    ranges = [2.0] * 360
    ranges[128] = invalid  # Native AMD renderer trial: -inf outside the measured body mask.
    result = scan.assess_scan(ranges, 0.08, 12.0)
    assert not result.usable and 'invalid ranges' in result.message


def test_scan_check_rejects_a_blind_all_infinite_scan():
    scan = _load('sim_scan_check_blind', 'scripts/sim-scan-check.py')
    assert not scan.assess_scan([float('inf')] * 360, 0.08, 12.0).usable


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
    (checkout / "scripts" / "lib").mkdir(parents=True)
    for name in ("sim-up.sh", "lib/launcher.sh"):
        shutil.copy2(ROOT / "scripts" / name, checkout / "scripts" / name)
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
    assert command == ["launch", "lekiwi_rmf", "bringup.launch.py", "profile:=sim", "slam_mode:=localization", "gui:=false"]
    # The recorded stack PID is the process that became ros2 launch.
    assert (logs / "runtime" / "stack.pid").read_text().strip() == pid


def test_shutdown_probe_reserves_an_isolated_domain_through_launch_failure(tmp_path, monkeypatch):
    import contextlib
    import json
    from types import SimpleNamespace

    import domain_coordinator
    import launch_ros.actions
    from lekiwi_rmf import moveit_config

    probe = _load("isolated_shutdown_probe", "scripts/moveit-shutdown-probe.py")
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "caller-logs"))
    output = tmp_path / "probe.json"
    monkeypatch.setattr(probe, "_parse_arguments",
                        lambda: SimpleNamespace(output=output, timeout=1))
    monkeypatch.setattr(probe, "_command_output", lambda *args, **kwargs: None)
    monkeypatch.setattr(probe, "_package_versions", lambda: {})
    monkeypatch.setattr(probe, "_package_prefix", lambda *args: None)
    monkeypatch.setattr(moveit_config, "moveit_config_builder",
                        lambda _: SimpleNamespace(to_moveit_configs=lambda: SimpleNamespace(to_dict=lambda: {})))
    held = []
    captured = {}

    @contextlib.contextmanager
    def lease(*, selector):
        domain = selector()
        assert 1 <= domain <= 100 and domain != 42
        held.append(domain)
        try:
            yield domain
        finally:
            held.clear()

    def node(**kwargs):
        captured.update(kwargs)
        assert held == [int(kwargs["additional_env"]["ROS_DOMAIN_ID"])]
        raise RuntimeError("stop before launching the isolated process")

    monkeypatch.setattr(domain_coordinator, "domain_id", lease)
    monkeypatch.setattr(launch_ros.actions, "Node", node)
    assert probe.main() == 1
    result = json.loads(output.read_text())
    assert result["ros_domain_id"] == int(captured["additional_env"]["ROS_DOMAIN_ID"])
    assert "stop before launching" in result["launch_error"]
    assert not held and os.environ["ROS_DOMAIN_ID"] == "42"


@pytest.mark.parametrize('missing_arm_joint', [False, True])
def test_workspace_gate_sends_only_complete_arm_joints(monkeypatch, missing_arm_joint):
    from types import SimpleNamespace

    from sensor_msgs.msg import JointState

    from lekiwi_rmf.arm_trajectory import ARM_JOINTS

    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    gate = _load('acceptance_gates_joint_scope', 'scripts/test-acceptance-gates.py')
    names = list(reversed(ARM_JOINTS)) + ['wheel_joint']
    if missing_arm_joint:
        names.remove('arm_gripper')
    joints = JointState(name=names, position=[0.] * len(names))
    joints.header.stamp.sec = 1
    sent = []
    mutations = []
    flags = dict(base_motion_permitted=True, arm_motion_permitted=True, driver='ARMED')

    def wait(condition, _seconds):
        assert condition()

    def scene(operation):
        mutations.append(operation)
        added = operation == gate.CollisionObject.ADD
        flags.update(collision=added, arm_motion_permitted=not added)

    def send(goal):
        sent.append(goal)
        return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(accepted=False))

    def tick(*_args, **_kwargs):
        joints.position[names.index('wheel_joint')] = .5

    reply = SimpleNamespace(valid=False, contacts=[SimpleNamespace(
        contact_body_1='arm_wrist_roll', contact_body_2='qualification_workspace_intrusion')])
    def ready():
        return True

    node = SimpleNamespace(
        case='workspace', checks={}, flags=flags, joints=joints, pose=(0., 0., 0.),
        wait_ready=lambda: None, wait=wait, tick=tick, scene_object=scene,
        scene=SimpleNamespace(service_is_ready=ready),
        validity=SimpleNamespace(service_is_ready=ready, call_async=lambda _: SimpleNamespace(
            done=ready, result=lambda: reply)),
        arm=SimpleNamespace(server_is_ready=ready, send_goal_async=send),
    )
    ticks = iter(range(20))
    monkeypatch.setattr(gate, 'time', SimpleNamespace(monotonic=lambda: next(ticks)))
    if missing_arm_joint:
        with pytest.raises(RuntimeError, match='complete finite arm joint snapshot'):
            gate.Gates.run(node)
        assert not sent and not mutations
    else:
        gate.Gates.run(node)
        trajectory = sent[0].trajectory
        assert list(trajectory.joint_names) == list(ARM_JOINTS)
        assert list(trajectory.points[0].positions) == [0., 0., 0., 0., .02, 0.]
        assert node.checks['arm_workspace_intrusion_stop']['maximum_joint_change_rad'] == 0
        assert mutations == [gate.CollisionObject.ADD, gate.CollisionObject.REMOVE]
