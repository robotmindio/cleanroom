"""Unit checks for the physical-torque request/reply contract."""

import dataclasses
import importlib.util
import json
import logging
import math
import pathlib
import sys
import types
import runpy

import pytest

from lekiwi_rmf.arm_trajectory import JOINT_LIMITS, action_positions, load_calibration
from lekiwi_rmf.motion_guards import load_base_speed_limits
from lekiwi_rmf.odometry import BASE_XY_SCALE, BASE_YAW_SCALE
from lekiwi_rmf.torque_control import (
    TorqueControlClient, TorqueControlError, enable_with_rollback,
    react_to_command_silence, run_all_safety_steps, torque_readback_matches,
    validate_action_payload, validated_bind_address,
)


def test_qualification_socket_changes_only_wire_data_and_expires():
    cls = runpy.run_path(str(pathlib.Path(__file__).parents[1] /
                            'scripts/test-host-telemetry.py'))['FaultSocket']
    sent, now = [], [0.0]
    socket = cls(types.SimpleNamespace(send_multipart=lambda f, **k: sent.append(f)),
                 lambda: now[0])
    frames = [json.dumps({'sequence': 1, '_lekiwi_motor_health': {
        'statuses': {'motor_bus': {'level': 0, 'message': 'healthy'}}}}).encode()]
    with pytest.raises(RuntimeError, match='before'):
        socket.inject('duplicate')
    socket.send_multipart(frames)
    socket.inject('duplicate')
    new = [frames[0].replace(b'1', b'2', 1)]
    socket.send_multipart(new)
    assert sent[-1] == frames
    now[0] = 8.0
    socket.send_multipart(new)
    assert sent[-1] == new
    socket.inject('diagnostic')
    socket.send_multipart(new)
    assert json.loads(sent[-1][0])['_lekiwi_motor_health']['statuses']['motor_bus']['level'] == 2
    assert json.loads(new[0])['_lekiwi_motor_health']['statuses']['motor_bus']['level'] == 0
    assert len(sent[-1]) == 1
    now[0] = 16.0
    socket.send_multipart(new)
    assert sent[-1] == new
    with pytest.raises(ValueError, match='unsupported'):
        socket.inject('unknown')

def test_wire_contract_matches_the_deployed_motor_host():
    from lekiwi_rmf import host_protocol as protocol

    payload = protocol.observation_payload(
        {"arm_shoulder_pan.pos": 1.5, "x.vel": 0.0}, session="s", sequence=3,
        sample_monotonic_ns=7, torque_enabled=True, motor_health={"version": 1},
        odometry={"pose": [0, 0, 0]}, arm_status=None,
    )
    # Byte-for-byte the camera-less message the Pi host already in service
    # sends (protocol 3); LeRobot's own client requires the empty "_cams".
    assert json.dumps(payload) == (
        '{"_cams": [], "arm_shoulder_pan.pos": 1.5, "x.vel": 0.0, '
        '"_lekiwi_protocol": 3, "_lekiwi_session": "s", "_lekiwi_sequence": 3, '
        '"_lekiwi_sample_monotonic_ns": 7, "_lekiwi_torque_enabled": true, '
        '"_lekiwi_motor_health": {"version": 1}, "_lekiwi_odometry": {"pose": [0, 0, 0]}, '
        '"_lekiwi_arm_trajectory": null}'
    )
    assert protocol.ARM_LEASE_KEYS == ("_lekiwi_arm_goal", "_lekiwi_arm_permission")
    assert protocol.STATE_KEYS == (
        "arm_shoulder_pan.pos", "arm_shoulder_lift.pos", "arm_elbow_flex.pos",
        "arm_wrist_flex.pos", "arm_wrist_roll.pos", "arm_gripper.pos",
        "x.vel", "y.vel", "theta.vel",
    )
    assert [protocol.TorqueCommand.ENABLE, protocol.TorqueCommand.DISABLE,
            protocol.TorqueCommand.STATE, protocol.TorqueCommand.TRAJECTORY_START,
            protocol.TorqueCommand.TRAJECTORY_CANCEL] == [
        "enable", "disable", "state", "trajectory_start", "trajectory_cancel"]
    assert protocol.valid_goal_id(1) and protocol.valid_goal_id(2**48 - 1)
    assert not any(map(protocol.valid_goal_id, (0, 2**48, True, 1.0, -1)))


class _Socket:
    def __init__(self, response):
        self.response = response
        self.options = []
        self.endpoint = None
        self.sent = None
        self.closed = False

    def setsockopt(self, option, value):
        self.options.append((option, value))

    def connect(self, endpoint):
        self.endpoint = endpoint

    def send_json(self, value):
        self.sent = value

    def recv_json(self):
        if isinstance(self.response, Exception):
            raise self.response
        return self.response

    def close(self):
        self.closed = True


class _Context:
    def __init__(self, response):
        self.socket_instance = _Socket(response)
        self.terminated = False

    def socket(self, kind):
        assert kind == 1
        return self.socket_instance

    def term(self):
        self.terminated = True


class _Zmq:
    REQ = 1
    LINGER = 2
    SNDTIMEO = 3
    RCVTIMEO = 4

    def __init__(self, response):
        self.context = _Context(response)

    def Context(self):
        return self.context


def test_client_requires_matching_host_confirmation():
    zmq = _Zmq({"ok": True, "torque_enabled": False})
    client = TorqueControlClient("127.0.0.1", 5557, 200, zmq)

    client.set_enabled(False)

    socket = zmq.context.socket_instance
    assert socket.endpoint == "tcp://127.0.0.1:5557"
    assert socket.sent == {"command": "disable"}
    assert socket.closed and zmq.context.terminated


def test_client_rejects_a_missing_or_wrong_confirmation():
    with pytest.raises(TorqueControlError, match="unexpected"):
        TorqueControlClient("127.0.0.1", zmq_module=_Zmq({"ok": True, "torque_enabled": True})).set_enabled(False)
    with pytest.raises(TorqueControlError, match="rejected"):
        TorqueControlClient("127.0.0.1", zmq_module=_Zmq({"ok": False, "error": "bus failure"})).set_enabled(True)


def test_torque_client_supports_an_unauthenticated_remote_host():
    client = TorqueControlClient("192.0.2.10", zmq_module=_Zmq({}))
    assert client.host == "192.0.2.10"


def test_driver_arms_and_disarms_through_the_torque_client():
    driver = (pathlib.Path(__file__).parents[1] / "lekiwi_rmf" / "driver.py").read_text()

    assert "self.set_servo_torque(False)" in driver
    assert "self.set_servo_torque(True)" in driver


def test_torque_confirmation_requires_every_expected_motor():
    expected = ("left", "right", "arm")
    assert torque_readback_matches({"left": 1, "right": 1, "arm": 1}, True, expected)
    assert torque_readback_matches({"left": 0, "right": 0, "arm": 0}, False, expected)
    assert not torque_readback_matches({"left": 1, "right": 1}, True, expected)
    assert not torque_readback_matches({"left": 1, "right": 1, "arm": 0}, True, expected)


def test_safety_steps_do_not_short_circuit_after_a_failure():
    called = []

    def fail_stop():
        called.append("stop")
        raise OSError("bus timeout")

    failures = run_all_safety_steps((
        ("stop", fail_stop),
        ("disable", lambda: called.append("disable")),
        ("verify", lambda: called.append("verify")),
    ))

    assert called == ["stop", "disable", "verify"]
    assert len(failures) == 1
    assert failures[0][0] == "stop"


def test_failed_enable_step_rolls_physical_torque_back_off():
    called = []

    def verify_enabled():
        called.append("verify enabled")
        raise RuntimeError("servo 3 still reads 0")

    with pytest.raises(RuntimeError, match="enable transaction failed"):
        enable_with_rollback(
            (
                ("enable", lambda: called.append("enable")),
                ("verify enabled", verify_enabled),
            ),
            (
                ("disable", lambda: called.append("disable")),
                ("verify disabled", lambda: called.append("verify disabled")),
            ),
        )

    assert called == ["enable", "verify enabled", "disable", "verify disabled"]


def test_control_listener_allows_the_all_interfaces_bind():
    assert validated_bind_address("127.0.0.1") == "127.0.0.1"
    assert validated_bind_address("192.0.2.20") == "192.0.2.20"
    assert validated_bind_address("0.0.0.0") == "0.0.0.0"
    for address in ("*", "robot.local", "224.0.0.1"):
        with pytest.raises(ValueError):
            validated_bind_address(address)


def test_host_action_decoder_requires_complete_strict_finite_json():
    keys = ("joint.pos", "x.vel")
    assert validate_action_payload('{"joint.pos": 1, "x.vel": 0}', keys) == {
        "joint.pos": 1.0, "x.vel": 0.0,
    }
    for malformed in (
        '{"joint.pos": 1}',
        '{"joint.pos": 1, "joint.pos": 2, "x.vel": 0}',
        '{"joint.pos": NaN, "x.vel": 0}',
        '{"joint.pos": true, "x.vel": 0}',
    ):
        with pytest.raises(ValueError):
            validate_action_payload(malformed, keys)


def test_command_silence_holds_the_robot_and_cuts_torque_only_when_opted_in():
    calls = []

    assert react_to_command_silence(
        False, lambda: calls.append("cut"), lambda: calls.append("hold")
    ) == "hold"
    assert calls == ["hold"]

    calls.clear()
    assert react_to_command_silence(
        True, lambda: calls.append("cut"), lambda: calls.append("hold")
    ) == "cut"
    assert calls == ["cut"]


class _SequencedZmq(_Zmq):
    """A new context per request, each answering with the next scripted response."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.contexts = []

    def Context(self):
        context = _Context(self.responses.pop(0))
        self.contexts.append(context)
        return context


def test_client_retries_once_when_a_request_gets_no_reply():
    zmq = _SequencedZmq([TimeoutError("no reply"), {"ok": True, "torque_enabled": False}])

    TorqueControlClient("127.0.0.1", zmq_module=zmq).set_enabled(False)

    assert len(zmq.contexts) == 2
    assert all(context.terminated and context.socket_instance.closed for context in zmq.contexts)


def test_client_gives_up_after_two_silent_attempts_and_never_retries_a_refusal():
    silent = _SequencedZmq([TimeoutError("no reply"), TimeoutError("still nothing")])
    with pytest.raises(TorqueControlError, match="did not reply: still nothing"):
        TorqueControlClient("127.0.0.1", zmq_module=silent).set_enabled(True)
    assert len(silent.contexts) == 2

    refused = _SequencedZmq([{"ok": False, "error": "bus failure"}, {"ok": True, "torque_enabled": True}])
    with pytest.raises(TorqueControlError, match="rejected"):
        TorqueControlClient("127.0.0.1", zmq_module=refused).set_enabled(True)
    assert len(refused.contexts) == 1


# ---------------------------------------------------------------------------
# lekiwi_rmf.motor_host, the bus-independent host core, against a fake motor bus.

ARM = ("arm_shoulder_pan", "arm_gripper")
BASE = ("base_left_wheel",)
MOTORS = ARM + BASE
ROOT = pathlib.Path(__file__).parents[1]


@pytest.fixture
def host():
    pytest.importorskip("zmq", reason="the motor host requires pyzmq")
    from lekiwi_rmf import motor_host

    return motor_host


def _again():
    import zmq

    return zmq.Again()


def _torque_host_script(monkeypatch, filename=None):
    """Load scripts/torque-host.py, which only adds LeRobot wiring, with LeRobot stubbed."""
    pytest.importorskip("zmq", reason="the motor host requires pyzmq")

    @dataclasses.dataclass
    class LeKiwiConfig:
        port: str = ""
        # LeRobot's own default opens the front and wrist cameras.
        cameras: dict = dataclasses.field(default_factory=lambda: {"front": object(), "wrist": object()})

    @dataclasses.dataclass
    class LeKiwiHostConfig:
        port_zmq_cmd: int = 5555
        port_zmq_observations: int = 5556
        watchdog_timeout_ms: int = 500
        max_loop_freq_hz: int = 30

    class LeKiwi:
        def __init__(self, config):
            self.config = config

    modules = {
        "draccus": types.SimpleNamespace(wrap=lambda: (lambda function: function)),
        "lerobot": types.ModuleType("lerobot"),
        "lerobot.motors": types.ModuleType("lerobot.motors"),
        "lerobot.motors.feetech": types.SimpleNamespace(OperatingMode=types.SimpleNamespace(
            POSITION=types.SimpleNamespace(value=0), VELOCITY=types.SimpleNamespace(value=1),
        )),
        "lerobot.robots": types.ModuleType("lerobot.robots"),
        "lerobot.robots.lekiwi": types.ModuleType("lerobot.robots.lekiwi"),
        "lerobot.robots.lekiwi.config_lekiwi": types.SimpleNamespace(
            LeKiwiConfig=LeKiwiConfig, LeKiwiHostConfig=LeKiwiHostConfig,
        ),
        "lerobot.robots.lekiwi.lekiwi": types.SimpleNamespace(LeKiwi=LeKiwi),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "torque_host", filename or ROOT / "scripts" / "torque-host.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("installed", [False, True])
def test_host_speed_profile_is_resolved_in_source_and_installed_layouts(monkeypatch, tmp_path, installed):
    root = pathlib.Path(__file__).parents[1]
    if installed:
        filename = tmp_path / "install/lekiwi_rmf/lib/lekiwi_rmf/torque-host.py"
        profile = tmp_path / "install/lekiwi_rmf/share/lekiwi_rmf/config/nav2_params.yaml"
    else:
        filename = tmp_path / "source/scripts/torque-host.py"
        profile = tmp_path / "source/config/nav2_params.yaml"
    filename.parent.mkdir(parents=True)
    profile.parent.mkdir(parents=True)
    filename.write_text((root / "scripts/torque-host.py").read_text())
    profile.write_text((root / "config/nav2_params.yaml").read_text())
    host = _torque_host_script(monkeypatch, filename)
    assert pathlib.Path(host.TorqueSafetyConfig().nav2_params_file) == profile
    assert host.load_base_speed_limits(profile) == (0.03, 0.06)


class _Bus:
    """Feetech bus stand-in: torque registers that follow enable/disable, plus scripted faults."""

    def __init__(self):
        self.motors = {motor: object() for motor in MOTORS}
        self.torque = dict.fromkeys(MOTORS, 0)
        self.registers = {
            "Present_Position": dict.fromkeys(MOTORS, 12.5),
            "Goal_Position": dict.fromkeys(MOTORS, 13.5),
            "Present_Load": dict.fromkeys(MOTORS, 100),
            "Present_Voltage": dict.fromkeys(MOTORS, 120),
            "Present_Temperature": dict.fromkeys(MOTORS, 35),
            "Status": dict.fromkeys(MOTORS, 0),
            "Present_Current": dict.fromkeys(MOTORS, 10),
            "Max_Temperature_Limit": dict.fromkeys(MOTORS, 70),
            "Min_Voltage_Limit": dict.fromkeys(MOTORS, 45),
            "Max_Voltage_Limit": dict.fromkeys(MOTORS, 140),
            "P_Coefficient": dict.fromkeys(MOTORS, 16),
            "I_Coefficient": dict.fromkeys(MOTORS, 0),
            "D_Coefficient": dict.fromkeys(MOTORS, 32),
            "Maximum_Velocity_Limit": dict.fromkeys(MOTORS, 80),
            "Velocity_Unit_factor": dict.fromkeys(MOTORS, 1),
            "Velocity_closed_loop_P_proportional_coefficient": dict.fromkeys(MOTORS, 20),
            "Velocity_closed_loop_I_integral_coefficient": dict.fromkeys(MOTORS, 0),
            "Max_Torque_Limit": dict.fromkeys(MOTORS, 1000),
            "Torque_Limit": dict.fromkeys(MOTORS, 1000),
            "Acceleration": dict.fromkeys(MOTORS, 0),
            "Maximum_Acceleration": dict.fromkeys(MOTORS, 250),
        }
        self.calls = []
        self.writes = {}
        self.faults = {}
        self.enable_skips = ()
        self.torque_off_skips = ()

    def _fault(self, name):
        if name in self.faults:
            raise self.faults[name]

    def enable_torque(self, num_retry=0):
        self.calls.append("enable_torque")
        self._fault("enable_torque")
        for motor in MOTORS:
            if motor not in self.enable_skips:
                self.torque[motor] = 1
            else:
                self.torque[motor] = 0

    def disable_torque(self, motors=None, num_retry=0):
        selected = MOTORS if motors is None else [motors] if isinstance(motors, str) else motors
        for motor in selected:
            self.calls.append(f"disable_torque {motor}")
            self._fault(f"disable_torque:{motor}")
            self._fault("disable_torque")
            self.torque[motor] = 0

    def sync_read(self, register, motors, normalize=True, num_retry=0):
        self.calls.append(f"read {register}")
        self._fault(register)
        source = self.torque if register == "Torque_Enable" else self.registers[register]
        return {motor: source[motor] for motor in motors if motor in source}

    def sync_write(self, register, values, num_retry=0):
        self.calls.append(f"write {register}")
        self._fault(f"{register}_write")
        self.written = (register, dict(values) if isinstance(values, dict) else values)
        if register == "Torque_Enable":
            self.torque.update({
                motor: values for motor in MOTORS if motor not in self.torque_off_skips
            })
        elif register in ("Goal_Position", "Goal_Velocity"):
            self.torque.update({motor: 1 for motor in values})

    def write(self, register, motor, value):
        self.calls.append(f"write {register} {motor}")
        self.writes[(register, motor)] = value
        if register in self.registers:
            self.registers[register][motor] = value

    def configure_motors(self):
        self.calls.append("configure_motors")


class _Robot:
    def __init__(self, bus):
        self.bus = bus
        self.arm_motors = list(ARM)
        self.base_motors = list(BASE)
        self.actions = []

    def stop_base(self):
        self.bus.calls.append("stop_base")

    def send_action(self, action):
        self.actions.append(action)

    def get_observation(self):
        return {"arm_gripper.pos": 1.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


class _RepSocket:
    def __init__(self):
        self.requests = []
        self.replies = []

    def setsockopt(self, *_args):
        pass

    def bind(self, endpoint):
        self.endpoint = endpoint

    def recv_json(self, flags=0):
        if not self.requests:
            raise _again()
        request = self.requests.pop(0)
        if isinstance(request, Exception):
            raise request
        return request

    def send_json(self, value):
        self.replies.append(value)

    def close(self):
        pass


def _control(host, tmp_path, bus=None):
    socket = _RepSocket()
    control = host.TorqueControlServer(
        socket, host.LocalArmExecutor(lambda: 0.0, 0.5),
        load_calibration(tmp_path / "arm_calibration.json"), "test-session",
    )
    robot = _Robot(bus or _Bus())
    return control, socket, robot


def _request(control, socket, robot, request):
    socket.requests.append(request)
    command = control.process_one(robot)
    return command, socket.replies[-1]


def test_configure_never_energizes_the_servos(monkeypatch):
    host = _torque_host_script(monkeypatch)
    robot = host.SafetyLeKiwi(host.LeKiwiConfig())
    robot.bus = _Bus()
    robot.arm_motors, robot.base_motors = list(ARM), list(BASE)

    robot.configure()

    assert robot.bus.calls[:4] == [
        "write Torque_Enable", "read Torque_Enable", "write Lock", "configure_motors",
    ]
    assert "enable_torque" not in robot.bus.calls


def test_wheel_conversion_covers_validated_translation_without_removing_saturation(monkeypatch):
    host = _torque_host_script(monkeypatch)
    robot = host.SafetyLeKiwi(host.LeKiwiConfig())
    monkeypatch.setattr(host.LeKiwi, '_body_to_wheel_raw', lambda self, *args: args, raising=False)
    assert robot._body_to_wheel_raw(.03, 0., 0.)[-1] == 3000
    robot.maximum_base_linear_speed_m_s = .30
    assert robot._body_to_wheel_raw(.30, 0., 0.) == (.30, 0., 0., .05, .125, 3912)
    assert robot._body_to_wheel_raw(0., .30, 0.)[-1] == 3912
    assert robot._body_to_wheel_raw(.30, 0., 0., max_raw=2000)[-1] == 2000


def test_loaded_arm_joints_get_tuned_position_gain(monkeypatch):
    host = _torque_host_script(monkeypatch)
    robot = host.SafetyLeKiwi(host.LeKiwiConfig())
    robot.bus = _Bus()
    robot.arm_motors = ["arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex"]
    robot.base_motors = list(BASE)

    robot.configure()

    assert robot.bus.writes[("P_Coefficient", "arm_shoulder_pan")] == 16
    assert robot.bus.writes[("P_Coefficient", "arm_shoulder_lift")] == 32
    assert robot.bus.writes[("P_Coefficient", "arm_elbow_flex")] == 64
    assert robot.bus.writes[("I_Coefficient", "arm_shoulder_lift")] == 1
    assert robot.bus.writes[("I_Coefficient", "arm_elbow_flex")] == 1
    assert robot.bus.writes[("I_Coefficient", "arm_wrist_flex")] == 1
    assert robot.bus.writes[("I_Coefficient", "arm_shoulder_pan")] == 0


def test_configure_rejects_unconfirmed_arm_gains(monkeypatch):
    host = _torque_host_script(monkeypatch)
    robot = host.SafetyLeKiwi(host.LeKiwiConfig())
    robot.bus = _Bus()
    robot.arm_motors, robot.base_motors = list(ARM), list(BASE)
    read = robot.bus.sync_read
    monkeypatch.setattr(robot.bus, "sync_read", lambda register, *args, **kwargs:
                        {} if register == "I_Coefficient" else read(register, *args, **kwargs))
    with pytest.raises(RuntimeError, match="I_Coefficient readback differs"):
        robot.configure()
    assert "enable_torque" not in robot.bus.calls


def test_velocity_readback_failure_reports_the_error_without_changing_torque(monkeypatch, caplog):
    host = _torque_host_script(monkeypatch)
    robot = host.SafetyLeKiwi(host.LeKiwiConfig())
    robot.bus,robot.arm_motors,robot.base_motors = _Bus(),list(ARM),list(BASE)
    robot.bus.faults['Maximum_Velocity_Limit'] = RuntimeError('readback failed')
    robot.configure()
    assert 'Base velocity settings readback unavailable: readback failed' in caplog.text
    assert 'enable_torque' not in robot.bus.calls


def test_motor_host_configures_the_robot_without_cameras(monkeypatch):
    host = _torque_host_script(monkeypatch)
    config = host.TorqueHostConfig()
    assert config.robot.cameras  # LeRobot's default, which draccus rebuilds
    configured = []

    def robot(robot_config):
        configured.append(robot_config)
        raise RuntimeError("stop before the serial bus")

    monkeypatch.setattr(host, "SafetyLeKiwi", robot)
    with pytest.raises(RuntimeError, match="serial bus"):
        host.main(config)
    assert configured[0].cameras == {} and configured[0].port == config.robot.port


def test_shutdown_signal_waits_for_the_serial_operation_to_finish(monkeypatch):
    host = _torque_host_script(monkeypatch)

    host._shutdown_signal(None, None)

    assert host._shutdown_requested is True


def test_enable_holds_the_measured_arm_pose_before_confirming_torque(host, tmp_path):
    control, socket, robot = _control(host, tmp_path)

    command, reply = _request(control, socket, robot, {"command": "enable"})

    assert (command, reply) == ("enable", {"ok": True, "torque_enabled": True})
    calls = robot.bus.calls
    assert calls[:4] == ["read Present_Position", "write Goal_Position", "stop_base", "enable_torque"]
    assert calls[4] == "read Torque_Enable"
    assert robot.bus.written == ("Goal_Position", dict.fromkeys(ARM, 12.5))
    # A second enable verifies the hardware rather than trusting the host flag.
    robot.bus.calls.clear()
    assert _request(control, socket, robot, {"command": "enable"})[1]["torque_enabled"] is True
    assert robot.bus.calls == ["read Torque_Enable"]


def test_enable_rolls_torque_back_off_when_a_servo_does_not_confirm(host, tmp_path):
    bus = _Bus()
    bus.enable_skips = ("arm_gripper",)
    control, socket, robot = _control(host, tmp_path, bus)

    command, reply = _request(control, socket, robot, {"command": "enable"})

    assert command is None
    assert reply["ok"] is False and reply["torque_enabled"] is False
    assert "enable transaction failed" in reply["error"]
    assert bus.torque == dict.fromkeys(MOTORS, 0)
    assert "write Torque_Enable" in bus.calls
    assert control.torque_enabled is False


def test_disable_broadcast_reaches_every_motor_past_an_overloaded_gripper(host, tmp_path):
    bus = _Bus()
    bus.faults["disable_torque"] = RuntimeError("gripper overload aborts per-servo write")
    control, socket, robot = _control(host, tmp_path, bus)
    _request(control, socket, robot, {"command": "enable"})

    command, reply = _request(control, socket, robot, {"command": "disable"})

    assert (command, reply) == ("disable", {"ok": True, "torque_enabled": False})
    assert bus.torque == dict.fromkeys(MOTORS, 0)
    assert "disable_torque" not in bus.calls


def test_shutdown_fallback_attempts_every_motor_after_an_overload(host):
    bus = _Bus()
    bus.torque = dict.fromkeys(MOTORS, 1)
    bus.faults["Torque_Enable_write"] = RuntimeError("broadcast failed")
    bus.faults["disable_torque:arm_gripper"] = RuntimeError("overload")

    with pytest.raises(RuntimeError, match="broadcast failed"):
        host.cut_torque_for_shutdown(_Robot(bus))

    assert bus.calls[0] == "write Torque_Enable"
    assert bus.calls[1:] == [f"disable_torque {motor}" for motor in MOTORS]
    assert bus.torque == {motor: int(motor == "arm_gripper") for motor in MOTORS}


def test_disable_reports_torque_on_until_the_bus_confirms_the_cut(host, tmp_path):
    control, socket, robot = _control(host, tmp_path)
    _request(control, socket, robot, {"command": "enable"})
    robot.bus.torque_off_skips = ("arm_gripper",)

    reply = _request(control, socket, robot, {"command": "disable"})[1]
    assert reply["ok"] is False and reply["torque_enabled"] is True

    # The failed cut left only the gripper energized; re-arm cannot succeed
    # using the host's old torque_enabled flag.
    assert _request(control, socket, robot, {"command": "enable"})[1]["ok"] is False

    robot.bus.torque_off_skips = ()
    command, reply = _request(control, socket, robot, {"command": "disable"})
    assert (command, reply) == ("disable", {"ok": True, "torque_enabled": False})


def test_torque_requests_answer_every_malformed_request(host, tmp_path):
    control, socket, robot = _control(host, tmp_path)

    assert control.process_one(robot) is None and socket.replies == []  # nothing pending
    assert _request(control, socket, robot, {"command": "state"}) == (
        "state", {"ok": True, "torque_enabled": False, "trajectory": None},
    )
    undecodable = _request(control, socket, robot, ValueError("Expecting value"))
    assert undecodable == (None, {"ok": False, "error": "invalid request: Expecting value"})
    for request in ({"command": "arm"}, ["enable"], {}):
        command, reply = _request(control, socket, robot, request)
        assert command is None and reply["ok"] is False and reply["torque_enabled"] is False
    assert "enable_torque" not in robot.bus.calls


class _Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class _CommandSocket:
    def __init__(self):
        self.messages = []

    def recv_string(self, flags=0):
        if not self.messages:
            raise _again()
        return self.messages.pop(0)


class _ObservationSocket:
    def __init__(self):
        self.sent = []
        self.full = False

    def send_multipart(self, frames, flags=0):
        if self.full:
            raise _again()
        self.sent.append(frames)


def _loop(host, tmp_path, *, disarm_on_failure=False):
    bound = types.SimpleNamespace(
        zmq_cmd_socket=_CommandSocket(), zmq_observation_socket=_ObservationSocket(),
        torque_socket=_RepSocket(), watchdog_timeout_ms=500,
    )
    clock = _Clock()
    health = types.SimpleNamespace(collect=lambda _robot, enabled: {"torque": enabled})
    robot = _Robot(_Bus())
    loop = host.HostLoop(
        robot, bound, health, disarm_on_failure=disarm_on_failure,
        arm_calibration=load_calibration(tmp_path / "arm_calibration.json"),
        base_limits=load_base_speed_limits(ROOT / "config/nav2_params.yaml"),
        base_scales=(BASE_XY_SCALE, BASE_YAW_SCALE), clock=clock,
    )
    return loop, clock, bound.torque_socket, robot


def _action(**overrides):
    action = {key: 0.0 for key in (
        "arm_shoulder_pan.pos", "arm_shoulder_lift.pos", "arm_elbow_flex.pos",
        "arm_wrist_flex.pos", "arm_wrist_roll.pos", "arm_gripper.pos",
        "x.vel", "y.vel", "theta.vel",
    )}
    action.update(overrides)
    return json.dumps(action)


def test_valid_commands_reach_the_robot_and_refresh_the_watchdog(host, tmp_path):
    assert set(json.loads(_action())) == set(host.STATE_KEYS)
    loop, clock, _socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True

    clock.now += 0.4
    loop.host.zmq_cmd_socket.messages.append(_action(**{"x.vel": 0.01}))
    loop.receive_command()
    clock.now += 0.4
    loop.enforce_watchdog()

    assert robot.actions[0]["x.vel"] == 0.01
    assert loop.watchdog_active is False
    assert "stop_base" not in robot.bus.calls


def test_out_of_envelope_commands_never_reach_motors_or_refresh_watchdog(host, tmp_path):
    loop, clock, _socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True
    last_command = loop.last_cmd_time
    clock.now += 0.4
    for changes in ({"x.vel": 1000000.0}, {"x.vel": 0.03, "y.vel": 0.03},
                    {"arm_shoulder_pan.pos": 180.0}, {"arm_gripper.pos": 101.0}):
        loop.host.zmq_cmd_socket.messages.append(_action(**changes))
        loop.receive_command()
    assert not robot.actions and loop.last_cmd_time == last_command
    assert loop.control.torque_enabled


@pytest.mark.parametrize('stage',[.20,.30])
def test_host_qualification_requires_stage_holds_arm_and_bounds_motion(host,tmp_path,stage):
    from lekiwi_rmf.host_protocol import BASE_TEST_STAGE_KEY
    from lekiwi_rmf.motion_guards import load_base_test_profile
    loop,clock,_socket,robot=_loop(host,tmp_path)
    loop.base_test_profiles={stage:load_base_test_profile(ROOT/'config/nav2_params.yaml',f'{stage:.2f}')}
    loop.control.torque_enabled=True
    robot.get_observation=lambda:json.loads(_action())
    loop.publish_observation()
    held={k:v for k,v in loop.local_observation.items() if k.endswith('.pos')}
    def command(**changes):
        loop.host.zmq_cmd_socket.messages.append(_action(**changes))
        loop.receive_command()
    command(**{'x.vel':stage/BASE_XY_SCALE})
    assert not robot.actions
    command(**{BASE_TEST_STAGE_KEY:stage,'x.vel':stage/BASE_XY_SCALE,'arm_shoulder_pan.pos':1.})
    assert robot.actions[-1]['x.vel']==pytest.approx(stage/BASE_XY_SCALE)
    assert {k:robot.actions[-1][k] for k in held}==held
    assert BASE_TEST_STAGE_KEY not in robot.actions[-1]
    count=len(robot.actions)
    for changes in ({BASE_TEST_STAGE_KEY:.4},
                    {BASE_TEST_STAGE_KEY:stage,'x.vel':(stage+.01)/BASE_XY_SCALE},
                    {BASE_TEST_STAGE_KEY:stage,'x.vel':stage/BASE_XY_SCALE,'theta.vel':1.},
                    {BASE_TEST_STAGE_KEY:stage,'_lekiwi_arm_goal':1.,'_lekiwi_arm_permission':1.}):
        command(**changes)
    assert len(robot.actions)==count
    loop.odometry.pose=(.451,0.,0.)
    command(**{BASE_TEST_STAGE_KEY:stage,'x.vel':stage/BASE_XY_SCALE})
    assert robot.actions[-1]['x.vel']==0.
    clock.now+=.251
    command(**{BASE_TEST_STAGE_KEY:stage,'x.vel':stage/BASE_XY_SCALE})
    assert len(robot.actions)==count+1
    loop.enforce_watchdog()
    assert loop.watchdog_active and 'stop_base' in robot.bus.calls and loop.control.torque_enabled
    loop.publish_observation()
    command(**{'x.vel':.01/BASE_XY_SCALE})
    assert loop._base_test_stage is None and robot.actions[-1]['x.vel']==pytest.approx(.01/BASE_XY_SCALE)
    command(**{'x.vel':stage/BASE_XY_SCALE})
    assert robot.actions[-1]['x.vel']==pytest.approx(.01/BASE_XY_SCALE)


def test_host_rejects_a_trajectory_that_changes_its_calibration(host, tmp_path):
    from test_local_arm_executor import setup_executor

    loop, _clock, socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True
    _executor, _now, _observation, trajectory = setup_executor()
    trajectory["zeros"]["arm_shoulder_pan"] = 1.0
    robot.bus.calls.clear()
    command, reply = _request(loop.control, socket, robot, {
        "command": "trajectory_start", "session": loop.telemetry_session, "trajectory": trajectory,
    })
    assert command is None and not reply["ok"]
    assert "calibration differs" in reply["error"]
    assert not robot.actions and not robot.bus.calls


def test_real_host_local_goal_holds_through_command_silence(host, tmp_path):
    from test_local_arm_executor import setup_executor

    loop, clock, socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True
    _executor, _now, observation, trajectory = setup_executor()
    robot.get_observation = lambda: {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0, **observation}
    command, reply = _request(loop.control, socket, robot, {
        "command": "trajectory_start", "session": loop.telemetry_session,
        "trajectory": trajectory,
    })
    assert command == "trajectory_start" and reply["ok"]
    for _ in range(20):
        clock.now += 0.033
        loop.host.zmq_cmd_socket.messages.append(_action(**{
            "_lekiwi_arm_goal": trajectory["id"], "_lekiwi_arm_permission": 1,
            "x.vel": 0.1, "arm_shoulder_pan.pos": 100.0,
        }))
        loop.step()
        observation.update({key: value for key, value in robot.actions[-1].items() if key.endswith(".pos")})
    elapsed = loop.arm_executor.status["elapsed"]
    clock.now += 12.0
    loop.step()
    assert loop.arm_executor.status["state"] == "paused"
    assert loop.arm_executor.status["elapsed"] == elapsed
    assert robot.actions[-1]["x.vel"] == 0
    assert robot.actions[-1]["arm_shoulder_pan.pos"] != 100.0
    clock.now += 0.033
    loop.host.zmq_cmd_socket.messages.append(_action(**{
        "_lekiwi_arm_goal": trajectory["id"], "_lekiwi_arm_permission": 1,
    }))
    loop.step()
    assert loop.arm_executor.status["state"] == "running"
    assert loop.arm_executor.status["elapsed"] == elapsed
    robot.get_observation = lambda: pytest.fail("state query must not read the motor bus")
    command, reply = _request(loop.control, socket, robot, {"command": "state"})
    assert command == "state" and reply["ok"]
    assert reply["trajectory"] == loop.arm_executor.status


def test_host_keeps_command_loop_fast_and_limits_observation_bandwidth(host, tmp_path):
    loop, clock, _socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True
    commands = loop.host.zmq_cmd_socket.messages
    commands.extend((
        _action(**{"x.vel": 0.01}),
        _action(**{"x.vel": 0.02}),
        _action(**{"x.vel": 0.03}),
    ))

    loop.step()
    loop.step()
    assert len(loop.host.zmq_observation_socket.sent) == 1
    clock.now += host.OBSERVATION_PERIOD_S
    loop.step()

    assert [action["x.vel"] for action in robot.actions] == [0.01, 0.02, 0.03]
    assert len(loop.host.zmq_observation_socket.sent) == 2


def test_host_reports_the_stage_causing_a_slow_observation(host, monkeypatch, tmp_path, caplog):
    loop, _clock, _socket, _robot = _loop(host, tmp_path)
    timings = iter((100.0, 100.01, 100.02, 100.03, 100.34))
    monkeypatch.setattr(host.time, "perf_counter", lambda: next(timings))

    with caplog.at_level(logging.WARNING):
        loop.step()

    assert "observation=0.310s" in caplog.text


def test_a_repeated_malformed_command_is_logged_once_per_distinct_error(host, tmp_path, caplog):
    loop, _clock, _socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True
    commands = loop.host.zmq_cmd_socket.messages

    def rejections():
        return [record for record in caplog.records if "Motion command rejected" in record.getMessage()]

    with caplog.at_level(logging.ERROR):
        for message in ("not json", "not json", "not json", '{"x.vel": 1}', "not json", _action(), "not json"):
            commands.append(message)
            loop.receive_command()

    assert robot.actions and len(robot.actions) == 1
    # not json, then the incomplete action, then not json again; after the valid
    # command the same error is news again.
    assert len(rejections()) == 4


def test_command_silence_holds_the_robot_once_with_torque_left_on(host, tmp_path):
    loop, clock, socket, robot = _loop(host, tmp_path)
    socket.requests.append({"command": "enable"})
    loop.handle_control_request()
    robot.bus.calls.clear()

    clock.now += 0.4
    loop.enforce_watchdog()
    assert robot.bus.calls == []  # still inside the grace period after arming

    clock.now += 0.2
    loop.enforce_watchdog()
    clock.now += 1.0
    loop.enforce_watchdog()

    assert robot.bus.calls == ["read Present_Position", "write Goal_Position", "stop_base"]
    assert robot.bus.torque == dict.fromkeys(MOTORS, 1)
    assert loop.watchdog_active is True


def test_command_silence_cuts_torque_only_in_strict_mode(host, tmp_path):
    loop, clock, socket, robot = _loop(host, tmp_path, disarm_on_failure=True)
    socket.requests.append({"command": "enable"})
    loop.handle_control_request()

    clock.now += 0.6
    loop.enforce_watchdog()

    assert robot.bus.torque == dict.fromkeys(MOTORS, 0)
    assert loop.control.torque_enabled is False


def test_an_unconfirmed_watchdog_action_is_retried_at_a_bounded_rate(host, tmp_path):
    loop, clock, _socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True
    robot.bus.faults["Present_Position"] = OSError("bus timeout")

    clock.now += 0.6
    loop.enforce_watchdog()
    clock.now += 0.1
    loop.enforce_watchdog()  # too soon to retry
    attempts = robot.bus.calls.count("read Present_Position")
    assert attempts == 1 and loop.watchdog_active is False

    del robot.bus.faults["Present_Position"]
    clock.now += 0.2
    loop.enforce_watchdog()
    assert robot.bus.calls.count("read Present_Position") == 2
    assert loop.watchdog_active is True


def test_a_disarm_request_stops_the_watchdog_from_acting_again(host, tmp_path):
    loop, clock, socket, robot = _loop(host, tmp_path)
    socket.requests.append({"command": "disable"})
    loop.handle_control_request()
    robot.bus.calls.clear()

    clock.now += 5
    loop.enforce_watchdog()

    assert robot.bus.calls == []


def test_disarmed_host_never_sends_goals_that_reenable_servo_torque(host, tmp_path):
    loop, clock, _socket, robot = _loop(host, tmp_path)
    clock.now += 1
    loop.enforce_watchdog()
    loop.host.zmq_cmd_socket.messages.append(_action())
    loop.receive_command()

    assert robot.bus.calls == []
    assert robot.actions == []
    assert robot.bus.torque == dict.fromkeys(MOTORS, 0)


def test_telemetry_reports_torque_state_health_and_a_gapless_sequence(host, tmp_path):
    loop, _clock, socket, _robot = _loop(host, tmp_path)
    observations = loop.host.zmq_observation_socket

    loop.publish_observation()
    observations.full = True
    loop.publish_observation()  # no client: dropped, but the sequence still advances
    observations.full = False
    socket.requests.append({"command": "enable"})
    loop.handle_control_request()
    loop.publish_observation()

    assert all(len(frames) == 1 for frames in observations.sent)
    first, second = (json.loads(frames[0]) for frames in observations.sent)
    assert first["arm_gripper.pos"] == 1.0 and first["_cams"] == []
    assert first["_lekiwi_motor_health"] == {"torque": False}
    assert second["_lekiwi_motor_health"] == {"torque": True}
    sequence = next(key for key in first if key.endswith("sequence"))
    assert (first[sequence], second[sequence]) == (0, 2)


def test_motor_action_limits_use_calibrated_joint_and_base_units(host):
    zeros = dict.fromkeys(JOINT_LIMITS, 0.0)
    directions = dict.fromkeys(JOINT_LIMITS, 1.0)
    zeros["arm_shoulder_pan"], directions["arm_shoulder_pan"] = 2.0, -1.0
    action = {f"{name}.pos": value for name, value in action_positions(
        tuple(JOINT_LIMITS), [0.0] * len(JOINT_LIMITS), zeros, directions,
    ).items()}
    action.update({"x.vel": 0.03 / 0.8, "y.vel": 0.0, "theta.vel": math.degrees(0.06 / 0.976)})
    args = ((0.03, 0.06), (0.8, 0.976), (zeros, directions))
    host.validate_motion_action(action, *args)
    for changes in (
        {"x.vel": 1000000.0}, {"y.vel": 0.01}, {"theta.vel": math.degrees(0.061 / 0.976)},
        {"arm_shoulder_pan.pos": 0.0}, {"arm_gripper.pos": -1.0}, {"arm_gripper.pos": 101.0},
        {"x.vel": float("nan")},
    ):
        with pytest.raises(ValueError):
            host.validate_motion_action({**action, **changes}, *args)


def test_motor_action_retains_an_out_of_bounds_measured_hold_without_extending_it(host):
    zeros, directions = dict.fromkeys(JOINT_LIMITS, 0.0), dict.fromkeys(JOINT_LIMITS, 1.0)
    action = {f"{name}.pos": 0.0 for name in JOINT_LIMITS}
    action.update({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
    action["arm_shoulder_lift.pos"] = math.degrees(JOINT_LIMITS["arm_shoulder_lift"][0] - 0.02)
    args = ((0.03, 0.06), (1.0, 1.0), (zeros, directions))
    with pytest.raises(ValueError, match="position limits"):
        host.validate_motion_action(action, *args)
    host.validate_motion_action(action, *args, held_positions=action)
    with pytest.raises(ValueError, match="position limits"):
        host.validate_motion_action({**action, "arm_shoulder_lift.pos": action["arm_shoulder_lift.pos"] - 1.0},
                               *args, held_positions=action)


def _collector(host, bus):
    robot = _Robot(bus)
    return host.MotorHealthCollector(robot), robot


def _levels(snapshot):
    return {name: status["level"] for name, status in snapshot["statuses"].items()}


def test_motor_health_reports_every_servo_with_its_limits(host, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(host.time, "monotonic", lambda: now[0])
    bus = _Bus()
    bus.registers["Present_Temperature"]["arm_gripper"] = 70
    collector, robot = _collector(host, bus)

    for index, _read in enumerate(host.HEALTH_READS):
        snapshot = collector.collect(robot, torque_enabled=False)
        if index + 1 < len(host.HEALTH_READS):
            now[0] += host.HEALTH_READ_PERIOD_S + 0.001

    levels = _levels(snapshot)
    assert levels["motor_bus"] == 0
    assert levels["servo/arm_gripper"] == 1  # at the programmed temperature limit
    assert levels["servo/arm_shoulder_pan"] == 0
    values = snapshot["statuses"]["servo/base_left_wheel"]["values"]
    assert values["present_voltage_v"] == "12.0" and values["maximum_temperature_c"] == "70"
    assert values["present_position"] == "12.5" and values["goal_position"] == "13.5"
    # Bounded rate: an immediate second call reuses the last read.
    reads = len(bus.calls)
    assert collector.collect(robot, torque_enabled=False) is snapshot
    assert len(bus.calls) == reads
    now[0] += host.HEALTH_READ_PERIOD_S / 2
    assert collector.collect(robot, torque_enabled=False) is snapshot
    assert len(bus.calls) == reads
    now[0] += host.HEALTH_READ_PERIOD_S / 2 + 0.001
    assert collector.collect(robot, torque_enabled=False) is snapshot
    assert len(bus.calls) == reads + 1


@pytest.mark.parametrize("fault", ["torque mismatch", "status", "incomplete", "bus error"])
def test_motor_health_fails_closed(host, monkeypatch, fault):
    bus = _Bus()
    if fault == "torque mismatch":
        bus.torque["arm_gripper"] = 1
    elif fault == "status":
        bus.registers["Status"]["base_left_wheel"] = 32
    elif fault == "incomplete":
        del bus.registers["Present_Load"]["arm_gripper"]
    else:
        bus.faults["Present_Current"] = OSError("no status packet")
    collector, robot = _collector(host, bus)

    now = [100.0]
    monkeypatch.setattr(host.time, "monotonic", lambda: now[0])
    for _ in range(2 * len(host.HEALTH_READS)):
        snapshot = collector.collect(robot, torque_enabled=False)
        now[0] += host.HEALTH_READ_PERIOD_S + 0.001

    assert _levels(snapshot)["motor_bus"] == 2
    assert all(level == 2 for level in _levels(snapshot).values())
