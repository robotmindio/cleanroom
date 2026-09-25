"""Unit checks for the physical-torque request/reply contract."""

import dataclasses
import importlib.util
import json
import logging
import pathlib
import sys
import types

import pytest

from lekiwi_rmf.torque_control import (
    TorqueControlClient, TorqueControlError, enable_with_rollback,
    react_to_command_silence, run_all_safety_steps, torque_readback_matches,
    validate_action_payload, validated_bind_address,
)


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


def test_safety_steps_do_not_short_circuit_after_persistence_failure():
    called = []

    def fail_persistence():
        called.append("persist")
        raise OSError("read-only filesystem")

    failures = run_all_safety_steps((
        ("persist", fail_persistence),
        ("stop", lambda: called.append("stop")),
        ("disable", lambda: called.append("disable")),
        ("verify", lambda: called.append("verify")),
    ))

    assert called == ["persist", "stop", "disable", "verify"]
    assert len(failures) == 1
    assert failures[0][0] == "persist"


def test_enable_persistence_failure_rolls_physical_torque_back_off():
    called = []

    def persist_enabled():
        called.append("persist enabled")
        raise OSError("disk full")

    with pytest.raises(RuntimeError, match="enable transaction failed"):
        enable_with_rollback(
            (
                ("enable", lambda: called.append("enable")),
                ("verify enabled", lambda: called.append("verify enabled")),
                ("persist enabled", persist_enabled),
            ),
            (
                ("disable", lambda: called.append("disable")),
                ("verify disabled", lambda: called.append("verify disabled")),
                ("persist disabled", lambda: called.append("persist disabled")),
            ),
        )

    assert called == [
        "enable", "verify enabled", "persist enabled",
        "disable", "verify disabled", "persist disabled",
    ]


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
# scripts/torque-host.py against a fake motor bus. LeRobot, draccus, OpenCV and
# pyzmq are replaced by minimal stand-ins so the host's own logic runs anywhere.

ARM = ("arm_shoulder_pan", "arm_gripper")
BASE = ("base_left_wheel",)
MOTORS = ARM + BASE


class _Again(Exception):
    pass


def _host_module(monkeypatch):
    fake_zmq = types.SimpleNamespace(
        Again=_Again, NOBLOCK=1, REP=4, PULL=7, PUSH=8, LINGER=17, CONFLATE=54, SNDHWM=23,
        HEARTBEAT_IVL=75, HEARTBEAT_TIMEOUT=77,
    )
    fake_cv2 = types.SimpleNamespace(IMWRITE_JPEG_QUALITY=1, imencode=lambda *_args: (True, b"jpeg"))
    fake_draccus = types.SimpleNamespace(wrap=lambda: (lambda function: function))

    @dataclasses.dataclass
    class LeKiwiConfig:
        port: str = ""

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
        "zmq": fake_zmq, "cv2": fake_cv2, "draccus": fake_draccus,
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
        "torque_host", pathlib.Path(__file__).parents[1] / "scripts" / "torque-host.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Bus:
    """Feetech bus stand-in: torque registers that follow enable/disable, plus scripted faults."""

    def __init__(self):
        self.motors = {motor: object() for motor in MOTORS}
        self.torque = dict.fromkeys(MOTORS, 0)
        self.registers = {
            "Present_Position": dict.fromkeys(MOTORS, 12.5),
            "Present_Load": dict.fromkeys(MOTORS, 100),
            "Present_Voltage": dict.fromkeys(MOTORS, 120),
            "Present_Temperature": dict.fromkeys(MOTORS, 35),
            "Status": dict.fromkeys(MOTORS, 0),
            "Present_Current": dict.fromkeys(MOTORS, 10),
            "Max_Temperature_Limit": dict.fromkeys(MOTORS, 70),
            "Min_Voltage_Limit": dict.fromkeys(MOTORS, 45),
            "Max_Voltage_Limit": dict.fromkeys(MOTORS, 140),
        }
        self.calls = []
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

    def configure_motors(self):
        self.calls.append("configure_motors")


class _Robot:
    def __init__(self, bus):
        self.bus = bus
        self.arm_motors = list(ARM)
        self.base_motors = list(BASE)
        self.cameras = {}
        self.actions = []

    def stop_base(self):
        self.bus.calls.append("stop_base")

    def send_action(self, action):
        self.actions.append(action)

    def get_observation(self):
        return {"arm_gripper.pos": 1.0}


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
            raise _Again()
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
    context = types.SimpleNamespace(socket=lambda _kind: socket)
    security = types.SimpleNamespace(configure_socket=lambda _socket: None)
    latch = host.TorqueLatch(str(tmp_path / "lekiwi" / "servo_torque_state"))
    control = host.TorqueControlServer(
        context, host.TorqueSafetyConfig(bind_address="127.0.0.1"), latch, security,
    )
    robot = _Robot(bus or _Bus())
    return control, socket, robot, latch


def _request(control, socket, robot, request):
    socket.requests.append(request)
    command = control.process_one(robot)
    return command, socket.replies[-1]


def test_torque_latch_is_written_atomically_and_privately(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    latch = host.TorqueLatch(str(tmp_path / "state" / "servo_torque_state"))

    latch.save(True)
    assert latch.path.read_text() == "enabled\n"
    assert latch.path.stat().st_mode & 0o777 == 0o600
    latch.save(False)
    assert latch.path.read_text() == "disabled\n"
    assert [path.name for path in latch.path.parent.iterdir()] == ["servo_torque_state"]


def test_configure_never_energizes_the_servos(monkeypatch):
    host = _host_module(monkeypatch)
    robot = host.SafetyLeKiwi(host.LeKiwiConfig())
    robot.bus = _Bus()
    robot.arm_motors, robot.base_motors = list(ARM), list(BASE)

    robot.configure()

    assert robot.bus.calls[:4] == [
        "write Torque_Enable", "read Torque_Enable", "write Lock", "configure_motors",
    ]
    assert "enable_torque" not in robot.bus.calls


def test_enable_holds_the_measured_arm_pose_before_confirming_torque(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    control, socket, robot, latch = _control(host, tmp_path)

    command, reply = _request(control, socket, robot, {"command": "enable"})

    assert (command, reply) == ("enable", {"ok": True, "torque_enabled": True})
    calls = robot.bus.calls
    assert calls[:4] == ["read Present_Position", "write Goal_Position", "stop_base", "enable_torque"]
    assert calls[4] == "read Torque_Enable"
    assert robot.bus.written == ("Goal_Position", dict.fromkeys(ARM, 12.5))
    assert latch.path.read_text() == "enabled\n"
    # A second enable verifies the hardware rather than trusting the host flag.
    robot.bus.calls.clear()
    assert _request(control, socket, robot, {"command": "enable"})[1]["torque_enabled"] is True
    assert robot.bus.calls == ["read Torque_Enable"]


def test_enable_rolls_torque_back_off_when_a_servo_does_not_confirm(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    bus = _Bus()
    bus.enable_skips = ("arm_gripper",)
    control, socket, robot, latch = _control(host, tmp_path, bus)

    command, reply = _request(control, socket, robot, {"command": "enable"})

    assert command is None
    assert reply["ok"] is False and reply["torque_enabled"] is False
    assert "enable transaction failed" in reply["error"]
    assert bus.torque == dict.fromkeys(MOTORS, 0)
    assert "write Torque_Enable" in bus.calls
    assert latch.path.read_text() == "disabled\n"
    assert control.torque_enabled is False


def test_disable_cuts_torque_even_when_the_latch_cannot_be_written(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    control, socket, robot, latch = _control(host, tmp_path)
    _request(control, socket, robot, {"command": "enable"})
    latch.path.unlink()
    latch.path.mkdir()  # os.replace onto a directory fails like a read-only filesystem

    command, reply = _request(control, socket, robot, {"command": "disable"})

    assert command is None
    assert reply["ok"] is False and "persist disabled latch" in reply["error"]
    assert robot.bus.torque == dict.fromkeys(MOTORS, 0)
    assert reply["torque_enabled"] is False and control.torque_enabled is False


def test_disable_broadcast_reaches_every_motor_past_an_overloaded_gripper(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    bus = _Bus()
    bus.faults["disable_torque"] = RuntimeError("gripper overload aborts per-servo write")
    control, socket, robot, _latch = _control(host, tmp_path, bus)
    _request(control, socket, robot, {"command": "enable"})

    command, reply = _request(control, socket, robot, {"command": "disable"})

    assert (command, reply) == ("disable", {"ok": True, "torque_enabled": False})
    assert bus.torque == dict.fromkeys(MOTORS, 0)
    assert "disable_torque" not in bus.calls


def test_shutdown_fallback_attempts_every_motor_after_an_overload(monkeypatch):
    host = _host_module(monkeypatch)
    bus = _Bus()
    bus.torque = dict.fromkeys(MOTORS, 1)
    bus.faults["Torque_Enable_write"] = RuntimeError("broadcast failed")
    bus.faults["disable_torque:arm_gripper"] = RuntimeError("overload")

    with pytest.raises(RuntimeError, match="broadcast failed"):
        host.cut_torque_for_shutdown(_Robot(bus))

    assert bus.calls[0] == "write Torque_Enable"
    assert bus.calls[1:] == [f"disable_torque {motor}" for motor in MOTORS]
    assert bus.torque == {motor: int(motor == "arm_gripper") for motor in MOTORS}


def test_disable_reports_torque_on_until_the_bus_confirms_the_cut(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    control, socket, robot, latch = _control(host, tmp_path)
    _request(control, socket, robot, {"command": "enable"})
    robot.bus.torque_off_skips = ("arm_gripper",)

    reply = _request(control, socket, robot, {"command": "disable"})[1]
    assert reply["ok"] is False and reply["torque_enabled"] is True
    assert latch.path.read_text() == "enabled\n"

    # The failed cut left only the gripper energized; re-arm cannot succeed
    # using the host's old torque_enabled flag.
    assert _request(control, socket, robot, {"command": "enable"})[1]["ok"] is False

    robot.bus.torque_off_skips = ()
    command, reply = _request(control, socket, robot, {"command": "disable"})
    assert (command, reply) == ("disable", {"ok": True, "torque_enabled": False})


def test_torque_requests_answer_every_malformed_request(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    control, socket, robot, _latch = _control(host, tmp_path)

    assert control.process_one(robot) is None and socket.replies == []  # nothing pending
    assert _request(control, socket, robot, {"command": "state"}) == (
        "state", {"ok": True, "torque_enabled": False},
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
            raise _Again()
        return self.messages.pop(0)


class _ObservationSocket:
    def __init__(self):
        self.sent = []
        self.full = False

    def send_multipart(self, frames, flags=0):
        if self.full:
            raise _Again()
        self.sent.append(frames)


def _loop(host, tmp_path, *, disarm_on_failure=False):
    control, socket, robot, _latch = _control(host, tmp_path)
    bound = types.SimpleNamespace(
        zmq_cmd_socket=_CommandSocket(), zmq_observation_socket=_ObservationSocket(),
        watchdog_timeout_ms=500, max_loop_freq_hz=30,
    )
    clock = _Clock()
    health = types.SimpleNamespace(collect=lambda _robot, enabled: {"torque": enabled})
    loop = host.HostLoop(robot, bound, control, health, disarm_on_failure, clock=clock)
    return loop, clock, socket, robot


def _action(**overrides):
    action = {key: 0.0 for key in (
        "arm_shoulder_pan.pos", "arm_shoulder_lift.pos", "arm_elbow_flex.pos",
        "arm_wrist_flex.pos", "arm_wrist_roll.pos", "arm_gripper.pos",
        "x.vel", "y.vel", "theta.vel",
    )}
    action.update(overrides)
    return json.dumps(action)


def test_valid_commands_reach_the_robot_and_refresh_the_watchdog(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    assert set(json.loads(_action())) == set(host.ACTION_KEYS)
    loop, clock, _socket, robot = _loop(host, tmp_path)
    loop.control.torque_enabled = True

    clock.now += 0.4
    loop.host.zmq_cmd_socket.messages.append(_action(**{"x.vel": 0.1}))
    loop.receive_command()
    clock.now += 0.4
    loop.enforce_watchdog()

    assert robot.actions[0]["x.vel"] == 0.1
    assert loop.watchdog_active is False
    assert "stop_base" not in robot.bus.calls


def test_a_repeated_malformed_command_is_logged_once_per_distinct_error(monkeypatch, tmp_path, caplog):
    host = _host_module(monkeypatch)
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


def test_command_silence_holds_the_robot_once_with_torque_left_on(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
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


def test_command_silence_cuts_torque_only_in_strict_mode(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    loop, clock, socket, robot = _loop(host, tmp_path, disarm_on_failure=True)
    socket.requests.append({"command": "enable"})
    loop.handle_control_request()

    clock.now += 0.6
    loop.enforce_watchdog()

    assert robot.bus.torque == dict.fromkeys(MOTORS, 0)
    assert loop.control.torque_enabled is False


def test_an_unconfirmed_watchdog_action_is_retried_at_a_bounded_rate(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
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


def test_a_disarm_request_stops_the_watchdog_from_acting_again(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    loop, clock, socket, robot = _loop(host, tmp_path)
    socket.requests.append({"command": "disable"})
    loop.handle_control_request()
    robot.bus.calls.clear()

    clock.now += 5
    loop.enforce_watchdog()

    assert robot.bus.calls == []


def test_disarmed_host_never_sends_goals_that_reenable_servo_torque(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    loop, clock, _socket, robot = _loop(host, tmp_path)
    clock.now += 1
    loop.enforce_watchdog()
    loop.host.zmq_cmd_socket.messages.append(_action())
    loop.receive_command()

    assert robot.bus.calls == []
    assert robot.actions == []
    assert robot.bus.torque == dict.fromkeys(MOTORS, 0)


def test_telemetry_reports_torque_state_health_and_a_gapless_sequence(monkeypatch, tmp_path):
    host = _host_module(monkeypatch)
    loop, _clock, socket, _robot = _loop(host, tmp_path)
    observations = loop.host.zmq_observation_socket

    loop.publish_observation()
    observations.full = True
    loop.publish_observation()  # no client: dropped, but the sequence still advances
    observations.full = False
    socket.requests.append({"command": "enable"})
    loop.handle_control_request()
    loop.publish_observation()

    first, second = (json.loads(frames[0]) for frames in observations.sent)
    assert first["arm_gripper.pos"] == 1.0 and first["_cams"] == []
    assert first["_lekiwi_motor_health"] == {"torque": False}
    assert second["_lekiwi_motor_health"] == {"torque": True}
    sequence = next(key for key in first if key.endswith("sequence"))
    assert (first[sequence], second[sequence]) == (0, 2)


def _collector(host, bus):
    robot = _Robot(bus)
    return host.MotorHealthCollector(robot), robot


def _levels(snapshot):
    return {name: status["level"] for name, status in snapshot["statuses"].items()}


def test_motor_health_reports_every_servo_with_its_limits(monkeypatch):
    host = _host_module(monkeypatch)
    bus = _Bus()
    bus.registers["Present_Temperature"]["arm_gripper"] = 70
    collector, robot = _collector(host, bus)

    snapshot = collector.collect(robot, torque_enabled=False)

    levels = _levels(snapshot)
    assert levels["motor_bus"] == 0
    assert levels["servo/arm_gripper"] == 1  # at the programmed temperature limit
    assert levels["servo/arm_shoulder_pan"] == 0
    values = snapshot["statuses"]["servo/base_left_wheel"]["values"]
    assert values["present_voltage_v"] == "12.0" and values["maximum_temperature_c"] == "70"
    # Bounded rate: an immediate second call reuses the last read.
    reads = len(bus.calls)
    assert collector.collect(robot, torque_enabled=False) is snapshot
    assert len(bus.calls) == reads


@pytest.mark.parametrize("fault", ["torque mismatch", "status", "incomplete", "bus error"])
def test_motor_health_fails_closed(monkeypatch, fault):
    host = _host_module(monkeypatch)
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

    snapshot = collector.collect(robot, torque_enabled=False)

    assert _levels(snapshot)["motor_bus"] == 2
    assert all(level == 2 for level in _levels(snapshot).values())
