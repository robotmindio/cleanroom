"""Unit checks for the physical-torque request/reply contract."""

import pathlib
import types

import pytest

from lekiwi_rmf.torque_control import (
    TorqueControlClient, TorqueControlError, enable_with_rollback,
    run_all_safety_steps, torque_readback_matches,
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


def test_torque_client_allows_an_explicit_insecure_test_fixture():
    client = TorqueControlClient(
        "192.0.2.10", zmq_module=_Zmq({}),
        allow_insecure_test_connection=True,
    )
    assert client.host == "192.0.2.10"


def test_driver_and_host_wire_disarm_to_the_serial_bus_owner():
    root = pathlib.Path(__file__).parents[1]
    driver = (root / "lekiwi_rmf" / "driver.py").read_text()
    host = (root / "scripts" / "torque-host.py").read_text()

    assert "self.set_servo_torque(False)" in driver
    assert "self.set_servo_torque(True)" in driver
    assert "robot.bus.disable_torque(num_retry=TORQUE_RETRIES)" in host
    assert "robot.bus.enable_torque(num_retry=TORQUE_RETRIES)" in host
    assert "self._hold_present_arm_position(robot)" in host
    assert "self.latch.save(False)" in host
    assert "except FileNotFoundError:" in host
    assert "return False" in host
    assert "control._disable(robot)" in host
    assert "cutting all servo torque" in host


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


def test_control_listener_requires_an_explicit_non_wildcard_interface():
    assert validated_bind_address("127.0.0.1") == "127.0.0.1"
    assert validated_bind_address("192.0.2.20") == "192.0.2.20"
    for address in ("0.0.0.0", "*", "robot.local", "224.0.0.1"):
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
