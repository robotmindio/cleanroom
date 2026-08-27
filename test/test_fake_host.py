"""Wire-level tests for the no-hardware LeKiwi host fixture."""

from __future__ import annotations

import json
import time

import pytest

zmq = pytest.importorskip("zmq", reason="fake host requires pyzmq (provided by the LeRobot test environment)")

from lekiwi_rmf.fake_host import FakeLeKiwiHost, ObservationFault
from lekiwi_rmf.odometry import (
    TELEMETRY_MONOTONIC_NS_KEY,
    TELEMETRY_SEQUENCE_KEY,
    TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)


@pytest.fixture
def host():
    with FakeLeKiwiHost() as fake:
        yield fake


@pytest.fixture
def context():
    ctx = zmq.Context()
    yield ctx
    ctx.term()


def _pull(context, endpoint):
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(endpoint)
    # PUSH intentionally drops rather than queues for an unconnected peer.
    # Let the TCP handshake complete before the test's first one-shot sample.
    time.sleep(0.05)
    return socket


def _wait_for(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    raise AssertionError("timed out waiting for fake host")


def _receive(socket):
    return _wait_for(lambda: socket.recv_multipart(zmq.NOBLOCK) if socket.poll(0) else None)


def test_fake_host_speaks_action_and_versioned_observation_protocol(host, context):
    command = context.socket(zmq.PUSH)
    command.setsockopt(zmq.LINGER, 0)
    command.connect(host.endpoints.command)
    observation = _pull(context, host.endpoints.observation)
    host.set_camera_frames({"front": b"jpeg-front"})
    host.set_state(**{"x.vel": 0.12, "arm_shoulder_pan.pos": 1.5})

    command.send_json({"y.vel": -0.2, "arm_shoulder_pan.pos": 2.0})
    _wait_for(lambda: host.process_actions() == 1)
    assert host.actions == [{"y.vel": -0.2, "arm_shoulder_pan.pos": 2.0}]

    host.publish_observation()
    frames = _receive(observation)
    payload = json.loads(frames[0])
    assert payload["_cams"] == ["front"]
    assert frames[1] == b"jpeg-front"
    assert payload["y.vel"] == -0.2
    assert payload["arm_shoulder_pan.pos"] == 2.0
    assert payload[TELEMETRY_SEQUENCE_KEY] == 0
    assert payload[TELEMETRY_SESSION_KEY] == host.session
    assert payload[TELEMETRY_MONOTONIC_NS_KEY] > 0
    assert payload[TELEMETRY_TORQUE_ENABLED_KEY] is False

    command.close()
    observation.close()


def test_fake_host_controls_drop_duplicate_stale_malformed_and_session_restart(host, context):
    observation = _pull(context, host.endpoints.observation)

    host.publish_observation()
    first = _receive(observation)
    first_payload = json.loads(first[0])

    host.queue_observation_fault(ObservationFault.DUPLICATE)
    host.publish_observation()
    assert _receive(observation) == first

    host.queue_observation_fault("stale")
    host.publish_observation()
    stale_payload = json.loads(_receive(observation)[0])
    assert stale_payload[TELEMETRY_SEQUENCE_KEY] == first_payload[TELEMETRY_SEQUENCE_KEY] + 1
    assert stale_payload[TELEMETRY_MONOTONIC_NS_KEY] == first_payload[TELEMETRY_MONOTONIC_NS_KEY]

    host.queue_observation_fault("malformed")
    host.publish_observation()
    assert _receive(observation) == [b"{malformed lekiwi observation"]

    host.queue_observation_fault("drop")
    assert host.publish_observation() is None
    assert not observation.poll(50)

    first_session = host.session
    host.torque_enabled = True
    second_session = host.restart_session()
    assert second_session != first_session
    assert host.torque_enabled is False
    host.publish_observation()
    restarted = json.loads(_receive(observation)[0])
    assert restarted[TELEMETRY_SESSION_KEY] == second_session
    assert restarted[TELEMETRY_SEQUENCE_KEY] == 0

    observation.close()


def test_fake_host_reports_torque_failures_and_recovers(host, context):
    torque = context.socket(zmq.REQ)
    torque.setsockopt(zmq.LINGER, 0)
    torque.connect(host.endpoints.torque)

    host.fail_next_torque("enable", "servo 3 did not acknowledge")
    torque.send_json({"command": "enable"})
    _wait_for(host.process_torque)
    assert torque.recv_json() == {
        "ok": False,
        "error": "servo 3 did not acknowledge",
        "torque_enabled": False,
    }

    torque.send_json({"command": "enable"})
    _wait_for(host.process_torque)
    assert torque.recv_json() == {"ok": True, "torque_enabled": True}
    assert host.torque_enabled

    torque.send_json({"command": "disable"})
    _wait_for(host.process_torque)
    assert torque.recv_json() == {"ok": True, "torque_enabled": False}
    assert host.torque_requests == ["enable", "enable", "disable"]
    torque.close()


def test_malformed_torque_request_does_not_kill_fake_host(host, context):
    torque = context.socket(zmq.REQ)
    torque.setsockopt(zmq.LINGER, 0)
    torque.connect(host.endpoints.torque)
    torque.send(b"{not-json")
    _wait_for(host.process_torque)
    response = torque.recv_json()
    assert response["ok"] is False
    assert response["torque_enabled"] is False

    torque.send_json({"command": "state"})
    _wait_for(host.process_torque)
    assert torque.recv_json() == {"ok": True, "torque_enabled": False}
    torque.close()


def test_fake_host_rejects_invalid_fault_and_never_binds_non_loopback_by_default(host):
    assert host.endpoints.command.startswith("tcp://127.0.0.1:")
    with pytest.raises(ValueError, match="unknown observation fault"):
        host.queue_observation_fault("replay")
    with pytest.raises(ValueError, match="enable or disable"):
        host.fail_next_torque("state")
