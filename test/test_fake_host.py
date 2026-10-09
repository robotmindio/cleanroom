"""Wire-level tests for the no-hardware LeKiwi host fixture."""

from __future__ import annotations

import json
import time

import pytest

zmq = pytest.importorskip("zmq", reason="fake host requires pyzmq (provided by the LeRobot test environment)")

from lekiwi_rmf.fake_host import FakeLeKiwiHost, ObservationFault
from lekiwi_rmf.host_protocol import (
    HOST_ODOMETRY_KEY,
    STATE_KEYS,
    TELEMETRY_MONOTONIC_NS_KEY,
    TELEMETRY_SEQUENCE_KEY,
    TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)
from lekiwi_rmf.motor_health import ERROR, fault_snapshot


@pytest.fixture
def host():
    with FakeLeKiwiHost() as fake:
        yield fake


@pytest.fixture
def context():
    ctx = zmq.Context()
    yield ctx
    ctx.destroy(linger=0)


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


def _publish(host):
    # Receipt does not synchronously free the lossy PUSH queue's capacity.
    _wait_for(lambda: host._observations.socket.poll(0, zmq.POLLOUT))
    return host.publish_observation()


def _torque(context, host, request):
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(host.endpoints.torque)
    try:
        if isinstance(request, bytes):
            socket.send(request)
        else:
            socket.send_json(request)

        def reply():
            host.step()
            return socket.recv_json() if socket.poll(0) else None

        return _wait_for(reply)
    finally:
        socket.close()


def _latest(socket):
    frames = None
    while socket.poll(0):
        frames = socket.recv_multipart()
    return frames


def _action(**changes):
    action = dict.fromkeys(STATE_KEYS, 0.0)
    action.update(changes)
    return action


def test_fake_host_speaks_action_and_versioned_observation_protocol(host, context):
    command = context.socket(zmq.PUSH)
    command.setsockopt(zmq.LINGER, 0)
    command.connect(host.endpoints.command)
    observation = _pull(context, host.endpoints.observation)
    host.set_state(**{"x.vel": 0.12, "arm_shoulder_pan.pos": 1.5})

    _publish(host)
    payload = json.loads(_receive(observation)[0])
    assert payload[TELEMETRY_SEQUENCE_KEY] == 0
    assert payload[TELEMETRY_SESSION_KEY] == host.session
    assert payload[TELEMETRY_TORQUE_ENABLED_KEY] is False
    assert payload["x.vel"] == 0.12 and payload["arm_shoulder_pan.pos"] == 1.5

    # Like the real host, a torque-off host ignores motion commands.
    command.send_json(_action(**{"y.vel": -0.02}))
    for _ in range(5):
        host.step()
        time.sleep(0.01)
    assert host.actions == []

    assert _torque(context, host, {"command": "enable"}) == {"ok": True, "torque_enabled": True}
    command.send_json(_action(**{"y.vel": -0.02, "arm_shoulder_pan.pos": 2.0}))
    _wait_for(lambda: (host.step(), host.actions)[1])
    assert host.actions[-1]["y.vel"] == -0.02
    assert host.actions[-1]["arm_shoulder_pan.pos"] == 2.0

    host.step()
    time.sleep(0.05)
    frames = _latest(observation)
    payload = json.loads(frames[0])
    assert len(frames) == 1 and payload["_cams"] == []
    assert payload["y.vel"] == -0.02
    assert payload["arm_shoulder_pan.pos"] == 2.0
    assert payload[TELEMETRY_MONOTONIC_NS_KEY] > 0
    assert payload[TELEMETRY_TORQUE_ENABLED_KEY] is True
    assert payload["_lekiwi_motor_health"]["statuses"]["motor_bus"]["level"] == 0
    assert set(payload[HOST_ODOMETRY_KEY]) == {"pose", "scales", "stamp_ns"}

    command.close()
    observation.close()


def test_fake_host_enforces_the_real_motion_envelope_and_watchdog(host, context):
    command = context.socket(zmq.PUSH)
    command.setsockopt(zmq.LINGER, 0)
    command.connect(host.endpoints.command)
    _torque(context, host, {"command": "enable"})

    command.send_json(_action(**{"x.vel": 1.0}))  # beyond the Nav2 speed limit
    time.sleep(0.05)
    host.step()
    assert host.actions == []

    host.set_state(**{"x.vel": 0.01})
    time.sleep(0.6)  # past the command watchdog: stop the base, keep torque on
    host.step()
    assert host.torque_enabled
    assert host.state["x.vel"] == 0.0
    command.close()


def test_qualification_stage_reaches_the_host_without_raising_untagged_caps():
    from pathlib import Path
    from lekiwi_rmf.motion_guards import load_base_speed_limits, load_base_test_profile
    from lekiwi_rmf.odometry import BASE_XY_SCALE
    from lekiwi_rmf.zmq_client import LeKiwiZmqClient
    from lekiwi_rmf.torque_control import TorqueControlClient
    profile=load_base_test_profile(Path(__file__).parents[1]/'config/nav2_params.yaml','0.20')
    with FakeLeKiwiHost(base_test_profiles={.20:profile}) as host:
        host.start(period_s=.01)
        client=LeKiwiZmqClient('127.0.0.1',host.command_endpoint_port,host.observation_endpoint_port,STATE_KEYS)
        client.connect()
        try:
            TorqueControlClient('127.0.0.1',host.torque_endpoint_port).set_enabled(True)
            _wait_for(lambda: client.zmq_cmd_socket.poll(0,zmq.POLLOUT))
            action=_action(**{'x.vel':.20/BASE_XY_SCALE})
            client.send_action(action,base_test_stage=.20)
            _wait_for(lambda:host.actions)
            assert host.actions[-1]['x.vel']==pytest.approx(.20/BASE_XY_SCALE)
            count=len(host.actions)
            production=load_base_speed_limits(Path(__file__).parents[1]/'config/nav2_params.yaml')[0]
            client.send_action(_action(**{'x.vel':(production+.01)/BASE_XY_SCALE}))
            time.sleep(.05)
            assert len(host.actions)==count and host.torque_enabled
        finally:
            client.disconnect()


def test_fake_host_can_inject_motor_health_fault(host, context):
    observation = _pull(context, host.endpoints.observation)
    host.set_motor_health(fault_snapshot(("wheel_left",), "injected bus failure"))
    _publish(host)
    payload = json.loads(_receive(observation)[0])
    assert payload["_lekiwi_motor_health"]["statuses"]["motor_bus"]["level"] == ERROR
    observation.close()


def test_fake_host_controls_drop_duplicate_stale_malformed_and_session_restart(host, context):
    observation = _pull(context, host.endpoints.observation)

    _publish(host)
    first = _receive(observation)
    first_payload = json.loads(first[0])

    host.queue_observation_fault(ObservationFault.DUPLICATE)
    _publish(host)
    assert _receive(observation) == first

    host.queue_observation_fault("stale")
    _publish(host)
    stale_payload = json.loads(_receive(observation)[0])
    assert stale_payload[TELEMETRY_SEQUENCE_KEY] > first_payload[TELEMETRY_SEQUENCE_KEY]
    assert stale_payload[TELEMETRY_MONOTONIC_NS_KEY] == first_payload[TELEMETRY_MONOTONIC_NS_KEY]

    host.queue_observation_fault("malformed")
    _publish(host)
    assert _receive(observation) == [b"{malformed lekiwi observation"]

    host.queue_observation_fault("drop")
    assert _publish(host) is None
    assert not observation.poll(50)

    first_session = host.session
    _torque(context, host, {"command": "enable"})
    assert host.torque_enabled
    second_session = host.restart_session()
    assert second_session != first_session
    assert host.torque_enabled is False
    time.sleep(0.05)
    _latest(observation)
    _publish(host)
    restarted = json.loads(_receive(observation)[0])
    assert restarted[TELEMETRY_SESSION_KEY] == second_session
    assert restarted[TELEMETRY_SEQUENCE_KEY] == 0

    observation.close()


def test_fake_host_reports_torque_failures_and_recovers(host, context):
    host.fail_next_torque("enable", "servo 3 did not acknowledge")
    failed = _torque(context, host, {"command": "enable"})
    assert failed["ok"] is False and failed["torque_enabled"] is False
    assert "servo 3 did not acknowledge" in failed["error"]

    assert _torque(context, host, {"command": "enable"}) == {"ok": True, "torque_enabled": True}
    assert host.torque_enabled

    # A cut the servos do not confirm leaves the host reporting torque on.
    host.fail_next_torque("disable", "servo 5 overload")
    failed = _torque(context, host, {"command": "disable"})
    assert failed["ok"] is False and failed["torque_enabled"] is True
    assert "servo 5 overload" in failed["error"]

    assert _torque(context, host, {"command": "disable"}) == {"ok": True, "torque_enabled": False}


def test_malformed_torque_request_does_not_kill_fake_host(host, context):
    response = _torque(context, host, b"{not-json")
    assert response["ok"] is False
    assert _torque(context, host, {"command": "state"}) == {
        "ok": True, "torque_enabled": False, "trajectory": None,
    }


def test_fake_host_rejects_invalid_fault_and_never_binds_non_loopback_by_default(host):
    assert host.endpoints.command.startswith("tcp://127.0.0.1:")
    with pytest.raises(ValueError, match="unknown observation fault"):
        host.queue_observation_fault("replay")
    with pytest.raises(ValueError, match="enable or disable"):
        host.fail_next_torque("state")
