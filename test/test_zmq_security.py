"""Behavioral checks for fail-closed CURVE transport configuration."""

import os
import json
from pathlib import Path
import threading

import pytest

from lekiwi_rmf.zmq_security import (
    CurveClientCredentials, CurveConfigurationError, CurveServerSecurity,
)
from lekiwi_rmf.odometry import (
    TELEMETRY_MONOTONIC_NS_KEY, TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_PROTOCOL_VERSION, TELEMETRY_SEQUENCE_KEY, TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)
from lekiwi_rmf.zmq_client import LeKiwiZmqClient


class _Context:
    pass


def test_unauthenticated_server_is_limited_to_loopback():
    security = CurveServerSecurity(_Context(), "127.0.0.1")
    assert security.enabled is False
    security.close()
    with pytest.raises(CurveConfigurationError, match="non-loopback"):
        CurveServerSecurity(_Context(), "192.0.2.10")
    with pytest.raises(CurveConfigurationError, match="unauthenticated"):
        LeKiwiZmqClient("192.0.2.10", 5555, 5556, ("x.vel",))


def test_partial_curve_configuration_fails_closed(tmp_path):
    public = tmp_path / "server.key"
    public.write_text("public", encoding="ascii")
    with pytest.raises(CurveConfigurationError, match="both client"):
        CurveClientCredentials(server_public_key_file=str(public)).validate()
    with pytest.raises(CurveConfigurationError, match="both server"):
        CurveServerSecurity(_Context(), "127.0.0.1", "secret", "")


def test_authenticated_server_requires_at_least_one_authorized_client(tmp_path):
    zmq = pytest.importorskip("zmq")
    from zmq.auth import create_certificates

    _public, secret = create_certificates(str(tmp_path), "server")
    os.chmod(secret, 0o600)
    empty = tmp_path / "authorized"
    empty.mkdir()
    context = zmq.Context()
    try:
        with pytest.raises(CurveConfigurationError, match="no public"):
            CurveServerSecurity(context, "127.0.0.1", secret, str(empty))
    finally:
        context.term()


def test_secret_key_permissions_are_enforced(tmp_path):
    secret = tmp_path / "client.key_secret"
    public = tmp_path / "server.key"
    secret.write_text("secret", encoding="ascii")
    public.write_text("public", encoding="ascii")
    os.chmod(secret, 0o644)
    with pytest.raises(CurveConfigurationError, match="group or other"):
        CurveClientCredentials(str(secret), str(public)).validate()


def _state_frames(sequence=0, value=1.25, torque_enabled=False):
    return [json.dumps({
        "_cams": [],
        "joint.pos": value,
        TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
        TELEMETRY_SESSION_KEY: "test-session",
        TELEMETRY_SEQUENCE_KEY: sequence,
        TELEMETRY_MONOTONIC_NS_KEY: 100 + sequence,
        TELEMETRY_TORQUE_ENABLED_KEY: torque_enabled,
    }).encode("utf-8")]


def test_repository_client_commits_only_decoded_complete_ordered_state():
    client = LeKiwiZmqClient("127.0.0.1", 5555, 5556, ("joint.pos",))
    client.connected = True
    queued = iter((_state_frames(), [b"{bad json"], _state_frames(), _state_frames(1, 2.5)))
    client._poll_latest = lambda: next(queued)

    assert client.get_observation() == {"joint.pos": 1.25}
    first_token = client.observation_token
    assert client.get_observation() == {"joint.pos": 1.25}
    assert client.observation_token == first_token
    # Replayed sequence zero also remains cached and cannot refresh freshness.
    assert client.get_observation() == {"joint.pos": 1.25}
    assert client.observation_token == first_token
    assert client.get_observation() == {"joint.pos": 2.5}
    assert client.observation_token == ("host", "test-session", 1)
    assert client.observation_torque_enabled is False

    client._poll_latest = lambda: [json.dumps({
        **json.loads(_state_frames()[0]),
        TELEMETRY_SESSION_KEY: "restarted-session",
        TELEMETRY_MONOTONIC_NS_KEY: 1,
    }).encode("utf-8")]
    client.get_observation()
    assert client.observation_session_changed is True


def test_repository_client_exposes_missing_state_without_zero_filling():
    client = LeKiwiZmqClient(
        "127.0.0.1", 5555, 5556, ("joint.pos", "x.vel")
    )
    client.connected = True
    client._poll_latest = lambda: _state_frames()

    assert client.get_observation() == {}
    assert client.missing_state_keys == ("x.vel",)
    assert client.observation_token is None


def test_repository_client_requires_protocol_metadata_unless_explicitly_compatible():
    legacy = [json.dumps({"_cams": [], "joint.pos": 1.0}).encode("utf-8")]
    production = LeKiwiZmqClient("127.0.0.1", 5555, 5556, ("joint.pos",))
    production.connected = True
    production._poll_latest = lambda: legacy
    assert production.get_observation() == {}
    assert production.observation_token is None

    compatibility = LeKiwiZmqClient(
        "127.0.0.1", 5555, 5556, ("joint.pos",), require_metadata=False
    )
    compatibility.connected = True
    compatibility._poll_latest = lambda: legacy
    assert compatibility.get_observation() == {"joint.pos": 1.0}
    assert compatibility.observation_token == ("legacy", 1)


def test_repository_client_command_send_is_nonblocking_and_reports_backpressure():
    class Again(Exception):
        pass

    class Zmq:
        NOBLOCK = 17

    Zmq.Again = Again

    class Socket:
        def __init__(self):
            self.calls = []
            self.fail = False

        def send_string(self, payload, flags=0):
            self.calls.append((payload, flags))
            if self.fail:
                raise Again("full")

    socket = Socket()
    client = LeKiwiZmqClient(
        "127.0.0.1", 5555, 5556, ("joint.pos",), zmq_module=Zmq
    )
    client._zmq = Zmq
    client.zmq_cmd_socket = socket
    client.connected = True

    assert client.send_action({"joint.pos": 2}) == {"joint.pos": 2.0}
    assert json.loads(socket.calls[-1][0]) == {"joint.pos": 2.0}
    assert socket.calls[-1][1] == Zmq.NOBLOCK

    socket.fail = True
    with pytest.raises(ConnectionError, match="action was not sent"):
        client.send_action({"joint.pos": 3})

    with pytest.raises(ValueError, match="exactly"):
        client.send_action({})


def test_curve_allows_authorized_client_and_rejects_unknown_client(tmp_path):
    zmq = pytest.importorskip("zmq")
    from zmq.auth import create_certificates

    server_public, server_secret = create_certificates(str(tmp_path), "server")
    client_source = tmp_path / "client-source"
    client_source.mkdir()
    client_public, client_secret = create_certificates(str(client_source), "driver")
    clients = tmp_path / "authorized"
    clients.mkdir()
    (clients / "driver.key").write_bytes(Path(client_public).read_bytes())
    unknown_dir = tmp_path / "unknown"
    unknown_dir.mkdir()
    unknown_public, unknown_secret = create_certificates(str(unknown_dir), "intruder")
    del unknown_public
    os.chmod(server_secret, 0o600)
    os.chmod(client_secret, 0o600)
    os.chmod(unknown_secret, 0o600)

    context = zmq.Context()
    server_security = CurveServerSecurity(
        context, "127.0.0.1", server_secret, str(clients)
    )
    server = context.socket(zmq.PULL)
    server.setsockopt(zmq.LINGER, 0)
    server_security.configure_socket(server)
    port = server.bind_to_random_port("tcp://127.0.0.1")
    try:
        intruder = context.socket(zmq.PUSH)
        intruder.setsockopt(zmq.LINGER, 0)
        intruder.setsockopt(zmq.SNDTIMEO, 200)
        CurveClientCredentials(unknown_secret, server_public).validate().configure_socket(intruder)
        intruder.connect(f"tcp://127.0.0.1:{port}")
        try:
            intruder.send(b"intruder")
        except zmq.Again:
            pass
        assert server.poll(250) == 0
        intruder.close()

        client = context.socket(zmq.PUSH)
        client.setsockopt(zmq.LINGER, 0)
        CurveClientCredentials(client_secret, server_public).validate().configure_socket(client)
        client.connect(f"tcp://127.0.0.1:{port}")
        client.send(b"authorized")
        assert server.poll(2000) & zmq.POLLIN
        assert server.recv() == b"authorized"
        client.close()
    finally:
        server.close()
        server_security.close()
        context.term()


def test_repository_client_speaks_authenticated_state_protocol(tmp_path):
    zmq = pytest.importorskip("zmq")
    from zmq.auth import create_certificates

    server_public, server_secret = create_certificates(str(tmp_path), "server")
    client_source = tmp_path / "client-source"
    client_source.mkdir()
    client_public, client_secret = create_certificates(str(client_source), "driver")
    authorized = tmp_path / "authorized"
    authorized.mkdir()
    (authorized / "driver.key").write_bytes(Path(client_public).read_bytes())
    os.chmod(server_secret, 0o600)
    os.chmod(client_secret, 0o600)

    context = zmq.Context()
    security = CurveServerSecurity(context, "127.0.0.1", server_secret, str(authorized))
    command = context.socket(zmq.PULL)
    observation = context.socket(zmq.PUSH)
    for socket in (command, observation):
        socket.setsockopt(zmq.LINGER, 0)
        security.configure_socket(socket)
    command_port = command.bind_to_random_port("tcp://127.0.0.1")
    observation_port = observation.bind_to_random_port("tcp://127.0.0.1")
    payload = {
        "_cams": [],
        "joint.pos": 1.25,
        TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
        TELEMETRY_SESSION_KEY: "test-session",
        TELEMETRY_SEQUENCE_KEY: 0,
        TELEMETRY_MONOTONIC_NS_KEY: 123,
        TELEMETRY_TORQUE_ENABLED_KEY: False,
    }
    publisher = threading.Thread(
        target=observation.send_multipart,
        args=([json.dumps(payload).encode("utf-8")],),
    )
    publisher.start()
    client = LeKiwiZmqClient(
        "127.0.0.1", command_port, observation_port, ("joint.pos",),
        curve_credentials=CurveClientCredentials(client_secret, server_public),
        connect_timeout_s=2,
    )
    try:
        client.connect()
        assert client.zmq_cmd_socket.getsockopt(zmq.SNDTIMEO) == 100
        assert client.zmq_cmd_socket.getsockopt(zmq.IMMEDIATE) == 1
        publisher.join(timeout=2)
        assert not publisher.is_alive()
        assert client.get_observation() == {"joint.pos": 1.25}
        assert client.observation_token == ("host", "test-session", 0)
        client.send_action({"joint.pos": 2.5})
        assert command.poll(2000) & zmq.POLLIN
        assert command.recv_json() == {"joint.pos": 2.5}
    finally:
        client.disconnect()
        command.close()
        observation.close()
        security.close()
        context.term()
