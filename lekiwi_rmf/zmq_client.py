"""Repository-owned, state-only LeKiwi ZeroMQ client.

Cameras travel over ROS in this repository. This client intentionally handles
only motor state and actions, avoiding LeRobot private APIs and NumPy state.
"""

from __future__ import annotations

import json
import math

from lekiwi_rmf.odometry import TelemetrySequenceTracker, accept_validated_telemetry
from lekiwi_rmf.motor_health import MOTOR_HEALTH_KEY, parse_motor_health
from lekiwi_rmf.zmq_security import (
    CurveClientCredentials,
)


class LeKiwiZmqClient:
    def __init__(
        self,
        remote_ip: str,
        command_port: int,
        observation_port: int,
        state_keys,
        *,
        curve_credentials: CurveClientCredentials | None = None,
        polling_timeout_ms: int = 15,
        connect_timeout_s: int = 5,
        command_timeout_ms: int = 100,
        zmq_module=None,
    ):
        if not isinstance(remote_ip, str) or not remote_ip.strip():
            raise ValueError("remote_ip must be a non-empty string")
        self.remote_ip = remote_ip
        self.command_port = command_port
        self.observation_port = observation_port
        self.state_keys = tuple(state_keys)
        self.curve_credentials = (
            curve_credentials or CurveClientCredentials()
        ).validate()
        self.polling_timeout_ms = polling_timeout_ms
        self.connect_timeout_s = connect_timeout_s
        if (
            isinstance(command_timeout_ms, bool)
            or not isinstance(command_timeout_ms, int)
            or command_timeout_ms <= 0
        ):
            raise ValueError("command_timeout_ms must be a positive integer")
        self.command_timeout_ms = command_timeout_ms
        self._provided_zmq = zmq_module
        self._zmq = None
        self.zmq_context = None
        self.zmq_cmd_socket = None
        self.zmq_observation_socket = None
        self.connected = False
        self.last_remote_state = {}
        self.missing_state_keys = self.state_keys
        self.observation_sequence = 0
        self.observation_token = None
        self.observation_sample_monotonic_ns = None
        self.observation_session_changed = False
        self.observation_torque_enabled = None
        self.observation_motor_health = None
        self.telemetry_sequences = TelemetrySequenceTracker()

    def connect(self):
        if self.connected:
            raise RuntimeError("LeKiwi client is already connected")
        if self._provided_zmq is None:
            import zmq
        else:
            zmq = self._provided_zmq
        self._zmq = zmq
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket = self.zmq_context.socket(zmq.PULL)
        try:
            self.zmq_cmd_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
            self.zmq_cmd_socket.setsockopt(zmq.SNDTIMEO, self.command_timeout_ms)
            # Do not silently queue motion for a host that is not currently
            # connected. A later reconnect must never receive an old action.
            self.zmq_cmd_socket.setsockopt(zmq.IMMEDIATE, 1)
            self.zmq_observation_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_observation_socket.setsockopt(zmq.RCVHWM, 2)
            self.curve_credentials.configure_socket(self.zmq_cmd_socket)
            self.curve_credentials.configure_socket(self.zmq_observation_socket)
            self.zmq_cmd_socket.connect(f"tcp://{self.remote_ip}:{self.command_port}")
            self.zmq_observation_socket.connect(
                f"tcp://{self.remote_ip}:{self.observation_port}"
            )
            poller = zmq.Poller()
            poller.register(self.zmq_observation_socket, zmq.POLLIN)
            if dict(poller.poll(self.connect_timeout_s * 1000)).get(
                self.zmq_observation_socket
            ) != zmq.POLLIN:
                raise ConnectionError("timeout waiting for LeKiwi host observation")
        except Exception:
            self._close_sockets()
            raise
        self.connected = True

    def _close_sockets(self):
        for socket in (self.zmq_observation_socket, self.zmq_cmd_socket):
            if socket is not None:
                socket.close()
        if self.zmq_context is not None:
            self.zmq_context.term()
        self.zmq_observation_socket = None
        self.zmq_cmd_socket = None
        self.zmq_context = None

    def disconnect(self):
        self._close_sockets()
        self.connected = False

    def _poll_latest(self):
        poller = self._zmq.Poller()
        poller.register(self.zmq_observation_socket, self._zmq.POLLIN)
        if dict(poller.poll(self.polling_timeout_ms)).get(
            self.zmq_observation_socket
        ) != self._zmq.POLLIN:
            return None
        latest = None
        while True:
            try:
                latest = self.zmq_observation_socket.recv_multipart(self._zmq.NOBLOCK)
            except self._zmq.Again:
                return latest

    def _decode(self, frames):
        if not frames:
            raise ValueError("empty observation multipart message")
        try:
            payload = json.loads(frames[0])
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError("malformed observation JSON") from error
        if not isinstance(payload, dict):
            raise ValueError("observation header must be a JSON object")
        cameras = payload.pop("_cams", None)
        if not isinstance(cameras, list) or any(not isinstance(name, str) for name in cameras):
            raise ValueError("observation camera manifest is malformed")
        if len(cameras) != len(frames) - 1:
            raise ValueError("observation camera manifest does not match multipart frames")
        return payload

    def get_observation(self):
        if not self.connected:
            raise RuntimeError("LeKiwi client is not connected")
        frames = self._poll_latest()
        if frames is None:
            return self.last_remote_state
        try:
            payload = self._decode(frames)
            self.missing_state_keys = tuple(
                key for key in self.state_keys if key not in payload
            )
            accepted = accept_validated_telemetry(
                self.telemetry_sequences, payload, self.state_keys
            )
            motor_health = parse_motor_health(payload.get(MOTOR_HEALTH_KEY))
            state = {key: float(payload[key]) for key in self.state_keys}
        except (TypeError, ValueError, OverflowError):
            return self.last_remote_state
        self.observation_sequence += 1
        self.observation_token = accepted.token
        self.observation_sample_monotonic_ns = accepted.sample_monotonic_ns
        self.observation_session_changed = accepted.session_changed
        self.observation_torque_enabled = accepted.torque_enabled
        self.observation_motor_health = motor_health
        self.last_remote_state = state
        return state

    def send_action(self, action):
        if not self.connected:
            raise RuntimeError("LeKiwi client is not connected")
        if not isinstance(action, dict) or set(action) != set(self.state_keys):
            raise ValueError("action must contain exactly every configured state key")
        encoded = {}
        for key, value in action.items():
            if not isinstance(key, str) or isinstance(value, bool):
                raise ValueError("action keys must be strings and values must be numeric")
            number = float(value)
            if not math.isfinite(number):
                raise ValueError(f"action {key!r} is not finite")
            encoded[str(key)] = number
        try:
            self.zmq_cmd_socket.send_string(
                json.dumps(encoded, allow_nan=False), flags=self._zmq.NOBLOCK
            )
        except self._zmq.Again as error:
            raise ConnectionError(
                "LeKiwi command transport is unavailable; action was not sent"
            ) from error
        return encoded
