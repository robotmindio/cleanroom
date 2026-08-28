"""Deterministic, no-hardware LeKiwi ZMQ host for integration tests.

The real motor host owns three endpoints: a PULL socket for JSON actions, a
PUSH socket for multipart observations, and a REP socket for the torque
interlock.  This module intentionally speaks that wire protocol without
importing LeRobot, ROS, cameras, or any motor library.  It is therefore safe
to use in unit tests, ``launch_testing`` fixtures, and a headless simulator.

The host is manual by default: a test calls :meth:`step` or
:meth:`publish_observation` at a known time.  ``start()`` is provided for
tests which need a continuously publishing peer.  All endpoints bind to
loopback by default and use ephemeral ports, so a test cannot discover or
command physical hardware by accident.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from typing import Mapping

import zmq

from lekiwi_rmf.odometry import (
    TELEMETRY_MONOTONIC_NS_KEY,
    TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_PROTOCOL_VERSION,
    TELEMETRY_SEQUENCE_KEY,
    TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)
from lekiwi_rmf.motor_health import healthy_snapshot


LEKIWI_STATE_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
    "x.vel",
    "y.vel",
    "theta.vel",
)


class ObservationFault(str, Enum):
    """The next observation fault to inject into the host stream."""

    VALID = "valid"
    DROP = "drop"
    MALFORMED = "malformed"
    DUPLICATE = "duplicate"
    STALE = "stale"


@dataclass(frozen=True)
class FakeHostEndpoints:
    """Loopback addresses published after the fake host binds its sockets."""

    command: str
    observation: str
    torque: str


@dataclass
class FakeLeKiwiHost:
    """A controllable implementation of the LeKiwi host's ZMQ boundary.

    The class owns an in-memory state model only.  Actions received at the
    command endpoint are recorded and update finite numeric state entries, so
    tests can verify the exact command that a driver emitted and then publish
    matching feedback.  Nothing in this class opens a serial device.
    """

    bind_host: str = "127.0.0.1"
    command_port: int = 0
    observation_port: int = 0
    torque_port: int = 0
    context: zmq.Context | None = None
    state: dict[str, float] = field(default_factory=lambda: {key: 0.0 for key in LEKIWI_STATE_KEYS})
    motor_health: dict | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.bind_host, str) or not self.bind_host:
            raise ValueError("bind_host must be a non-empty string")
        for name, port in (
            ("command_port", self.command_port),
            ("observation_port", self.observation_port),
            ("torque_port", self.torque_port),
        ):
            if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
                raise ValueError(f"{name} must be an integer from 0 through 65535")
        self._owns_context = self.context is None
        self._context = self.context or zmq.Context()
        self._command_socket = self._context.socket(zmq.PULL)
        self._observation_socket = self._context.socket(zmq.PUSH)
        self._torque_socket = self._context.socket(zmq.REP)
        for socket in (self._command_socket, self._observation_socket, self._torque_socket):
            socket.setsockopt(zmq.LINGER, 0)
        self._command_socket.setsockopt(zmq.CONFLATE, 1)
        self._observation_socket.setsockopt(zmq.SNDHWM, 2)
        self._command_socket.bind(f"tcp://{self.bind_host}:{self.command_port}")
        self._observation_socket.bind(f"tcp://{self.bind_host}:{self.observation_port}")
        self._torque_socket.bind(f"tcp://{self.bind_host}:{self.torque_port}")
        self.endpoints = FakeHostEndpoints(
            command=self._command_socket.getsockopt_string(zmq.LAST_ENDPOINT),
            observation=self._observation_socket.getsockopt_string(zmq.LAST_ENDPOINT),
            torque=self._torque_socket.getsockopt_string(zmq.LAST_ENDPOINT),
        )
        self.actions: list[dict[str, float]] = []
        self.torque_enabled = False
        self.torque_requests: list[str] = []
        self._torque_failures: dict[str, deque[str]] = {"enable": deque(), "disable": deque()}
        self._faults: deque[ObservationFault] = deque()
        self._session = uuid.uuid4().hex
        self._sequence = 0
        self._protocol_lock = threading.Lock()
        self._last_frames: list[bytes] | None = None
        self._last_sample_ns: int | None = None
        self._camera_frames: dict[str, bytes] = {}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._motor_names = (
            "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex",
            "arm_wrist_roll", "arm_gripper", "wheel_left", "wheel_right", "wheel_back",
        )
        self._motor_health_injected = self.motor_health is not None
        if self.motor_health is None:
            self.motor_health = healthy_snapshot(self._motor_names, False)

    @property
    def session(self) -> str:
        """The session identifier that will be used for the next sample."""
        with self._protocol_lock:
            return self._session

    @property
    def sequence(self) -> int:
        """The sequence number that will be used for the next sample."""
        with self._protocol_lock:
            return self._sequence

    @staticmethod
    def _endpoint_port(endpoint: str) -> int:
        return int(endpoint.rsplit(":", 1)[1])

    @property
    def command_endpoint_port(self) -> int:
        return self._endpoint_port(self.endpoints.command)

    @property
    def observation_endpoint_port(self) -> int:
        return self._endpoint_port(self.endpoints.observation)

    @property
    def torque_endpoint_port(self) -> int:
        return self._endpoint_port(self.endpoints.torque)

    def queue_observation_fault(self, fault: ObservationFault | str, count: int = 1) -> None:
        """Inject ``fault`` into the next ``count`` published observations."""
        try:
            selected = ObservationFault(fault)
        except ValueError as error:
            raise ValueError(f"unknown observation fault {fault!r}") from error
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("fault count must be a positive integer")
        self._faults.extend([selected] * count)

    def fail_next_torque(self, command: str, message: str = "injected torque failure") -> None:
        """Make the next ``enable`` or ``disable`` request return an error."""
        if command not in self._torque_failures:
            raise ValueError("command must be enable or disable")
        if not isinstance(message, str) or not message:
            raise ValueError("message must be a non-empty string")
        self._torque_failures[command].append(message)

    def restart_session(self) -> str:
        """Simulate a host restart; state survives but packet identity restarts."""
        with self._protocol_lock:
            self._session = uuid.uuid4().hex
            self._sequence = 0
            self.torque_enabled = False
            self._last_frames = None
            self._last_sample_ns = None
            return self._session

    def set_state(self, **updates: float) -> None:
        """Set finite feedback values that are included in later observations."""
        for key, value in updates.items():
            if key not in LEKIWI_STATE_KEYS:
                raise KeyError(f"unknown LeKiwi state key {key!r}")
            value = float(value)
            if not isfinite(value):
                raise ValueError(f"state {key!r} must be finite")
            self.state[key] = value

    def set_camera_frames(self, frames: Mapping[str, bytes]) -> None:
        """Set already-JPEG-encoded camera frames for multipart observations."""
        copied: dict[str, bytes] = {}
        for name, frame in frames.items():
            if not isinstance(name, str) or not name:
                raise ValueError("camera names must be non-empty strings")
            if not isinstance(frame, bytes):
                raise TypeError("camera frames must be JPEG bytes")
            copied[name] = frame
        self._camera_frames = copied

    def set_motor_health(self, snapshot: Mapping) -> None:
        """Set an arbitrary diagnostic snapshot for wire/fault-injection tests."""
        if not isinstance(snapshot, Mapping):
            raise TypeError("motor health snapshot must be a mapping")
        self.motor_health = dict(snapshot)
        self._motor_health_injected = True

    def _next_fault(self) -> ObservationFault:
        return self._faults.popleft() if self._faults else ObservationFault.VALID

    def _valid_frames(self, *, stale_timestamp: bool = False) -> list[bytes]:
        with self._protocol_lock:
            if not self._motor_health_injected:
                self.motor_health = healthy_snapshot(self._motor_names, self.torque_enabled)
            sample_ns = self._last_sample_ns if stale_timestamp and self._last_sample_ns is not None else time.monotonic_ns()
            payload = {
                "_cams": list(self._camera_frames),
                **self.state,
                TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
                TELEMETRY_SESSION_KEY: self._session,
                TELEMETRY_SEQUENCE_KEY: self._sequence,
                TELEMETRY_MONOTONIC_NS_KEY: sample_ns,
                TELEMETRY_TORQUE_ENABLED_KEY: self.torque_enabled,
                "_lekiwi_motor_health": self.motor_health,
            }
            self._sequence += 1
            self._last_sample_ns = sample_ns
        return [json.dumps(payload, sort_keys=True).encode("utf-8"), *self._camera_frames.values()]

    def publish_observation(self) -> list[bytes] | None:
        """Publish one valid or fault-injected observation and return its frames.

        ``DROP`` deliberately sends nothing.  ``MALFORMED`` sends invalid JSON;
        ``DUPLICATE`` repeats the prior packet byte-for-byte; ``STALE`` creates a
        new sequence carrying the previous source timestamp.  These distinct
        cases exercise decoder, ordering, and watchdog behavior separately.
        """
        fault = self._next_fault()
        if fault is ObservationFault.DROP:
            return None
        if fault is ObservationFault.MALFORMED:
            frames = [b"{malformed lekiwi observation"]
        elif fault is ObservationFault.DUPLICATE:
            if self._last_frames is None:
                raise RuntimeError("cannot duplicate before a valid observation")
            frames = list(self._last_frames)
        else:
            frames = self._valid_frames(stale_timestamp=fault is ObservationFault.STALE)
            if fault is ObservationFault.VALID:
                self._last_frames = list(frames)
        try:
            self._observation_socket.send_multipart(frames, flags=zmq.NOBLOCK)
        except zmq.Again:
            # Match the real host: observations are lossy when no client is
            # connected, and teardown must never strand the fake's thread in send().
            pass
        return frames

    def process_actions(self) -> int:
        """Drain available JSON actions and return how many were accepted."""
        accepted = 0
        while True:
            try:
                encoded = self._command_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                return accepted
            try:
                decoded = json.loads(encoded.decode("utf-8"))
                if not isinstance(decoded, dict):
                    raise ValueError("action must be a JSON object")
                action = {str(key): float(value) for key, value in decoded.items()}
                if not all(isfinite(value) for value in action.values()):
                    raise ValueError("action contains non-finite value")
            except (UnicodeDecodeError, TypeError, ValueError, json.JSONDecodeError):
                continue
            self.actions.append(action)
            for key, value in action.items():
                if key in self.state:
                    self.state[key] = value
            accepted += 1

    def process_torque(self) -> bool:
        """Process at most one torque request; return whether a request arrived."""
        try:
            request = self._torque_socket.recv_json(zmq.NOBLOCK)
        except zmq.Again:
            return False
        except Exception as error:
            # Match the real REP endpoint: malformed JSON consumes one request
            # but must not kill the continuously running fake-host thread.
            self._torque_socket.send_json({
                "ok": False, "error": f"invalid request: {error}",
                "torque_enabled": self.torque_enabled,
            })
            return True
        if not isinstance(request, dict) or not isinstance(request.get("command"), str):
            self._torque_socket.send_json({"ok": False, "error": "command must be enable, disable, or state"})
            return True
        command = request["command"]
        self.torque_requests.append(command)
        if command in self._torque_failures and self._torque_failures[command]:
            self._torque_socket.send_json({
                "ok": False,
                "error": self._torque_failures[command].popleft(),
                "torque_enabled": self.torque_enabled,
            })
        elif command == "enable":
            self.torque_enabled = True
            self._torque_socket.send_json({"ok": True, "torque_enabled": True})
        elif command == "disable":
            self.torque_enabled = False
            self._torque_socket.send_json({"ok": True, "torque_enabled": False})
        elif command == "state":
            self._torque_socket.send_json({"ok": True, "torque_enabled": self.torque_enabled})
        else:
            self._torque_socket.send_json({"ok": False, "error": "command must be enable, disable, or state"})
        return True

    def step(self, *, publish: bool = True) -> list[bytes] | None:
        """Advance all protocol endpoints once without sleeping."""
        self.process_actions()
        self.process_torque()
        return self.publish_observation() if publish else None

    def start(self, period_s: float = 1 / 30) -> None:
        """Start a daemon thread which steps the host at ``period_s`` intervals."""
        if not isinstance(period_s, (int, float)) or period_s <= 0:
            raise ValueError("period_s must be positive")
        if self._thread and self._thread.is_alive():
            raise RuntimeError("fake host is already running")
        self._stop.clear()

        def run() -> None:
            while not self._stop.is_set():
                started = time.monotonic()
                self.step()
                self._stop.wait(max(0.0, period_s - (time.monotonic() - started)))

        self._thread = threading.Thread(target=run, name="fake-lekiwi-host", daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stop the fake and release its loopback ports."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("fake host thread did not stop")
            self._thread = None
        for socket in (self._torque_socket, self._observation_socket, self._command_socket):
            socket.close()
        if self._owns_context:
            self._context.term()

    def __enter__(self) -> "FakeLeKiwiHost":
        return self

    def __exit__(self, *_unused) -> None:
        self.close()
