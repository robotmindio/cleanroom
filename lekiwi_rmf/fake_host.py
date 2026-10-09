"""Deterministic, no-hardware LeKiwi motor host for integration tests.

This is the real host core (:mod:`lekiwi_rmf.motor_host`) on an in-memory
motor bus. It binds the same three endpoints as the Pi host -- a PULL socket
for JSON actions, a PUSH socket for JSON observations, and a REP socket
for the torque interlock -- and enforces the same torque-off, motion-envelope,
arm-lease and command-watchdog rules, including host odometry in every
observation. It imports no LeRobot, ROS or serial library.

The host is manual by default: a test calls :meth:`step` or
:meth:`publish_observation` at a known time. ``start()`` is provided for
tests which need a continuously publishing peer. All endpoints bind to
loopback by default and use ephemeral ports, so a test cannot discover or
command physical hardware by accident.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import isfinite
from pathlib import Path
from types import SimpleNamespace
from typing import Mapping

import zmq

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, load_calibration
from lekiwi_rmf.host_protocol import (
    BASE_VELOCITY_KEYS, MOTOR_NAMES, STATE_KEYS, TELEMETRY_MONOTONIC_NS_KEY, TorqueCommand,
)
from lekiwi_rmf.motion_guards import load_base_speed_limits
from lekiwi_rmf.motor_health import healthy_snapshot
from lekiwi_rmf.motor_host import HostLoop
from lekiwi_rmf.odometry import BASE_XY_SCALE, BASE_YAW_SCALE


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


def _default_nav2_params() -> Path:
    path = Path(__file__).parents[1] / "config/nav2_params.yaml"
    if path.is_file():
        return path
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("lekiwi_rmf")) / "config/nav2_params.yaml"


class FakeMotorBus:
    """Torque and position registers of the nine LeKiwi servos, with injectable faults."""

    def __init__(self, state: dict[str, float]):
        self.state = state
        self.motors = dict.fromkeys(MOTOR_NAMES)
        self.torque = dict.fromkeys(MOTOR_NAMES, 0)
        self.failures: dict[str, deque[str]] = {
            TorqueCommand.ENABLE: deque(), TorqueCommand.DISABLE: deque(),
        }

    def _fail(self, command: str) -> None:
        if self.failures[command]:
            raise RuntimeError(self.failures[command].popleft())

    def enable_torque(self, num_retry=0) -> None:
        self._fail(TorqueCommand.ENABLE)
        self.torque = dict.fromkeys(MOTOR_NAMES, 1)

    def disable_torque(self, motor, num_retry=0) -> None:
        self.torque[motor] = 0

    def sync_read(self, register, motors, normalize=True, num_retry=0) -> dict:
        if register == "Torque_Enable":
            return {motor: self.torque[motor] for motor in motors}
        if register == "Present_Position":
            return {motor: self.state[f"{motor}.pos"] for motor in motors}
        raise KeyError(f"the fake motor bus does not model {register}")

    def sync_write(self, register, values, num_retry=0) -> None:
        if register == "Torque_Enable":
            if not values:
                self._fail(TorqueCommand.DISABLE)
            self.torque = dict.fromkeys(MOTOR_NAMES, int(values))
        elif register != "Goal_Position":
            raise KeyError(f"the fake motor bus does not model {register}")


class FakeLeKiwiRobot:
    """The LeKiwi surface used by the host core: feedback follows each accepted command."""

    def __init__(self, state: dict[str, float], actions: list[dict[str, float]]):
        self.state = state
        self.actions = actions
        self.bus = FakeMotorBus(state)
        self.arm_motors = list(ARM_JOINTS)

    def get_observation(self) -> dict:
        return dict(self.state)

    def send_action(self, action: Mapping[str, float]) -> dict[str, float]:
        action = dict(action)
        self.actions.append(action)
        self.state.update((key, value) for key, value in action.items() if key in self.state)
        return action

    def stop_base(self) -> None:
        self.state.update(dict.fromkeys(BASE_VELOCITY_KEYS, 0.0))


class FakeMotorHealth:
    """All-OK health for the reported torque state, unless a test injects a snapshot."""

    def __init__(self, snapshot: Mapping | None = None):
        self.snapshot = None if snapshot is None else dict(snapshot)

    def collect(self, _robot, torque_enabled: bool) -> dict:
        return self.snapshot if self.snapshot is not None else healthy_snapshot(MOTOR_NAMES, torque_enabled)


class FaultInjectingSocket:
    """Wrap the observation PUSH socket; only transmitted frames change."""

    def __init__(self, socket):
        self.socket = socket
        self.faults: deque[ObservationFault] = deque()
        self.last_sent: list[bytes] | None = None
        self._last_valid: list[bytes] | None = None
        self._last_sample_ns: int | None = None

    def send_multipart(self, frames, flags=0) -> None:
        fault = self.faults.popleft() if self.faults else ObservationFault.VALID
        frames = [bytes(frame) for frame in frames]
        header = json.loads(frames[0])
        sample_ns = header[TELEMETRY_MONOTONIC_NS_KEY]
        if fault is ObservationFault.DROP:
            self.last_sent = None
            return
        if fault is ObservationFault.MALFORMED:
            outgoing = [b"{malformed lekiwi observation"]
        elif fault is ObservationFault.DUPLICATE:
            if self._last_valid is None:
                raise RuntimeError("cannot duplicate before a valid observation")
            outgoing = list(self._last_valid)
        elif fault is ObservationFault.STALE and self._last_sample_ns is not None:
            header[TELEMETRY_MONOTONIC_NS_KEY] = self._last_sample_ns
            outgoing = [json.dumps(header).encode()]
        else:
            outgoing = frames
            self._last_valid = list(frames)
            self._last_sample_ns = sample_ns
        self.last_sent = outgoing
        # Lossy when no client is connected, like the real host; teardown must
        # never strand the fake's thread in send().
        self.socket.send_multipart(outgoing, flags=flags)


class FakeLeKiwiHost:
    """The LeKiwi host's ZMQ boundary and host core over an in-memory robot.

    Accepted actions are recorded in ``actions`` and move the in-memory
    feedback, so tests can verify the exact command a driver emitted and see
    it reflected in later observations. Nothing in this class opens a serial
    device.
    """

    def __init__(
        self, bind_host: str = "127.0.0.1", command_port: int = 0, observation_port: int = 0,
        torque_port: int = 0, context: zmq.Context | None = None,
        state: dict[str, float] | None = None, motor_health: Mapping | None = None, *,
        disarm_on_failure: bool = False, watchdog_timeout_s: float = 0.5,
        arm_calibration_file: str = "", nav2_params_file: str | Path | None = None,
        base_scales: tuple[float, float] = (BASE_XY_SCALE, BASE_YAW_SCALE),
        base_test_profiles: dict | None = None,
    ):
        if not isinstance(bind_host, str) or not bind_host:
            raise ValueError("bind_host must be a non-empty string")
        for name, port in (
            ("command_port", command_port),
            ("observation_port", observation_port),
            ("torque_port", torque_port),
        ):
            if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
                raise ValueError(f"{name} must be an integer from 0 through 65535")
        self.state = {key: 0.0 for key in STATE_KEYS} if state is None else state
        self.actions: list[dict[str, float]] = []
        self.robot = FakeLeKiwiRobot(self.state, self.actions)
        self.health = FakeMotorHealth(motor_health)
        self._settings = dict(
            disarm_on_failure=disarm_on_failure,
            # Like the real host, a missing calibration file means identity calibration.
            arm_calibration=load_calibration(arm_calibration_file or "/nonexistent/arm-calibration.json"),
            base_limits=load_base_speed_limits(nav2_params_file or _default_nav2_params()),
            base_test_profiles=base_test_profiles,
            base_scales=base_scales,
            # Tests observe at every step.
            observation_period_s=0.0,
        )
        self._owns_context = context is None
        self._context = context or zmq.Context()
        command = self._context.socket(zmq.PULL)
        observation = self._context.socket(zmq.PUSH)
        torque = self._context.socket(zmq.REP)
        for socket in (command, observation, torque):
            socket.setsockopt(zmq.LINGER, 0)
        command.setsockopt(zmq.CONFLATE, 1)
        observation.setsockopt(zmq.SNDHWM, 2)
        command.bind(f"tcp://{bind_host}:{command_port}")
        observation.bind(f"tcp://{bind_host}:{observation_port}")
        torque.bind(f"tcp://{bind_host}:{torque_port}")
        self.endpoints = FakeHostEndpoints(
            command=command.getsockopt_string(zmq.LAST_ENDPOINT),
            observation=observation.getsockopt_string(zmq.LAST_ENDPOINT),
            torque=torque.getsockopt_string(zmq.LAST_ENDPOINT),
        )
        self._observations = FaultInjectingSocket(observation)
        self.transport = SimpleNamespace(
            zmq_cmd_socket=command, zmq_observation_socket=self._observations,
            torque_socket=torque, watchdog_timeout_ms=watchdog_timeout_s * 1000,
        )
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.loop = HostLoop(self.robot, self.transport, self.health, **self._settings)

    @property
    def session(self) -> str:
        """The session identifier that will be used for the next sample."""
        return self.loop.telemetry_session

    @property
    def sequence(self) -> int:
        """The sequence number that will be used for the next sample."""
        return self.loop.telemetry_sequence

    @property
    def torque_enabled(self) -> bool:
        return self.loop.control.torque_enabled

    @property
    def arm_executor(self):
        return self.loop.arm_executor

    @property
    def motor_health(self) -> dict:
        return self.health.collect(self.robot, self.torque_enabled)

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
        with self._lock:
            self._observations.faults.extend([selected] * count)

    def fail_next_torque(self, command: str, message: str = "injected torque failure") -> None:
        """Make the servo bus fail the next ``enable`` or ``disable`` torque write."""
        if command not in self.robot.bus.failures:
            raise ValueError("command must be enable or disable")
        if not isinstance(message, str) or not message:
            raise ValueError("message must be a non-empty string")
        with self._lock:
            self.robot.bus.failures[command].append(message)

    def restart_session(self) -> str:
        """Simulate a host restart: torque off and a new telemetry session; feedback survives."""
        with self._lock:
            self.robot.bus.torque = dict.fromkeys(MOTOR_NAMES, 0)
            self.loop = HostLoop(self.robot, self.transport, self.health, **self._settings)
            return self.loop.telemetry_session

    def set_state(self, **updates: float) -> None:
        """Set finite feedback values that are included in later observations."""
        checked = {}
        for key, value in updates.items():
            if key not in STATE_KEYS:
                raise KeyError(f"unknown LeKiwi state key {key!r}")
            value = float(value)
            if not isfinite(value):
                raise ValueError(f"state {key!r} must be finite")
            checked[key] = value
        with self._lock:
            self.state.update(checked)

    def set_motor_health(self, snapshot: Mapping) -> None:
        """Set an arbitrary diagnostic snapshot for wire/fault-injection tests."""
        if not isinstance(snapshot, Mapping):
            raise TypeError("motor health snapshot must be a mapping")
        with self._lock:
            self.health.snapshot = dict(snapshot)

    def publish_observation(self) -> list[bytes] | None:
        """Publish one valid or fault-injected observation and return its frames.

        ``DROP`` deliberately sends nothing. ``MALFORMED`` sends invalid JSON;
        ``DUPLICATE`` repeats the prior packet byte-for-byte; ``STALE`` sends a
        new sequence carrying the previous source timestamp. These distinct
        cases exercise decoder, ordering, and watchdog behavior separately.
        """
        with self._lock:
            self.loop.publish_observation()
            return self._observations.last_sent

    def step(self) -> list[bytes] | None:
        """Run one host-loop pass without sleeping; return the observation it sent."""
        with self._lock:
            self.loop.step()
            return self._observations.last_sent

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
        for socket in (
            self.transport.torque_socket, self._observations.socket, self.transport.zmq_cmd_socket,
        ):
            socket.close()
        if self._owns_context:
            self._context.term()

    def __enter__(self) -> "FakeLeKiwiHost":
        return self

    def __exit__(self, *_unused) -> None:
        self.close()
