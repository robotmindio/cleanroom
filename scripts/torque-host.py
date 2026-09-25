#!/usr/bin/env python3
"""LeRobot motor host with a repository-owned physical-torque safety channel.

The stock LeRobot host owns the serial bus but only exposes motion commands.
This process keeps its command/observation protocol intact and adds a separate
ZMQ REP endpoint for the ROS driver's explicit arm/disarm transactions.
"""

import json
import logging
import math
import os
import signal
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import draccus
import zmq

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiConfig, LeKiwiHostConfig
from lerobot.robots.lekiwi.lekiwi import LeKiwi

from lekiwi_rmf.torque_control import (
    enable_with_rollback, react_to_command_silence, run_all_safety_steps,
    torque_readback_matches, validate_action_payload, validated_bind_address,
)
from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.zmq_security import CurveServerSecurity, configure_link_liveness
from lekiwi_rmf.odometry import (
    TELEMETRY_MONOTONIC_NS_KEY, TELEMETRY_PROTOCOL_KEY,
    TELEMETRY_PROTOCOL_VERSION, TELEMETRY_SEQUENCE_KEY, TELEMETRY_SESSION_KEY,
    TELEMETRY_TORQUE_ENABLED_KEY,
)
from lekiwi_rmf.motor_health import fault_snapshot, healthy_snapshot


TORQUE_RETRIES = 5
HEALTH_PERIOD_S = 0.10
ACTION_KEYS = tuple(f"{joint}.pos" for joint in ARM_JOINTS) + (
    "x.vel", "y.vel", "theta.vel",
)


def broadcast_torque_off(bus):
    # Feetech's per-servo disable aborts on an overload response and skips
    # every later motor. Broadcast to all nine; callers verify each readback.
    bus.sync_write("Torque_Enable", 0, num_retry=TORQUE_RETRIES)


def disable_torque_per_motor(bus):
    failures = run_all_safety_steps(
        (
            (
                f"disable motor torque on {motor}",
                lambda motor=motor: bus.disable_torque(motor, num_retry=TORQUE_RETRIES),
            )
            for motor in bus.motors
        )
    )
    if failures:
        raise RuntimeError("; ".join(f"{name}: {error}" for name, error in failures))


@dataclass
class TorqueSafetyConfig:
    port_zmq: int = 5557
    state_file: str = "~/.ros/lekiwi/servo_torque_state"
    bind_address: str = "0.0.0.0"
    # False: command silence stops the base and freezes the arm with torque left on; the
    # driver re-arms itself. True (larger robots): it cuts all servo torque.
    disarm_on_failure: bool = False


@dataclass
class CurveServerConfig:
    server_secret_key_file: str = ""
    authorized_clients_dir: str = ""


@dataclass
class TorqueHostConfig:
    robot: LeKiwiConfig = field(default_factory=LeKiwiConfig)
    host: LeKiwiHostConfig = field(default_factory=LeKiwiHostConfig)
    safety: TorqueSafetyConfig = field(default_factory=TorqueSafetyConfig)
    curve: CurveServerConfig = field(default_factory=CurveServerConfig)


class BoundLeKiwiHost:
    """Vendor-compatible sockets bound to one configured control interface."""

    def __init__(
        self, config: LeKiwiHostConfig, bind_address: str,
        curve: CurveServerConfig | None = None,
    ):
        address = validated_bind_address(bind_address)
        curve = curve or CurveServerConfig()
        self.zmq_context = zmq.Context()
        self.security = None
        self.zmq_cmd_socket = None
        self.zmq_observation_socket = None
        try:
            self.security = CurveServerSecurity(
                self.zmq_context, curve.server_secret_key_file,
                curve.authorized_clients_dir,
            )
            self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
            self.zmq_cmd_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
            configure_link_liveness(self.zmq_cmd_socket, zmq)
            self.security.configure_socket(self.zmq_cmd_socket)
            self.zmq_cmd_socket.bind(f"tcp://{address}:{config.port_zmq_cmd}")
            self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
            self.zmq_observation_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_observation_socket.setsockopt(zmq.SNDHWM, 2)
            configure_link_liveness(self.zmq_observation_socket, zmq)
            self.security.configure_socket(self.zmq_observation_socket)
            self.zmq_observation_socket.bind(f"tcp://{address}:{config.port_zmq_observations}")
        except Exception:
            self.disconnect()
            raise
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        if self.zmq_observation_socket is not None:
            self.zmq_observation_socket.close()
            self.zmq_observation_socket = None
        if self.zmq_cmd_socket is not None:
            self.zmq_cmd_socket.close()
            self.zmq_cmd_socket = None
        if self.security is not None:
            self.security.close()
            self.security = None
        self.zmq_context.term()


class TorqueLatch:
    """Record the last confirmed torque state in a file an operator can inspect.

    The host never reads it back: a process or machine restart is never
    permission to restore actuator energy, so the host always starts
    torque-off and the ROS driver arms it explicitly once it has complete,
    fresh telemetry.
    """

    def __init__(self, path: str):
        self.path = Path(path).expanduser()

    def save(self, enabled: bool) -> None:
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.tmp")
        temporary.write_text("enabled\n" if enabled else "disabled\n", encoding="ascii")
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.path)


class SafetyLeKiwi(LeKiwi):
    """Configure the vendor robot but leave torque off until an explicit arm request."""

    def configure(self):
        # This is LeRobot 0.6.1's LeKiwi.configure() without its final
        # enable_torque(). Keep its modes and gains identical to the supported
        # vendor implementation.
        broadcast_torque_off(self.bus)
        TorqueControlServer._verify_torque(self, False)
        self.bus.sync_write("Lock", 0, num_retry=TORQUE_RETRIES)
        self.bus.configure_motors()
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus.write("P_Coefficient", name, 16)
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 32)
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)


class TorqueControlServer:
    def __init__(
        self, context, config: TorqueSafetyConfig, latch: TorqueLatch,
        security: CurveServerSecurity,
    ):
        if not 1 <= config.port_zmq <= 65535:
            raise ValueError("safety.port_zmq must be between 1 and 65535")
        self.socket = context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        security.configure_socket(self.socket)
        self.socket.bind(f"tcp://{validated_bind_address(config.bind_address)}:{config.port_zmq}")
        self.latch = latch
        self.torque_enabled = False

    def disconnect(self):
        self.socket.close()

    @staticmethod
    def _hold_present_arm_position(robot: SafetyLeKiwi) -> None:
        positions = robot.bus.sync_read("Present_Position", robot.arm_motors, num_retry=TORQUE_RETRIES)
        robot.bus.sync_write("Goal_Position", positions, num_retry=TORQUE_RETRIES)
        robot.stop_base()

    @staticmethod
    def _verify_torque(robot: SafetyLeKiwi, enabled: bool) -> None:
        states = robot.bus.sync_read(
            "Torque_Enable", list(robot.bus.motors), normalize=False,
            num_retry=TORQUE_RETRIES,
        )
        if not torque_readback_matches(states, enabled, robot.bus.motors):
            raise RuntimeError(
                f"servo torque readback did not confirm every motor {'enabled' if enabled else 'disabled'}"
            )

    def _enable(self, robot: SafetyLeKiwi) -> None:
        if self.torque_enabled:
            # A failed cut can leave this cached flag true with only some
            # motors energized; never acknowledge an arm from the flag alone.
            self._verify_torque(robot, True)
            return
        # A torque-off arm may have sagged. Never re-enable against its old
        # target: first command each arm joint to its measured current position
        # and command zero wheel velocity, then apply torque.
        self._hold_present_arm_position(robot)
        enable_with_rollback(
            (
                ("enable motor torque", lambda: robot.bus.enable_torque(num_retry=TORQUE_RETRIES)),
                ("verify motor torque enabled", lambda: self._verify_torque(robot, True)),
                ("persist enabled latch", lambda: self.latch.save(True)),
            ),
            (("disable motor torque", lambda: self._disable(robot)),),
        )
        self.torque_enabled = True

    def _disable(self, robot: SafetyLeKiwi) -> None:
        # Attempt the physical cut even if stopping fails; record "disabled"
        # only after every servo confirms its torque register is zero.
        failures = run_all_safety_steps((
            ("stop base", robot.stop_base),
            ("disable motor torque", lambda: broadcast_torque_off(robot.bus)),
            ("verify motor torque disabled", lambda: self._verify_torque(robot, False)),
        ))
        if not any(name == "verify motor torque disabled" for name, _error in failures):
            self.torque_enabled = False
            failures.extend(run_all_safety_steps((
                ("persist disabled latch", lambda: self.latch.save(False)),
            )))
        if failures:
            raise RuntimeError("; ".join(f"{name}: {error}" for name, error in failures))

    def process_one(self, robot: SafetyLeKiwi) -> str | None:
        try:
            request = self.socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        except Exception as error:
            self.socket.send_json({"ok": False, "error": f"invalid request: {error}"})
            return None

        try:
            command = request.get("command") if isinstance(request, dict) else None
            if command == "enable":
                self._enable(robot)
            elif command == "disable":
                self._disable(robot)
            elif command != "state":
                raise ValueError("command must be enable, disable, or state")
            self.socket.send_json({"ok": True, "torque_enabled": self.torque_enabled})
            return command
        except Exception as error:
            logging.exception("Torque-control request failed")
            self.socket.send_json({"ok": False, "error": str(error), "torque_enabled": self.torque_enabled})
            return None


def cut_torque_for_shutdown(robot: SafetyLeKiwi) -> None:
    try:
        broadcast_torque_off(robot.bus)
        TorqueControlServer._verify_torque(robot, False)
    except Exception as error:
        try:
            disable_torque_per_motor(robot.bus)
        except Exception as fallback_error:
            logging.error(
                "Broadcast torque cut failed (%s); per-motor fallback failed: %s",
                error, fallback_error,
            )
        raise


class MotorHealthCollector:
    """Read only health collector run by the process that already owns the bus."""

    def __init__(self, robot: SafetyLeKiwi):
        self.motors = tuple(robot.bus.motors)
        self._last_read = 0.0
        self._snapshot = fault_snapshot(self.motors, "health read has not completed")
        self._last_error = None
        self._limits = None

    def _read_limits(self, robot: SafetyLeKiwi) -> None:
        """Read the servo-programmed protective limits once after connection."""
        limits = {}
        for register in ("Max_Temperature_Limit", "Min_Voltage_Limit", "Max_Voltage_Limit"):
            values = robot.bus.sync_read(register, list(self.motors), normalize=False, num_retry=TORQUE_RETRIES)
            if not isinstance(values, dict) or set(values) != set(self.motors):
                raise RuntimeError(f"incomplete {register} readback")
            limits[register] = {motor: int(values[motor]) for motor in self.motors}
        self._limits = limits

    def collect(self, robot: SafetyLeKiwi, torque_enabled: bool) -> dict:
        """Return the last bounded-rate readback, failing closed on every error.

        The STS3215 register names and units are supplied by the installed
        LeRobot control table. Values that need a robot-specific operating
        envelope remain advisory until bench-qualified.
        """
        now = time.monotonic()
        if now - self._last_read < HEALTH_PERIOD_S:
            return self._snapshot
        self._last_read = now
        try:
            if self._limits is None:
                self._read_limits(robot)
            torque = robot.bus.sync_read(
                "Torque_Enable", list(self.motors), normalize=False, num_retry=TORQUE_RETRIES,
            )
            positions = robot.bus.sync_read(
                "Present_Position", list(self.motors), num_retry=TORQUE_RETRIES,
            )
            feedback = {
                "Present_Load": robot.bus.sync_read("Present_Load", list(self.motors), normalize=False, num_retry=TORQUE_RETRIES),
                "Present_Voltage": robot.bus.sync_read("Present_Voltage", list(self.motors), normalize=False, num_retry=TORQUE_RETRIES),
                "Present_Temperature": robot.bus.sync_read("Present_Temperature", list(self.motors), normalize=False, num_retry=TORQUE_RETRIES),
                "Status": robot.bus.sync_read("Status", list(self.motors), normalize=False, num_retry=TORQUE_RETRIES),
                "Present_Current": robot.bus.sync_read("Present_Current", list(self.motors), normalize=False, num_retry=TORQUE_RETRIES),
            }
            all_readbacks = {"Torque_Enable": torque, "Present_Position": positions, **feedback}
            for register, values in all_readbacks.items():
                if not isinstance(values, dict) or set(values) != set(self.motors):
                    raise RuntimeError(f"incomplete {register} readback")
            details = {}
            warnings = {}
            for motor in self.motors:
                reported_torque = torque[motor]
                if isinstance(reported_torque, bool) or int(reported_torque) not in (0, 1):
                    raise RuntimeError(f"invalid torque readback from {motor}")
                if bool(reported_torque) != bool(torque_enabled):
                    detail = "torque readback differs from safety latch"
                    if detail != self._last_error:
                        logging.warning("%s: %s", detail, motor)
                        self._last_error = detail
                    self._snapshot = fault_snapshot(
                        self.motors, detail, failed_motor=motor,
                    )
                    return self._snapshot
                position = float(positions[motor])
                if not math.isfinite(position):
                    raise RuntimeError(f"non-finite present position from {motor}")
                load = float(feedback["Present_Load"][motor])
                voltage_raw = int(feedback["Present_Voltage"][motor])
                temperature = int(feedback["Present_Temperature"][motor])
                current_raw = int(feedback["Present_Current"][motor])
                status_raw = int(feedback["Status"][motor])
                if not math.isfinite(load) or not 0 <= voltage_raw <= 255 or not 0 <= temperature <= 255:
                    raise RuntimeError(f"invalid electrical feedback from {motor}")
                if status_raw != 0:
                    detail = f"servo Status register is nonzero ({status_raw})"
                    if detail != self._last_error:
                        logging.warning("%s: %s", detail, motor)
                        self._last_error = detail
                    self._snapshot = fault_snapshot(self.motors, detail, failed_motor=motor)
                    return self._snapshot
                minimum_voltage = self._limits["Min_Voltage_Limit"][motor] / 10.0
                maximum_voltage = self._limits["Max_Voltage_Limit"][motor] / 10.0
                voltage = voltage_raw / 10.0
                maximum_temperature = self._limits["Max_Temperature_Limit"][motor]
                if voltage <= minimum_voltage or voltage >= maximum_voltage:
                    warnings[motor] = "supply voltage is at or beyond the configured servo limit"
                elif temperature >= maximum_temperature:
                    warnings[motor] = "temperature is at or beyond the configured servo limit"
                details[motor] = {
                    "present_position": position,
                    "torque_enabled": bool(reported_torque),
                    "present_load_raw": int(load),
                    "present_load_duty_cycle": load / 1000.0,
                    "present_voltage_v": voltage,
                    "present_temperature_c": temperature,
                    "present_current_raw": current_raw,
                    "present_current_ma": current_raw * 6.5,
                    "status_raw": status_raw,
                    "minimum_voltage_v": minimum_voltage,
                    "maximum_voltage_v": maximum_voltage,
                    "maximum_temperature_c": maximum_temperature,
                }
            self._snapshot = healthy_snapshot(
                self.motors, torque_enabled, detail=details, warnings=warnings,
            )
            if self._last_error is not None:
                logging.info("Motor-health read recovered")
                self._last_error = None
        except Exception as error:
            detail = f"motor-health read failed: {error}"
            if detail != self._last_error:
                logging.warning("%s", detail)
                self._last_error = detail
            self._snapshot = fault_snapshot(self.motors, detail)
        return self._snapshot


class HostLoop:
    """One pass of the motor host: motion commands, torque requests, the command
    watchdog, and the telemetry that reports all of them."""

    def __init__(
        self, robot: SafetyLeKiwi, host: BoundLeKiwiHost, control: TorqueControlServer,
        health: MotorHealthCollector, disarm_on_failure: bool, clock=time.monotonic,
    ):
        self.robot = robot
        self.host = host
        self.control = control
        self.health = health
        self.disarm_on_failure = disarm_on_failure
        self.clock = clock
        self.last_cmd_time = clock()
        self.watchdog_active = False
        self.next_watchdog_attempt = 0.0
        self.telemetry_session = uuid.uuid4().hex
        self.telemetry_sequence = 0
        self._last_command_error = None

    def step(self) -> None:
        self.receive_command()
        self.handle_control_request()
        self.enforce_watchdog()
        self.publish_observation()

    def receive_command(self) -> None:
        try:
            message = self.host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
            action = validate_action_payload(message, ACTION_KEYS)
            if not self.control.torque_enabled:
                raise RuntimeError("servo torque is disabled")
            self.robot.send_action(action)
        except zmq.Again:
            # Between commands is the normal state; silence past the watchdog
            # timeout is handled by enforce_watchdog().
            return
        except Exception as error:
            # A client repeating one malformed command would otherwise log at the
            # loop rate. Report each distinct failure once, until a command succeeds.
            detail = f"{type(error).__name__}: {error}"
            if detail != self._last_command_error:
                logging.error("Motion command rejected: %s", detail)
                self._last_command_error = detail
            return
        self._last_command_error = None
        self.last_cmd_time = self.clock()
        self.watchdog_active = False

    def handle_control_request(self) -> None:
        command = self.control.process_one(self.robot)
        if command == "enable":
            # _enable() held the measured arm pose and stopped the base. Treat
            # that physical hold as the start of a short grace period in which
            # the newly armed driver must submit a fresh action.
            self.last_cmd_time = self.clock()
            self.watchdog_active = False
        elif command == "disable":
            self.watchdog_active = True

    def enforce_watchdog(self) -> None:
        now = self.clock()
        if (
            now - self.last_cmd_time <= self.host.watchdog_timeout_ms / 1000
            or self.watchdog_active
            or now < self.next_watchdog_attempt
        ):
            return
        if not self.control.torque_enabled:
            self.watchdog_active = True
            return
        self.next_watchdog_attempt = now + 0.25
        try:
            outcome = react_to_command_silence(
                self.disarm_on_failure,
                lambda: self.control._disable(self.robot),
                lambda: self.control._hold_present_arm_position(self.robot),
            )
        except Exception:
            # A failed bus transaction is retried at a bounded rate; it must not
            # permanently suppress the host's autonomous fail-safe.
            logging.exception("Command watchdog action was not confirmed")
            return
        logging.warning(
            "Command watchdog elapsed; %s",
            "cut all servo torque" if outcome == "cut"
            else "stopped the base and froze the arm, torque unchanged",
        )
        self.watchdog_active = True

    def publish_observation(self) -> None:
        observation = self.robot.get_observation()
        sample_monotonic_ns = time.monotonic_ns()
        motor_health = self.health.collect(self.robot, self.control.torque_enabled)
        camera_keys = list(self.robot.cameras.keys())
        jpeg_frames = []
        for camera_key in camera_keys:
            valid, jpeg = cv2.imencode(
                ".jpg", observation.pop(camera_key), [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            )
            jpeg_frames.append(jpeg if valid else b"")
        payload = {
            "_cams": camera_keys,
            **observation,
            TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
            TELEMETRY_SESSION_KEY: self.telemetry_session,
            TELEMETRY_SEQUENCE_KEY: self.telemetry_sequence,
            TELEMETRY_MONOTONIC_NS_KEY: sample_monotonic_ns,
            TELEMETRY_TORQUE_ENABLED_KEY: self.control.torque_enabled,
            "_lekiwi_motor_health": motor_health,
        }
        try:
            self.host.zmq_observation_socket.send_multipart(
                [json.dumps(payload).encode()] + jpeg_frames, flags=zmq.NOBLOCK,
            )
        except zmq.Again:
            logging.info("Dropping observation, no client connected")
        self.telemetry_sequence += 1


def _shutdown_signal(_signum, _frame):
    # Translate systemd's SIGTERM so the finally block cuts and verifies torque.
    raise KeyboardInterrupt


def connect_when_servos_powered(robot: SafetyLeKiwi) -> None:
    """Wait on one servo without restarting this LeRobot process every few seconds."""
    robot.bus.connect(handshake=False)
    try:
        while robot.bus.ping("arm_shoulder_pan") is None:
            logging.info("Waiting for servo power")
            time.sleep(1)
    finally:
        robot.bus.disconnect(disable_torque=False)
    robot.connect()


@draccus.wrap()
def main(cfg: TorqueHostConfig):
    latch = TorqueLatch(cfg.safety.state_file)
    robot = SafetyLeKiwi(cfg.robot)
    host = None
    control = None
    signal.signal(signal.SIGTERM, _shutdown_signal)
    signal.signal(signal.SIGHUP, _shutdown_signal)
    try:
        logging.info("Waiting for servo power")
        connect_when_servos_powered(robot)
        # configure() leaves torque off, but a write returning successfully
        # is not proof. Reissue it and require every servo's register readback
        # before opening any network control endpoint.
        broadcast_torque_off(robot.bus)
        TorqueControlServer._verify_torque(robot, False)
        latch.save(False)
        host = BoundLeKiwiHost(cfg.host, cfg.safety.bind_address, cfg.curve)
        control = TorqueControlServer(host.zmq_context, cfg.safety, latch, host.security)
        loop = HostLoop(robot, host, control, MotorHealthCollector(robot), cfg.safety.disarm_on_failure)
        while True:
            loop_start = time.monotonic()
            loop.step()
            time.sleep(max(1 / host.max_loop_freq_hz - (time.monotonic() - loop_start), 0))
    except KeyboardInterrupt:
        logging.info("Stopping LeKiwi torque host")
    finally:
        torque_cut = False
        try:
            if robot.is_connected:
                cut_torque_for_shutdown(robot)
                torque_cut = True
        finally:
            try:
                if robot.is_connected:
                    # LeRobot.disconnect() writes zero wheel goals, which
                    # re-enables Feetech torque after our verified cut.
                    try:
                        robot.bus.disconnect(disable_torque=not torque_cut)
                    finally:
                        for camera in robot.cameras.values():
                            camera.disconnect()
            finally:
                if control is not None:
                    control.disconnect()
                if host is not None:
                    host.disconnect()


if __name__ == "__main__":
    main()
