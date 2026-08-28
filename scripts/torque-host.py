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
    enable_with_rollback, run_all_safety_steps, torque_readback_matches,
    validate_action_payload, validated_bind_address,
)
from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.zmq_security import CurveServerSecurity
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


@dataclass
class TorqueSafetyConfig:
    port_zmq: int = 5557
    state_file: str = "~/.ros/lekiwi/servo_torque_state"
    bind_address: str = "0.0.0.0"


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
                self.zmq_context, address, curve.server_secret_key_file,
                curve.authorized_clients_dir,
            )
            self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
            self.zmq_cmd_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
            self.security.configure_socket(self.zmq_cmd_socket)
            self.zmq_cmd_socket.bind(f"tcp://{address}:{config.port_zmq_cmd}")
            self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
            self.zmq_observation_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_observation_socket.setsockopt(zmq.SNDHWM, 2)
            self.security.configure_socket(self.zmq_observation_socket)
            self.zmq_observation_socket.bind(f"tcp://{address}:{config.port_zmq_observations}")
        except Exception:
            self.disconnect()
            raise
        self.connection_time_s = config.connection_time_s
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
    """Persist an explicit disarm across a host crash or systemd restart."""

    def __init__(self, path: str):
        self.path = Path(path).expanduser()

    def initial_enabled(self) -> bool:
        try:
            state = self.path.read_text(encoding="ascii").strip()
        except FileNotFoundError:
            # No prior safety decision is never permission to energize motors.
            # The ROS driver will explicitly arm after it receives complete,
            # fresh telemetry and has sent its initial zero command.
            return False
        except OSError as error:
            logging.error("Cannot read torque latch %s: %s; keeping torque off", self.path, error)
            return False
        if state == "enabled":
            # A process or machine restart is never permission to restore
            # actuator energy. Keep the value only as crash evidence; the ROS
            # driver must receive healthy telemetry and an explicit arm request.
            logging.warning("Previous host exited while torque was enabled; restarting torque-off")
            try:
                self.save(False)
            except OSError as error:
                logging.error("Could not persist restart disarm latch: %s", error)
            return False
        if state == "disabled":
            return False
        logging.error("Invalid torque latch %s; keeping torque off", self.path)
        return False

    def save(self, enabled: bool) -> None:
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.tmp")
        temporary.write_text("enabled\n" if enabled else "disabled\n", encoding="ascii")
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.path)


class SafetyLeKiwi(LeKiwi):
    """Configure the vendor robot without overriding a persisted disarm latch."""

    def __init__(self, config: LeKiwiConfig, torque_enabled: bool):
        self.torque_enabled = torque_enabled
        super().__init__(config)

    def configure(self):
        # This is LeRobot 0.6.1's LeKiwi.configure(), with its final
        # enable_torque() guarded by the persisted safety latch. Keep its
        # modes and gains identical to the supported vendor implementation.
        self.bus.disable_torque()
        self.bus.configure_motors()
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus.write("P_Coefficient", name, 16)
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 32)
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        if self.torque_enabled:
            self.bus.enable_torque()


class TorqueControlServer:
    def __init__(
        self, context, config: TorqueSafetyConfig, latch: TorqueLatch,
        torque_enabled: bool, security: CurveServerSecurity,
    ):
        if not 1 <= config.port_zmq <= 65535:
            raise ValueError("safety.port_zmq must be between 1 and 65535")
        self.socket = context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        security.configure_socket(self.socket)
        self.socket.bind(f"tcp://{validated_bind_address(config.bind_address)}:{config.port_zmq}")
        self.latch = latch
        self.torque_enabled = torque_enabled

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
            (
                ("disable motor torque", lambda: robot.bus.disable_torque(num_retry=TORQUE_RETRIES)),
                ("verify motor torque disabled", lambda: self._verify_torque(robot, False)),
                ("persist disabled latch", lambda: self.latch.save(False)),
            ),
        )
        self.torque_enabled = True
        robot.torque_enabled = True

    def _disable(self, robot: SafetyLeKiwi) -> None:
        # Persistence, stopping, the bus write, and readback are independent
        # safety layers. A filesystem failure must never skip the physical cut.
        failures = run_all_safety_steps((
            ("persist disabled latch", lambda: self.latch.save(False)),
            ("stop base", robot.stop_base),
            ("disable motor torque", lambda: robot.bus.disable_torque(num_retry=TORQUE_RETRIES)),
            ("verify motor torque disabled", lambda: self._verify_torque(robot, False)),
        ))
        physical_failure = any(name in {
            "disable motor torque", "verify motor torque disabled",
        } for name, _error in failures)
        if not physical_failure:
            self.torque_enabled = False
            robot.torque_enabled = False
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


def _shutdown_signal(_signum, _frame):
    # LeRobot's stock main only runs its disconnect/finally path for a
    # KeyboardInterrupt. systemd uses SIGTERM, so translate it and guarantee
    # its configured disconnect disables torque as the process exits.
    raise KeyboardInterrupt


@draccus.wrap()
def main(cfg: TorqueHostConfig):
    latch = TorqueLatch(cfg.safety.state_file)
    initial_torque = latch.initial_enabled()
    robot = SafetyLeKiwi(cfg.robot, initial_torque)
    host = None
    control = None
    signal.signal(signal.SIGTERM, _shutdown_signal)
    signal.signal(signal.SIGHUP, _shutdown_signal)
    try:
        logging.info("Connecting LeKiwi (initial torque: %s)", "enabled" if initial_torque else "disabled")
        robot.connect()
        # configure() requests torque-off, but a write returning successfully
        # is not proof. Reissue it and require every servo's register readback
        # before opening any network control endpoint.
        robot.bus.disable_torque(num_retry=TORQUE_RETRIES)
        TorqueControlServer._verify_torque(robot, False)
        latch.save(False)
        host = BoundLeKiwiHost(cfg.host, cfg.safety.bind_address, cfg.curve)
        control = TorqueControlServer(
            host.zmq_context, cfg.safety, latch, initial_torque, host.security
        )
        health = MotorHealthCollector(robot)
        last_cmd_time = time.monotonic()
        watchdog_active = False
        next_watchdog_attempt = 0.0
        telemetry_session = uuid.uuid4().hex
        telemetry_sequence = 0
        start = time.perf_counter()
        while time.perf_counter() - start < host.connection_time_s:
            loop_start = time.monotonic()
            try:
                message = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                robot.send_action(validate_action_payload(message, ACTION_KEYS))
                last_cmd_time = time.monotonic()
                watchdog_active = False
            except zmq.Again:
                if not watchdog_active:
                    logging.warning("No command available")
            except Exception as error:
                logging.error("Message fetching failed: %s", error)

            control_command = control.process_one(robot)
            if control_command == "enable":
                # _enable() held the measured arm pose and stopped the base.
                # Treat that physical hold as the start of a short grace period
                # in which the newly armed driver must submit a fresh action.
                last_cmd_time = time.monotonic()
                watchdog_active = False
            elif control_command == "disable":
                watchdog_active = True

            watchdog_now = time.monotonic()
            if (
                watchdog_now - last_cmd_time > host.watchdog_timeout_ms / 1000
                and not watchdog_active
                and watchdog_now >= next_watchdog_attempt
            ):
                logging.warning("Command watchdog elapsed; cutting all servo torque")
                next_watchdog_attempt = watchdog_now + 0.25
                try:
                    control._disable(robot)
                    watchdog_active = True
                except Exception:
                    # A failed physical cut is retried at a bounded rate; one
                    # failed bus transaction must not permanently suppress the
                    # host's autonomous fail-safe.
                    logging.exception("Command watchdog could not confirm torque-off")

            observation = robot.get_observation()
            sample_monotonic_ns = time.monotonic_ns()
            motor_health = health.collect(robot, control.torque_enabled)
            camera_keys = list(robot.cameras.keys())
            jpeg_frames = []
            for camera_key in camera_keys:
                valid, jpeg = cv2.imencode(
                    ".jpg", observation.pop(camera_key), [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
                jpeg_frames.append(jpeg if valid else b"")
            try:
                payload = {
                    "_cams": camera_keys,
                    **observation,
                    TELEMETRY_PROTOCOL_KEY: TELEMETRY_PROTOCOL_VERSION,
                    TELEMETRY_SESSION_KEY: telemetry_session,
                    TELEMETRY_SEQUENCE_KEY: telemetry_sequence,
                    TELEMETRY_MONOTONIC_NS_KEY: sample_monotonic_ns,
                    TELEMETRY_TORQUE_ENABLED_KEY: control.torque_enabled,
                    "_lekiwi_motor_health": motor_health,
                }
                host.zmq_observation_socket.send_multipart(
                    [json.dumps(payload).encode()] + jpeg_frames,
                    flags=zmq.NOBLOCK,
                )
            except zmq.Again:
                logging.info("Dropping observation, no client connected")
            telemetry_sequence += 1
            time.sleep(max(1 / host.max_loop_freq_hz - (time.monotonic() - loop_start), 0))
    except KeyboardInterrupt:
        logging.info("Stopping LeKiwi torque host")
    finally:
        # disconnect() calls bus.disconnect(disable_torque=True) in the
        # supported LeRobot version. This remains the last line of defence for
        # a clean host shutdown, independently of the persisted latch.
        try:
            if robot.is_connected:
                robot.disconnect()
        finally:
            if control is not None:
                control.disconnect()
            if host is not None:
                host.disconnect()


if __name__ == "__main__":
    main()
