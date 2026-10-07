"""Bus-independent core of the LeKiwi motor host.

``scripts/torque-host.py`` runs this against LeRobot's LeKiwi and its Feetech
bus; ``lekiwi_rmf.fake_host`` runs it against an in-memory bus. Both therefore
enforce the same torque, motion-envelope, lease and watchdog rules.

The robot is the subset of LeRobot's ``LeKiwi`` the host needs: ``bus``
(``motors``, ``sync_read``, ``sync_write``, ``enable_torque``,
``disable_torque``), ``arm_motors``, ``cameras``, ``get_observation()``,
``send_action()`` and ``stop_base()``. The transport (``host``) provides the
bound ``zmq_cmd_socket`` (PULL), ``zmq_observation_socket`` (PUSH),
``torque_socket`` (REP) and ``watchdog_timeout_ms``.
"""

import json
import logging
import math
import time
import uuid

import zmq

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS, joint_positions
from lekiwi_rmf.host_protocol import (
    ARM_LEASE_KEYS, STATE_KEYS, TorqueCommand, observation_payload, valid_goal_id,
)
from lekiwi_rmf.local_arm_executor import LocalArmExecutor
from lekiwi_rmf.motor_health import fault_snapshot, healthy_snapshot
from lekiwi_rmf.odometry import HostOdometry
from lekiwi_rmf.torque_control import (
    enable_with_rollback, react_to_command_silence, run_all_safety_steps,
    torque_readback_matches, validate_action_payload,
)


TORQUE_RETRIES = 5
# Spread grouped register reads across host cycles; one burst per snapshot can
# starve the position loop on the shared Feetech bus.
HEALTH_READ_PERIOD_S = 0.10
# State and the full per-servo diagnostic snapshot share the narrow Pi uplink.
# Ten hertz keeps feedback fresh while leaving command and watchdog handling at 30 Hz.
OBSERVATION_PERIOD_S = 0.10
# An unconfirmed watchdog action is retried no more often than this.
WATCHDOG_RETRY_S = 0.25
HEALTH_READS = (
    ("Present_Position", True),
    # Read back the register the servo actually received so tracking failures
    # can be separated from command transport or calibration errors.
    ("Goal_Position", True),
    ("Present_Load", False),
    ("Present_Voltage", False),
    ("Present_Temperature", False),
    ("Present_Current", False),
    ("Status", False),
    ("Torque_Enable", False),
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


def verify_torque(robot, enabled: bool) -> None:
    """Require every servo's torque register to read back the requested state."""
    states = robot.bus.sync_read(
        "Torque_Enable", list(robot.bus.motors), normalize=False,
        num_retry=TORQUE_RETRIES,
    )
    if not torque_readback_matches(states, enabled, robot.bus.motors):
        raise RuntimeError(
            f"servo torque readback did not confirm every motor {'enabled' if enabled else 'disabled'}"
        )


def hold_present_arm_position(robot) -> None:
    positions = robot.bus.sync_read("Present_Position", robot.arm_motors, num_retry=TORQUE_RETRIES)
    robot.bus.sync_write("Goal_Position", positions, num_retry=TORQUE_RETRIES)
    robot.stop_base()


def cut_torque_for_shutdown(robot) -> None:
    try:
        broadcast_torque_off(robot.bus)
        verify_torque(robot, False)
    except Exception as error:
        try:
            disable_torque_per_motor(robot.bus)
        except Exception as fallback_error:
            logging.error(
                "Broadcast torque cut failed (%s); per-motor fallback failed: %s",
                error, fallback_error,
            )
        raise


def validate_motion_action(action, base_limits, base_scales, calibration, held_positions=None):
    """Enforce the tracked envelope in calibrated units at the motor boundary.

    A previously measured hold may lie outside a joint limit after torque-off
    sag. Retaining that exact target is permitted; a new target outside is not.
    """
    if any(key not in action or type(action[key]) not in (int, float)
           or not math.isfinite(action[key]) for key in STATE_KEYS):
        raise ValueError("motion action must contain every finite motor command")
    linear, angular = base_limits
    xy_scale, yaw_scale = base_scales
    if (math.hypot(action["x.vel"], action["y.vel"]) * xy_scale > linear + 1e-9
            or abs(math.radians(action["theta.vel"]) * yaw_scale) > angular + 1e-9):
        raise ValueError("motion action exceeds the configured base speed limits")
    if not 0 <= action["arm_gripper.pos"] <= 100:
        raise ValueError("motion action exceeds the gripper's 0-100 range")
    positions = joint_positions(action, *calibration)
    for name, value in positions.items():
        if name == "arm_gripper":
            continue  # Its calibrated reachable range is the normalized servo range above.
        lower, upper = JOINT_LIMITS[name]
        key = f"{name}.pos"
        if not lower - 1e-9 <= value <= upper + 1e-9 and (
            held_positions is None or action[key] != held_positions.get(key)
        ):
            raise ValueError(f"motion action exceeds {name} position limits")


class TorqueControlServer:
    """The torque REP endpoint: explicit arm/disarm and local arm-goal requests."""

    def __init__(self, socket, arm_executor: LocalArmExecutor, arm_calibration, host_session: str):
        self.socket = socket
        self.arm_executor = arm_executor
        self.arm_calibration = arm_calibration
        self.host_session = host_session
        self.torque_enabled = False

    def enable(self, robot) -> None:
        if self.torque_enabled:
            # A failed cut can leave this cached flag true with only some
            # motors energized; never acknowledge an arm from the flag alone.
            verify_torque(robot, True)
            return
        # A torque-off arm may have sagged. Never re-enable against its old
        # target: first command each arm joint to its measured current position
        # and command zero wheel velocity, then apply torque.
        hold_present_arm_position(robot)
        enable_with_rollback(
            (
                ("enable motor torque", lambda: robot.bus.enable_torque(num_retry=TORQUE_RETRIES)),
                ("verify motor torque enabled", lambda: verify_torque(robot, True)),
            ),
            (("disable motor torque", lambda: self.disable(robot)),),
        )
        self.torque_enabled = True

    def disable(self, robot) -> None:
        # Attempt the physical cut even if stopping fails; record "disabled"
        # only after every servo confirms its torque register is zero.
        failures = run_all_safety_steps((
            ("stop base", robot.stop_base),
            ("disable motor torque", lambda: broadcast_torque_off(robot.bus)),
            ("verify motor torque disabled", lambda: verify_torque(robot, False)),
        ))
        if not any(name == "verify motor torque disabled" for name, _error in failures):
            self.torque_enabled = False
        if failures:
            raise RuntimeError("; ".join(f"{name}: {error}" for name, error in failures))

    def process_one(self, robot) -> str | None:
        try:
            request = self.socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        except Exception as error:
            self.socket.send_json({"ok": False, "error": f"invalid request: {error}"})
            return None

        try:
            command = request.get("command") if isinstance(request, dict) else None
            if command == TorqueCommand.ENABLE:
                self.enable(robot)
            elif command == TorqueCommand.DISABLE:
                self.disable(robot)
                self.arm_executor.cancel()
                self.arm_executor.hold = {}
            elif command == TorqueCommand.TRAJECTORY_START:
                if not self.torque_enabled or request.get("session") != self.host_session:
                    raise ValueError("trajectory requires enabled torque and the current host session")
                trajectory = request["trajectory"]
                if (trajectory["zeros"], trajectory["directions"]) != self.arm_calibration:
                    raise ValueError("trajectory calibration differs from the motor host")
                # Stop wheels before validation, without an extra arm read/write
                # transaction that delays the host's regular feedback publication.
                robot.stop_base()
                observation = robot.get_observation()
                status = self.arm_executor.start(request["trajectory"], observation)
                self.socket.send_json({"ok": True, "trajectory": status})
                return command
            elif command == TorqueCommand.TRAJECTORY_CANCEL:
                if request.get("session") != self.host_session:
                    raise ValueError("trajectory cancellation belongs to another host session")
                if self.arm_executor.status is None or request.get("id") != self.arm_executor.status["id"]:
                    raise ValueError("trajectory cancellation belongs to another arm goal")
                self.arm_executor.cancel(request.get("id"))
                hold_present_arm_position(robot)
                measured = robot.get_observation()
                self.arm_executor.hold = {f"{name}.pos": measured[f"{name}.pos"] for name in ARM_JOINTS}
            elif command != TorqueCommand.STATE:
                raise ValueError("command must be enable, disable, or state")
            response = {"ok": True, "torque_enabled": self.torque_enabled}
            if command == TorqueCommand.STATE:
                # Read-only observers must use REP. A second telemetry PULL
                # steals samples from the driver's point-to-point PUSH stream.
                response["trajectory"] = self.arm_executor.status
            self.socket.send_json(response)
            return command
        except Exception as error:
            logging.exception("Torque-control request failed")
            self.socket.send_json({"ok": False, "error": str(error), "torque_enabled": self.torque_enabled})
            return None


class MotorHealthCollector:
    """Read only health collector run by the process that already owns the bus."""

    def __init__(self, robot):
        self.motors = tuple(robot.bus.motors)
        self._last_read = 0.0
        self._snapshot = fault_snapshot(self.motors, "health read has not completed")
        self._last_error = None
        self._limits = None
        self._read_index = 0
        self._readbacks = {}

    def _read_limits(self, robot) -> None:
        """Read the servo-programmed protective limits once after connection."""
        limits = {}
        for register in ("Max_Temperature_Limit", "Min_Voltage_Limit", "Max_Voltage_Limit"):
            values = robot.bus.sync_read(register, list(self.motors), normalize=False, num_retry=TORQUE_RETRIES)
            if not isinstance(values, dict) or set(values) != set(self.motors):
                raise RuntimeError(f"incomplete {register} readback")
            limits[register] = {motor: int(values[motor]) for motor in self.motors}
        self._limits = limits

    def collect(self, robot, torque_enabled: bool) -> dict:
        """Return the last bounded-rate readback, failing closed on every error.

        The STS3215 register names and units are supplied by the installed
        LeRobot control table. Values that need a robot-specific operating
        envelope remain advisory until bench-qualified.
        """
        now = time.monotonic()
        if now - self._last_read < HEALTH_READ_PERIOD_S:
            return self._snapshot
        self._last_read = now
        try:
            if self._limits is None:
                self._read_limits(robot)
            register, normalize = HEALTH_READS[self._read_index]
            values = robot.bus.sync_read(
                register, list(self.motors), normalize=normalize,
                num_retry=TORQUE_RETRIES,
            )
            if not isinstance(values, dict) or set(values) != set(self.motors):
                raise RuntimeError(f"incomplete {register} readback")
            self._readbacks[register] = values
            self._read_index += 1
            if self._read_index < len(HEALTH_READS):
                return self._snapshot

            all_readbacks = self._readbacks
            self._readbacks = {}
            self._read_index = 0
            torque = all_readbacks["Torque_Enable"]
            positions = all_readbacks["Present_Position"]
            goals = all_readbacks["Goal_Position"]
            feedback = {
                key: all_readbacks[key] for key in (
                    "Present_Load", "Present_Voltage", "Present_Temperature",
                    "Present_Current", "Status",
                )
            }
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
                goal = float(goals[motor])
                if not math.isfinite(position) or not math.isfinite(goal):
                    raise RuntimeError(f"non-finite position readback from {motor}")
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
                    "goal_position": goal,
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
            self._readbacks = {}
            self._read_index = 0
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
        self, robot, host, health, *, disarm_on_failure: bool, arm_calibration,
        base_limits, base_scales, encode_camera, observation_period_s=OBSERVATION_PERIOD_S,
        clock=time.monotonic,
    ):
        self.robot = robot
        self.host = host
        self.health = health
        self.disarm_on_failure = disarm_on_failure
        self.arm_calibration = arm_calibration
        self.base_limits = base_limits
        self.encode_camera = encode_camera
        self.observation_period_s = observation_period_s
        self.clock = clock
        self.last_cmd_time = clock()
        self.watchdog_active = False
        self.next_watchdog_attempt = 0.0
        self.telemetry_session = uuid.uuid4().hex
        self.arm_executor = LocalArmExecutor(clock, host.watchdog_timeout_ms / 1000)
        self.control = TorqueControlServer(
            host.torque_socket, self.arm_executor, arm_calibration, self.telemetry_session,
        )
        self.local_observation = None
        self.local_feedback_at = None
        self.telemetry_sequence = 0
        self.odometry = HostOdometry(*base_scales)
        self.next_observation_at = None
        self._last_command_error = None
        self._last_step_started = None
        self._last_slow_log = 0.0

    def step(self) -> None:
        started = time.perf_counter()
        self.receive_command()
        command_done = time.perf_counter()
        self.handle_control_request()
        control_done = time.perf_counter()
        self.enforce_watchdog()
        watchdog_done = time.perf_counter()
        now = self.clock()
        if self.next_observation_at is None or now >= self.next_observation_at:
            self.publish_observation()
            self.next_observation_at = now + self.observation_period_s
        if self.control.torque_enabled and self.arm_executor.active:
            action = self.arm_executor.step(self.local_observation, self.local_feedback_at)
            if action is not None:
                self.send_action(action)
        finished = time.perf_counter()
        gap = 0.0 if self._last_step_started is None else started - self._last_step_started
        self._last_step_started = started
        if (gap > 0.2 or finished - started > 0.15) and started - self._last_slow_log > 1.0:
            logging.warning(
                "Motor-host loop delay: gap=%.3fs command=%.3fs control=%.3fs "
                "watchdog=%.3fs observation=%.3fs",
                gap, command_done - started, control_done - command_done,
                watchdog_done - control_done, finished - watchdog_done,
            )
            self._last_slow_log = started

    def send_action(self, action) -> None:
        validate_motion_action(
            action, self.base_limits, self.odometry.scales, self.arm_calibration,
            self.arm_executor.hold or self.local_observation,
        )
        self.robot.send_action(action)

    def receive_command(self) -> None:
        try:
            message = self.host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
            action = validate_action_payload(message, STATE_KEYS, ARM_LEASE_KEYS)
            if not self.control.torque_enabled:
                raise RuntimeError("servo torque is disabled")
            goal_id = action.pop(ARM_LEASE_KEYS[0], None)
            permission = action.pop(ARM_LEASE_KEYS[1], None)
            if (goal_id is None) != (permission is None) or (
                goal_id is not None and (
                    not goal_id.is_integer() or not valid_goal_id(int(goal_id)) or permission not in (0, 1)
                )
            ):
                raise ValueError("invalid arm trajectory lease")
            if goal_id is not None:
                self.arm_executor.renew(int(goal_id), bool(permission))
            # While a local goal owns the arm, legacy streamed setpoints cannot
            # overwrite it. The retained final target also survives reply latency.
            if not self.arm_executor.active:
                action.update(self.arm_executor.hold)
                self.send_action(action)
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
        if command == TorqueCommand.ENABLE:
            # enable() held the measured arm pose and stopped the base. Treat
            # that physical hold as the start of a short grace period in which
            # the newly armed driver must submit a fresh action.
            self.last_cmd_time = self.clock()
            self.watchdog_active = False
        elif command == TorqueCommand.DISABLE:
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
        if self.arm_executor.status is not None:
            self.arm_executor.renew(self.arm_executor.status["id"], False)
        if self.disarm_on_failure:
            self.arm_executor.cancel()
            self.arm_executor.hold = {}
        self.next_watchdog_attempt = now + WATCHDOG_RETRY_S
        try:
            outcome = react_to_command_silence(
                self.disarm_on_failure,
                lambda: self.control.disable(self.robot),
                lambda: hold_present_arm_position(self.robot),
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
        self.local_observation = dict(observation)
        self.local_feedback_at = self.clock()
        sample_monotonic_ns = time.monotonic_ns()
        odometry = self.odometry.update((
            float(observation["x.vel"]), float(observation["y.vel"]),
            math.radians(float(observation["theta.vel"])),
        ), sample_monotonic_ns, time.time_ns())
        motor_health = self.health.collect(self.robot, self.control.torque_enabled)
        camera_keys = list(self.robot.cameras.keys())
        jpeg_frames = [self.encode_camera(observation.pop(camera_key)) for camera_key in camera_keys]
        payload = observation_payload(
            observation, camera_keys,
            session=self.telemetry_session,
            sequence=self.telemetry_sequence,
            sample_monotonic_ns=sample_monotonic_ns,
            torque_enabled=self.control.torque_enabled,
            motor_health=motor_health,
            odometry=odometry,
            arm_status=self.arm_executor.status,
        )
        try:
            self.host.zmq_observation_socket.send_multipart(
                [json.dumps(payload).encode()] + jpeg_frames, flags=zmq.NOBLOCK,
            )
        except zmq.Again:
            logging.info("Dropping observation, no client connected")
        self.telemetry_sequence += 1
