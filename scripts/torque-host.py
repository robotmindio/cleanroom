#!/usr/bin/env python3
"""LeRobot motor host with a repository-owned physical-torque safety channel.

The stock LeRobot host owns the serial bus but only exposes motion commands.
This process keeps its command/observation protocol intact and adds a separate
ZMQ REP endpoint for the ROS driver's explicit arm/disarm transactions. The
command, torque, watchdog and telemetry rules live in lekiwi_rmf.motor_host;
this script wires them to LeRobot's LeKiwi and its serial bus. The host opens
no cameras; ROS camera nodes own them.
"""

import logging
import math
import os
import signal
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import draccus
import zmq

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiConfig, LeKiwiHostConfig
from lerobot.robots.lekiwi.lekiwi import LeKiwi

from lekiwi_rmf.torque_control import validated_bind_address
from lekiwi_rmf.arm_trajectory import load_calibration
from lekiwi_rmf.motion_guards import load_base_speed_limits, load_base_test_profile
from lekiwi_rmf.host_protocol import BASE_TEST_STAGES
from lekiwi_rmf.motor_host import (
    TORQUE_RETRIES, HostLoop, MotorHealthCollector, broadcast_torque_off,
    cut_torque_for_shutdown, verify_torque,
)
from lekiwi_rmf.zmq_security import CurveServerSecurity, configure_link_liveness
from lekiwi_rmf.odometry import load_base_scales


# P=96 made the unsupported folded lift oscillate 0.0245 rad at a constant
# goal. Integral correction below handles static load error; soften its P loop.
ARM_P_COEFFICIENTS = {"arm_shoulder_lift": 32, "arm_elbow_flex": 64}
# Proportional control alone leaves a load-dependent position error. Use the
# smallest integral gain on gravity-loaded joints; keep jaw contact unchanged.
ARM_I_COEFFICIENTS = {"arm_shoulder_lift": 1, "arm_elbow_flex": 1, "arm_wrist_flex": 1}


@dataclass
class TorqueSafetyConfig:
    port_zmq: int = 5557
    bind_address: str = "0.0.0.0"
    arm_calibration_file: str = "~/.ros/lekiwi_arm_calibration.json"
    nav2_params_file: str = str(
        Path(__file__).resolve().parents[1] / "config/nav2_params.yaml"
        if (Path(__file__).resolve().parents[1] / "config/nav2_params.yaml").is_file()
        else Path(__file__).resolve().parents[2] / "share/lekiwi_rmf/config/nav2_params.yaml"
    )
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
    """Vendor-compatible motion sockets plus the torque REP socket, bound to one interface."""

    def __init__(
        self, config: LeKiwiHostConfig, safety: TorqueSafetyConfig,
        curve: CurveServerConfig | None = None,
    ):
        address = validated_bind_address(safety.bind_address)
        if not 1 <= safety.port_zmq <= 65535:
            raise ValueError("safety.port_zmq must be between 1 and 65535")
        curve = curve or CurveServerConfig()
        self.zmq_context = zmq.Context()
        self.security = None
        self.zmq_cmd_socket = None
        self.zmq_observation_socket = None
        self.torque_socket = None
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
            self.torque_socket = self.zmq_context.socket(zmq.REP)
            self.torque_socket.setsockopt(zmq.LINGER, 0)
            self.torque_socket.setsockopt(zmq.MAXMSGSIZE, 2 * 1024 * 1024)
            self.security.configure_socket(self.torque_socket)
            self.torque_socket.bind(f"tcp://{address}:{safety.port_zmq}")
        except Exception:
            self.disconnect()
            raise
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        for name in ("torque_socket", "zmq_observation_socket", "zmq_cmd_socket"):
            socket = getattr(self, name)
            if socket is not None:
                socket.close()
                setattr(self, name, None)
        if self.security is not None:
            self.security.close()
            self.security = None
        self.zmq_context.term()


class SafetyLeKiwi(LeKiwi):
    """Configure the vendor robot but leave torque off until an explicit arm request."""

    maximum_base_linear_speed_m_s = 0.0

    def _body_to_wheel_raw(self, x, y, theta, wheel_radius=.05, base_radius=.125, max_raw=None):
        # Vendor's 3000 tick/s cap clips 0.30 m/s; cover the validated pure translation ceiling.
        if max_raw is None:
            max_raw = max(3000, math.ceil(self.maximum_base_linear_speed_m_s / wheel_radius * 4096 / (2 * math.pi)))
        return super()._body_to_wheel_raw(x, y, theta, wheel_radius, base_radius, max_raw)

    def configure(self):
        # This is LeRobot 0.6.1's LeKiwi.configure() without its final
        # enable_torque(). Position gains are tuned for this robot's load.
        broadcast_torque_off(self.bus)
        verify_torque(self, False)
        self.bus.sync_write("Lock", 0, num_retry=TORQUE_RETRIES)
        self.bus.configure_motors()
        expected_gains = {"P_Coefficient": {}, "I_Coefficient": {}, "D_Coefficient": {}}
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            for register, value in (
                ("P_Coefficient", ARM_P_COEFFICIENTS.get(name, 16)),
                ("I_Coefficient", ARM_I_COEFFICIENTS.get(name, 0)),
                ("D_Coefficient", 32),
            ):
                self.bus.write(register, name, value)
                expected_gains[register][name] = value
        for register, expected in expected_gains.items():
            actual = self.bus.sync_read(register, self.arm_motors, normalize=False, num_retry=TORQUE_RETRIES)
            if actual != expected:
                raise RuntimeError(f"{register} readback differs from configured arm gains")
        logging.info("Arm position gains verified: %s", expected_gains)
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)


_shutdown_requested = False


def _shutdown_signal(_signum, _frame):
    # Finish an in-flight serial transaction before the torque-off readback.
    # Raising inside Feetech's port handler leaves its port-in-use flag set.
    global _shutdown_requested
    _shutdown_requested = True


def connect_when_servos_powered(robot: SafetyLeKiwi) -> None:
    """Wait on one servo without restarting this LeRobot process every few seconds."""
    robot.bus.connect(handshake=False)
    try:
        while not _shutdown_requested and robot.bus.ping("arm_shoulder_pan") is None:
            logging.info("Waiting for servo power")
            time.sleep(1)
    finally:
        robot.bus.disconnect(disable_torque=False)
    if not _shutdown_requested:
        robot.connect()


@draccus.wrap()
def main(cfg: TorqueHostConfig):
    global _shutdown_requested
    _shutdown_requested = False
    base_limits = load_base_speed_limits(cfg.safety.nav2_params_file)
    base_test_profiles = {stage: load_base_test_profile(cfg.safety.nav2_params_file, f'{stage:.2f}') for stage in BASE_TEST_STAGES}
    # LeRobot's LeKiwiConfig defaults to two cameras, and draccus rebuilds that
    # default. The motor host serves none; ROS camera nodes own the devices.
    robot = SafetyLeKiwi(replace(cfg.robot, cameras={}))
    robot.maximum_base_linear_speed_m_s = max(base_limits[0], *(p['linear_speed_m_s'] for p in base_test_profiles.values()))
    host = None
    signal.signal(signal.SIGTERM, _shutdown_signal)
    signal.signal(signal.SIGHUP, _shutdown_signal)
    try:
        logging.info("Waiting for servo power")
        connect_when_servos_powered(robot)
        if _shutdown_requested:
            raise KeyboardInterrupt
        # configure() leaves torque off, but a write returning successfully
        # is not proof. Reissue it and require every servo's register readback
        # before opening any network control endpoint.
        broadcast_torque_off(robot.bus)
        verify_torque(robot, False)
        if _shutdown_requested:
            raise KeyboardInterrupt
        host = BoundLeKiwiHost(cfg.host, cfg.safety, cfg.curve)
        loop = HostLoop(
            robot, host, MotorHealthCollector(robot),
            disarm_on_failure=cfg.safety.disarm_on_failure,
            arm_calibration=load_calibration(cfg.safety.arm_calibration_file),
            base_limits=base_limits,
            base_test_profiles=base_test_profiles,
            base_scales=load_base_scales(os.environ.get(
                "LEKIWI_LAUNCH_CALIBRATION", "~/.ros/lekiwi_launch_calibration.conf")),
        )
        while not _shutdown_requested:
            loop_start = time.monotonic()
            loop.step()
            time.sleep(max(1 / host.max_loop_freq_hz - (time.monotonic() - loop_start), 0))
        logging.info("Stopping LeKiwi torque host")
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
                    robot.bus.disconnect(disable_torque=not torque_cut)
            finally:
                if host is not None:
                    host.disconnect()


if __name__ == "__main__":
    main()
