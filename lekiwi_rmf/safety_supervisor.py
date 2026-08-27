#!/usr/bin/env python3
"""Continuously arbitrate base and arm motion permissions.

Readiness gates answer whether a dependency became available once.  This node
answers the different, safety-relevant question: are all configured inputs
healthy *now*?  Missing and stale required inputs deny motion, and a fault
observed after startup latches until an explicit reset while the robot is
healthy and disarmed.

The ROS wrapper deliberately uses standard messages so deployments can wire a
hardware safety controller, BMS, or bumper bridge without a repository-specific
message package.  The pure :class:`SafetyStateMachine` is directly unit-testable.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class SafetyState(str, Enum):
    BOOT = "BOOT"
    DISARMED = "DISARMED"
    READY = "READY"
    ARMED = "ARMED"
    FAULT_LATCHED = "FAULT_LATCHED"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class Requirement:
    max_age_ns: int
    base: bool = True
    arm: bool = True


@dataclass
class InputSample:
    healthy: bool
    stamp_ns: int
    detail: str = ""


@dataclass
class SafetyDecision:
    state: SafetyState
    base_permitted: bool
    arm_permitted: bool
    faults: tuple[str, ...] = ()


@dataclass
class SafetyStateMachine:
    """Default-deny, fault-latching health evaluator."""

    requirements: dict[str, Requirement]
    samples: dict[str, InputSample] = field(default_factory=dict)
    driver_state: str = "DISARMED"
    arm_stowed: bool = False
    fault_latched: bool = False
    estop_latched: bool = False
    ever_ready: bool = False
    base_ever_ready: bool = False
    arm_ever_ready: bool = False

    def update(self, name: str, healthy: bool, stamp_ns: int, detail: str = "") -> None:
        if name not in self.requirements:
            return
        self.samples[name] = InputSample(bool(healthy), int(stamp_ns), detail)
        if name == "estop" and not healthy:
            self.estop_latched = True
        elif not healthy:
            requirement = self.requirements[name]
            if (
                (requirement.base and self.base_ever_ready)
                or (requirement.arm and self.arm_ever_ready)
            ):
                self.fault_latched = True

    def _faults(self, now_ns: int, for_base: bool, for_arm: bool) -> tuple[str, ...]:
        faults: list[str] = []
        for name, requirement in self.requirements.items():
            if not ((for_base and requirement.base) or (for_arm and requirement.arm)):
                continue
            sample = self.samples.get(name)
            if sample is None:
                faults.append(f"{name}: missing")
                continue
            age = now_ns - sample.stamp_ns
            if age < 0 or age > requirement.max_age_ns:
                faults.append(f"{name}: stale")
            elif not sample.healthy:
                faults.append(f"{name}: {sample.detail or 'unhealthy'}")
        return tuple(faults)

    def decision(self, now_ns: int) -> SafetyDecision:
        base_faults = self._faults(now_ns, True, False)
        arm_faults = self._faults(now_ns, False, True)
        all_faults = tuple(dict.fromkeys((*base_faults, *arm_faults)))
        if self.driver_state == "LINK_LOST":
            all_faults = (*all_faults, "driver: link lost")
            base_faults = (*base_faults, "driver: link lost")
            arm_faults = (*arm_faults, "driver: link lost")
            if self.base_ever_ready or self.arm_ever_ready:
                self.fault_latched = True
        elif (
            (base_faults and self.base_ever_ready)
            or (arm_faults and self.arm_ever_ready)
        ):
            # A dependency becoming stale after its scope was ready is a
            # global runtime fault, not a return to partial startup state.
            self.fault_latched = True

        if self.estop_latched:
            return SafetyDecision(SafetyState.ESTOP, False, False, all_faults)
        if self.fault_latched:
            return SafetyDecision(SafetyState.FAULT_LATCHED, False, False, all_faults)

        base_ready = not base_faults
        arm_ready = not arm_faults
        if not base_ready and not arm_ready:
            return SafetyDecision(SafetyState.BOOT, False, False, all_faults)

        self.base_ever_ready = self.base_ever_ready or base_ready
        self.arm_ever_ready = self.arm_ever_ready or arm_ready
        self.ever_ready = self.base_ever_ready or self.arm_ever_ready
        armed = self.driver_state == "ARMED"
        base_permitted = armed and base_ready and self.arm_stowed
        arm_permitted = armed and arm_ready
        if armed:
            return SafetyDecision(
                SafetyState.ARMED, base_permitted, arm_permitted, all_faults
            )
        # In READY these are short-lived capability-readiness leases, not
        # evidence of motion or torque. The disarmed driver still rejects all
        # commands, but its explicit arm transaction may use either healthy
        # capability to bootstrap the shared physical torque bus.
        return SafetyDecision(
            SafetyState.READY,
            base_ready and self.arm_stowed,
            arm_ready,
            all_faults,
        )

    def reset(self, now_ns: int) -> tuple[bool, str]:
        if self.driver_state == "ARMED":
            return False, "disarm the driver before resetting safety faults"
        faults = self._faults(now_ns, True, True)
        if faults:
            return False, "; ".join(faults)
        self.fault_latched = False
        self.estop_latched = False
        self.ever_ready = True
        self.base_ever_ready = True
        self.arm_ever_ready = True
        return True, "safety fault reset; robot remains disarmed"


def _seconds_to_ns(value: float, name: str) -> int:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return int(value * 1_000_000_000)


def _valid_scan_ranges(message: LaserScan, minimum_valid_fraction: float) -> bool:
    """Reject empty, corrupt, and effectively blind scans."""
    if not message.ranges:
        return False
    finite_returns = 0
    for value in message.ranges:
        value = float(value)
        if math.isinf(value) and value > 0.0:
            continue
        if not math.isfinite(value) or value < message.range_min or value > message.range_max:
            return False
        finite_returns += 1
    return finite_returns / len(message.ranges) >= minimum_valid_fraction


def _point_field_format(field: PointField) -> tuple[str, int] | None:
    if field.count != 1:
        return None
    if field.datatype == PointField.FLOAT32:
        return "f", 4
    if field.datatype == PointField.FLOAT64:
        return "d", 8
    return None


def _valid_depth_points(
    message: PointCloud2,
    minimum_valid_points: int,
    minimum_valid_fraction: float,
    minimum_range: float,
    maximum_range: float,
    maximum_samples: int = 1024,
) -> bool:
    """Validate the layout and sampled XYZ content of a depth point cloud."""
    point_count = int(message.width) * int(message.height)
    if (
        point_count < minimum_valid_points
        or message.point_step <= 0
        or message.row_step < message.width * message.point_step
        or len(message.data) < message.height * message.row_step
    ):
        return False
    fields = {field.name: field for field in message.fields}
    layouts: list[tuple[int, str]] = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None:
            return False
        field_format = _point_field_format(field)
        if field_format is None:
            return False
        code, size = field_format
        if field.offset < 0 or field.offset + size > message.point_step:
            return False
        layouts.append((int(field.offset), code))

    sample_count = min(point_count, maximum_samples)
    required = max(
        minimum_valid_points,
        math.ceil(sample_count * minimum_valid_fraction),
    )
    endian = ">" if message.is_bigendian else "<"
    valid = 0
    for sample in range(sample_count):
        linear_index = sample * point_count // sample_count
        row, column = divmod(linear_index, int(message.width))
        base = row * int(message.row_step) + column * int(message.point_step)
        try:
            x, y, z = (
                struct.unpack_from(endian + code, message.data, base + offset)[0]
                for offset, code in layouts
            )
        except (struct.error, TypeError):
            return False
        if all(math.isfinite(value) for value in (x, y, z)):
            distance = math.sqrt(x * x + y * y + z * z)
            if minimum_range <= distance <= maximum_range:
                valid += 1
                if valid >= required:
                    return True
    return False


def _valid_battery(
    message: BatteryState,
    minimum_voltage: float,
    minimum_percentage: float,
) -> bool:
    if not math.isfinite(message.voltage) or message.voltage < minimum_voltage:
        return False
    # REP-147 uses NaN, not negative values or infinity, for an unknown SOC.
    if math.isnan(message.percentage):
        return True
    return (
        math.isfinite(message.percentage)
        and 0.0 <= message.percentage <= 1.0
        and message.percentage >= minimum_percentage
    )


def _polygon(value, description: str) -> tuple[tuple[float, float], ...]:
    """Parse one simple finite polygon from a YAML value or ROS string value."""
    if isinstance(value, str):
        try:
            value = yaml.safe_load(value)
        except yaml.YAMLError as error:
            raise ValueError(f"{description} is not valid YAML") from error
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError(f"{description} must contain at least three points")
    points = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"{description} points must be [x, y] pairs")
        if any(isinstance(coordinate, bool) for coordinate in point):
            raise ValueError(f"{description} coordinates must be numeric")
        try:
            parsed = tuple(float(coordinate) for coordinate in point)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{description} coordinates must be numeric") from error
        if not all(math.isfinite(coordinate) for coordinate in parsed):
            raise ValueError(f"{description} coordinates must be finite")
        points.append(parsed)
    if len(set(points)) != len(points):
        raise ValueError(f"{description} contains duplicate points")
    twice_area = sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )
    if abs(twice_area) <= 1e-12:
        raise ValueError(f"{description} has zero area")
    return tuple(points)


def _same_polygon(first, second, tolerance: float = 1e-9) -> bool:
    """Compare polygon vertices independent of start vertex and winding."""
    if len(first) != len(second):
        return False
    for candidate in (second, tuple(reversed(second))):
        for offset in range(len(candidate)):
            rotated = candidate[offset:] + candidate[:offset]
            if all(
                math.dist(left, right) <= tolerance
                for left, right in zip(first, rotated)
            ):
                return True
    return False


def _point_in_polygon(point, polygon) -> bool:
    """Return true for points inside or on the boundary of a simple polygon."""
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) <= 1e-12 and min(x1, x2) - 1e-12 <= x <= max(x1, x2) + 1e-12 \
                and min(y1, y2) - 1e-12 <= y <= max(y1, y2) + 1e-12:
            return True
        if (y1 > y) != (y2 > y):
            boundary_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < boundary_x:
                inside = not inside
    return inside


def _point_segment_distance(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 0.0:
        return math.dist(point, start)
    projection = max(0.0, min(1.0, (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / denominator))
    return math.dist(point, (start[0] + projection * dx, start[1] + projection * dy))


def _segment_distance(first_start, first_end, second_start, second_end) -> float:
    # Proper or endpoint intersection has zero clearance. Collinear cases are
    # also caught by the endpoint-to-segment distances below.
    def orientation(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    orientations = (
        orientation(first_start, first_end, second_start),
        orientation(first_start, first_end, second_end),
        orientation(second_start, second_end, first_start),
        orientation(second_start, second_end, first_end),
    )
    if orientations[0] * orientations[1] < 0.0 and orientations[2] * orientations[3] < 0.0:
        return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance(second_start, first_start, first_end),
        _point_segment_distance(second_end, first_start, first_end),
    )


def _polygon_boundary_distance(first, second) -> float:
    return min(
        _segment_distance(first_start, first_end, second_start, second_end)
        for first_start, first_end in zip(first, first[1:] + first[:1])
        for second_start, second_end in zip(second, second[1:] + second[:1])
    )


def _nav2_stop_zone_clearance(nav2_path: str | Path, acceptance: dict) -> tuple[bool, str]:
    """Bind acceptance to Nav2's footprint and measured StopZone clearance."""
    try:
        nav2 = yaml.safe_load(Path(nav2_path).expanduser().read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        return False, f"cannot read Nav2 safety configuration: {error}"
    try:
        expected = _polygon(
            acceptance.get("expected_base_footprint"), "accepted base footprint"
        )
        expected_padding = acceptance.get("expected_footprint_padding_m")
        if (
            not isinstance(expected_padding, (int, float))
            or isinstance(expected_padding, bool)
            or not math.isfinite(expected_padding)
            or expected_padding < 0.0
        ):
            return False, "accepted footprint padding is invalid"

        for costmap_name in ("local_costmap", "global_costmap"):
            parameters = nav2[costmap_name][costmap_name]["ros__parameters"]
            configured = _polygon(
                parameters.get("footprint"), f"Nav2 {costmap_name} footprint"
            )
            if not _same_polygon(expected, configured):
                return False, f"Nav2 {costmap_name} footprint differs from accepted footprint"
            padding = parameters.get("footprint_padding", 0.0)
            if (
                not isinstance(padding, (int, float))
                or isinstance(padding, bool)
                or not math.isfinite(padding)
                or abs(float(padding) - float(expected_padding)) > 1e-9
            ):
                return False, f"Nav2 {costmap_name} footprint padding differs from acceptance"

        monitor = nav2["collision_monitor"]["ros__parameters"]
        if "StopZone" not in monitor.get("polygons", []):
            return False, "Nav2 collision monitor does not enable StopZone"
        stop = monitor["StopZone"]
        if (
            stop.get("type") != "polygon"
            or stop.get("action_type") != "stop"
            or stop.get("enabled") is not True
            or isinstance(stop.get("min_points"), bool)
            or not isinstance(stop.get("min_points"), int)
            or stop.get("min_points") < 1
        ):
            return False, "Nav2 StopZone is not an enabled obstacle stop polygon"
        stop_polygon = _polygon(stop.get("points"), "Nav2 StopZone")
    except (KeyError, TypeError, ValueError) as error:
        return False, f"invalid Nav2 safety geometry: {error}"

    if not all(_point_in_polygon(point, stop_polygon) for point in expected):
        return False, "Nav2 StopZone does not enclose the accepted base footprint"
    measured = max(
        float(result["worst_stopping_distance_m"])
        for result in acceptance["directions"].values()
    )
    required_clearance = measured + float(acceptance["measurement_uncertainty_m"])
    actual_clearance = _polygon_boundary_distance(expected, stop_polygon)
    if actual_clearance + 1e-9 < required_clearance:
        return False, (
            f"Nav2 StopZone clearance {actual_clearance:.3f} m is smaller than "
            f"measured stopping distance plus uncertainty {required_clearance:.3f} m"
        )
    return True, "Nav2 StopZone encloses the accepted footprint and stopping clearance"


def validate_acceptance_file(
    path: str | Path, nav2_params_file: str | Path = "",
    expected_stow: dict[str, float] | None = None,
) -> tuple[bool, str]:
    """Validate the measured physical stopping/fault acceptance record."""
    try:
        data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        return False, f"cannot read safety acceptance: {error}"
    if not isinstance(data, dict) or data.get("schema_version") != 2:
        return False, "unsupported safety acceptance schema"
    if data.get("validated") is not True:
        return False, "physical safety acceptance is not validated"
    minimum_trials = data.get("minimum_trials_per_direction")
    if isinstance(minimum_trials, bool) or not isinstance(minimum_trials, int) or minimum_trials < 30:
        return False, "at least 30 trials per direction are required"
    latency = data.get("maximum_command_stop_latency_s")
    if not isinstance(latency, (int, float)) or isinstance(latency, bool) or not math.isfinite(latency) or latency <= 0:
        return False, "measured stop latency is invalid"
    allowed_latency = data.get("maximum_allowed_command_stop_latency_s")
    if (
        not isinstance(allowed_latency, (int, float))
        or isinstance(allowed_latency, bool)
        or not math.isfinite(allowed_latency)
        or allowed_latency <= 0
        or latency > allowed_latency
    ):
        return False, "measured stop latency exceeds or lacks its acceptance limit"
    allowed_distance = data.get("maximum_allowed_stopping_distance_m")
    uncertainty = data.get("measurement_uncertainty_m")
    if (
        not isinstance(allowed_distance, (int, float))
        or isinstance(allowed_distance, bool)
        or not math.isfinite(allowed_distance)
        or allowed_distance <= 0
        or not isinstance(uncertainty, (int, float))
        or isinstance(uncertainty, bool)
        or not math.isfinite(uncertainty)
        or uncertainty < 0
    ):
        return False, "stopping-distance limit or measurement uncertainty is invalid"
    directions = data.get("directions")
    required_directions = {
        "forward", "reverse", "left", "right", "rotation_cw", "rotation_ccw"
    }
    if not isinstance(directions, dict) or set(directions) != required_directions:
        return False, "directional stopping results are incomplete"
    for direction, result in directions.items():
        trials = result.get("trials") if isinstance(result, dict) else None
        if (
            not isinstance(trials, int)
            or isinstance(trials, bool)
            or trials < minimum_trials
        ):
            return False, f"{direction} has too few stopping trials"
        distance = result.get("worst_stopping_distance_m")
        if not isinstance(distance, (int, float)) or isinstance(distance, bool) or not math.isfinite(distance) or distance <= 0:
            return False, f"{direction} stopping distance is invalid"
        if distance + uncertainty > allowed_distance:
            return False, f"{direction} stopping distance plus uncertainty exceeds its acceptance limit"
    fault_tests = data.get("fault_tests")
    required_fault_tests = {
        "scan_disconnect", "depth_disconnect", "imu_disconnect",
        "battery_low_or_disconnect", "motor_diagnostic_fault", "bumper",
        "estop_independent_of_ros", "telemetry_loss",
        "telemetry_replay_or_duplicate", "host_restart_stays_disarmed",
        "ros_restart_stays_disarmed", "zmq_unauthorized_client_rejected",
        "dds_control_plane_isolated_or_authenticated",
        "rosbridge_disabled_or_authenticated",
        "collision_monitor_obstacle_stop",
        "arm_workspace_intrusion_stop",
    }
    if (
        not isinstance(fault_tests, dict)
        or not required_fault_tests.issubset(fault_tests)
        or not all(fault_tests[name] is True for name in required_fault_tests)
    ):
        return False, "required fault-response tests have not all passed"
    payload = data.get("payload_kg")
    if (
        not isinstance(payload, (int, float)) or isinstance(payload, bool)
        or not math.isfinite(payload) or payload < 0
        or not isinstance(data.get("surface"), str) or not data["surface"].strip()
    ):
        return False, "acceptance must identify a valid payload and test surface"
    if not all(
        isinstance(data.get(name), str) and data[name].strip()
        for name in ("software_revision", "sensor_configuration", "validated_at")
    ):
        return False, "acceptance must identify revision, sensor configuration, and validation time"
    accepted_stow = data.get("accepted_stow_joint_positions")
    if expected_stow is None:
        return False, "configured arm stow is required to verify acceptance"
    if not isinstance(accepted_stow, dict) or set(accepted_stow) != set(expected_stow):
        return False, "accepted arm stow does not contain exactly the configured joints"
    for name, configured in expected_stow.items():
        accepted = accepted_stow[name]
        if (
            not isinstance(accepted, (int, float))
            or isinstance(accepted, bool)
            or not math.isfinite(accepted)
            or not math.isfinite(configured)
            or abs(float(accepted) - float(configured)) > 1e-9
        ):
            return False, f"accepted arm stow differs from configured {name} position"
    if not nav2_params_file:
        return False, "Nav2 parameters are required to verify stopping clearance"
    nav2_valid, nav2_detail = _nav2_stop_zone_clearance(nav2_params_file, data)
    if not nav2_valid:
        return False, nav2_detail
    return True, "physical safety acceptance validated"


class SafetySupervisor(Node):
    """ROS health adapters around :class:`SafetyStateMachine`."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameter("publish_frequency", 20.0)
        self.declare_parameter("sensor_timeout", 0.75)
        self.declare_parameter("state_timeout", 1.0)
        # Consumers use this receive-time lease because Bool has no source
        # timestamp. Keep it shorter than the supervisor's input deadline so
        # a stopped publisher fails closed before a normal state timeout.
        self.declare_parameter("permission_timeout", 0.5)
        self.declare_parameter("battery_timeout", 2.0)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_driver_state", True)
        self.declare_parameter("require_full_scan", True)
        self.declare_parameter("minimum_scan_coverage", 6.0)
        self.declare_parameter("minimum_scan_valid_fraction", 0.05)
        self.declare_parameter("require_depth", True)
        self.declare_parameter("minimum_depth_valid_points", 16)
        self.declare_parameter("minimum_depth_valid_fraction", 0.01)
        self.declare_parameter("minimum_depth_range", 0.05)
        self.declare_parameter("maximum_depth_range", 10.0)
        self.declare_parameter("require_bumper", True)
        self.declare_parameter("require_estop", True)
        self.declare_parameter("require_battery", True)
        self.declare_parameter("require_motor_health", True)
        self.declare_parameter("require_odometry", True)
        self.declare_parameter("require_imu", True)
        self.declare_parameter("require_joint_states", True)
        self.declare_parameter("require_arm_workspace", True)
        self.declare_parameter("arm_workspace_topic", "/safety/arm_workspace_clear")
        self.declare_parameter("minimum_battery_voltage", 10.5)
        self.declare_parameter("minimum_battery_percentage", 0.10)
        self.declare_parameter("stow_tolerance", 0.08)
        self.declare_parameter("stow_joint_names", [
            "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
            "arm_wrist_flex", "arm_wrist_roll", "arm_gripper",
        ])
        self.declare_parameter("stow_joint_positions", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("require_acceptance", True)
        self.declare_parameter("acceptance_file", "")
        self.declare_parameter("nav2_params_file", "")

        sensor_timeout = _seconds_to_ns(float(self.get_parameter("sensor_timeout").value), "sensor_timeout")
        state_timeout = _seconds_to_ns(float(self.get_parameter("state_timeout").value), "state_timeout")
        permission_timeout = _seconds_to_ns(
            float(self.get_parameter("permission_timeout").value), "permission_timeout"
        )
        if permission_timeout >= state_timeout:
            raise ValueError("permission_timeout must be shorter than state_timeout")
        battery_timeout = _seconds_to_ns(float(self.get_parameter("battery_timeout").value), "battery_timeout")
        requirements: dict[str, Requirement] = {}
        require_driver_state = bool(self.get_parameter("require_driver_state").value)
        if require_driver_state:
            requirements["driver"] = Requirement(state_timeout)
        if self.get_parameter("require_scan").value:
            requirements["scan"] = Requirement(sensor_timeout, base=True, arm=False)
        if self.get_parameter("require_depth").value:
            requirements["depth"] = Requirement(sensor_timeout, base=False, arm=True)
        if self.get_parameter("require_bumper").value:
            requirements["bumper"] = Requirement(state_timeout)
        if self.get_parameter("require_estop").value:
            requirements["estop"] = Requirement(state_timeout)
        if self.get_parameter("require_battery").value:
            requirements["battery"] = Requirement(battery_timeout)
        if self.get_parameter("require_motor_health").value:
            requirements["motor_health"] = Requirement(battery_timeout)
        if self.get_parameter("require_odometry").value:
            requirements["odometry"] = Requirement(state_timeout, base=True, arm=False)
        if self.get_parameter("require_imu").value:
            requirements["imu"] = Requirement(state_timeout, base=True, arm=False)
        require_joint_states = bool(self.get_parameter("require_joint_states").value)
        if require_joint_states:
            requirements["joints"] = Requirement(state_timeout)
        if bool(self.get_parameter("require_arm_workspace").value):
            requirements["arm_workspace"] = Requirement(
                state_timeout, base=False, arm=True
            )

        stow_names = tuple(self.get_parameter("stow_joint_names").value)
        stow_positions = tuple(
            float(value) for value in self.get_parameter("stow_joint_positions").value
        )
        self._stow = dict(zip(stow_names, stow_positions))
        if (
            not self._stow
            or len(stow_names) != len(stow_positions)
            or len(self._stow) != len(stow_names)
            or not all(math.isfinite(value) for value in self._stow.values())
        ):
            raise ValueError("stow joint names and positions must be non-empty and one-to-one")

        self._machine = SafetyStateMachine(
            requirements,
            driver_state="DISARMED" if require_driver_state else "ARMED",
            arm_stowed=not require_joint_states,
        )
        if bool(self.get_parameter("require_acceptance").value):
            requirements["acceptance"] = Requirement(2**62)
            healthy, detail = validate_acceptance_file(
                str(self.get_parameter("acceptance_file").value),
                str(self.get_parameter("nav2_params_file").value),
                self._stow,
            )
            self._machine.update("acceptance", healthy, 0, detail)
        self._minimum_scan_coverage = float(self.get_parameter("minimum_scan_coverage").value)
        self._require_full_scan = bool(self.get_parameter("require_full_scan").value)
        self._minimum_scan_valid_fraction = float(
            self.get_parameter("minimum_scan_valid_fraction").value
        )
        self._minimum_depth_valid_points = int(
            self.get_parameter("minimum_depth_valid_points").value
        )
        self._minimum_depth_valid_fraction = float(
            self.get_parameter("minimum_depth_valid_fraction").value
        )
        self._minimum_depth_range = float(self.get_parameter("minimum_depth_range").value)
        self._maximum_depth_range = float(self.get_parameter("maximum_depth_range").value)
        if not 0.0 < self._minimum_scan_valid_fraction <= 1.0:
            raise ValueError("minimum_scan_valid_fraction must be in (0, 1]")
        if self._minimum_depth_valid_points <= 0:
            raise ValueError("minimum_depth_valid_points must be positive")
        if not 0.0 < self._minimum_depth_valid_fraction <= 1.0:
            raise ValueError("minimum_depth_valid_fraction must be in (0, 1]")
        if not (
            math.isfinite(self._minimum_depth_range)
            and math.isfinite(self._maximum_depth_range)
            and 0.0 <= self._minimum_depth_range < self._maximum_depth_range
        ):
            raise ValueError("depth range must be finite, non-negative, and increasing")
        self._minimum_battery_voltage = float(self.get_parameter("minimum_battery_voltage").value)
        self._minimum_battery_percentage = float(self.get_parameter("minimum_battery_percentage").value)
        self._stow_tolerance = float(self.get_parameter("stow_tolerance").value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._state_pub = self.create_publisher(String, "safety/supervisor_state", latched)
        self._base_pub = self.create_publisher(Bool, "safety/base_motion_permitted", latched)
        self._arm_pub = self.create_publisher(Bool, "safety/arm_motion_permitted", latched)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(String, "safety/driver_state", self._on_driver, latched)
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.create_subscription(PointCloud2, "/camera/depth/points", self._on_depth, sensor_qos)
        self.create_subscription(Odometry, "/odom", self._on_odometry, sensor_qos)
        self.create_subscription(Imu, "/imu/data", self._on_imu, sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, sensor_qos)
        self.create_subscription(
            Bool,
            str(self.get_parameter("arm_workspace_topic").value),
            self._on_arm_workspace,
            latched,
        )
        self.create_subscription(Bool, "safety/bumper_active", self._on_bumper, latched)
        self.create_subscription(Bool, "safety/estop_active", self._on_estop, latched)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, sensor_qos)
        self.create_subscription(
            DiagnosticArray, "/hardware/diagnostics", self._on_hardware_diagnostics, sensor_qos
        )
        self.create_service(Trigger, "safety/reset_fault", self._reset)

        frequency = float(self.get_parameter("publish_frequency").value)
        if not math.isfinite(frequency) or frequency <= 0.0:
            raise ValueError("publish_frequency must be finite and positive")
        if 1_000_000_000 / frequency >= permission_timeout:
            raise ValueError(
                "publish_frequency must refresh permission before permission_timeout"
            )
        self.create_timer(1.0 / frequency, self._publish)
        self._publish()

    def _now(self) -> int:
        return self.get_clock().now().nanoseconds

    @staticmethod
    def _source_stamp(message) -> int:
        stamp = message.header.stamp
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return value

    def _on_driver(self, message: String) -> None:
        self._machine.driver_state = message.data
        healthy = message.data in {"DISARMED", "ARMED"}
        self._machine.update("driver", healthy, self._now(), message.data or "empty state")

    def _on_scan(self, message: LaserScan) -> None:
        coverage = abs(float(message.angle_increment)) * max(0, len(message.ranges) - 1)
        healthy = (
            bool(message.ranges)
            and math.isfinite(message.angle_increment)
            and message.angle_increment != 0.0
            and math.isfinite(message.range_min)
            and math.isfinite(message.range_max)
            and 0.0 <= message.range_min < message.range_max
            and _valid_scan_ranges(message, self._minimum_scan_valid_fraction)
            and (not self._require_full_scan or coverage >= self._minimum_scan_coverage)
            and self._source_stamp(message) > 0
        )
        self._machine.update("scan", healthy, self._source_stamp(message) or self._now(), f"coverage={coverage:.2f} rad")

    def _on_depth(self, message: PointCloud2) -> None:
        healthy = (
            _valid_depth_points(
                message,
                self._minimum_depth_valid_points,
                self._minimum_depth_valid_fraction,
                self._minimum_depth_range,
                self._maximum_depth_range,
            )
            and self._source_stamp(message) > 0
        )
        self._machine.update(
            "depth", healthy, self._source_stamp(message) or self._now(),
            "invalid, blind, or stampless point cloud",
        )

    def _on_odometry(self, message: Odometry) -> None:
        values = (
            message.pose.pose.position.x, message.pose.pose.position.y,
            message.pose.pose.orientation.z, message.pose.pose.orientation.w,
            message.twist.twist.linear.x, message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        )
        healthy = all(math.isfinite(value) for value in values) and self._source_stamp(message) > 0
        self._machine.update("odometry", healthy, self._source_stamp(message) or self._now(), "non-finite/stampless odometry")

    def _on_imu(self, message: Imu) -> None:
        values = (
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x, message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        healthy = all(math.isfinite(value) for value in values) and self._source_stamp(message) > 0
        self._machine.update("imu", healthy, self._source_stamp(message) or self._now(), "non-finite/stampless IMU feedback")

    def _on_joints(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        healthy = (
            self._source_stamp(message) > 0
            and all(name in positions and math.isfinite(positions[name]) for name in self._stow)
        )
        self._machine.arm_stowed = healthy and all(
            abs(positions[name] - expected) <= self._stow_tolerance
            for name, expected in self._stow.items()
        )
        self._machine.update("joints", healthy, self._source_stamp(message) or self._now(), "incomplete/stampless joint feedback")

    def _on_bumper(self, message: Bool) -> None:
        self._machine.update("bumper", not message.data, self._now(), "bumper active")

    def _on_arm_workspace(self, message: Bool) -> None:
        self._machine.update(
            "arm_workspace", bool(message.data), self._now(),
            "MoveIt reports collision or workspace monitor is unhealthy",
        )

    def _on_estop(self, message: Bool) -> None:
        self._machine.update("estop", not message.data, self._now(), "E-stop active")

    def _on_battery(self, message: BatteryState) -> None:
        healthy = _valid_battery(
            message, self._minimum_battery_voltage, self._minimum_battery_percentage
        ) and self._source_stamp(message) > 0
        self._machine.update("battery", healthy, self._source_stamp(message) or self._now(), "battery below threshold or stampless")

    def _on_hardware_diagnostics(self, message: DiagnosticArray) -> None:
        healthy = self._source_stamp(message) > 0 and bool(message.status) and all(
            status.level == DiagnosticStatus.OK for status in message.status
        )
        detail = "; ".join(
            f"{status.name}: {status.message}"
            for status in message.status if status.level != DiagnosticStatus.OK
        ) or "no motor diagnostics"
        self._machine.update("motor_health", healthy, self._source_stamp(message) or self._now(), detail)

    def _reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._machine.reset(self._now())
        self._publish()
        return response

    def _publish(self) -> None:
        decision = self._machine.decision(self._now())
        state = String()
        state.data = decision.state.value
        base = Bool()
        base.data = decision.base_permitted
        arm = Bool()
        arm.data = decision.arm_permitted
        self._state_pub.publish(state)
        self._base_pub.publish(base)
        self._arm_pub.publish(arm)

        status = DiagnosticStatus()
        status.name = "lekiwi/safety_supervisor"
        status.hardware_id = "lekiwi"
        status.level = DiagnosticStatus.OK if not decision.faults else DiagnosticStatus.ERROR
        status.message = decision.state.value
        status.values = [
            KeyValue(key="base_motion_permitted", value=str(decision.base_permitted).lower()),
            KeyValue(key="arm_motion_permitted", value=str(decision.arm_permitted).lower()),
            KeyValue(key="faults", value="; ".join(decision.faults)),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SafetySupervisor] = None
    try:
        node = SafetySupervisor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
