"""Validate the physical safety acceptance record against the tracked configuration.

Pure functions (no ROS): the safety supervisor checks the record at startup
and ``scripts/check-release.py`` checks the tracked record before a release
is sealed, so an inconsistent footprint, StopZone, speed or stow is caught
before deployment as well as on the robot.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from lekiwi_rmf.geometry import point_in_polygon, polygon, polygon_boundary_distance, same_polygon
from lekiwi_rmf.motion_guards import load_base_speed_limits


def finite_number(value: object) -> bool:
    """A real YAML number: not a bool, NaN, or infinity."""
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def nav2_stop_zone_clearance(nav2_path: str | Path, acceptance: dict) -> tuple[bool, str]:
    """Bind acceptance to the footprint and static or predictive braking guard."""
    try:
        limits = load_base_speed_limits(nav2_path)
    except ValueError as error:
        return False, str(error)
    for name, required in zip(
        ("maximum_tested_linear_speed_m_s", "maximum_tested_angular_speed_rad_s"), limits
    ):
        measured = acceptance.get(name)
        if not finite_number(measured) or measured <= 0:
            return False, f"acceptance lacks a positive {name}"
        if measured + 1e-9 < required:
            return False, f"{name} {measured} is below configured speed {required}"
    try:
        nav2 = yaml.safe_load(Path(nav2_path).expanduser().read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        return False, f"cannot read Nav2 safety configuration: {error}"
    try:
        expected = polygon(
            acceptance.get("expected_base_footprint"), "accepted base footprint"
        )
        expected_padding = acceptance.get("expected_footprint_padding_m")
        if (
            not finite_number(expected_padding)
            or expected_padding < 0.0
        ):
            return False, "accepted footprint padding is invalid"

        for costmap_name in ("local_costmap", "global_costmap"):
            parameters = nav2[costmap_name][costmap_name]["ros__parameters"]
            configured = polygon(
                parameters.get("footprint"), f"Nav2 {costmap_name} footprint"
            )
            if not same_polygon(expected, configured):
                return False, f"Nav2 {costmap_name} footprint differs from accepted footprint"
            padding = parameters.get("footprint_padding", 0.0)
            if (
                not finite_number(padding)
                or abs(float(padding) - float(expected_padding)) > 1e-9
            ):
                return False, f"Nav2 {costmap_name} footprint padding differs from acceptance"

        monitor = nav2["collision_monitor"]["ros__parameters"]
        if monitor.get("enabled", True) is not True:
            return False, "Nav2 collision monitor is disabled"
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
        stop_polygon = polygon(stop.get("points"), "Nav2 StopZone")
    except (KeyError, TypeError, ValueError) as error:
        return False, f"invalid Nav2 safety geometry: {error}"

    if not all(point_in_polygon(point, stop_polygon) for point in expected):
        return False, "Nav2 StopZone does not enclose the accepted base footprint"
    measured = max(
        float(result["worst_stopping_distance_m"])
        for result in acceptance["directions"].values()
    )
    required_clearance = measured + float(acceptance["measurement_uncertainty_m"])
    actual_clearance = polygon_boundary_distance(expected, stop_polygon)
    if "FootprintApproach" in monitor.get("polygons", []):
        approach = monitor["FootprintApproach"]
        if not isinstance(approach, dict):
            return False, "invalid Nav2 predictive guard parameters"
        try:
            predicted_body = polygon(approach.get("points"), "Nav2 FootprintApproach")
        except (TypeError, ValueError) as error:
            return False, f"invalid Nav2 predictive safety geometry: {error}"
        if not same_polygon(expected, stop_polygon) or not same_polygon(expected, predicted_body):
            return False, "predictive braking requires the exact accepted body in both polygons"
        if (approach.get("type") != "polygon" or approach.get("action_type") != "approach"
                or approach.get("enabled") is not True or type(approach.get("min_points")) is not int
                or approach["min_points"] != 1 or stop["min_points"] != 1
                or "max_points" in approach or "max_points" in stop):
            return False, "Nav2 FootprintApproach is not an enabled single-point predictive guard"
        sources = monitor.get("observation_sources", [])
        scan = monitor.get("scan")
        source_groups = (sources, approach.get("sources_names", sources), stop.get("sources_names", sources))
        if (any(not isinstance(group, list) or "scan" not in group for group in source_groups)
                or not isinstance(scan, dict) or scan.get("enabled") is not True
                or scan.get("type") != "scan" or scan.get("topic") != "/scan"):
            return False, "Nav2 predictive guards must use the enabled accepted /scan source"
        horizon = approach.get("time_before_collision")
        step = approach.get("simulation_time_step")
        if (not finite_number(horizon) or horizon <= 0 or not finite_number(step)
                or not 0 < step <= 0.05):
            return False, "Nav2 predictive braking needs a positive horizon and <=0.05 s step"
        radius = max(math.hypot(x, y) for x, y in expected)
        linear = float(acceptance["maximum_tested_linear_speed_m_s"])
        angular = float(acceptance["maximum_tested_angular_speed_rad_s"])
        stopping_times = [
            (float(result["worst_stopping_distance_m"]) + float(acceptance["measurement_uncertainty_m"]))
            / (radius * angular if direction.startswith("rotation_") else linear)
            for direction, result in acceptance["directions"].items()
        ]
        required_horizon = max(
            float(acceptance["maximum_allowed_command_stop_latency_s"]), *stopping_times,
        ) + float(step)
        if horizon + 1e-9 < required_horizon:
            return False, (
                f"Nav2 predictive horizon {horizon:.3f} s is below the measured braking "
                f"and uncertainty bound {required_horizon:.3f} s"
            )
        return True, "Nav2 uses the accepted body and a predictive horizon covering measured braking"
    if actual_clearance + 1e-9 < required_clearance:
        return False, (
            f"Nav2 StopZone clearance {actual_clearance:.3f} m is smaller than "
            f"measured stopping distance plus uncertainty {required_clearance:.3f} m"
        )
    return True, "Nav2 StopZone encloses the accepted footprint and stopping clearance"


def validate_acceptance_file(
    path: str | Path, nav2_params_file: str | Path = "",
    expected_stow: dict[str, float] | None = None,
    installed_hardware: dict[str, bool] | None = None,
) -> tuple[bool, str]:
    """Validate the measured physical stopping/fault acceptance record."""
    try:
        data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        return False, f"cannot read safety acceptance: {error}"
    if not isinstance(data, dict) or data.get("schema_version") != 4:
        return False, "unsupported safety acceptance schema"
    if data.get("validated") is not True:
        return False, "physical safety acceptance is not validated"
    scope = data.get("operating_scope")
    if not isinstance(scope, dict) or scope.get("mode") != "attended_autonomous_base" or (
        scope.get("operator_at_motor_power_stop") is not True
    ):
        return False, "acceptance requires an attended operator at the physical motor-power stop"
    hardware = data.get("installed_hardware")
    if not isinstance(hardware, dict) or set(hardware) != {"bumper", "imu", "battery_monitor"} or (
        not all(type(value) is bool for value in hardware.values())
    ):
        return False, "accepted installed hardware must identify bumper, IMU, and battery monitor"
    if installed_hardware is not None and hardware != installed_hardware:
        return False, "accepted installed hardware differs from the production safety profile"
    minimum_trials = data.get("minimum_trials_per_direction")
    if isinstance(minimum_trials, bool) or not isinstance(minimum_trials, int) or minimum_trials < 5:
        return False, "at least 5 trials per direction are required"
    latency = data.get("maximum_command_stop_latency_s")
    if not finite_number(latency) or latency <= 0:
        return False, "measured stop latency is invalid"
    allowed_latency = data.get("maximum_allowed_command_stop_latency_s")
    if (
        not finite_number(allowed_latency)
        or allowed_latency <= 0
        or latency > allowed_latency
    ):
        return False, "measured stop latency exceeds or lacks its acceptance limit"
    allowed_distance = data.get("maximum_allowed_stopping_distance_m")
    uncertainty = data.get("measurement_uncertainty_m")
    if (
        not finite_number(allowed_distance)
        or allowed_distance <= 0
        or not finite_number(uncertainty)
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
        if not finite_number(distance) or distance <= 0:
            return False, f"{direction} stopping distance is invalid"
        if distance + uncertainty > allowed_distance:
            return False, f"{direction} stopping distance plus uncertainty exceeds its acceptance limit"
    fault_tests = data.get("fault_tests")
    required_fault_tests = {
        "scan_disconnect", "depth_disconnect", "motor_diagnostic_fault",
        "estop_independent_of_ros", "telemetry_loss",
        "telemetry_replay_or_duplicate", "host_restart_stops_then_gated_rearm",
        "ros_restart_stops_then_gated_rearm", "zmq_unauthorized_client_rejected",
        "dds_control_plane_isolated_or_authenticated",
        "rosbridge_disabled_or_authenticated",
        "collision_monitor_obstacle_stop",
        "arm_workspace_intrusion_stop",
    }
    if not isinstance(fault_tests, dict) or not all(
        fault_tests.get(name) is True for name in required_fault_tests
    ):
        return False, "required fault-response tests have not all passed"
    for hardware_name, test_name in (
        ("imu", "imu_disconnect"),
        ("battery_monitor", "battery_low_or_disconnect"),
        ("bumper", "bumper"),
    ):
        expected = True if hardware[hardware_name] else None
        if test_name not in fault_tests or fault_tests[test_name] is not expected:
            return False, f"{test_name} result does not match installed hardware"
    payload = data.get("payload_kg")
    if (
        not finite_number(payload) or payload < 0
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
            not finite_number(accepted)
            or not math.isfinite(configured)
            or abs(float(accepted) - float(configured)) > 1e-9
        ):
            return False, f"accepted arm stow differs from configured {name} position"
    if not nav2_params_file:
        return False, "Nav2 parameters are required to verify stopping clearance"
    nav2_valid, nav2_detail = nav2_stop_zone_clearance(nav2_params_file, data)
    if not nav2_valid:
        return False, nav2_detail
    return True, "physical safety acceptance validated"


def installed_hardware(parameters: dict) -> dict[str, bool]:
    """The optional hardware a safety profile requires, as the record names it."""
    return {
        "bumper": bool(parameters["require_bumper"]),
        "imu": bool(parameters["require_imu"]),
        "battery_monitor": bool(parameters["require_battery"]),
    }


def validate_tracked_acceptance(config_directory: str | Path) -> tuple[bool, str]:
    """Validate the tracked record against the tracked Nav2 and production profiles."""
    config = Path(config_directory)
    try:
        parameters = yaml.safe_load((config / "safety_production.yaml").read_text(encoding="utf-8"))[
            "safety_supervisor"]["ros__parameters"]
        stow = dict(zip(parameters["stow_joint_names"],
                        (float(value) for value in parameters["stow_joint_positions"])))
        hardware = installed_hardware(parameters)
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        return False, f"cannot read the production safety profile: {error}"
    return validate_acceptance_file(
        config / "safety_acceptance.yaml", config / "nav2_params.yaml", stow, hardware)
