"""Behavioral tests for the continuous motion-permission state machine."""

import math
import struct
from pathlib import Path

import yaml
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2, PointField

from lekiwi_rmf.safety_supervisor import (
    Requirement, SafetyState, SafetyStateMachine, _valid_battery,
    _valid_depth_points, _valid_scan_ranges, validate_acceptance_file,
)


SECOND = 1_000_000_000


def _machine() -> SafetyStateMachine:
    return SafetyStateMachine({
        "driver": Requirement(SECOND),
        "scan": Requirement(SECOND, base=True, arm=False),
        "depth": Requirement(SECOND, base=False, arm=True),
        "estop": Requirement(SECOND),
    })


def _healthy(machine: SafetyStateMachine, now: int = SECOND) -> None:
    machine.driver_state = "DISARMED"
    for name in machine.requirements:
        machine.update(name, True, now)
    machine.arm_stowed = True


def test_tracked_permission_lease_is_shorter_than_supervisor_state_deadline():
    root = Path(__file__).parents[1]
    for name in ("safety_simulation.yaml", "safety_production.yaml"):
        parameters = yaml.safe_load(
            (root / "config" / name).read_text(encoding="utf-8")
        )["safety_supervisor"]["ros__parameters"]
        assert 0.0 < parameters["permission_timeout"] < parameters["state_timeout"]


def test_production_requires_arm_workspace_gate_but_simulation_profile_does_not():
    root = Path(__file__).parents[1]
    production = yaml.safe_load(
        (root / "config" / "safety_production.yaml").read_text(encoding="utf-8")
    )
    simulation = yaml.safe_load(
        (root / "config" / "safety_simulation.yaml").read_text(encoding="utf-8")
    )
    assert production["safety_supervisor"]["ros__parameters"]["require_arm_workspace"] is True
    assert simulation["safety_supervisor"]["ros__parameters"]["require_arm_workspace"] is False
    assert production["arm_workspace_monitor"]["ros__parameters"]["group_name"] == "arm"


def test_missing_required_input_denies_all_motion():
    machine = _machine()
    decision = machine.decision(SECOND)
    assert decision.state == SafetyState.BOOT
    assert not decision.base_permitted
    assert not decision.arm_permitted
    assert "scan: missing" in decision.faults


def test_armed_driver_needs_stow_for_base_but_not_arm():
    machine = _machine()
    _healthy(machine)
    machine.driver_state = "ARMED"
    machine.arm_stowed = False
    decision = machine.decision(SECOND)
    assert decision.state == SafetyState.ARMED
    assert not decision.base_permitted
    assert decision.arm_permitted


def test_ready_state_grants_stowed_capability_leases_for_explicit_arm_request():
    machine = _machine()
    _healthy(machine)
    decision = machine.decision(SECOND)
    assert decision.state == SafetyState.READY
    assert decision.base_permitted
    assert decision.arm_permitted


def test_ready_base_scope_can_bootstrap_shared_driver_without_arm_scope():
    machine = SafetyStateMachine({
        "driver": Requirement(SECOND),
        "scan": Requirement(SECOND, base=True, arm=False),
        "arm_workspace": Requirement(SECOND, base=False, arm=True),
    })
    machine.driver_state = "DISARMED"
    machine.arm_stowed = True
    machine.update("driver", True, SECOND)
    machine.update("scan", True, SECOND)

    decision = machine.decision(SECOND)

    assert decision.state == SafetyState.READY
    assert decision.base_permitted and not decision.arm_permitted
    assert "arm_workspace: missing" in decision.faults


def test_arm_only_missing_input_does_not_block_base_startup_or_latch():
    machine = SafetyStateMachine({
        "driver": Requirement(SECOND),
        "scan": Requirement(SECOND, base=True, arm=False),
        "arm_workspace": Requirement(SECOND, base=False, arm=True),
    })
    machine.driver_state = "ARMED"
    machine.arm_stowed = True
    machine.update("driver", True, SECOND)
    machine.update("scan", True, SECOND)

    first = machine.decision(SECOND)
    repeated = machine.decision(SECOND)

    assert first.state == repeated.state == SafetyState.ARMED
    assert first.base_permitted and not first.arm_permitted
    assert "arm_workspace: missing" in first.faults
    assert not machine.fault_latched


def test_base_only_missing_input_does_not_block_arm_startup():
    machine = SafetyStateMachine({
        "driver": Requirement(SECOND),
        "scan": Requirement(SECOND, base=True, arm=False),
        "arm_workspace": Requirement(SECOND, base=False, arm=True),
    })
    machine.driver_state = "ARMED"
    machine.arm_stowed = True
    machine.update("driver", True, SECOND)
    machine.update("arm_workspace", True, SECOND)

    decision = machine.decision(SECOND)

    assert decision.state == SafetyState.ARMED
    assert not decision.base_permitted and decision.arm_permitted
    assert "scan: missing" in decision.faults


def test_arm_only_input_failure_globally_latches_after_arm_scope_was_ready():
    machine = SafetyStateMachine({
        "driver": Requirement(SECOND),
        "scan": Requirement(SECOND, base=True, arm=False),
        "arm_workspace": Requirement(SECOND, base=False, arm=True),
    })
    machine.driver_state = "ARMED"
    machine.arm_stowed = True
    for name in machine.requirements:
        machine.update(name, True, SECOND)
    assert machine.decision(SECOND).arm_permitted

    decision = machine.decision(3 * SECOND)

    assert decision.state == SafetyState.FAULT_LATCHED
    assert not decision.base_permitted and not decision.arm_permitted


def test_stale_scan_removes_only_base_permission_and_latches_after_ready():
    machine = _machine()
    _healthy(machine)
    assert machine.decision(SECOND).state == SafetyState.READY
    machine.driver_state = "ARMED"
    decision = machine.decision(SECOND * 3)
    assert decision.state == SafetyState.FAULT_LATCHED
    assert not decision.base_permitted
    assert not decision.arm_permitted


def test_estop_latches_and_cannot_reset_until_healthy_and_disarmed():
    machine = _machine()
    _healthy(machine)
    assert machine.decision(SECOND).state == SafetyState.READY
    machine.update("estop", False, SECOND, "pressed")
    assert machine.decision(SECOND).state == SafetyState.ESTOP
    success, _ = machine.reset(SECOND)
    assert not success
    machine.update("estop", True, SECOND)
    success, _ = machine.reset(SECOND)
    assert success
    assert machine.decision(SECOND).state == SafetyState.READY


def test_one_good_message_followed_by_silence_does_not_remain_permitted():
    machine = _machine()
    _healthy(machine)
    machine.driver_state = "ARMED"
    assert machine.decision(SECOND).base_permitted
    decision = machine.decision(SECOND * 3)
    assert decision.state == SafetyState.FAULT_LATCHED
    assert not decision.base_permitted
    assert not decision.arm_permitted


def test_unknown_driver_state_is_a_fault_not_healthy_disarmed_state():
    machine = _machine()
    for name in machine.requirements:
        machine.update(name, True, SECOND)
    machine.arm_stowed = True
    machine.driver_state = "BROKEN_STATE"
    machine.update("driver", False, SECOND, machine.driver_state)
    decision = machine.decision(SECOND)
    assert decision.state == SafetyState.BOOT
    assert "driver: BROKEN_STATE" in decision.faults


def test_physical_acceptance_requires_measured_all_direction_and_fault_results(tmp_path):
    path = tmp_path / "acceptance.yaml"
    nav2_path = tmp_path / "nav2.yaml"
    nav2_path.write_text(yaml.safe_dump({
        "local_costmap": {"local_costmap": {"ros__parameters": {
            "footprint": "[[-0.22, -0.22], [0.22, -0.22], [0.22, 0.22], [-0.22, 0.22]]",
            "footprint_padding": 0.0,
        }}},
        "global_costmap": {"global_costmap": {"ros__parameters": {
            "footprint": "[[0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22]]",
            "footprint_padding": 0.0,
        }}},
        "collision_monitor": {"ros__parameters": {
            "polygons": ["StopZone"],
            "StopZone": {
                "type": "polygon", "action_type": "stop", "enabled": True,
                "min_points": 1,
                "points": "[[-0.45, -0.45], [0.45, -0.45], [0.45, 0.45], [-0.45, 0.45]]",
            },
        }},
    }), encoding="utf-8")
    path.write_text(yaml.safe_dump({"schema_version": 2, "validated": False}), encoding="utf-8")
    assert not validate_acceptance_file(path)[0]

    path.write_text(yaml.safe_dump({
        "schema_version": 2,
        "validated": True,
        "software_revision": "abc123",
        "sensor_configuration": "scanner-v1",
        "validated_at": "2026-08-27T12:00:00Z",
        "payload_kg": 1.0,
        "surface": "sealed concrete",
        "expected_base_footprint": [
            [-0.22, -0.22], [0.22, -0.22], [0.22, 0.22], [-0.22, 0.22],
        ],
        "expected_footprint_padding_m": 0.0,
        "accepted_stow_joint_positions": {
            "arm_shoulder_pan": 0.0,
            "arm_shoulder_lift": 0.0,
            "arm_elbow_flex": 0.0,
            "arm_wrist_flex": 0.0,
            "arm_wrist_roll": 0.0,
            "arm_gripper": 0.0,
        },
        "minimum_trials_per_direction": 30,
        "maximum_command_stop_latency_s": 0.08,
        "maximum_allowed_command_stop_latency_s": 0.10,
        "maximum_allowed_stopping_distance_m": 0.22,
        "measurement_uncertainty_m": 0.01,
        "directions": {
            name: {"trials": 30, "worst_stopping_distance_m": 0.18}
            for name in ("forward", "reverse", "left", "right", "rotation_cw", "rotation_ccw")
        },
        "fault_tests": {
            "scan_disconnect": True, "depth_disconnect": True, "imu_disconnect": True,
            "battery_low_or_disconnect": True, "motor_diagnostic_fault": True,
            "bumper": True,
            "estop_independent_of_ros": True, "host_restart_stays_disarmed": True,
            "ros_restart_stays_disarmed": True,
            "telemetry_loss": True, "telemetry_replay_or_duplicate": True,
            "zmq_unauthorized_client_rejected": True,
            "dds_control_plane_isolated_or_authenticated": True,
            "rosbridge_disabled_or_authenticated": True,
            "collision_monitor_obstacle_stop": True,
            "arm_workspace_intrusion_stop": True,
        },
    }), encoding="utf-8")
    expected_stow = {
        "arm_shoulder_pan": 0.0,
        "arm_shoulder_lift": 0.0,
        "arm_elbow_flex": 0.0,
        "arm_wrist_flex": 0.0,
        "arm_wrist_roll": 0.0,
        "arm_gripper": 0.0,
    }
    assert validate_acceptance_file(path, nav2_path, expected_stow)[0]

    acceptance = yaml.safe_load(path.read_text(encoding="utf-8"))
    acceptance["accepted_stow_joint_positions"]["arm_elbow_flex"] = 0.1
    path.write_text(yaml.safe_dump(acceptance), encoding="utf-8")
    valid, detail = validate_acceptance_file(path, nav2_path, expected_stow)
    assert not valid
    assert "arm_elbow_flex" in detail
    acceptance["accepted_stow_joint_positions"]["arm_elbow_flex"] = 0.0
    path.write_text(yaml.safe_dump(acceptance), encoding="utf-8")

    nav2 = yaml.safe_load(nav2_path.read_text(encoding="utf-8"))
    nav2["collision_monitor"]["ros__parameters"]["StopZone"]["points"] = (
        "[[-0.30, -0.30], [0.30, -0.30], [0.30, 0.30], [-0.30, 0.30]]"
    )
    nav2_path.write_text(yaml.safe_dump(nav2), encoding="utf-8")
    valid, detail = validate_acceptance_file(path, nav2_path, expected_stow)
    assert not valid
    assert "clearance" in detail


def test_physical_acceptance_rejects_self_selected_weak_limits(tmp_path):
    path = tmp_path / "acceptance.yaml"
    template = {
        "schema_version": 2,
        "validated": True,
        "software_revision": "abc123",
        "sensor_configuration": "scanner-v1",
        "validated_at": "2026-08-27T12:00:00Z",
        "payload_kg": 1.0,
        "surface": "sealed concrete",
        "minimum_trials_per_direction": 30,
        "maximum_command_stop_latency_s": 0.20,
        "maximum_allowed_command_stop_latency_s": 0.10,
        "maximum_allowed_stopping_distance_m": 0.22,
        "measurement_uncertainty_m": 0.01,
        "directions": {
            name: {"trials": 30, "worst_stopping_distance_m": 0.18}
            for name in ("forward", "reverse", "left", "right", "rotation_cw", "rotation_ccw")
        },
        "fault_tests": {},
    }
    path.write_text(yaml.safe_dump(template), encoding="utf-8")
    valid, detail = validate_acceptance_file(path)
    assert not valid
    assert "latency" in detail


def test_scan_health_rejects_blind_and_malformed_payloads():
    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, 2.0, math.inf, math.inf]
    assert _valid_scan_ranges(scan, 0.25)

    scan.ranges = [math.inf] * 20
    assert not _valid_scan_ranges(scan, 0.05)
    for invalid in (math.nan, -math.inf, 50.0):
        scan.ranges[0] = invalid
        assert not _valid_scan_ranges(scan, 0.05)


def _cloud(points: list[tuple[float, float, float]]) -> PointCloud2:
    cloud = PointCloud2()
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = cloud.width * cloud.point_step
    cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
    return cloud


def test_depth_health_requires_xyz_layout_and_enough_finite_in_range_points():
    cloud = _cloud([(0.1, 0.2, 1.0)] * 20)
    assert _valid_depth_points(cloud, 16, 0.01, 0.05, 10.0)

    blind = _cloud([(math.nan, math.nan, math.nan)] * 20)
    assert not _valid_depth_points(blind, 1, 0.01, 0.05, 10.0)
    too_close = _cloud([(0.0, 0.0, 0.0)] * 20)
    assert not _valid_depth_points(too_close, 1, 0.01, 0.05, 10.0)
    cloud.fields.pop()
    assert not _valid_depth_points(cloud, 1, 0.01, 0.05, 10.0)

    undersized = _cloud([(0.1, 0.2, 1.0)])
    assert not _valid_depth_points(undersized, 16, 0.01, 0.05, 10.0)


def test_battery_unknown_soc_is_nan_only_and_invalid_values_fail_closed():
    battery = BatteryState()
    battery.voltage = 12.0
    battery.percentage = math.nan
    assert _valid_battery(battery, 10.5, 0.1)
    for invalid in (-1.0, math.inf, -math.inf, 1.1, 0.05):
        battery.percentage = invalid
        assert not _valid_battery(battery, 10.5, 0.1)
    battery.percentage = 0.5
    battery.voltage = math.nan
    assert not _valid_battery(battery, 10.5, 0.1)
