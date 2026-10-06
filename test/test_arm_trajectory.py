import json
import math
import pathlib
import types

import pytest
import yaml

from lekiwi_rmf.arm_trajectory import (
    JOINT_ACCELERATION_LIMITS,
    JOINT_LIMITS,
    JOINT_VELOCITY_LIMITS,
    action_positions,
    joint_positions,
    load_calibration,
    position_tolerances,
    prepare_trajectory,
    sample_trajectory,
    validate_motion_action,
)


ROOT = pathlib.Path(__file__).parents[1]


def test_motor_action_limits_use_calibrated_joint_and_base_units():
    zeros = dict.fromkeys(JOINT_LIMITS, 0.0)
    directions = dict.fromkeys(JOINT_LIMITS, 1.0)
    zeros["arm_shoulder_pan"], directions["arm_shoulder_pan"] = 2.0, -1.0
    action = {f"{name}.pos": value for name, value in action_positions(
        tuple(JOINT_LIMITS), [0.0] * len(JOINT_LIMITS), zeros, directions,
    ).items()}
    action.update({"x.vel": 0.03 / 0.8, "y.vel": 0.0, "theta.vel": math.degrees(0.06 / 0.976)})
    args = ((0.03, 0.06), (0.8, 0.976), (zeros, directions))
    validate_motion_action(action, *args)
    for changes in (
        {"x.vel": 1000000.0}, {"y.vel": 0.01}, {"theta.vel": math.degrees(0.061 / 0.976)},
        {"arm_shoulder_pan.pos": 0.0}, {"arm_gripper.pos": -1.0}, {"arm_gripper.pos": 101.0},
        {"x.vel": float("nan")},
    ):
        with pytest.raises(ValueError):
            validate_motion_action({**action, **changes}, *args)


def test_motor_action_retains_an_out_of_bounds_measured_hold_without_extending_it():
    zeros, directions = dict.fromkeys(JOINT_LIMITS, 0.0), dict.fromkeys(JOINT_LIMITS, 1.0)
    action = {f"{name}.pos": 0.0 for name in JOINT_LIMITS}
    action.update({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
    action["arm_shoulder_lift.pos"] = math.degrees(JOINT_LIMITS["arm_shoulder_lift"][0] - 0.02)
    args = ((0.03, 0.06), (1.0, 1.0), (zeros, directions))
    with pytest.raises(ValueError, match="position limits"):
        validate_motion_action(action, *args)
    validate_motion_action(action, *args, held_positions=action)
    with pytest.raises(ValueError, match="position limits"):
        validate_motion_action({**action, "arm_shoulder_lift.pos": action["arm_shoulder_lift.pos"] - 1.0},
                               *args, held_positions=action)


def test_zero_time_start_uses_local_feedback_within_moveit_start_tolerance():
    joint = "arm_shoulder_pan"
    points = [(0.0, (0.0,), ()), (2.0, (0.1,), ())]
    measured = {joint: 0.003}
    trajectory = prepare_trajectory((joint,), points, measured)
    assert trajectory[0].positions[joint] == measured[joint]
    assert sample_trajectory((joint,), measured, trajectory, 0.0)[0] == measured
    assert trajectory[-1].positions[joint] == 0.1
    with pytest.raises(ValueError, match="no time"):
        prepare_trajectory((joint,), points, {joint: 0.011})


def test_start_feedback_offset_does_not_deform_short_moveit_segments():
    joint = "arm_shoulder_lift"
    points = [
        (0.0, (0.0,), (0.0,), (0.0,), ()),
        (0.01, (0.00001,), (0.001,), (0.0,), ()),
        (2.0, (0.1,), (0.0,), (0.0,), ()),
    ]
    measured = {joint: 0.003}
    trajectory = prepare_trajectory((joint,), points, measured)
    assert sample_trajectory((joint,), measured, trajectory, 0.0)[0] == measured
    assert trajectory[1].positions[joint] == 0.0
    assert trajectory[2].time - trajectory[1].time == pytest.approx(0.01)
    assert trajectory[-1].time == pytest.approx(2.25)


def test_action_positions_converts_ros_radians_to_lerobot_units():
    gripper_midpoint = sum(JOINT_LIMITS["arm_gripper"]) / 2
    converted = action_positions(
        ("arm_shoulder_pan", "arm_gripper"), (math.pi / 2, gripper_midpoint)
    )
    assert converted == pytest.approx({"arm_shoulder_pan": 90.0, "arm_gripper": 50.0})


def test_gripper_observation_maps_lerobot_endpoints_to_so101_angles():
    zeros = dict.fromkeys(JOINT_LIMITS, 0.0)
    directions = dict.fromkeys(JOINT_LIMITS, 1.0)
    assert joint_positions({"arm_gripper.pos": 0}, zeros, directions)[
        "arm_gripper"
    ] == pytest.approx(JOINT_LIMITS["arm_gripper"][0])
    assert joint_positions({"arm_gripper.pos": 100}, zeros, directions)[
        "arm_gripper"
    ] == pytest.approx(JOINT_LIMITS["arm_gripper"][1])


def test_action_positions_rejects_invalid_joint_lists():
    with pytest.raises(ValueError):
        action_positions(("arm_shoulder_pan", "unknown"), (0.0, 0.0))
    with pytest.raises(ValueError):
        action_positions(("arm_shoulder_pan",), (2.0,))
    with pytest.raises(ValueError, match="joint limits"):
        action_positions(("arm_wrist_roll",), (math.pi + 0.01,))
    with pytest.raises(ValueError, match="finite"):
        action_positions(("arm_wrist_roll",), (math.nan,))


def test_gripper_calibration_rejects_goals_outside_reachable_servo_range():
    zeros = dict.fromkeys(JOINT_LIMITS, 0.0)
    directions = dict.fromkeys(JOINT_LIMITS, 1.0)
    zeros["arm_gripper"] = -0.13763789773507207
    lower = JOINT_LIMITS["arm_gripper"][0]
    reachable_closed = lower - zeros["arm_gripper"]

    assert action_positions(
        ("arm_gripper",), (reachable_closed,), zeros, directions
    )["arm_gripper"] == pytest.approx(0.0)
    configured_closed = -0.036895102265
    assert action_positions(
        ("arm_gripper",), (configured_closed,), zeros, directions
    )["arm_gripper"] == pytest.approx(0.0)
    prepare_trajectory(
        ("arm_gripper",), [(1.0, (configured_closed,), ())],
        {"arm_gripper": 0.0}, zeros, directions,
    )
    with pytest.raises(ValueError, match="outside the servo's 0-100 range"):
        action_positions(("arm_gripper",), (lower,), zeros, directions)
    with pytest.raises(ValueError, match="outside the servo's 0-100 range"):
        prepare_trajectory(
            ("arm_gripper",), [(1.0, (lower,), ())], {"arm_gripper": 0.0},
            zeros, directions,
        )


def test_trajectory_timing_and_reported_velocities_are_bounded():
    start = {"arm_shoulder_pan": 0.0}
    prepare_trajectory(
        ("arm_shoulder_pan",),
        [(1.5, (1.0,), (0.0,)), (4.0, (0.0,), (0.0,))],
        start,
    )

    with pytest.raises(ValueError, match="velocity limits"):
        prepare_trajectory(
            ("arm_shoulder_pan",), [(0.1, (1.0,), ())], start
        )
    with pytest.raises(ValueError, match="velocity exceeds"):
        prepare_trajectory(
            ("arm_shoulder_pan",), [(1.0, (1.0,), (2.1,))], start
        )


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_trajectory_rejects_non_finite_positions_and_velocities(value):
    start = {"arm_shoulder_pan": 0.0}
    with pytest.raises(ValueError, match="finite"):
        prepare_trajectory(
            ("arm_shoulder_pan",), [(1.0, (value,), ())], start
        )
    with pytest.raises(ValueError, match="finite"):
        prepare_trajectory(
            ("arm_shoulder_pan",), [(1.0, (0.0,), (value,))], start
        )


def test_supplied_derivatives_select_cubic_and_quintic_interpolation():
    names = ("arm_shoulder_pan",)
    start = {"arm_shoulder_pan": 0.0}
    cubic = prepare_trajectory(names, [(2.0, (1.0,), (0.0,))], start)
    position, velocity, acceleration = sample_trajectory(names, start, cubic, 1.0)
    assert position["arm_shoulder_pan"] == pytest.approx(0.5)
    assert velocity["arm_shoulder_pan"] == pytest.approx(0.75)
    assert acceleration["arm_shoulder_pan"] == pytest.approx(0.0)

    quintic = prepare_trajectory(
        names, [(2.0, (1.0,), (0.0,), (0.0,), ())], start
    )
    position, velocity, acceleration = sample_trajectory(names, start, quintic, 1.0)
    assert position["arm_shoulder_pan"] == pytest.approx(0.5)
    assert velocity["arm_shoulder_pan"] == pytest.approx(0.9375)
    assert acceleration["arm_shoulder_pan"] == pytest.approx(0.0)


def test_trajectory_rejects_acceleration_and_interpolated_limit_violations():
    names = ("arm_shoulder_pan",)
    with pytest.raises(ValueError, match="acceleration exceeds"):
        prepare_trajectory(
            names,
            [(2.0, (0.1,), (0.0,), (3.1,), ())],
            {"arm_shoulder_pan": 0.0},
        )
    with pytest.raises(ValueError, match="interpolation exceeds.*position"):
        prepare_trajectory(
            names,
            [(0.5, (1.8,), (-2.0,))],
            {"arm_shoulder_pan": 1.8},
        )
    with pytest.raises(ValueError, match="end at zero velocity"):
        prepare_trajectory(
            names,
            [(2.0, (0.1,), (0.1,))],
            {"arm_shoulder_pan": 0.0},
        )
    with pytest.raises(ValueError, match="supplied consistently"):
        prepare_trajectory(
            names,
            [(1.0, (0.1,), (0.1,)), (2.0, (0.2,), ())],
            {"arm_shoulder_pan": 0.0},
        )


@pytest.mark.parametrize("side", ["low", "high"])
def test_slightly_out_of_bounds_start_can_only_recover_monotonically(side):
    lower, upper = JOINT_LIMITS["arm_shoulder_lift"]
    start = lower - 0.07 if side == "low" else upper + 0.07
    target = lower + 0.04 if side == "low" else upper - 0.04
    names = ("arm_shoulder_lift",)
    start_positions = {names[0]: start}
    trajectory = prepare_trajectory(
        names, [(1.0, (target,), ())], start_positions
    )
    positions = [
        sample_trajectory(names, start_positions, trajectory, step / 4)[0][names[0]]
        for step in range(5)
    ]
    assert positions[-1] == pytest.approx(target)
    assert positions == sorted(positions, reverse=(side == "high"))

    with pytest.raises(ValueError, match="too far outside limits"):
        prepare_trajectory(
            names,
            [(1.0, (target,), ())],
            {names[0]: start + (-0.1 if side == "low" else 0.1)},
        )


def test_boundary_recovery_rejects_a_nonmonotonic_interpolation():
    upper = JOINT_LIMITS["arm_shoulder_lift"][1]
    with pytest.raises(ValueError, match="monotonically"):
        prepare_trajectory(
            ("arm_shoulder_lift",),
            [(1.0, (upper - 0.04,), (0.0,), (3.0,), ())],
            {"arm_shoulder_lift": upper + 0.07},
        )


def test_bounded_terminal_acceleration_is_allowed_but_terminal_velocity_is_not():
    names = ("arm_shoulder_pan",)
    start = {"arm_shoulder_pan": 0.0}
    # MoveIt's time parameterization may carry a finite final acceleration.
    # The validator still bounds it, while requiring the actuator to stop.
    prepare_trajectory(names, [(2.0, (0.1,), (0.0,), (0.2,), ())], start)
    with pytest.raises(ValueError, match="end at zero velocity"):
        prepare_trajectory(names, [(2.0, (0.1,), (0.1,), (0.2,), ())], start)


def test_requested_position_tolerances_override_disable_and_validate():
    names = ("arm_shoulder_pan", "arm_elbow_flex")
    def tolerance(**values):
        return types.SimpleNamespace(
            name=values["name"], position=values.get("position", 0.0),
            velocity=values.get("velocity", 0.0), acceleration=values.get("acceleration", 0.0),
        )
    resolved = position_tolerances(
        names,
        [tolerance(name="arm_shoulder_pan", position=0.02),
         tolerance(name="arm_elbow_flex", position=-1.0)],
        dict.fromkeys(names, 0.05),
    )
    assert resolved == {"arm_shoulder_pan": 0.02}
    with pytest.raises(ValueError, match="unsupported joint"):
        position_tolerances(names, [tolerance(name="unknown", position=0.1)])
    with pytest.raises(ValueError, match="measured derivatives"):
        position_tolerances(names, [tolerance(name="arm_shoulder_pan", velocity=0.1)])


def test_driver_limits_match_moveit_configuration():
    configured = yaml.safe_load((ROOT / "config" / "joint_limits.yaml").read_text())[
        "joint_limits"
    ]
    for name, (lower, upper) in JOINT_LIMITS.items():
        assert configured[name]["min_position"] == pytest.approx(lower)
        assert configured[name]["max_position"] == pytest.approx(upper)
        assert configured[name]["max_velocity"] == pytest.approx(JOINT_VELOCITY_LIMITS[name])
        assert configured[name]["max_acceleration"] == pytest.approx(
            JOINT_ACCELERATION_LIMITS[name]
        )


def test_calibration_maps_urdf_zero_to_the_lerobot_position(tmp_path):
    calibration = tmp_path / "arm.json"
    zeros = {
        "arm_shoulder_pan": 0.5,
        "arm_shoulder_lift": 0,
        "arm_elbow_flex": 0,
        "arm_wrist_flex": 0,
        "arm_wrist_roll": 0,
        "arm_gripper": 0,
    }
    calibration.write_text(json.dumps({"zero_positions": zeros, "directions": dict.fromkeys(zeros, 1)}))
    zero_positions, directions = load_calibration(calibration)
    positions = joint_positions(
        {"arm_shoulder_pan.pos": math.degrees(0.5)}, zero_positions, directions
    )
    assert positions["arm_shoulder_pan"] == 0
    assert action_positions(("arm_shoulder_pan",), (0.0,), zero_positions, directions) == {
        "arm_shoulder_pan": math.degrees(0.5)
    }


@pytest.mark.parametrize("contents", ["[]", '"not an object"'])
def test_calibration_with_wrong_json_shape_is_reported_as_invalid(tmp_path, contents):
    calibration = tmp_path / "arm.json"
    calibration.write_text(contents)

    with pytest.raises(ValueError, match="invalid arm calibration"):
        load_calibration(calibration)
