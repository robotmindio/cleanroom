"""Behavioral checks for the Gazebo-only actuation and sensor model."""

from __future__ import annotations

import math
import pathlib
import subprocess
import tempfile
import threading
import types
import xml.etree.ElementTree as ET

import pytest

from lekiwi_rmf.arm_trajectory import duration_seconds, stamp_nanoseconds
from lekiwi_rmf.sim_arm_controller import SimArmController, permission_is_fresh
from lekiwi_rmf.sim_omni_controller import (
    MotionLimiter,
    body_to_wheels,
    saturate_wheels,
    wheels_to_body,
)
from lekiwi_rmf.sim_sdf import GZ_XML_NAMESPACE, render_simulation_sdf
from lekiwi_rmf.sim_topics import ARM_TRAJECTORY_HEARTBEAT_TOPIC


ROOT = pathlib.Path(__file__).parents[1]


def test_simulated_arm_permission_is_a_receive_time_lease():
    assert permission_is_fresh(True, 10.0, 10.49, 0.5)
    assert not permission_is_fresh(True, 10.0, 10.51, 0.5)
    assert not permission_is_fresh(False, 10.0, 10.1, 0.5)
    assert not permission_is_fresh(True, None, 10.1, 0.5)
    assert not permission_is_fresh(True, 10.2, 10.1, 0.5)


def test_simulated_arm_rejects_malformed_ros_durations_and_stamps():
    with pytest.raises(ValueError, match="duration"):
        duration_seconds(types.SimpleNamespace(sec=0, nanosec=1_000_000_000))
    with pytest.raises(ValueError, match="timestamp"):
        stamp_nanoseconds(types.SimpleNamespace(sec=-1, nanosec=0))


def test_simulated_arm_stale_feedback_abort_still_publishes_cached_hold():
    node = SimArmController.__new__(SimArmController)
    node._state_lock = threading.Lock()
    node._positions = {"joint": 0.25}
    node._position_stamps_ns = {"joint": 0}
    node.state_timeout = 0.1
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=1_000_000_000)
    )
    published = []
    node._trajectory_publisher = types.SimpleNamespace(publish=published.append)

    assert node._fresh_positions(("joint",)) is None
    node._hold(("joint",))

    assert len(published) == 1
    assert published[0].joint_names == ["joint"]
    assert list(published[0].points[0].positions) == [0.25]


def test_simulated_arm_publishes_native_watchdog_heartbeat_while_executing():
    controller = (ROOT / "lekiwi_rmf" / "sim_arm_controller.py").read_text()
    assert "ARM_TRAJECTORY_HEARTBEAT_TOPIC" in controller
    assert ARM_TRAJECTORY_HEARTBEAT_TOPIC == "/sim/arm/trajectory_heartbeat"
    assert "self._trajectory_heartbeat.publish(Bool(data=True))" in controller


def description(sim: bool) -> ET.Element:
    output = subprocess.check_output(
        ["xacro", str(ROOT / "urdf" / "lekiwi.urdf.xacro"), f"sim:={str(sim).lower()}"],
        text=True,
    )
    return ET.fromstring(output)


@pytest.mark.parametrize(
    "velocity",
    [(0.3, 0.0, 0.0), (0.0, -0.25, 0.0), (0.0, 0.0, 1.2), (0.12, -0.08, 0.4)],
)
def test_three_wheel_kinematics_round_trip(velocity):
    assert wheels_to_body(*body_to_wheels(*velocity)) == pytest.approx(velocity)


def test_wheel_saturation_preserves_chassis_direction():
    requested = body_to_wheels(0.3, 0.3, 1.57)
    limited = saturate_wheels(requested, 5.0)
    scale = limited[0] / requested[0]
    assert max(abs(value) for value in limited) == pytest.approx(5.0)
    assert limited == pytest.approx(tuple(value * scale for value in requested))


def test_motion_limiter_respects_acceleration_and_jerk():
    limiter = MotionLimiter()
    previous_velocity = limiter.velocity
    previous_acceleration = limiter.acceleration
    for _ in range(50):
        velocity = limiter.update((0.3, -0.3, 1.57), 0.02, (0.6, 0.6, 3.14), (2.0, 2.0, 10.0))
        acceleration = tuple((new - old) / 0.02 for new, old in zip(velocity, previous_velocity))
        assert abs(acceleration[0]) <= 0.6 + 1e-9
        assert abs(acceleration[1]) <= 0.6 + 1e-9
        assert abs(acceleration[2]) <= 3.14 + 1e-9
        assert all(
            abs(new - old) <= limit * 0.02 + 1e-9
            for new, old, limit in zip(acceleration, previous_acceleration, (2.0, 2.0, 10.0))
        )
        previous_velocity, previous_acceleration = velocity, acceleration


def test_real_description_is_untouched_by_simulation_extensions():
    robot = description(False)
    assert robot.find("./joint[@name='sim_base_left_wheel_joint']") is None
    assert robot.find("./gazebo/plugin[@filename='gz-sim-joint-controller-system']") is None
    assert robot.find(".//sensor[@name='front_depth']") is None


def test_simulation_has_physical_wheels_arm_actuator_and_depth():
    robot = description(True)
    for name in (
        "sim_base_left_wheel_joint",
        "sim_base_back_wheel_joint",
        "sim_base_right_wheel_joint",
    ):
        joint = robot.find(f"./joint[@name='{name}']")
        assert joint is not None and joint.attrib["type"] == "continuous"
        assert math.isclose(float(joint.find("limit").attrib["effort"]), 1.2)
    assert robot.find("./gazebo/plugin[@filename='gz-sim-velocity-control-system']") is None
    trajectory = robot.find(
        "./gazebo/plugin[@filename='gz-sim-joint-trajectory-controller-system']"
    )
    assert trajectory is not None
    assert trajectory.findtext("topic") == "/sim/arm/native_joint_trajectory"
    failsafe = robot.find(
        "./gazebo/plugin[@filename='liblekiwi_sim_native_failsafe.so']"
    )
    assert failsafe is not None
    assert [element.text for element in trajectory.findall("joint_name")] == [
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
    ]
    depth = robot.find(".//sensor[@name='front_depth']")
    assert depth is not None
    assert depth.findtext("topic") == "/camera/depth"
    assert float(depth.findtext("update_rate")) == 10.0
    assert float(depth.findtext("camera/noise/stddev")) > 0.0


def test_native_failsafe_owns_the_actual_actuator_topics():
    source = (ROOT / "src" / "sim_native_failsafe.cpp").read_text()
    assert '"/sim/sim_base_left_wheel/cmd_vel"' in source
    assert '"/sim/sim_base_left_wheel/native_cmd_vel"' in source
    assert '"/sim/arm/joint_trajectory"' in source
    assert '"/sim/arm/native_joint_trajectory"' in source
    assert '"/sim/arm/trajectory_heartbeat"' in source
    assert "std::chrono::milliseconds(250)" in source
    assert "HoldTrajectory" in source


def test_simulation_urdf_converts_to_valid_sdf():
    sdf_text = render_simulation_sdf(ROOT / "urdf" / "lekiwi.urdf.xacro")
    with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf") as temporary:
        temporary.write(sdf_text)
        temporary.flush()
        result = subprocess.run(
            ["gz", "sdf", "-k", temporary.name], text=True, capture_output=True, check=True
        )
    assert "Valid." in result.stdout
    sdf = ET.fromstring(sdf_text)
    model = sdf.find("model")
    assert model is not None
    assert model.find("./joint[@name='sim_base_left_wheel_joint']") is not None
    assert model.find(".//sensor[@name='front_depth']") is not None
    directions = {
        "sim_base_left_wheel_contact": "-0.866025 0.5 0",
        "sim_base_back_wheel_contact": "0 -1 0",
        "sim_base_right_wheel_contact": "0.866025 0.5 0",
    }
    for link_name, direction in directions.items():
        link = model.find(f"./link[@name='{link_name}']")
        assert link is not None
        ode = link.find("collision/surface/friction/ode")
        assert ode is not None
        assert float(ode.findtext("mu")) == 1.2
        assert float(ode.findtext("mu2")) == 0.02
        fdir = ode.find("fdir1")
        assert fdir.text == direction
        assert fdir.attrib[f"{{{GZ_XML_NAMESPACE}}}expressed_in"] == "base_footprint"
        assert float(ode.findtext("slip1")) == 0.01
        assert float(ode.findtext("slip2")) == 0.10
