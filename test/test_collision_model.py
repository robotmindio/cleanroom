"""Regression checks for the conservative CAD-backed MoveIt collision model."""

import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = pathlib.Path(__file__).parents[1]


def _real_robot() -> ET.Element:
    description = subprocess.check_output(
        ["xacro", str(ROOT / "urdf" / "lekiwi.urdf.xacro"), "sim:=false"], text=True
    )
    return ET.fromstring(description)


def _sim_robot() -> ET.Element:
    description = subprocess.check_output(
        ["xacro", str(ROOT / "urdf" / "lekiwi.urdf.xacro"), "sim:=true"], text=True
    )
    return ET.fromstring(description)


def test_arm_has_complete_link_and_servo_collision_envelopes():
    robot = _real_robot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    expected = {
        "arm_pedestal_collision_proxy",
        "front_camera_collision_proxy",
        "shoulder_collision_proxy",
        "upper_arm_collision_proxy",
        "forearm_collision_proxy",
        "wrist_collision_proxy",
        "roll_collision_proxy",
        "gripper_collision_proxy",
        "shoulder_pan_servo_collision_proxy",
        "shoulder_lift_servo_collision_proxy",
        "elbow_servo_collision_proxy",
        "wrist_flex_servo_collision_proxy",
        "wrist_roll_servo_collision_proxy",
        "gripper_servo_collision_proxy",
    }

    assert expected <= links.keys()
    assert all(links[name].find("collision") is not None for name in expected)


def test_long_arm_sections_use_capsules_not_joint_center_spheres():
    robot = _real_robot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for name in (
        "shoulder_collision_proxy",
        "upper_arm_collision_proxy",
        "forearm_collision_proxy",
        "roll_collision_proxy",
        "gripper_collision_proxy",
    ):
        geometries = [collision.find("geometry") for collision in links[name].findall("collision")]
        assert any(geometry.find("cylinder") is not None for geometry in geometries)
        assert sum(geometry.find("sphere") is not None for geometry in geometries) == 2


def test_srdf_collision_exemptions_reference_real_links_only():
    robot = _real_robot()
    links = {link.attrib["name"] for link in robot.findall("link")}
    srdf = ET.parse(ROOT / "config" / "lekiwi.srdf").getroot()

    for exemption in srdf.findall("disable_collisions"):
        assert exemption.attrib["link1"] in links
        assert exemption.attrib["link2"] in links


def test_wrist_roll_is_bounded_identically_on_hardware_and_simulation():
    real_joint = _real_robot().find("./joint[@name='arm_wrist_roll']")
    assert real_joint is not None
    assert real_joint.attrib["type"] == "revolute"
    limit = real_joint.find("limit")
    assert limit is not None
    assert float(limit.attrib["lower"]) == pytest.approx(-math.pi)
    assert float(limit.attrib["upper"]) == pytest.approx(math.pi)
    assert float(limit.attrib["velocity"]) == pytest.approx(3.0)

    sim_joint = _sim_robot().find("./joint[@name='arm_wrist_roll']")
    assert sim_joint is not None
    assert sim_joint.attrib["type"] == "revolute"
    sim_limit = sim_joint.find("limit")
    assert sim_limit is not None
    assert float(sim_limit.attrib["lower"]) == pytest.approx(-math.pi)
    assert float(sim_limit.attrib["upper"]) == pytest.approx(math.pi)


def test_rmf_circle_encloses_the_tracked_nav2_polygon():
    nav2 = yaml.safe_load((ROOT / "config" / "nav2_params.yaml").read_text())
    polygon = yaml.safe_load(
        nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )
    local_parameters = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    global_parameters = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    polygon_radius = max(math.hypot(float(x), float(y)) for x, y in polygon)
    fleet = yaml.safe_load((ROOT / "config" / "fleet_config.yaml").read_text())
    rmf_radius = float(fleet["rmf_fleet"]["profile"]["footprint"])
    bundle = yaml.safe_load(
        (ROOT / "maps" / "bundles" / "cleanroom-development.yaml").read_text()
    )

    assert rmf_radius >= polygon_radius
    assert float(bundle["robot_footprint_radius"]) == rmf_radius
    # The map/RMF radius is the whole live Nav2 envelope, not the raw polygon
    # plus a hidden padding margin.
    assert float(local_parameters["footprint_padding"]) == 0.0
    assert float(global_parameters["footprint_padding"]) == 0.0


def test_moveit_and_rviz_share_tracked_scaling_and_depth_defaults():
    limits = yaml.safe_load((ROOT / "config" / "joint_limits.yaml").read_text())
    rviz = yaml.safe_load((ROOT / "config" / "lekiwi.rviz").read_text())
    planning_display = next(
        display for display in rviz["Visualization Manager"]["Displays"]
        if display.get("Class") == "moveit_rviz_plugin/MotionPlanning"
    )
    sensors = yaml.safe_load((ROOT / "config" / "moveit_sensors.yaml").read_text())

    assert planning_display["Planning Group"] == "arm"
    assert planning_display["Velocity_Scaling_Factor"] == pytest.approx(
        limits["default_velocity_scaling_factor"]
    )
    assert planning_display["Acceleration_Scaling_Factor"] == pytest.approx(
        limits["default_acceleration_scaling_factor"]
    )
    assert sensors["point_cloud"]["point_cloud_topic"] == "/camera/depth/points"
    assert sensors["point_cloud"]["sensor_plugin"] == (
        "occupancy_map_monitor/PointCloudOctomapUpdater"
    )
