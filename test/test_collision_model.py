"""Regression checks for the conservative CAD-backed MoveIt collision model."""

import pathlib
import subprocess
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).parents[1]


def _real_robot() -> ET.Element:
    description = subprocess.check_output(
        ["xacro", str(ROOT / "urdf" / "lekiwi.urdf.xacro"), "sim:=false"], text=True
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
