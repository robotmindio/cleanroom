"""Keep the LD06 auto-selection tied to the actual stable CP2102 port."""

import math
import pathlib
import xml.etree.ElementTree as ET

from launch_snapshot import find_node, find_nodes, resolve_bringup

_URDF_SOURCE = (pathlib.Path(__file__).parents[1] / "urdf" / "lekiwi.urdf.xacro").read_text()
_CAD = ET.parse(pathlib.Path(__file__).parents[1] / "urdf" / "lekiwi_cad.urdf").getroot()
ACTUAL = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
LEGACY = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0"


def test_actual_cp2102_interface_name_is_detected_and_selected():
    records = resolve_bringup(profile="wired", serial_devices=[ACTUAL])
    assert find_node(records, name="ld06_lidar")["parameters"]["port_name"] == ACTUAL
    assert not find_nodes(records, name="free_space")


def test_auto_detection_keeps_the_legacy_interface_name_compatible():
    records = resolve_bringup(profile="wired", serial_devices=[LEGACY])
    assert find_node(records, name="ld06_lidar")["parameters"]["port_name"] == LEGACY


def test_auto_detection_without_an_ld06_uses_the_camera_laser():
    records = resolve_bringup(profile="wired")
    assert not find_nodes(records, name="ld06_lidar")
    assert ["scan", "/scan"] in find_node(records, name="free_space")["remappings"]


def test_scan_filter_subscribes_before_the_pi_publisher_appears():
    # A typed subscription needs no publisher yet, unlike a type-inferring relay.
    records = resolve_bringup(profile="split")
    assert find_node(records, name="scan_self_filter")["parameters"]["input_topic"] == "/pi/lidar/scan"
    assert not find_nodes(records, name="ld06_lidar")


def test_laser_frame_has_a_measured_correction_after_the_nominal_cad_pose():
    assert '<joint name="laser_calibration" type="fixed">' in _URDF_SOURCE
    assert '${lidar_offset_xyz}' in _URDF_SOURCE
    root = ET.fromstring(_URDF_SOURCE)
    yaw = root.find("{http://www.ros.org/wiki/xacro}property[@name='lidar_offset_yaw']")
    assert math.isclose(float(yaw.get("value")), -math.pi / 2, abs_tol=1e-9)


def test_ld06_stays_on_its_robotskin_mount_at_the_installed_plate_pose():
    joints = {joint.get("name"): joint for joint in _CAD.findall("joint")}
    mount = joints["robotskin_lidar_mount_joint"]
    body = joints["ld06_body_mount"]

    assert mount.find("parent").get("link") == "base_plate_layer2-v3"
    assert mount.find("child").get("link") == "robotskin_lidar_mount"
    mount_origin = mount.find("origin")
    assert tuple(map(float, mount_origin.get("xyz").split())) == (0.0, -0.115, 0.007)
    assert tuple(map(float, mount_origin.get("rpy").split())) == (0.0, 0.0, -1.5707963267948966)
    assert not {"Bottom-V2-v3", "Top-V2-v2"} & {
        link.get("name") for link in _CAD.findall("link")
    }
    assert body.find("parent").get("link") == "robotskin_lidar_mount"
    assert body.find("child").get("link") == "ld06_body"
    body_origin = body.find("origin")
    assert tuple(map(float, body_origin.get("xyz").split())) == (0.02, -0.005, 0.012)
    assert tuple(map(float, body_origin.get("rpy").split())) == (0.0, 0.0, 0.0)
