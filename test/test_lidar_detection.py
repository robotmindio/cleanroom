"""Keep the LD06 auto-selection tied to the actual stable CP2102 port."""

import ast
import pathlib
import types
import xml.etree.ElementTree as ET


_SOURCE = (pathlib.Path(__file__).parents[1] / "launch" / "bringup.launch.py").read_text()
_URDF_SOURCE = (pathlib.Path(__file__).parents[1] / "urdf" / "lekiwi.urdf.xacro").read_text()
_CAD = ET.parse(pathlib.Path(__file__).parents[1] / "urdf" / "lekiwi_cad.urdf").getroot()
_TREE = ast.parse(_SOURCE)
_NAMES = {"LD06_SERIAL_PORTS", "_lidar_serial_present", "_lidar_default_port"}
_NODES = [
    node
    for node in _TREE.body
    if (isinstance(node, ast.Assign) and any(getattr(target, "id", None) in _NAMES for target in node.targets))
    or getattr(node, "name", None) in _NAMES
]
lidar = types.ModuleType("lidar_detection_under_test")
exec(compile(ast.Module(body=_NODES, type_ignores=[]), "bringup.launch.py", "exec"), lidar.__dict__)


def test_actual_cp2102_interface_name_is_detected_and_selected(monkeypatch):
    actual = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    lidar.os = types.SimpleNamespace(path=types.SimpleNamespace(exists=lambda path: path == actual))

    assert lidar._lidar_serial_present()
    assert lidar._lidar_default_port() == actual


def test_auto_detection_keeps_the_legacy_interface_name_compatible():
    legacy = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0"
    lidar.os = types.SimpleNamespace(path=types.SimpleNamespace(exists=lambda path: path == legacy))

    assert lidar._lidar_serial_present()
    assert lidar._lidar_default_port() == legacy


def test_remote_relay_has_a_laserscan_type_before_the_pi_publisher_appears():
    assert 'arguments=["/pi/lidar/scan", "/scan", "sensor_msgs/msg/LaserScan"]' in _SOURCE


def test_laser_frame_has_a_measured_correction_after_the_nominal_cad_pose():
    assert '<joint name="laser_calibration" type="fixed">' in _URDF_SOURCE
    assert '${lidar_offset_xyz}' in _URDF_SOURCE


def test_ld06_stays_on_its_robotskin_mount_at_the_installed_plate_pose():
    joints = {joint.get("name"): joint for joint in _CAD.findall("joint")}
    mount = joints["robotskin_lidar_mount_joint"]
    body = joints["ld06_body_mount"]

    assert mount.find("parent").get("link") == "base_plate_layer1-v5"
    assert mount.find("child").get("link") == "robotskin_lidar_mount"
    assert mount.find("origin").attrib == {"xyz": "0.055 0.08 0", "rpy": "0 0 0"}
    assert body.find("parent").get("link") == "robotskin_lidar_mount"
    assert body.find("child").get("link") == "ld06_body"
    assert body.find("origin").attrib == {"xyz": "0.02 -0.005 0.012", "rpy": "0 0 0"}
