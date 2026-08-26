"""Keep the LD06 auto-selection tied to the actual stable CP2102 port."""

import ast
import pathlib
import types


_SOURCE = (pathlib.Path(__file__).parents[1] / "launch" / "bringup.launch.py").read_text()
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
