import time
from pathlib import Path
import runpy

import rclpy


def spin_until(node, predicate, timeout: float = 5.0, period: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=period)
        if predicate():
            return True
    return False


def bringup_configuration(monkeypatch, **arguments):
    """Resolve actual launch inputs without starting ROS nodes or processes."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.substitutions import TextSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare

    root = Path(__file__).resolve().parents[1]
    module = runpy.run_path(str(root / "launch/bringup.launch.py"))
    generate = module["generate_launch_description"]
    specs = []

    def capture_node(**kwargs):
        specs.append(kwargs)
        return Node(**kwargs)

    def capture_include(*args, **kwargs):
        action = IncludeLaunchDescription(*args, **kwargs)
        specs.append({"include": action})
        return action

    monkeypatch.setitem(generate.__globals__, "IncludeLaunchDescription", capture_include)
    monkeypatch.setitem(generate.__globals__, "Node", capture_node)
    monkeypatch.setitem(generate.__globals__, "get_package_share_directory", lambda _name: str(root))
    monkeypatch.setitem(generate.__globals__, "FindPackageShare", lambda name: (
        TextSubstitution(text=str(root)) if name == "lekiwi_rmf" else FindPackageShare(name)
    ))
    description = generate()
    context = LaunchContext()
    context.launch_configurations.update({
        "profile": "wired", "rtabmap_database": "/test/uncreated-rtabmap.db", **arguments,
    })
    for action in module["_deployment_defaults"](context):
        action.execute(context)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for action in module["_mapping_relocalization_gate"](context):
        action.execute(context)
    return context, specs


def node_parameters(context, spec):
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    import yaml

    parameters = {}
    for value in evaluate_parameters(context, normalize_parameters(spec["parameters"])):
        if isinstance(value, Path):
            parameters.update(yaml.safe_load(value.read_text())[spec["name"]]["ros__parameters"])
        else:
            parameters.update(value)
    return parameters
