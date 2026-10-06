#!/usr/bin/env python3
"""Reload the managed planner's collision rules while its driver is stopped.

Source scripts/setup.bash first. Does not restart motor/device services or
request a torque change. The existing launch process respawns move_group.
"""
import os
from pathlib import Path
import signal
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml


def refresh_collision_pairs(current, source):
    """Preserve live poses/calibration, replace only tracked collision pairs."""
    live = ET.fromstring(current)
    tracked = ET.fromstring(source)
    if live.get("name") != tracked.get("name"):
        raise ValueError("live and installed robot names differ")
    for pair in live.findall("disable_collisions"):
        live.remove(pair)
    for pair in tracked.findall("disable_collisions"):
        live.append(pair)
    return ET.tostring(live, encoding="unicode")


def managed_planners():
    matches = []
    for process in Path("/proc").glob("[0-9]*"):
        try:
            if process.stat().st_uid != os.getuid():
                continue
            args = process.joinpath("cmdline").read_bytes().decode().rstrip("\0").split("\0")
            if args and args[0].endswith("/moveit_ros_move_group/move_group") and (
                "lekiwi-stack.service" in process.joinpath("cgroup").read_text()
            ):
                matches.append((int(process.name), args))
        except FileNotFoundError:
            continue  # Processes can exit during discovery.
    return matches


def planner_parameter_path(pid, args):
    paths = [Path(args[i + 1]) for i, arg in enumerate(args) if arg == "--params-file"]
    if len(paths) != 1 or not paths[0].is_absolute():
        raise RuntimeError("expected one absolute managed planner parameter file")
    # systemd PrivateTmp gives the planner a different /tmp. Access its mount
    # namespace; the same path in our shell refers to an unrelated file.
    return Path(f"/proc/{pid}/root") / paths[0].relative_to("/")


def write_parameters(path, parameters):
    # YAML [] loses its array type in rcl's parser. Empty arrays observed in
    # MoveIt are declared defaults; let the plugin declare them on restart.
    parameters = {name: value for name, value in parameters.items() if value != []}
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as temporary:
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}, temporary)
        temporary_path = Path(temporary.name)
    try:
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def main():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rcl_interfaces.msg import Parameter as ParameterMessage
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    planners = managed_planners()
    if len(planners) != 1:
        raise RuntimeError(f"expected one managed move_group, found {len(planners)}")
    pid, args = planners[0]
    parameter_path = planner_parameter_path(pid, args)
    rclpy.init()
    node = rclpy.create_node("stationary_moveit_reload")
    state = []
    node.create_subscription(String, "/safety/driver_state", lambda msg: state.append(msg.data), 10)
    clients = {}

    def call(service, kind, request):
        if service not in clients:
            clients[service] = node.create_client(kind, service)
        client = clients[service]
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"service unavailable: {service}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError(f"service timed out: {service}")
        return future.result()

    try:
        deadline = time.monotonic() + 3
        while not state and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not state or state[-1] != "DISARMED":
            raise RuntimeError("planner reload requires the already-stopped driver")
        names = call("/move_group/list_parameters", ListParameters, ListParameters.Request()).result.names
        parameters = {}
        uninitialized = []
        # MoveIt declares optional limits without initializing them. One such
        # name makes rclcpp reject an entire batch, so read each name separately.
        for name in names:
            response = call("/move_group/get_parameters", GetParameters, GetParameters.Request(names=[name]))
            if not response.values:
                uninitialized.append(name)
                continue
            if len(response.values) != 1:
                raise RuntimeError(f"unexpected parameter response: {name}")
            value = response.values[0]
            if value.type:
                decoded = Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
                parameters[name] = list(decoded) if value.type >= 5 else decoded
        if parameters.get("collision_detector") != "lekiwi_rmf/RestFCL":
            raise RuntimeError("managed planner is not using the repository collision plugin")
        if uninitialized:
            print(f"preserved {len(parameters)} parameters; omitted {len(uninitialized)} uninitialized declarations")
        source = Path(get_package_share_directory("lekiwi_rmf")) / "config/lekiwi.srdf"
        parameters["robot_description_semantic"] = refresh_collision_pairs(
            parameters["robot_description_semantic"], source.read_text()
        )
        # Render initialized live values into the planner's startup overrides.
        write_parameters(parameter_path, parameters)
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            replacements = managed_planners()
            if (len(replacements) == 1 and replacements[0][0] != pid and
                    clients["/move_group/get_parameters"].wait_for_service(timeout_sec=0.25)):
                response = call("/move_group/get_parameters", GetParameters,
                                GetParameters.Request(names=["robot_description_semantic"]))
                if response.values and response.values[0].string_value == parameters["robot_description_semantic"]:
                    print(f"planner respawned with verified collision rules: {pid} -> {replacements[0][0]}")
                    return
            time.sleep(0.25)
        raise RuntimeError("managed planner did not respawn within 30 seconds")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
