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
    paths = [Path(args[i + 1]) for i, arg in enumerate(args) if arg == "--params-file"]
    if len(paths) != 1:
        raise RuntimeError("expected one managed planner parameter file")
    rclpy.init()
    node = rclpy.create_node("stationary_moveit_reload")
    state = []
    node.create_subscription(String, "/safety/driver_state", lambda msg: state.append(msg.data), 10)

    def call(service, kind, request):
        client = node.create_client(kind, service)
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
        response = call("/move_group/get_parameters", GetParameters, GetParameters.Request(names=names))
        parameters = {}
        for name, value in zip(names, response.values):
            if value.type:
                decoded = Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
                parameters[name] = list(decoded) if value.type >= 5 else decoded
        if parameters.get("collision_detector") != "lekiwi_rmf/RestFCL":
            raise RuntimeError("managed planner is not using the repository collision plugin")
        source = Path(get_package_share_directory("lekiwi_rmf")) / "config/lekiwi.srdf"
        parameters["robot_description_semantic"] = refresh_collision_pairs(
            parameters["robot_description_semantic"], source.read_text()
        )
        # Launch's temporary file may have been removed while its child stayed
        # alive. Recreate it from live parameters so respawn remains repeatable.
        with tempfile.NamedTemporaryFile("w", dir=paths[0].parent, delete=False) as temporary:
            yaml.safe_dump({"/**": {"ros__parameters": parameters}}, temporary)
            temporary_path = Path(temporary.name)
        try:
            os.replace(temporary_path, paths[0])
        finally:
            temporary_path.unlink(missing_ok=True)
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            replacements = managed_planners()
            if len(replacements) == 1 and replacements[0][0] != pid:
                print(f"planner respawned: {pid} -> {replacements[0][0]}")
                return
            time.sleep(0.25)
        raise RuntimeError("managed planner did not respawn within 30 seconds")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
