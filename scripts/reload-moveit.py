#!/usr/bin/env python3
"""Reload the managed planner's collision rules while its driver is stopped.

Source scripts/setup.bash first. Does not restart motor/device services or
request a torque change. The existing launch process respawns move_group.
--release selects a sealed compatible planner-only release; finish the ordinary
deployment after recovering travel_stow. Deployment markers are never updated here.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml


def planner_release(release):
    """Allow a sealed planner-only update; other running nodes keep identical inputs."""
    target = Path(release).expanduser().resolve(strict=True)
    current = target.parent.parent / 'current'
    previous = current.resolve(strict=True)
    if target.parent.name != 'releases' or not current.is_symlink() or previous.parent != target.parent:
        raise ValueError('planner recovery requires a release beside the selected workspace release')
    manifests = []
    verifier = Path(__file__).with_name('check-release.py')
    for path in (previous, target):
        manifest = json.loads((path / 'release.json').read_text())
        subprocess.run([sys.executable, str(verifier), 'verify', str(path), path.name, 'compute'], check=True)
        manifests.append(manifest)
    old, new = manifests
    if old['dependency_overlay'] != new['dependency_overlay'] or old['settings'] != new['settings']:
        raise ValueError('planner recovery cannot change the dependency overlay or local settings')
    changed = subprocess.check_output([
        'git', '-C', str(target / 'source'), 'diff', '--name-only', previous.name, target.name,
    ], text=True).splitlines()
    allowed = {'src/rest_collision_env.hpp', 'config/lekiwi.srdf', 'scripts/reload-moveit.py'}
    if any(name not in allowed and not name.startswith(('test/', 'docs/')) for name in changed):
        raise ValueError('planner recovery refuses changes outside collision rules, tests and documentation')
    return current, previous, target


def select_planner_release(current, previous, target):
    if current.resolve(strict=True) != previous:
        raise RuntimeError('selected release changed during planner recovery')
    with tempfile.TemporaryDirectory(dir=current.parent, prefix='.planner-release-') as directory:
        link = Path(directory) / 'current'
        link.symlink_to(target)
        os.replace(link, current)


def refresh_collision_pairs(current, source):
    """Refresh collision pairs/support reference, preserving calibrated poses."""
    live = ET.fromstring(current)
    tracked = ET.fromstring(source)
    if live.get("name") != tracked.get("name"):
        raise ValueError("live and installed robot names differ")
    for pair in live.findall("disable_collisions"):
        live.remove(pair)
    for pair in tracked.findall("disable_collisions"):
        live.append(pair)
    reference = "group_state[@name='supported_rest'][@group='arm']"
    supports = tracked.findall(reference)
    if len(supports) != 1:
        raise ValueError("tracked collision rules require one supported_rest arm reference")
    for state in live.findall(reference):
        live.remove(state)
    live.append(supports[0])
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

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, help='sealed planner-only compute release to select without a torque change')
    options = parser.parse_args()
    logs = Path(os.environ.get('LEKIWI_LOGS', Path.home() / '.ros/lekiwi'))
    logs.mkdir(parents=True, exist_ok=True)
    deployment_lock = (logs / 'deploy.lock').open('a')
    fcntl.flock(deployment_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    release = planner_release(options.release) if options.release else None
    planners = managed_planners()
    if len(planners) != 1:
        raise RuntimeError(f"expected one managed move_group, found {len(planners)}")
    pid, args = planners[0]
    if release:
        environment = dict(item.split(b'=', 1) for item in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0') if b'=' in item)
        prefix = str(release[0] / 'install/lekiwi_rmf')
        if (prefix not in environment.get(b'AMENT_PREFIX_PATH', b'').decode().split(':') or
                prefix + '/lib' not in environment.get(b'LD_LIBRARY_PATH', b'').decode().split(':')):
            raise RuntimeError('managed planner does not follow workspace/current for its model and libraries')
    parameter_path = planner_parameter_path(pid, args)
    rclpy.init()
    node = rclpy.create_node("stationary_moveit_reload")
    state = []
    node.create_subscription(String, "/safety/driver_state", lambda msg: state.append((msg.data, time.monotonic())), 10)
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
        if not state or state[-1][0] != "DISARMED" or time.monotonic() - state[-1][1] > 0.5:
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
        package = release[2] / 'install/lekiwi_rmf/share/lekiwi_rmf' if release else Path(get_package_share_directory("lekiwi_rmf"))
        source = package / "config/lekiwi.srdf"
        parameters["robot_description_semantic"] = refresh_collision_pairs(
            parameters["robot_description_semantic"], source.read_text()
        )
        # Render initialized live values into the planner's startup overrides.
        rclpy.spin_once(node, timeout_sec=0.1)
        if not state or state[-1][0] != 'DISARMED' or time.monotonic() - state[-1][1] > 0.5:
            raise RuntimeError('driver is no longer confirmed stopped; planner recovery refused')
        write_parameters(parameter_path, parameters)
        if release:
            select_planner_release(*release)
            print(f'selected sealed planner-only release {release[2].name}; deployment markers remain unchanged')
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            replacements = managed_planners()
            if (len(replacements) == 1 and replacements[0][0] != pid and
                    clients["/move_group/get_parameters"].wait_for_service(timeout_sec=0.25)):
                response = call("/move_group/get_parameters", GetParameters,
                                GetParameters.Request(names=["robot_description_semantic"]))
                if response.values and response.values[0].string_value == parameters["robot_description_semantic"]:
                    if release:
                        library = release[2] / 'install/lekiwi_rmf/lib/liblekiwi_rest_collision.so'
                        if str(library.resolve(strict=True)) not in Path(f'/proc/{replacements[0][0]}/maps').read_text():
                            time.sleep(0.25)
                            continue
                    print(f"planner respawned with verified collision rules: {pid} -> {replacements[0][0]}")
                    return
            time.sleep(0.25)
        raise RuntimeError("managed planner did not respawn within 30 seconds")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        deployment_lock.close()


if __name__ == "__main__":
    main()
