"""Semantic validation for bringup launch arguments.

``DeclareLaunchArgument(..., choices=...)`` catches spelling errors, but it
cannot reject combinations that start a partially functional robot.  Keep this
module ROS-free so the same rules have ordinary, fast unit tests.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import ipaddress
import math
import os
from pathlib import Path
import subprocess
from typing import TYPE_CHECKING

import yaml

if TYPE_CHECKING:
    from lekiwi_rmf.map_bundle import ValidatedMapBundle

# RFC 6598 CGNAT space, which Tailscale allocates every tailnet address from.
# A rosbridge bound here is only reachable over the authenticated, encrypted
# WireGuard mesh -- not the open network -- so it satisfies the same intent
# as "authenticated TLS" without rosbridge having to speak TLS itself.
_TAILSCALE_CGNAT_RANGE = ipaddress.ip_network("100.64.0.0/10")


def _is_tailscale_address(address: str) -> bool:
    try:
        return ipaddress.ip_address(address) in _TAILSCALE_CGNAT_RANGE
    except ValueError:
        return False


# Each deployment profile fills in topology arguments the operator did not
# pass explicitly, so previously installed service arguments keep working.
PROFILES = {
    "sim": {"mode": "sim", "camera_source": "local", "lidar_source": "local", "laser_source": "auto"},
    "wired": {"mode": "real", "camera_source": "local", "lidar_source": "local", "laser_source": "auto"},
    "split": {"mode": "real", "camera_source": "remote", "lidar_source": "remote", "laser_source": "ld06"},
}
CHOICES = {
    "mode": ("real", "sim"),
    "localization": ("amcl", "visual_slam"),
    "slam_mode": ("mapping", "localization"),
    "camera_source": ("local", "remote"),
    "laser_source": ("auto", "camera", "ld06", "none"),
    "lidar_source": ("local", "remote"),
    "safety_policy": ("hold", "strict"),
}
LD06_SERIAL_PORTS = (
    # CP2102's usual Linux interface suffix. This is the device presently
    # attached to this robot (ID_SERIAL_SHORT=0001).
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    # Kept for an earlier udev naming variant seen on the same adapter family.
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0",
)
_TOPOLOGY_KEY = "lekiwi_topology"


def lidar_serial_present() -> bool:
    # /dev/ttyUSB0 is only a kernel-assigned slot: it can just as easily be a
    # console cable or another USB-UART.  Auto mode is deliberately conservative.
    return any(os.path.exists(path) for path in LD06_SERIAL_PORTS)


def lidar_default_port() -> str:
    """Select the stable serial name that actually exists at launch time."""
    return next((path for path in LD06_SERIAL_PORTS if os.path.exists(path)), LD06_SERIAL_PORTS[0])


ARGUMENT_NAMES = (
    "mode",
    "remote_ip",
    "curve_client_secret_key_file",
    "curve_server_public_key_file",
    "safety_policy",
    "start_rmf",
    "start_moveit",
    "start_foxglove",
    "foxglove_address",
    "foxglove_port",
    "start_rosbridge",
    "rosbridge_address",
    "rosbridge_port",
    "rosbridge_domain",
    "localization",
    "slam_mode",
    "publish_camera",
    "publish_astra",
    "hardware_config",
    "camera_source",
    "laser_source",
    "lidar_source",
    "xy_velocity_scale",
    "yaw_velocity_scale",
    "rtabmap_database",
    "rtabmap_wm_nodes",
    "rtabmap_mapping_max_bytes",
    "rtabmap_mapping_max_seconds",
    "map_bundle",
    "static_map",
)


def _bool(value: object, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.lower() in {"true", "false"}:
        return value.lower() == "true"
    raise ValueError(f"{name} must be true or false, got {value!r}")


def _positive_float(value: object, name: str) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a positive number, got {value!r}") from error
    if not math.isfinite(numeric) or numeric <= 0:
        raise ValueError(f"{name} must be finite and positive, got {value!r}")
    return numeric


def _finite_float(value: object, name: str) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number, got {value!r}") from error
    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return numeric


def _port(value: object, name: str) -> int:
    try:
        port = int(str(value), 10)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a TCP port, got {value!r}") from error
    if not 1 <= port <= 65535:
        raise ValueError(f"{name} must be between 1 and 65535, got {value!r}")
    return port


def _nonnegative_int(value: object, name: str, maximum: int | None = None) -> int:
    try:
        number = int(str(value), 10)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a non-negative integer, got {value!r}") from error
    if number < 0:
        raise ValueError(f"{name} must be non-negative, got {value!r}")
    if maximum is not None and number > maximum:
        raise ValueError(f"{name} must be at most {maximum}, got {value!r}")
    return number


def _positive_int(value: object, name: str) -> int:
    number = _nonnegative_int(value, name)
    if number == 0:
        raise ValueError(f"{name} must be a positive integer, got {value!r}")
    return number


def astra_serial_from_hardware_config(path: str | Path, *, required: bool) -> str:
    """Read the deployment-pinned Astra identity from tracked YAML."""
    config_path = Path(path).expanduser()
    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read hardware configuration {config_path}: {error}") from error
    try:
        serial = data["astra"]["serial_number"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "hardware configuration must contain astra.serial_number"
        ) from error
    if not isinstance(serial, str):
        raise ValueError("hardware configuration astra.serial_number must be a string")
    serial = serial.strip()
    if required and not serial:
        raise ValueError(
            "real local Astra RGB-D requires a non-empty tracked astra.serial_number"
        )
    return serial


def permission_timeout(config_directory: str | Path) -> float:
    """The supervisor's motion-permission lease from the base safety profile."""
    path = Path(config_directory) / "safety_production.yaml"
    parameters = yaml.safe_load(path.read_text(encoding="utf-8"))["safety_supervisor"]["ros__parameters"]
    return float(parameters["permission_timeout"])


def validate_launch_arguments(
    arguments: Mapping[str, object],
    *,
    trusted_rosbridge_addresses: frozenset[str] = frozenset(),
) -> ValidatedMapBundle | None:
    """Raise ``ValueError`` unless resolved bringup arguments are coherent.

    Returns the validated map bundle when the configuration uses one, else ``None``.
    """
    missing = [name for name in ARGUMENT_NAMES if name not in arguments]
    if missing:
        raise ValueError(f"missing launch arguments: {', '.join(missing)}")

    mode = str(arguments["mode"])
    localization = str(arguments["localization"])
    slam_mode = str(arguments["slam_mode"])
    camera_source = str(arguments["camera_source"])
    laser_source = str(arguments["laser_source"])
    lidar_source = str(arguments["lidar_source"])
    remote_ip = str(arguments["remote_ip"]).strip()
    curve_client_secret = str(arguments["curve_client_secret_key_file"]).strip()
    curve_server_public = str(arguments["curve_server_public_key_file"]).strip()
    rtabmap_database = str(arguments["rtabmap_database"]).strip()
    map_bundle = str(arguments["map_bundle"]).strip()
    for name, choices in CHOICES.items():
        if str(arguments[name]) not in choices:
            raise ValueError(f"{name} must be one of {', '.join(choices)}, got {arguments[name]!r}")

    start_rmf = _bool(arguments["start_rmf"], "start_rmf")
    _bool(arguments["start_moveit"], "start_moveit")
    start_foxglove = _bool(arguments["start_foxglove"], "start_foxglove")
    foxglove_address = str(arguments["foxglove_address"]).strip()
    start_rosbridge = _bool(arguments["start_rosbridge"], "start_rosbridge")
    rosbridge_address = str(arguments["rosbridge_address"]).strip()
    publish_camera = _bool(arguments["publish_camera"], "publish_camera")
    _bool(arguments["publish_astra"], "publish_astra")
    static_map = _bool(arguments["static_map"], "static_map")
    _port(arguments["foxglove_port"], "foxglove_port")
    _port(arguments["rosbridge_port"], "rosbridge_port")
    _nonnegative_int(arguments["rosbridge_domain"], "rosbridge_domain", maximum=232)
    _positive_float(arguments["xy_velocity_scale"], "xy_velocity_scale")
    _positive_float(arguments["yaw_velocity_scale"], "yaw_velocity_scale")
    _positive_int(arguments["rtabmap_wm_nodes"], "rtabmap_wm_nodes")
    _positive_int(arguments["rtabmap_mapping_max_bytes"], "rtabmap_mapping_max_bytes")
    _positive_float(arguments["rtabmap_mapping_max_seconds"], "rtabmap_mapping_max_seconds")
    if not remote_ip:
        raise ValueError("remote_ip must be non-empty")
    if bool(curve_client_secret) != bool(curve_server_public):
        raise ValueError("both CURVE client secret and server public key paths are required")
    if not rtabmap_database:
        raise ValueError("rtabmap_database must be non-empty")
    if not map_bundle:
        raise ValueError("map_bundle must be non-empty")
    if not str(arguments["hardware_config"]).strip():
        raise ValueError("hardware_config must be non-empty")
    # Simulation is a disposable test topology: its ZMQ clients and WebSocket
    # endpoints may be reached by the integration-test runner. Real robot
    # deployments remain loopback-only, or on this tailnet's own CGNAT
    # address (already authenticated and encrypted by Tailscale), until the
    # bridge speaks TLS itself.
    if (
        mode == "real"
        and start_foxglove
        and foxglove_address not in {"127.0.0.1", "::1"}
        and not _is_tailscale_address(foxglove_address)
    ):
        raise ValueError(
            "foxglove may bind only to loopback or a tailnet address until authenticated TLS is configured"
        )
    if (
        mode == "real"
        and start_rosbridge
        and rosbridge_address not in {"127.0.0.1", "::1"}
        and rosbridge_address not in trusted_rosbridge_addresses
    ):
        raise ValueError("rosbridge may bind only to loopback or an assigned Tailscale address")

    if localization == "visual_slam" and not publish_camera:
        raise ValueError("visual_slam requires publish_camera:=true")
    if mode == "real" and laser_source == "none":
        raise ValueError("real navigation requires laser_source:=camera, ld06, or auto")
    if mode == "real" and laser_source == "camera" and not publish_camera:
        raise ValueError("laser_source:=camera requires publish_camera:=true")
    if mode == "real" and lidar_source == "remote" and laser_source == "auto":
        # auto only looks for a lidar on this machine, so it would start the
        # camera laser and never relay the device's LD06.
        raise ValueError("lidar_source:=remote requires laser_source:=ld06")
    if mode == "sim" and camera_source == "remote":
        raise ValueError("camera_source:=remote is unsupported in simulation")
    if mode == "sim" and lidar_source == "remote":
        raise ValueError("lidar_source:=remote is unsupported in simulation")
    # slam_mode configures RTAB-Map only; amcl always localizes on a fixed map.
    if start_rmf and localization == "visual_slam" and slam_mode == "mapping":
        raise ValueError("start_rmf:=true requires slam_mode:=localization and a validated map bundle")
    if start_rmf and localization != "amcl":
        raise ValueError("RMF operation requires amcl with the immutable occupancy-map bundle")
    if start_rmf or localization == "amcl" or static_map:
        from lekiwi_rmf.map_bundle import validate_map_bundle

        return validate_map_bundle(map_bundle, require_approved=True)
    return None


@dataclass(frozen=True)
class Topology:
    """Which parts of the stack one validated bringup starts."""

    sim: bool
    amcl: bool
    visual_slam: bool
    slam_mapping: bool
    static_map: bool
    camera_on: bool
    # Front/wrist cameras read by v4l2 on this machine.
    local_cameras: bool
    astra_here: bool
    # The cameras face different directions: never assign Astra depth to front
    # RGB. Native RGB-D synchronization preserves each camera's measured view.
    dual_rgbd: bool
    remote_camera: bool
    # Gazebo supplies /scan in sim; RTAB-Map subscribes to it there too.
    lidar_on: bool
    camera_laser: bool
    ld06: bool
    remote_ld06: bool
    start_rmf: bool
    start_moveit: bool
    start_foxglove: bool
    start_rosbridge: bool

    @property
    def real(self) -> bool:
        return not self.sim


def resolve_topology(arguments: Mapping[str, object], *, lidar_detected: bool) -> Topology:
    """Derive the started components from validated launch arguments."""
    real = str(arguments["mode"]) == "real"
    flag = {name: _bool(arguments[name], name) for name in (
        "publish_camera", "publish_astra", "static_map",
        "start_rmf", "start_moveit", "start_foxglove", "start_rosbridge")}
    camera_on = flag["publish_camera"]
    camera_local = str(arguments["camera_source"]) == "local"
    laser = str(arguments["laser_source"])
    lidar_local = str(arguments["lidar_source"]) == "local"
    visual_slam = str(arguments["localization"]) == "visual_slam"
    # Whoever owns /scan owns what Nav2 dodges: a real LD06 on its RobotSkin
    # base, the front camera's floor-geometry trick, or nobody at all.
    auto_ld06 = laser == "auto" and lidar_detected
    return Topology(
        sim=not real,
        amcl=not visual_slam,
        visual_slam=visual_slam,
        slam_mapping=str(arguments["slam_mode"]) == "mapping",
        static_map=flag["static_map"],
        camera_on=camera_on,
        local_cameras=camera_on and camera_local and real,
        astra_here=camera_on and camera_local and real and flag["publish_astra"],
        dual_rgbd=camera_on and real and flag["publish_astra"],
        remote_camera=camera_on and real and not camera_local,
        lidar_on=laser != "none",
        camera_laser=real and (laser == "camera" or (laser == "auto" and not lidar_detected)),
        ld06=real and lidar_local and (laser == "ld06" or auto_ld06),
        remote_ld06=real and not lidar_local and laser == "ld06",
        start_rmf=flag["start_rmf"],
        start_moveit=flag["start_moveit"],
        start_foxglove=flag["start_foxglove"],
        start_rosbridge=flag["start_rosbridge"],
    )


def launch_topology(context) -> Topology:
    """The topology ``validate_context`` resolved once for this launch."""
    return context.get_locals_as_dict()[_TOPOLOGY_KEY]


def _tailscale_ipv4_addresses() -> frozenset[str]:
    """Return IPv4 addresses assigned to the authenticated Tailscale interface."""
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "address", "show", "dev", "tailscale0"],
            check=False, capture_output=True, text=True,
        )
    except OSError:
        return frozenset()
    addresses = set()
    for line in result.stdout.splitlines():
        fields = line.split()
        if "inet" not in fields:
            continue
        address = fields[fields.index("inet") + 1].split("/", 1)[0]
        try:
            addresses.add(str(ipaddress.IPv4Address(address)))
        except ipaddress.AddressValueError:
            continue
    return frozenset(addresses)


def validate_context(context, *_args, **_kwargs):
    """``launch.actions.OpaqueFunction`` adapter for ``bringup.launch.py``.

    Add ``OpaqueFunction(function=validate_context)`` after declarations and
    before every node/include action.  Raising here stops launch before any
    partially configured process is spawned.
    """
    from launch.substitutions import LaunchConfiguration

    values = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENT_NAMES}
    bundle = validate_launch_arguments(
        values, trusted_rosbridge_addresses=_tailscale_ipv4_addresses()
    )
    # Resolve key existence and secret-file permissions in the preflight
    # OpaqueFunction, before any camera, mapper, or driver process starts.
    from lekiwi_rmf.zmq_security import CurveClientCredentials

    CurveClientCredentials(
        str(values["curve_client_secret_key_file"]),
        str(values["curve_server_public_key_file"]),
    ).validate()
    astra_required = (
        str(values["mode"]) == "real"
        and _bool(values["publish_camera"], "publish_camera")
        and str(values["camera_source"]) == "local"
        and _bool(values["publish_astra"], "publish_astra")
    )
    astra_serial = astra_serial_from_hardware_config(
        str(values["hardware_config"]), required=astra_required
    )
    # Resolve the topology once, from exactly the values just validated and
    # one look at the serial bus; every bringup launch file reads this copy.
    context.extend_globals({_TOPOLOGY_KEY: resolve_topology(
        values, lidar_detected=lidar_serial_present())})
    from launch.actions import SetLaunchConfiguration

    selected = [SetLaunchConfiguration("astra_serial", astra_serial)]
    if bundle is not None:
        selected.append(SetLaunchConfiguration("selected_map", str(bundle.occupancy_yaml)))
        if _bool(values["start_rmf"], "start_rmf"):
            selected.extend([
                SetLaunchConfiguration("selected_nav_graph", str(bundle.navigation_graph)),
                SetLaunchConfiguration("selected_fleet_config", str(bundle.fleet_config)),
            ])
    return selected
