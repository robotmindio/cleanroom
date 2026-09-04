"""Semantic validation for bringup launch arguments.

``DeclareLaunchArgument(..., choices=...)`` catches spelling errors, but it
cannot reject combinations that start a partially functional robot.  Keep this
module ROS-free so the same rules have ordinary, fast unit tests.
"""

from __future__ import annotations

from collections.abc import Mapping
import ipaddress
import math
from pathlib import Path

import yaml

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

ARGUMENT_NAMES = (
    "mode",
    "remote_ip",
    "curve_client_secret_key_file",
    "curve_server_public_key_file",
    "auto_arm_on_startup",
    "start_rmf",
    "rmf_domain",
    "start_moveit",
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


def validate_launch_arguments(arguments: Mapping[str, object]) -> None:
    """Raise ``ValueError`` unless resolved bringup arguments are coherent."""
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
    if mode not in {"real", "sim"}:
        raise ValueError(f"mode must be real or sim, got {mode!r}")
    if localization not in {"amcl", "visual_slam"}:
        raise ValueError(f"localization must be amcl or visual_slam, got {localization!r}")
    if slam_mode not in {"mapping", "localization"}:
        raise ValueError(f"slam_mode must be mapping or localization, got {slam_mode!r}")
    if camera_source not in {"local", "remote"}:
        raise ValueError(f"camera_source must be local or remote, got {camera_source!r}")
    if laser_source not in {"auto", "camera", "ld06", "none"}:
        raise ValueError(f"laser_source must be auto, camera, ld06, or none, got {laser_source!r}")
    if lidar_source not in {"local", "remote"}:
        raise ValueError(f"lidar_source must be local or remote, got {lidar_source!r}")

    start_rmf = _bool(arguments["start_rmf"], "start_rmf")
    _bool(arguments["auto_arm_on_startup"], "auto_arm_on_startup")
    _bool(arguments["start_moveit"], "start_moveit")
    start_rosbridge = _bool(arguments["start_rosbridge"], "start_rosbridge")
    rosbridge_address = str(arguments["rosbridge_address"]).strip()
    publish_camera = _bool(arguments["publish_camera"], "publish_camera")
    _bool(arguments["publish_astra"], "publish_astra")
    static_map = _bool(arguments["static_map"], "static_map")
    _port(arguments["rosbridge_port"], "rosbridge_port")
    rmf_domain = _nonnegative_int(arguments["rmf_domain"], "rmf_domain", maximum=232)
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
    # Simulation is a disposable test topology: its ZMQ clients and rosbridge
    # endpoint may be reached by the integration-test runner. Real robot
    # deployments remain loopback-only, or on this tailnet's own CGNAT
    # address (already authenticated and encrypted by Tailscale), until
    # rosbridge speaks TLS itself.
    if (
        mode == "real"
        and start_rosbridge
        and rosbridge_address not in {"127.0.0.1", "::1"}
        and not _is_tailscale_address(rosbridge_address)
    ):
        raise ValueError(
            "rosbridge may bind only to loopback or a tailnet address until authenticated TLS is configured"
        )

    if localization == "visual_slam" and not publish_camera:
        raise ValueError("visual_slam requires publish_camera:=true")
    if mode == "real" and laser_source == "none":
        raise ValueError("real navigation requires laser_source:=camera, ld06, or auto")
    if mode == "real" and laser_source == "camera" and not publish_camera:
        raise ValueError("laser_source:=camera requires publish_camera:=true")
    if mode == "sim" and camera_source == "remote":
        raise ValueError("camera_source:=remote is unsupported in simulation")
    if mode == "sim" and lidar_source == "remote":
        raise ValueError("lidar_source:=remote is unsupported in simulation")
    if start_rmf and slam_mode == "mapping":
        raise ValueError("start_rmf:=true requires slam_mode:=localization and a validated map bundle")
    if start_rmf and localization != "amcl":
        raise ValueError("RMF operation requires amcl with the immutable occupancy-map bundle")
    if start_rmf and rmf_domain != 0:
        raise ValueError(
            "rmf_domain must be 0 until a tracked cross-domain bridge is configured"
        )
    if start_rmf or localization == "amcl" or static_map:
        from lekiwi_rmf.map_bundle import validate_map_bundle

        validate_map_bundle(map_bundle, require_approved=True)


def validate_context(context, *_args, **_kwargs):
    """``launch.actions.OpaqueFunction`` adapter for ``bringup.launch.py``.

    Add ``OpaqueFunction(function=validate_context)`` after declarations and
    before every node/include action.  Raising here stops launch before any
    partially configured process is spawned.
    """
    from launch.substitutions import LaunchConfiguration

    values = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENT_NAMES}
    validate_launch_arguments(values)
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
    from launch.actions import SetLaunchConfiguration

    selected = [SetLaunchConfiguration("astra_serial", astra_serial)]
    uses_bundle = (
        _bool(values["start_rmf"], "start_rmf")
        or str(values["localization"]) == "amcl"
        or _bool(values["static_map"], "static_map")
    )
    if uses_bundle:
        from lekiwi_rmf.map_bundle import validate_map_bundle

        bundle = validate_map_bundle(str(values["map_bundle"]), require_approved=True)
        selected.append(SetLaunchConfiguration("selected_map", str(bundle.occupancy_yaml)))
        if _bool(values["start_rmf"], "start_rmf"):
            selected.extend([
            SetLaunchConfiguration("selected_nav_graph", str(bundle.navigation_graph)),
            SetLaunchConfiguration("selected_fleet_config", str(bundle.fleet_config)),
            ])
        return selected
    return selected
