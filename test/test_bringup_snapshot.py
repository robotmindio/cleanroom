"""Pin everything bringup.launch.py starts, per deployment profile and flag combination.

Each golden file in ``bringup_snapshots/`` is the normalized, order-independent
description produced by ``launch_snapshot.resolve_bringup``: every node and
process with its effective parameters, remappings, environment and respawn
policy, plus what each readiness gate starts.  A refactor of the launch files
or a re-layering of parameter files must leave these files unchanged.

Regenerate deliberately with ``LEKIWI_UPDATE_SNAPSHOTS=1`` and review the diff.
"""

import json
import os
from pathlib import Path

import pytest

from launch_snapshot import declared_arguments, resolve_bringup

LD06_PORTS = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0",
)

SNAPSHOTS = Path(__file__).parent / "bringup_snapshots"
CASES = {
    "sim": {"profile": "sim"},
    "sim_localization_moveit": {"profile": "sim", "slam_mode": "localization", "start_moveit": "true"},
    "sim_static_map": {"profile": "sim", "static_map": "true", "map_bundle_approved": True},
    "sim_amcl_rmf_rosbridge": {
        "profile": "sim", "localization": "amcl", "slam_mode": "localization", "start_rmf": "true",
        "start_rosbridge": "true", "start_foxglove": "false", "map_bundle_approved": True,
    },
    "wired": {"profile": "wired"},
    "wired_ld06_detected": {"profile": "wired", "serial_devices": [LD06_PORTS[0]]},
    "wired_ld06_bounded_strict": {
        "profile": "wired", "laser_source": "ld06", "publish_astra": "false",
        "bounded_base_test": "true", "safety_policy": "strict",
        "wrist_camera_device": "/dev/wrist",
    },
    "wired_remote_camera": {"profile": "wired", "camera_source": "remote", "laser_source": "camera"},
    "split": {"profile": "split"},
    # An installed service from before profiles existed: explicit topology, no profile.
    "service_without_profile": {
        "mode": "real", "camera_source": "remote", "camera_device": "none", "wrist_camera_device": "none",
        "camera_height": "0.101259", "camera_pitch": "0.056677", "remote_ip": "192.0.2.66",
        "laser_source": "ld06", "lidar_source": "remote", "start_moveit": "true",
    },
    "split_amcl_rmf": {
        "profile": "split", "localization": "amcl", "slam_mode": "localization",
        "publish_camera": "false", "start_rmf": "true", "map_bundle_approved": True,
    },
}


@pytest.mark.parametrize("case", sorted(CASES))
def test_bringup_matches_its_snapshot(case):
    actual = resolve_bringup(**CASES[case])
    path = SNAPSHOTS / f"{case}.json"
    if os.environ.get("LEKIWI_UPDATE_SNAPSHOTS") == "1":
        path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(actual, indent=1, sort_keys=True) + "\n")
    assert actual == json.loads(path.read_text())


def test_every_snapshot_has_a_case():
    assert {path.stem for path in SNAPSHOTS.glob("*.json")} == set(CASES)


def test_user_facing_arguments_match_their_snapshot():
    """Scripts, services and operators pass these names; keep them stable."""
    actual = declared_arguments(profile="wired")
    path = Path(__file__).parent / "bringup_arguments.json"
    if os.environ.get("LEKIWI_UPDATE_SNAPSHOTS") == "1":
        path.write_text(json.dumps(actual, indent=1, sort_keys=True) + "\n")
    assert actual == json.loads(path.read_text())
