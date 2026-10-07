"""Keep the checked-in Foxglove dashboard aligned with the launch contract."""

from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import subprocess
import xml.etree.ElementTree as ElementTree

from launch_snapshot import find_node, find_nodes, resolve_bringup


ROOT = Path(__file__).parents[1]


def test_dashboard_has_the_operator_views_and_live_robot_model():
    layout = json.loads((ROOT / "config" / "foxglove-layout.json").read_text())
    config = layout["configById"]
    tabs = config["Tab!lekiwi"]["tabs"]

    assert [tab["title"] for tab in tabs] == [
        "Overview", "Navigation", "Perception", "Telemetry & Health",
    ]
    for panel_id in ("3D!overview", "3D!navigation", "3D!perception"):
        robot = next(
            layer for layer in config[panel_id]["layers"].values()
            if layer["layerId"] == "foxglove.Urdf"
        )
        assert robot["sourceType"] == "topic"
        assert robot["topic"] == "/robot_description"

    overview_topics = config["3D!overview"]["topics"]
    for topic in (
        "/map", "/global_costmap/costmap", "/local_costmap/costmap", "/scan",
        "/camera/depth/points", "/plan", "/safety/marker",
    ):
        assert overview_topics[topic]["visible"]

    # The driver's safety/state is remapped to safety/driver_state by bringup.
    assert config["RawMessages!driver_state"]["topicPath"] == "/safety/driver_state"


def test_foxglove_bridge_and_desktop_are_installed_with_the_stack():
    package = ElementTree.parse(ROOT / "package.xml").getroot()
    installer = (ROOT / "scripts" / "install.sh").read_text()

    for profile in ("sim", "wired", "split"):
        bridge = find_node(resolve_bringup(profile=profile), node="foxglove_bridge/foxglove_bridge")
        # Clients may inspect but never publish into the robot graph.
        assert bridge["parameters"]["capabilities"] == ["connectionGraph", "assets"]
    assert not find_nodes(resolve_bringup(profile="sim", start_foxglove="false"),
                          node="foxglove_bridge/foxglove_bridge")
    assert "foxglove_bridge" in [depend.text for depend in package.iter("exec_depend")]
    assert '"ros-$ROS_DISTRO-foxglove-bridge"' in installer
    assert "FOXGLOVE_ARCH=amd64" in installer
    assert "FOXGLOVE_ARCH=arm64" in installer
    assert "foxglove-studio-${FOXGLOVE_VERSION}-linux-${FOXGLOVE_ARCH}.deb" in installer
    assert '"$FOXGLOVE_SHA256"' in installer


def test_desktop_launcher_opens_the_local_bridge(tmp_path):
    calls = tmp_path / "calls"
    studio = tmp_path / "bin" / "foxglove-studio"
    studio.parent.mkdir()
    studio.write_text(f'#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "{calls}"\n')
    studio.chmod(studio.stat().st_mode | stat.S_IXUSR)

    def run(path):
        return subprocess.run(["bash", str(ROOT / "scripts" / "foxglove.sh")],
                              env={**os.environ, "PATH": path}, capture_output=True, text=True)

    assert run(f"{studio.parent}:/usr/bin:/bin").returncode == 0
    assert calls.read_text().splitlines() == [
        "foxglove://open?ds=foxglove-websocket&ds.url=ws://127.0.0.1:8765/",
    ]
    missing = run("/usr/bin:/bin")
    assert missing.returncode == 1 and "not installed" in missing.stderr
