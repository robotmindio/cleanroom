"""Keep the checked-in Foxglove dashboard aligned with the launch contract."""

from __future__ import annotations

import json
from pathlib import Path


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
        "/camera/depth/points", "/plan", "/local_plan", "/safety/marker",
    ):
        assert overview_topics[topic]["visible"]


def test_foxglove_bridge_is_read_only_and_installed_with_the_stack():
    launcher = (ROOT / "launch" / "bringup.launch.py").read_text()
    package = (ROOT / "package.xml").read_text()
    installer = (ROOT / "scripts" / "install.sh").read_text()

    assert 'package="foxglove_bridge"' in launcher
    assert '"capabilities": ["connectionGraph", "assets"]' in launcher
    assert "clientPublish" not in launcher
    assert "<exec_depend>foxglove_bridge</exec_depend>" in package
    assert '"ros-$ROS_DISTRO-foxglove-bridge"' in installer
