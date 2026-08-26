"""Keep launch dependency gates tied to concrete ROS message types."""

import importlib.util
import pathlib
import types

from rclpy.qos import DurabilityPolicy

from lekiwi_rmf.readiness_gate import TOPIC_TYPES, topic_qos


ROOT = pathlib.Path(__file__).parents[1]


def test_readiness_gate_supports_the_bringup_dependencies():
    assert set(TOPIC_TYPES) == {"image", "odom", "map"}


def test_map_readiness_receives_rtabmaps_latched_grid():
    assert topic_qos("map").durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert topic_qos("image").durability == DurabilityPolicy.VOLATILE


def test_failed_gate_does_not_start_its_dependents():
    spec = importlib.util.spec_from_file_location("bringup_under_test", ROOT / "launch" / "bringup.launch.py")
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)

    after_camera = bringup._after_success("camera", ["rtabmap"])
    assert after_camera(types.SimpleNamespace(returncode=0), None) == ["rtabmap"]

    failure = after_camera(types.SimpleNamespace(returncode=1), None)
    assert len(failure) == 1
    assert "dependents remain stopped" in failure[0].msg[0].text


def test_simulation_bridge_consumes_only_the_guarded_velocity_topic():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert '("/cmd_vel", "/cmd_vel_safe")' in source
    assert 'package="topic_tools"' not in source
