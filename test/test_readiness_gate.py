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
    running_context = types.SimpleNamespace(is_shutdown=False)
    assert after_camera(types.SimpleNamespace(returncode=0), running_context) == ["rtabmap"]

    failure = after_camera(types.SimpleNamespace(returncode=1), running_context)
    assert len(failure) == 1
    assert "dependents remain stopped" in failure[0].msg[0].text


def test_shutdown_gate_does_not_start_dependents():
    spec = importlib.util.spec_from_file_location("bringup_under_test", ROOT / "launch" / "bringup.launch.py")
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)

    after_camera = bringup._after_success("camera", ["rtabmap"])
    assert after_camera(
        types.SimpleNamespace(returncode=0), types.SimpleNamespace(is_shutdown=True)
    ) == []
    assert after_camera(
        types.SimpleNamespace(returncode=130), types.SimpleNamespace(is_shutdown=False)
    ) == []


def test_simulation_bridge_consumes_only_the_guarded_velocity_topic():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert '("/cmd_vel", "/cmd_vel_safe")' in source
    assert 'package="topic_tools"' not in source


def test_simulation_uses_a_database_separate_from_the_real_robot():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "lekiwi_rtabmap_sim.db" in source


def test_simulation_exports_a_resource_path_for_vendored_cad_meshes():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "SetEnvironmentVariable" in source
    assert "GZ_SIM_RESOURCE_PATH" in source


def test_simulation_selects_ogre2_without_an_unsupported_host_override():
    source = (ROOT / "worlds" / "cleanroom.sdf").read_text()
    assert "<render_engine>ogre2</render_engine>" in source
    assert "render_engine_api_backend" not in source


def test_rviz_replaces_only_the_recorded_instance():
    source = (ROOT / "scripts" / "rviz.sh").read_text()
    assert "rviz_pid_file" in source
    assert "pgrep -f" not in source
    assert 'printf \'%s\\n\' "$$" > "$rviz_pid_file"' in source


def test_camera_supervisor_uses_a_valid_compressed_parameter_name():
    source = (ROOT / "scripts" / "camera-supervisor.sh").read_text()
    assert '"image_raw.compressed.jpeg_quality:=$jpeg_quality"' in source
    assert '".image_raw.compressed.jpeg_quality:=$jpeg_quality"' not in source


def test_camera_safety_path_keeps_only_the_latest_frame():
    relay = (ROOT / "lekiwi_rmf" / "camera_relay.py").read_text()
    free_space = (ROOT / "scripts" / "free_space.py").read_text()
    assert "self.raw_qos = QoSProfile(depth=1" in relay
    assert "camera_qos = QoSProfile(depth=1" in free_space


def test_pi_camera_service_uses_the_low_latency_calibrated_front_size():
    source = (ROOT / "launch" / "pi_cameras.launch.py").read_text()
    assert '"--frame", "front_camera_optical_frame", "--size", "[320, 240]"' in source


def test_long_lived_nodes_have_idempotent_ros_shutdown():
    for relative_path in (
        "lekiwi_rmf/cmd_vel_mux.py",
        "lekiwi_rmf/driver.py",
        "lekiwi_rmf/readiness_gate.py",
    ):
        source = (ROOT / relative_path).read_text()
        assert "rclpy.try_shutdown()" in source
    assert "A timer already in flight" in (ROOT / "lekiwi_rmf/cmd_vel_mux.py").read_text()


def test_interrupted_readiness_gate_is_not_a_successful_dependency():
    source = (ROOT / "lekiwi_rmf/readiness_gate.py").read_text()
    assert "raise SystemExit(130)" in source
