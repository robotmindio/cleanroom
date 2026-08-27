from __future__ import annotations

import pytest

from lekiwi_rmf.launch_validation import astra_serial_from_hardware_config, validate_launch_arguments


def valid_arguments(**overrides):
    arguments = {
        "mode": "real",
        "remote_ip": "192.0.2.10",
        "curve_client_secret_key_file": "/tmp/client.key_secret",
        "curve_server_public_key_file": "/tmp/server.key",
        "start_rmf": "false",
        "rmf_domain": "0",
        "start_moveit": "false",
        "start_rosbridge": "false",
        "rosbridge_address": "127.0.0.1",
        "rosbridge_port": "9090",
        "rosbridge_domain": "0",
        "localization": "visual_slam",
        "slam_mode": "mapping",
        "publish_camera": "true",
        "publish_astra": "true",
        "hardware_config": "/tmp/hardware.yaml",
        "camera_source": "local",
        "laser_source": "camera",
        "xy_velocity_scale": "1.0",
        "yaw_velocity_scale": "0.9",
        "rtabmap_database": "/tmp/lekiwi.db",
        "rtabmap_wm_nodes": "300",
        "rtabmap_mapping_max_bytes": "536870912",
        "rtabmap_mapping_max_seconds": "14400",
        "map_bundle": "/tmp/not-used-without-rmf.yaml",
        "static_map": "false",
    }
    arguments.update(overrides)
    return arguments


def test_accepts_a_coherent_real_mapping_configuration():
    validate_launch_arguments(valid_arguments())


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"publish_camera": "false"}, "visual_slam requires"),
        ({"laser_source": "none"}, "real navigation requires"),
        ({"localization": "amcl", "publish_camera": "false", "laser_source": "camera"}, "laser_source:=camera requires"),
        ({"start_rmf": "true"}, "requires slam_mode:=localization"),
        ({"start_rmf": "true", "slam_mode": "localization", "localization": "amcl", "rmf_domain": "55"}, "rmf_domain must be 0"),
        ({"mode": "sim", "camera_source": "remote"}, "unsupported in simulation"),
        ({"rosbridge_port": "0"}, "between 1 and 65535"),
        ({"xy_velocity_scale": "0"}, "finite and positive"),
        ({"yaw_velocity_scale": "inf"}, "finite and positive"),
        ({"rtabmap_mapping_max_seconds": "nan"}, "finite and positive"),
        ({"rtabmap_wm_nodes": "0"}, "positive integer"),
        ({"rtabmap_mapping_max_bytes": "1.5"}, "non-negative integer"),
        ({"remote_ip": ""}, "must be non-empty"),
        ({"curve_client_secret_key_file": ""}, "both CURVE"),
        ({"start_rosbridge": "true", "rosbridge_address": "0.0.0.0"}, "only to loopback"),
    ],
)
def test_rejects_unsafe_or_incoherent_combinations(overrides, message):
    with pytest.raises(ValueError, match=message):
        validate_launch_arguments(valid_arguments(**overrides))


def test_requires_all_semantic_inputs():
    arguments = valid_arguments()
    arguments.pop("laser_source")
    with pytest.raises(ValueError, match="missing launch arguments: laser_source"):
        validate_launch_arguments(arguments)


def test_hardware_config_requires_a_pinned_astra_serial_when_rgbd_is_enabled(tmp_path):
    config = tmp_path / "hardware.yaml"
    config.write_text("astra:\n  serial_number: ''\n", encoding="utf-8")
    with pytest.raises(ValueError, match="non-empty tracked astra.serial_number"):
        astra_serial_from_hardware_config(config, required=True)


def test_hardware_config_allows_front_camera_fallback_without_astra(tmp_path):
    config = tmp_path / "hardware.yaml"
    config.write_text("astra:\n  serial_number: ''\n", encoding="utf-8")
    assert astra_serial_from_hardware_config(config, required=False) == ""


def test_simulation_accepts_moveit_with_the_physics_arm_controller():
    validate_launch_arguments(valid_arguments(mode="sim", start_moveit="true"))


def test_simulation_allows_insecure_test_transports():
    validate_launch_arguments(valid_arguments(
        mode="sim", curve_client_secret_key_file="", curve_server_public_key_file="",
        remote_ip="192.0.2.10", start_rosbridge="true", rosbridge_address="0.0.0.0",
    ))


def test_real_robot_accepts_only_the_assigned_tailnet_rosbridge_address():
    validate_launch_arguments(
        valid_arguments(start_rosbridge="true", rosbridge_address="100.87.252.60"),
        trusted_rosbridge_addresses=frozenset({"100.87.252.60"}),
    )


def test_real_remote_host_allows_an_unauthenticated_zmq_transport():
    validate_launch_arguments(valid_arguments(
        curve_client_secret_key_file="", curve_server_public_key_file="",
    ))


def test_rmf_rejects_mutable_visual_slam_even_in_localization_mode():
    with pytest.raises(ValueError, match="requires amcl"):
        validate_launch_arguments(valid_arguments(start_rmf="true", slam_mode="localization"))


def test_amcl_never_silently_uses_the_default_synthetic_map():
    with pytest.raises(ValueError, match="cannot read YAML"):
        validate_launch_arguments(valid_arguments(
            localization="amcl",
            laser_source="ld06",
            publish_camera="false",
        ))


def test_static_map_requires_an_approved_bundle_even_without_rmf():
    with pytest.raises(ValueError, match="cannot read YAML"):
        validate_launch_arguments(valid_arguments(static_map="true"))
