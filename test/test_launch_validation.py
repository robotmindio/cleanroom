from __future__ import annotations

import pytest

from lekiwi_rmf.launch_validation import astra_serial_from_hardware_config, validate_launch_arguments
from launch_snapshot import find_node, find_nodes, gate_stage, resolve_bringup, resolve_launch


@pytest.mark.parametrize("profile,mode,source,laser", [
    ("sim", "sim", "local", "auto"), ("wired", "real", "local", "auto"),
    ("split", "real", "remote", "ld06"),
])
def test_deployment_profiles_resolve_coherent_defaults(profile, mode, source, laser):
    _, values = resolve_launch(profile=profile)
    assert (values["mode"], values["camera_source"], values["lidar_source"], values["laser_source"]) == (
        mode, source, source, laser,
    )
    assert values["safety_policy"] == "hold"
    assert values["slam_mode"] == "mapping"
    validate_launch_arguments(values)


def test_profiles_preserve_installed_explicit_topology_arguments():
    _, values = resolve_launch(
        profile="wired", mode="real", camera_source="remote",
        lidar_source="remote", laser_source="ld06",
    )
    assert values["camera_source"] == "remote"
    assert values["lidar_source"] == "remote"
    validate_launch_arguments(values)


@pytest.mark.parametrize("profile,publish_astra,strategy,visual,rgb,depth,rgbd,cameras", [
    ("split", "true", "2", "0", False, False, True, 0),
    ("wired", "false", "2", "1", True, True, False, 1),
    ("sim", "true", "2", "1", True, True, False, 1),
])
def test_mapper_yaml_and_dynamic_view_parameters_preserve_behavior(
    profile, publish_astra, strategy, visual, rgb, depth, rgbd, cameras,
):
    parameters = find_node(resolve_bringup(profile=profile, publish_astra=publish_astra),
                           name="rtabmap")["parameters"]
    assert parameters["Reg/Strategy"] == strategy
    assert parameters["Vis/EstimationType"] == visual
    assert (parameters["subscribe_rgb"], parameters["subscribe_depth"], parameters["subscribe_rgbd"],
            parameters["rgbd_cameras"]) == (rgb, depth, rgbd, cameras)
    assert parameters["Mem/IncrementalMemory"] == "True"
    assert parameters["Mem/InitWMWithAllNodes"] == "True"
    for key, expected in {
        "Kp/MaxFeatures": "500", "RGBD/NeighborLinkRefining": "false",
        "RGBD/LoopCovLimited": "true", "RGBD/LinearUpdate": "0.04",
        "RGBD/ProximityMaxGraphDepth": "0", "RGBD/ProximityOdomGuess": "true",
        "Rtabmap/ImagesAlreadyRectified": "false", "Mem/NotLinkedNodesKept": "false",
        "odom_sensor_sync": True, "qos_image": 1, "sync_queue_size": 5, "topic_queue_size": 5,
    }.items():
        assert parameters[key] == expected


@pytest.mark.parametrize("profile,local", [("wired", True), ("split", False), ("sim", False)])
def test_shared_camera_launches_preserve_local_configuration(profile, local):
    records = resolve_bringup(profile=profile, wrist_camera_device="/dev/wrist")
    commands = [part for record in records if "process" in record for part in record["process"]]
    astra = find_nodes(records, node="astra_camera/astra_camera_node") + find_nodes(
        records, name="astra_cloud_filter")
    if not local:
        assert "--camera-name" not in commands and not astra
        return
    assert "/camera/front" in commands and "/camera/wrist" in commands
    assert "[320, 240]" in commands and "[352, 288]" in commands
    assert commands[commands.index("--jpeg-quality") + 1] == ""
    assert len(astra) == 2
    assert all(node["respawn"] is False for node in astra)
    assert astra[0]["parameters"]["serial_number"]


def test_camera_scan_uses_tracked_offsets_and_signed_saved_geometry():
    camera = find_node(resolve_bringup(profile="wired", camera_height="0.2", camera_pitch="-0.031"),
                       name="free_space")
    assert camera["parameters"] == {
        "camera_height": 0.2, "camera_pitch": -0.031, "camera_offset_x": 0.03,
        "camera_offset_y": 0.0, "camera_yaw": 0.0, "camera_roll": 0.0,
    }


def test_bounded_launch_defaults_follow_current_production_speed_limits():
    from pathlib import Path
    from lekiwi_rmf.motion_guards import load_base_speed_limits

    root = Path(__file__).parents[1]
    driver = find_node(resolve_bringup(profile="wired", bounded_base_test="true"),
                       node="lekiwi_rmf/lekiwi_driver")["parameters"]
    assert (driver["base_test_linear_limit"], driver["base_test_angular_limit"]) == (
        load_base_speed_limits(root / "config/nav2_params.yaml"))


@pytest.mark.parametrize('stage',['0.20','0.25','0.30'])
def test_qualification_changes_manual_caps_and_zones_without_raising_nav2_speed(stage):
    records=resolve_bringup(profile='split',bounded_base_test='true',base_test_stage=stage)
    driver=find_node(records,node='lekiwi_rmf/lekiwi_driver')['parameters']
    assert driver['base_test_stage']==stage
    navigation=gate_stage(records,'wait_for_map')['start']
    (include,)=[item for item in navigation if 'include' in item]
    parameters=include['arguments']['params_file']['generated_yaml']
    controller=parameters['controller_server']['ros__parameters']['FollowPath']
    assert (controller['vx_max'],controller['wz_max'])==(.03,.06)
    import yaml
    stop=parameters['collision_monitor']['ros__parameters']['StopZone']
    assert stop['type']=='velocity_polygon' and stop['holonomic']
    assert stop['min_points']==1
    assert stop['velocity_polygons']==['rotation','return_forward','return_reverse','return_left','return_right',
        'forward','reverse','left','right','fallback']
    uncertainty=.05 if stage=='0.25' else .04
    margin=float(stage)*1.15*1.5+uncertainty
    for direction in ('forward','reverse','left','right'):
        points=yaml.safe_load(stop[direction]['points'])
        assert stop[direction]['theta_min']==stop[direction]['theta_max']==0.
        if direction=='forward':
            assert max(p[0] for p in points)==pytest.approx(.24+margin)
            assert max(p[1] for p in points)==pytest.approx(.27,abs=1e-6)
        elif direction=='reverse':
            assert min(p[0] for p in points)==pytest.approx(-.22-margin)
            assert stop[direction]['direction_start_angle']>stop[direction]['direction_end_angle']
        elif direction=='left':
            assert max(p[1] for p in points)==pytest.approx(.22+margin)
        else:
            assert min(p[1] for p in points)==pytest.approx(-.22-margin)
    rotation=yaml.safe_load(stop['rotation']['points'])
    assert max(p[0] for p in rotation)>.326+.049
    assert stop['rotation']['linear_max']==0.
    assert stop['return_reverse']['linear_max']==pytest.approx(.04)
    returning=yaml.safe_load(stop['return_reverse']['points'])
    assert max(p[1] for p in returning)<.44
    assert min(p[0] for p in returning)==pytest.approx(-.22-(.04+.33*.06)*1.15*1.5-uncertainty)
    points=yaml.safe_load(stop['fallback']['points'])
    assert points[0]==pytest.approx([.89,.87])
    with pytest.raises(ValueError,match='requires bounded_base_test'):
        resolve_bringup(profile='split',base_test_stage='0.30')


def test_held_stow_reference_is_used_only_for_the_025_attended_stage():
    for stage,expected in [(None,[0.,-1.8167,1.6295,1.2367,-.0169,.2095]),
                           ('0.20',[0.,-1.8167,1.6295,1.2367,-.0169,.2095]),
                           ('0.25',[.0092,-1.7967,1.6249,1.2306,-.0077,.2161])]:
        arguments={'bounded_base_test':'true','base_test_stage':stage} if stage else {}
        supervisor=find_node(resolve_bringup(profile='split',**arguments),
            node='lekiwi_rmf/safety_supervisor')['parameters']
        assert supervisor['stow_joint_positions']==expected
        assert supervisor['stow_tolerance']==.02


def test_nav2_bounded_profile_matches_the_driver_and_preserves_production():
    def navigation_parameters(bounded):
        stage = gate_stage(resolve_bringup(profile="wired", bounded_base_test=bounded), "wait_for_map")
        (navigation,) = [item for item in stage["start"] if "include" in item]
        supervisor = find_node(resolve_bringup(profile="wired", bounded_base_test=bounded),
                               name="safety_supervisor")["parameters"]
        # The supervisor validates its stop zone against the exact file Nav2 runs.
        assert supervisor["nav2_params_file"] == navigation["arguments"]["params_file"]
        return navigation["arguments"]["params_file"]

    assert navigation_parameters("false") == "$(share lekiwi_rmf)/config/nav2_params.yaml"
    parameters = navigation_parameters("true")["generated_yaml"]["controller_server"]["ros__parameters"]
    controller = parameters['FollowPath']
    from pathlib import Path
    from lekiwi_rmf.motion_guards import load_base_speed_limits

    linear, angular = load_base_speed_limits(Path(__file__).parents[1] / "config/nav2_params.yaml")
    assert (controller['vx_max'], controller['vx_min'], controller['vy_max'], controller['wz_max']) == (
        linear, -linear, linear, angular)
    assert (controller['vx_std'], controller['vy_std'], controller['wz_std']) == (0.012, 0.012, 0.04)
    assert parameters['progress_checker']['required_movement_radius'] == 0.02
    assert parameters['goal_checker']['xy_goal_tolerance'] == 0.03


def valid_arguments(**overrides):
    arguments = {
        "mode": "real",
        "remote_ip": "192.0.2.10",
        "curve_client_secret_key_file": "/tmp/client.key_secret",
        "curve_server_public_key_file": "/tmp/server.key",
        "safety_policy": "hold",
        "start_rmf": "false",
        "start_moveit": "false",
        "start_foxglove": "true",
        "foxglove_address": "127.0.0.1",
        "foxglove_port": "8765",
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
        "lidar_source": "local",
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
        ({"safety_policy": "maybe"}, "safety_policy must be one of hold, strict"),
        ({"laser_source": "sonar"}, "laser_source must be one of auto, camera, ld06, none"),
        ({"mode": "hybrid"}, "mode must be one of real, sim"),
        ({"mode": "sim", "camera_source": "remote"}, "unsupported in simulation"),
        ({"mode": "sim", "lidar_source": "remote"}, "unsupported in simulation"),
        ({"foxglove_port": "0"}, "between 1 and 65535"),
        ({"rosbridge_port": "0"}, "between 1 and 65535"),
        ({"xy_velocity_scale": "0"}, "finite and positive"),
        ({"yaw_velocity_scale": "inf"}, "finite and positive"),
        ({"rtabmap_mapping_max_seconds": "nan"}, "finite and positive"),
        ({"rtabmap_wm_nodes": "0"}, "positive integer"),
        ({"rtabmap_mapping_max_bytes": "1.5"}, "non-negative integer"),
        ({"remote_ip": ""}, "must be non-empty"),
        ({"curve_client_secret_key_file": ""}, "both CURVE"),
        ({"start_rosbridge": "true", "rosbridge_address": "0.0.0.0"}, "only to loopback"),
        ({"foxglove_address": "0.0.0.0"}, "only to loopback"),
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


def test_real_stack_can_relay_the_device_ld06():
    validate_launch_arguments(valid_arguments(laser_source="ld06", lidar_source="remote"))
def test_real_mode_allows_rosbridge_on_a_tailnet_address():
    validate_launch_arguments(valid_arguments(
        start_rosbridge="true", rosbridge_address="100.87.252.60",
    ), trusted_rosbridge_addresses=frozenset({"100.87.252.60"}))


def test_real_mode_still_rejects_a_non_tailnet_non_loopback_address():
    with pytest.raises(ValueError, match="only to loopback"):
        validate_launch_arguments(valid_arguments(
            start_rosbridge="true", rosbridge_address="192.0.2.55",
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


def test_a_remote_lidar_needs_the_ld06_laser_source_not_auto():
    with pytest.raises(ValueError, match="lidar_source:=remote requires laser_source:=ld06"):
        validate_launch_arguments(valid_arguments(laser_source="auto", lidar_source="remote"))

    validate_launch_arguments(valid_arguments(laser_source="ld06", lidar_source="remote"))


def test_slam_mode_matters_to_rmf_only_with_visual_slam():
    # amcl ignores slam_mode, so RMF with amcl gets past that rule and reaches
    # the map-bundle check instead.
    with pytest.raises(ValueError, match="cannot read YAML"):
        validate_launch_arguments(valid_arguments(
            start_rmf="true", localization="amcl", slam_mode="mapping",
            laser_source="ld06", publish_camera="false",
        ))


def test_the_map_bundle_is_validated_once_and_returned(monkeypatch):
    import lekiwi_rmf.map_bundle as map_bundle

    calls = []
    monkeypatch.setattr(
        map_bundle, "validate_map_bundle",
        lambda path, require_approved: calls.append((path, require_approved)) or "bundle",
    )
    assert validate_launch_arguments(valid_arguments(static_map="true")) == "bundle"
    assert calls == [("/tmp/not-used-without-rmf.yaml", True)]
    assert validate_launch_arguments(valid_arguments()) is None
