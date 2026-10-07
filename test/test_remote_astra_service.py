from pathlib import Path

from test_service_installation import _unit


ROOT = Path(__file__).parents[1]


def test_remote_astra_service_publishes_the_canonical_cloud_without_other_hardware():
    launch = (ROOT / "launch" / "pi_astra.launch.py").read_text(encoding="utf-8")
    service = _unit("lekiwi-astra.service")
    installer = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")

    assert 'package="astra_camera", executable="astra_camera_node"' in launch
    assert '("/depth/points", "/camera/depth/points_raw")' in launch
    assert 'package="lekiwi_rmf", executable="astra_cloud_filter"' in launch
    assert '"config" / "astra_cloud_filter.yaml"' in launch
    assert 'executable="republish"' not in launch
    preprocessing = (ROOT / "lekiwi_rmf/astra_cloud_filter.py").read_text()
    assert 'f"/camera/astra/{camera}/image_raw/compressed"' in preprocessing
    assert 'raw=True' in preprocessing
    assert "astra_serial_from_hardware_config" in launch
    assert service["Service"]["ExecStart"] == ["@PROJECT_ROOT@/scripts/ros-astra.sh"]
    assert service["Service"]["Restart"] == ["always"]
    # The camera is independent of the motor host: no ordering or binding to it.
    assert not any("lekiwi-host.service" in value for values in service["Unit"].values() for value in values)
    assert 'install_unit lekiwi-astra.service' in installer
    assert '[[ $astra_ros_available == true ]] && units+=(lekiwi-astra.service)' in installer
    assert 'systemctl enable --now --no-block "${units[@]}"' in installer


def test_optional_2d_cameras_wait_without_blocking_the_independent_astra_service():
    service = _unit("lekiwi-cameras.service")
    cameras = (ROOT / "scripts" / "ros-cameras.sh").read_text(encoding="utf-8")
    deploy = (ROOT / "scripts" / "deploy-split.sh").read_text(encoding="utf-8")

    assert service["Service"]["Restart"] == ["always"]
    assert "while :; do" in cameras
    assert "waiting for front camera" in cameras
    assert "remote_front_camera_present" in deploy
    assert "No front camera is attached" in deploy
