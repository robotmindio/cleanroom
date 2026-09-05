from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_remote_astra_service_publishes_the_canonical_cloud_without_other_hardware():
    launch = (ROOT / "launch" / "pi_astra.launch.py").read_text(encoding="utf-8")
    service = (ROOT / "systemd" / "lekiwi-astra.service").read_text(encoding="utf-8")
    installer = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")

    assert 'package="astra_camera", executable="astra_camera_node"' in launch
    assert '("/depth/points", "/camera/depth/points_raw")' in launch
    assert 'package="lekiwi_rmf", executable="astra_cloud_filter"' in launch
    assert "astra_serial_from_hardware_config" in launch
    assert "ExecStart=@PROJECT_ROOT@/scripts/ros-astra.sh" in service
    assert "Restart=always" in service
    assert "lekiwi-host.service" not in service
    assert 'install_unit lekiwi-astra.service' in installer
    assert 'systemctl enable --now lekiwi-host.service lekiwi-astra.service' in installer


def test_optional_2d_cameras_wait_without_blocking_the_independent_astra_service():
    service = (ROOT / "systemd" / "lekiwi-cameras.service").read_text(encoding="utf-8")
    cameras = (ROOT / "scripts" / "ros-cameras.sh").read_text(encoding="utf-8")
    deploy = (ROOT / "scripts" / "deploy-split.sh").read_text(encoding="utf-8")

    assert "Restart=always" in service
    assert "while :; do" in cameras
    assert "waiting for front camera" in cameras
    assert "remote_front_camera_present" in deploy
    assert "No front camera is attached" in deploy
