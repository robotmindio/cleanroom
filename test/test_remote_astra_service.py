from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_remote_astra_service_publishes_the_canonical_cloud_without_other_hardware():
    launch = (ROOT / "launch" / "pi_astra.launch.py").read_text(encoding="utf-8")
    service = (ROOT / "systemd" / "lekiwi-astra.service").read_text(encoding="utf-8")
    installer = (ROOT / "scripts" / "install-astra-service.sh").read_text(encoding="utf-8")

    assert 'package="astra_camera", executable="astra_camera_node"' in launch
    assert '("/depth/points", "/camera/depth/points")' in launch
    assert "astra_serial_from_hardware_config" in launch
    assert "ExecStart=@PROJECT_ROOT@/scripts/ros-astra.sh" in service
    assert "lekiwi-host.service" not in service
    assert "lekiwi-cameras.service" not in installer
    assert "lekiwi-lidar.service" not in installer
