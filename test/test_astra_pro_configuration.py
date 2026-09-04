from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_astra_pro_publishes_registered_rgbd_in_the_robot_camera_frame():
    parameters = yaml.safe_load((ROOT / "config" / "astra_pro.yaml").read_text())["/**"]["ros__parameters"]

    assert parameters["depth_registration"] is True
    assert parameters["color_depth_synchronization"] is True
    assert parameters["enable_point_cloud"] is True
    assert parameters["publish_tf"] is False
    assert parameters["color_optical_frame_id"] == "astra_camera_optical_frame"
    assert parameters["depth_optical_frame_id"] == "astra_camera_optical_frame"


def test_real_bringup_keeps_navigation_on_the_front_camera_when_astra_fails():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()

    assert 'package="astra_camera", executable="astra_camera_node"' in source
    assert '("/color/image_raw", "/camera/astra/color/image_raw")' in source
    assert '("/depth/image_raw", "/camera/astra/depth/image_raw")' in source
    assert '"--namespace", "/camera/front"' in source
    assert '"--namespace", "/camera/wrist"' in source
    assert 'slam_rgb_topic = "/camera/front/image_raw"' in source
    assert '"subscribe_depth": False' in source


def test_astra_identity_is_pinned_in_tracked_hardware_configuration():
    hardware = yaml.safe_load((ROOT / "config" / "hardware.yaml").read_text())
    assert set(hardware["astra"]) == {"serial_number"}
    assert "LEKIWI_ASTRA_SERIAL" not in (ROOT / "scripts" / "ros-start.sh").read_text()


def test_astra_has_its_own_tracked_robot_frame():
    description = (ROOT / "urdf" / "lekiwi.urdf.xacro").read_text()

    assert 'link name="astra_camera_link"' in description
    assert 'link name="astra_camera_optical_frame"' in description


def test_rviz_shows_astra_from_a_fixed_frame_available_without_odometry():
    rviz = (ROOT / "config" / "lekiwi.rviz").read_text()
    launcher = (ROOT / "scripts" / "rviz.sh").read_text()

    assert "Fixed Frame: base_link" in rviz
    assert "Name: Astra RGB" in rviz
    assert "Topic: /camera/astra/color" in rviz
    assert "Value: /camera/front/image_raw" in rviz
    assert "Value: /camera/wrist/image_raw" in rviz
    assert "Value: /camera/depth/points" in rviz
    assert "astra_topic=/camera/astra/color/image_raw" in launcher
