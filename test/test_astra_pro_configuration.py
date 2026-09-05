from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import pytest
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
    robot = ET.fromstring(subprocess.check_output(
        ["xacro", str(ROOT / "urdf/lekiwi.urdf.xacro")], text=True
    ))
    camera = robot.find("link[@name='astra_camera_link']")
    assert camera.find("visual/geometry/box").get("size") == "0.040 0.165 0.048"
    assert camera.find("collision/geometry/box").attrib == camera.find("visual/geometry/box").attrib
    assert camera.find("collision/origin").attrib == camera.find("visual/origin").attrib
    assert '<parent link="astra_pro_compact_mount"/><child link="astra_camera_link"/>' in description
    assert 'property name="astra_mount_xyz" value="0 0 0.0155"' in description


def test_astra_optical_axis_faces_left_rear_and_down_in_the_complete_robot():
    robot = ET.fromstring(subprocess.check_output(
        ["xacro", str(ROOT / "urdf/lekiwi.urdf.xacro")], text=True
    ))
    parents = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
    frame, axis = "astra_camera_optical_frame", (0, 0, 1)
    while frame != "base_link":
        joint = parents[frame]
        roll, pitch, yaw = map(float, joint.find("origin").get("rpy", "0 0 0").split())
        x, y, z = axis
        y, z = math.cos(roll) * y - math.sin(roll) * z, math.sin(roll) * y + math.cos(roll) * z
        x, z = math.cos(pitch) * x + math.sin(pitch) * z, -math.sin(pitch) * x + math.cos(pitch) * z
        axis = (math.cos(yaw) * x - math.sin(yaw) * y, math.sin(yaw) * x + math.cos(yaw) * y, z)
        frame = joint.find("parent").get("link")
    horizontal = math.cos(math.radians(8)) / math.sqrt(5)
    assert axis == pytest.approx((-horizontal, 2 * horizontal, -math.sin(math.radians(8))), abs=1e-9)
    mount = robot.find("joint[@name='astra_pro_compact_mount_joint']/origin")
    assert tuple(map(float, mount.get("xyz").split())) == (-0.09, -0.04, 0.007)


def test_late_rviz_receives_the_latched_robot_description():
    rviz = yaml.safe_load((ROOT / "config/lekiwi.rviz").read_text())
    model = next(display for display in rviz["Visualization Manager"]["Displays"]
                 if display.get("Class") == "rviz_default_plugins/RobotModel")
    assert model["Description Topic"]["Durability Policy"] == "Transient Local"


def test_sensor_calibration_has_one_xacro_source_for_all_model_consumers():
    description = (ROOT / "urdf" / "lekiwi.urdf.xacro").read_text()
    bringup = (ROOT / "launch" / "bringup.launch.py").read_text()
    rviz = (ROOT / "scripts" / "rviz.sh").read_text()
    moveit = (ROOT / "launch" / "moveit.launch.py").read_text()

    assert 'property name="astra_mount_xyz"' in description
    assert 'property name="wrist_camera_xyz"' in description
    assert 'property name="lidar_offset_xyz"' in description
    assert "lidar_offset_x:=" not in bringup
    assert 'xacro "$package_share/urdf/lekiwi.urdf.xacro" sim:=false' in rviz
    assert 'file_path="urdf/lekiwi.urdf.xacro"' in moveit


def test_rviz_shows_astra_from_a_fixed_frame_available_without_odometry():
    rviz = (ROOT / "config" / "lekiwi.rviz").read_text()
    launcher = (ROOT / "scripts" / "rviz.sh").read_text()

    assert "Fixed Frame: base_link" in rviz
    assert "Name: Astra RGB" in rviz
    assert "Value: /camera/astra/color/image_raw" in rviz
    assert "Value: /camera/front/image_raw" in rviz
    assert "Value: /camera/wrist/image_raw" in rviz
    assert "Value: /camera/depth/points" in rviz
    assert "astra_topic=/camera/astra/color/image_raw" in launcher


def test_rviz_hides_the_duplicate_moveit_scene_robot_and_keeps_live_tf_model():
    rviz = (ROOT / "config" / "lekiwi.rviz").read_text()

    assert "Robot Alpha: 0" in rviz
    assert "Name: Live RobotModel (TF)" in rviz


def test_cyclonedds_keeps_rgbd_udp_datagrams_below_the_network_mtu():
    cyclonedds = (ROOT / "config" / "cyclonedds.xml").read_text()

    assert "<MaxMessageSize>1200B</MaxMessageSize>" in cyclonedds
    assert "<FragmentSize>1192B</FragmentSize>" in cyclonedds
    assert '<SocketReceiveBufferSize min="default" max="8MiB"/>' in cyclonedds
