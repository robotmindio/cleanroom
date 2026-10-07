"""Real-robot cameras and range sensors; included by bringup in real mode.

Whoever owns /scan owns what Nav2 dodges: a real LD06 on its RobotSkin base,
the front camera's floor-geometry trick, or nobody at all. The topology was
resolved once by bringup's argument validation.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from lekiwi_rmf.launch_validation import launch_topology


def _sensors(context):
    topology = launch_topology(context)
    package = FindPackageShare("lekiwi_rmf")
    actions = []
    if topology.astra_here:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([package, "launch", "pi_astra.launch.py"])),
            launch_arguments={"hardware_config": LaunchConfiguration("hardware_config"),
                              "respawn": "false"}.items(),
        ))
    if topology.local_cameras:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([package, "launch", "pi_cameras.launch.py"])),
            launch_arguments={
                "front_device": LaunchConfiguration("camera_device"),
                "wrist_device": LaunchConfiguration("wrist_camera_device"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
                "wrist_camera_info_url": LaunchConfiguration("wrist_camera_info_url"),
                "camera_namespace": "/camera", "jpeg_quality": "",
            }.items(),
        ))
    if topology.remote_camera:
        # Remote topology: the cameras are read by v4l2_camera on the machine they
        # are plugged into (launch/pi_cameras.launch.py, usually via the
        # lekiwi-cameras service) and only compressed frames cross the network.
        # This relay re-creates what a local v4l2_camera would have published --
        # raw images and CameraInfo on the canonical topics -- so nothing
        # downstream can tell the topologies apart. Frames keep their original
        # stamps: RTAB-Map syncs approximately (approx_sync), which ordinary
        # NTP-synced clocks comfortably satisfy.
        actions.append(Node(
            package="lekiwi_rmf", executable="camera_relay", name="camera_relay", output="screen",
        ))
    if topology.camera_laser:
        # There is no depth sensor, but the floor is flat, which makes every
        # floor pixel a known distance, and the first pixel that stops looking
        # like floor is an obstacle. Nav2's obstacle layer reads /scan and needs
        # nothing else. The geometry was measured with the checkerboard;
        # `free_space.py --ros-args -p calibrate:=true` prints it again, and
        # wrong numbers put phantom walls in the costmap.
        actions.append(Node(
            package="lekiwi_rmf", executable="free_space.py", name="free_space",
            parameters=[PathJoinSubstitution([package, "config", "camera_scan.yaml"]), {
                "camera_height": ParameterValue(LaunchConfiguration("camera_height"), value_type=float),
                "camera_pitch": ParameterValue(LaunchConfiguration("camera_pitch"), value_type=float),
            }],
            remappings=[("image", "/camera/front/image_raw"), ("camera_info", "/camera/front/camera_info"),
                        ("scan", "/scan")],
            output="screen",
        ))
    if topology.ld06:
        # frame_id is the URDF's `laser` link, so robot_state_publisher already
        # provides its pose -- the stock upstream launch adds a static TF that
        # would fight it. The LD06's 12 m range dwarfs the camera trick; keep
        # both off Nav2 at once by never enabling them together.
        actions.append(Node(
            package="ldlidar_stl_ros2",
            executable="ldlidar_stl_ros2_node",
            name="ld06_lidar",
            parameters=[{
                "product_name": "LDLiDAR_LD06",
                "topic_name": "/lidar/scan_raw",
                "frame_id": "laser",
                "port_name": LaunchConfiguration("lidar_port"),
                "port_baudrate": 230400,
            }],
            output="screen",
        ))
    if topology.remote_camera or topology.remote_ld06:
        # Device sensors cross the network only through zenoh: DDS stays on
        # each machine (cyclonedds.xml), and the device's lekiwi-zenoh
        # service exports exactly the topics this bridge imports.
        actions.append(ExecuteProcess(
            cmd=[
                "zenoh-bridge-ros2dds",
                "-c", PathJoinSubstitution([package, "config", "zenoh_compute.json5"]),
                "-e", ["tls/", LaunchConfiguration("remote_ip"), ":7447"],
            ],
            respawn=True, respawn_delay=5.0, output="screen",
        ))
    if topology.ld06 or topology.remote_ld06:
        # Both LD06 paths end here: the robot's own body is blanked out of the scan
        # (config/lidar_self_mask.yaml) before anything reads /scan. The node
        # subscribes without needing the Pi's publisher to exist yet, so a compute
        # host that boots first simply waits.
        actions.append(Node(
            package="lekiwi_rmf",
            executable="scan_self_filter",
            name="scan_self_filter",
            parameters=[
                PathJoinSubstitution([package, "config", "lidar_self_mask.yaml"]),
                {"input_topic": "/pi/lidar/scan" if topology.remote_ld06 else "/lidar/scan_raw"},
            ],
            output="screen",
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_sensors)])
