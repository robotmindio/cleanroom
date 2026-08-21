import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("lekiwi_rmf")
    # ponytail: LeRobot pins numpy>=2 while ROS's C extensions are built against 1.26.
    # Mixing them segfaults rmf_adapter, so LeRobot lives in its own venv and only the
    # driver runs there -- its `#!/usr/bin/env python3` picks up whichever PATH we give it.
    lerobot_bin = os.path.join(
        os.environ.get("LEKIWI_WS", os.path.expanduser("~/lekiwi_ws")), ".venv-lerobot", "bin"
    )
    lerobot_env = {"PATH": lerobot_bin + os.pathsep + os.environ.get("PATH", "")}
    mode = LaunchConfiguration("mode")
    remote_ip = LaunchConfiguration("remote_ip")
    start_rmf = LaunchConfiguration("start_rmf")
    rmf_domain = LaunchConfiguration("rmf_domain")
    start_rosbridge = LaunchConfiguration("start_rosbridge")
    start_moveit = LaunchConfiguration("start_moveit")
    rosbridge_address = LaunchConfiguration("rosbridge_address")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    rosbridge_domain = LaunchConfiguration("rosbridge_domain")
    localization = LaunchConfiguration("localization")
    slam_mode = LaunchConfiguration("slam_mode")
    publish_camera = LaunchConfiguration("publish_camera")
    camera_info_url = LaunchConfiguration("camera_info_url")
    camera_device = LaunchConfiguration("camera_device")
    camera_source = LaunchConfiguration("camera_source")
    wrist_device = LaunchConfiguration("wrist_camera_device")
    xy_velocity_scale = LaunchConfiguration("xy_velocity_scale")
    yaw_velocity_scale = LaunchConfiguration("yaw_velocity_scale")
    rtabmap_database = LaunchConfiguration("rtabmap_database")
    headless = LaunchConfiguration("headless")
    sim = PythonExpression(["'", mode, "' == 'sim'"])
    real = PythonExpression(["'", mode, "' == 'real'"])
    amcl = PythonExpression(["'", localization, "' == 'amcl'"])
    visual_slam = PythonExpression(["'", localization, "' == 'visual_slam'"])
    slam_localization = PythonExpression(["'", slam_mode, "' == 'localization'"])
    camera_on = PythonExpression(["'", publish_camera, "' == 'true'"])
    camera_here = PythonExpression([camera_on, " and '", camera_source, "' == 'local'"])
    wrist_here = PythonExpression([camera_here, " and '", wrist_device, "' != 'none'"])
    # In remote mode the LeKiwi driver receives the host's front JPEG over ZMQ and
    # republishes it on the same canonical ROS topics as a local V4L2 camera.
    camera_info_topic = "/camera/front/camera_info"
    slam_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    static_map = LaunchConfiguration("static_map")
    canned_map = PythonExpression([visual_slam, " and '", static_map, "' == 'true'"])
    # Whoever owns /map owns what Nav2 plans against: the map server when a floor plan is
    # supplied, RTAB-Map's own grid when the robot is left to draw one.
    rtabmap_map_topic = PythonExpression([
        "'/rtabmap/map' if '", static_map, "' == 'true' else '/map'"])
    scanning = PythonExpression(["'", LaunchConfiguration("free_space"), "' == 'true'"])
    nav2_share = FindPackageShare("nav2_bringup")
    map_file = PathJoinSubstitution([package, "maps", "cleanroom.yaml"])
    params_file = PathJoinSubstitution([nav2_share, "params", "nav2_params.yaml"])
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([package, "urdf", "lekiwi.urdf.xacro"]), " sim:=", sim]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"]),
            # RViz is the normal visualization for this stack. Running Gazebo's server
            # only also works from CI and a machine reached over SSH without an X/GLX
            # display. Pass headless:=false to open Gazebo's own GUI.
            DeclareLaunchArgument("headless", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("remote_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("start_rmf", default_value="true"),
            DeclareLaunchArgument("rmf_domain", default_value="55"),
            DeclareLaunchArgument("start_rosbridge", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("rosbridge_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("rosbridge_domain", default_value="0"),
            DeclareLaunchArgument("localization", default_value="visual_slam", choices=["amcl", "visual_slam"]),
            DeclareLaunchArgument("slam_mode", default_value="mapping", choices=["mapping", "localization"]),
            DeclareLaunchArgument("publish_camera", default_value="true"),
            # Prefer a /dev/v4l/by-id/... path: /dev/videoN is reassigned on every USB
            # re-enumeration, and on a laptop video0 is usually the built-in webcam.
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument(
                "camera_source", default_value="local", choices=["local", "remote"]
            ),
            # "none" leaves the wrist camera out. Both cameras hang off one USB 2.0 hub and
            # neither can be compressed here, so the wrist runs small -- see the node below.
            DeclareLaunchArgument("wrist_camera_device", default_value="none"),
            # LeRobot's kinematics assume base_radius=0.125 m. Measure your own robot --
            # wheel-centre to wheel-centre, divided by sqrt(3), gives the real radius --
            # and set yaw_velocity_scale to 0.125 / that. Wheels 24 cm apart give 0.90.
            DeclareLaunchArgument("xy_velocity_scale", default_value="1.0"),
            DeclareLaunchArgument("yaw_velocity_scale", default_value="0.90"),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=["file://", EnvironmentVariable("HOME"), "/.ros/camera_info/lekiwi_front.yaml"],
            ),
            DeclareLaunchArgument(
                "rtabmap_database",
                default_value=[EnvironmentVariable("HOME"), "/.ros/lekiwi_rtabmap.db"],
            ),
            # RTAB-Map keeps its working memory in RAM and, unbounded, grows without end:
            # an hour of mapping at 1 Hz took 6.6 GB and starved the rest of the machine.
            # Past this many nodes the oldest move to the database and come back only when
            # the robot returns near them, so loop closure still works. Raise it on a
            # machine with memory to spare -- larger working memory closes loops sooner.
            DeclareLaunchArgument("rtabmap_wm_nodes", default_value="300"),
            # The checked-in PGM is a floor plan of a room that does not exist. Left false,
            # nothing serves it and RTAB-Map draws the map itself from what the robot sees.
            DeclareLaunchArgument("static_map", default_value="false"),
            # Camera-as-laser obstacle detection, and the geometry it stands on.
            DeclareLaunchArgument("free_space", default_value="true"),
            # Measured with the checkerboard on the floor, not taken from the URDF: the
            # camera sits 9.3 cm up and all but level, which is why it sees a chair leg at
            # 20 cm. Re-measure after touching the mount.
            DeclareLaunchArgument("camera_height", default_value="0.093"),
            DeclareLaunchArgument("camera_pitch", default_value="0.031"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": ParameterValue(sim, value_type=bool)}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=IfCondition(sim),
                parameters=[{"use_sim_time": True}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
                launch_arguments={"gz_args": [
                    PythonExpression([
                        "'-r -s --headless-rendering ' if '", headless,
                        "' == 'true' else '-r '"
                    ]),
                    PathJoinSubstitution([package, "worlds", "cleanroom.sdf"]),
                ]}.items(),
                condition=IfCondition(sim),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "lekiwi_1", "-topic", "robot_description", "-x", "-4", "-y", "-2.5"],
                condition=IfCondition(sim),
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    # gz publishes the image on <topic> itself and derives the info topic from
                    # the parent namespace, so <topic>/camera/front gives /camera/camera_info.
                    "/camera/front@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                remappings=[
                    ("/camera/front", "/camera/front/image_raw"),
                    ("/camera/camera_info", "/camera/front/camera_info"),
                ],
                condition=IfCondition(sim),
                output="screen",
            ),
            # The front camera is read straight off V4L2 rather than relayed through the
            # LeRobot host: the host aborts the whole robot -- motor control included --
            # when a camera frame arrives half a second late, which a USB webcam does. Off
            # the critical path, a stalled camera costs frames instead of the robot.
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="front_camera",
                # A namespace rather than remappings: image_transport builds its extra
                # transport topics from the unremapped base name, so remapping image_raw
                # leaves compressed advertised at /image_raw/compressed.
                namespace="/camera/front",
                parameters=[{
                    "video_device": camera_device,
                    "camera_info_url": camera_info_url,
                    "camera_frame_id": "front_camera_optical_frame",
                    "camera_name": "lekiwi_front",
                    "pixel_format": "YUYV",
                    "output_encoding": "rgb8",
                    "image_size": [640, 480],
                }],
                condition=IfCondition(PythonExpression([camera_here, " and ", real])),
                output="screen",
            ),
            # The wrist camera is for watching the gripper, not for SLAM: nothing subscribes
            # to it and it carries no calibration. camera_frame_id is the arm's tool link
            # rather than an optical frame, since the mount has never been measured.
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="wrist_camera",
                namespace="/camera/wrist",
                parameters=[{
                    "video_device": wrist_device,
                    "camera_frame_id": "tool",
                    "camera_name": "lekiwi_wrist",
                    # v4l2_camera has no JPEG decoder -- ask it for MJPG and it aborts on the
                    # first frame, so this is uncompressed and has to stay small: both cameras
                    # share one USB 2.0 hub, and a second 640x480 YUYV feed starves the front
                    # camera into solid green frames. The camera answers with the nearest mode
                    # it has, 352x288, which is what actually leaves the bus.
                    "pixel_format": "YUYV",
                    "output_encoding": "rgb8",
                    "image_size": [160, 120],
                }],
                condition=IfCondition(PythonExpression([wrist_here, " and ", real])),
                output="screen",
            ),
            # There is no laser on this robot, so obstacles come from the front camera: the
            # floor is flat, which makes every floor pixel a known distance, and the first
            # pixel that stops looking like floor is an obstacle. Nav2's obstacle layer
            # reads /scan and needs nothing else. The geometry below was measured with the
            # checkerboard; `free_space.py --ros-args -p calibrate:=true` prints it again,
            # and wrong numbers put phantom walls in the costmap.
            Node(
                package="lekiwi_rmf",
                executable="free_space.py",
                name="free_space",
                parameters=[{
                    "camera_height": ParameterValue(
                        LaunchConfiguration("camera_height"), value_type=float),
                    "camera_pitch": ParameterValue(
                        LaunchConfiguration("camera_pitch"), value_type=float),
                }],
                remappings=[
                    ("image", "/camera/front/image_raw"),
                    ("camera_info", camera_info_topic),
                    ("scan", "/scan"),
                ],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration("free_space"), "' == 'true' and ", real])),
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([package, "launch", "moveit.launch.py"])),
                condition=IfCondition(PythonExpression([real, " and '", start_moveit, "' == 'true'"])),
            ),
            Node(
                package="lekiwi_rmf",
                executable="lekiwi_driver",
                parameters=[{
                    "remote_ip": remote_ip,
                    "camera_info_url": camera_info_url,
                    # Odometry starts where the map says the robot starts. Without a floor
                    # plan there is no such place, so the robot's own start is the origin.
                    "initial_x": ParameterValue(
                        PythonExpression(["-4.0 if '", static_map, "' == 'true' else 0.0"]),
                        value_type=float),
                    "initial_y": ParameterValue(
                        PythonExpression(["-2.5 if '", static_map, "' == 'true' else 0.0"]),
                        value_type=float),
                    "xy_velocity_scale": ParameterValue(xy_velocity_scale, value_type=float),
                    "yaw_velocity_scale": ParameterValue(yaw_velocity_scale, value_type=float),
                }],
                condition=IfCondition(real),
                additional_env=lerobot_env,
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_share, "launch", "localization_launch.py"])),
                launch_arguments={"map": map_file, "params_file": params_file, "use_sim_time": sim}.items(),
                condition=IfCondition(amcl),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[{"yaml_filename": map_file, "use_sim_time": ParameterValue(sim, value_type=bool)}],
                condition=IfCondition(canned_map),
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                parameters=[{"autostart": True, "node_names": ["map_server"], "use_sim_time": ParameterValue(sim, value_type=bool)}],
                condition=IfCondition(canned_map),
                output="screen",
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                parameters=[{
                    "use_sim_time": ParameterValue(sim, value_type=bool),
                    "frame_id": "base_footprint",
                    "map_frame_id": "map",
                    "odom_frame_id": "odom",
                    "database_path": rtabmap_database,
                    "subscribe_rgb": True,
                    "subscribe_depth": False,
                    "subscribe_rgbd": False,
                    "subscribe_scan": ParameterValue(scanning, value_type=bool),
                    "subscribe_odom_info": False,
                    "approx_sync": True,
                    "publish_tf": True,
                    "qos_image": 2,
                    "qos_camera_info": 2,
                    "qos_scan": 2,
                    "qos_odom": 1,
                    "Rtabmap/MemoryThr": ParameterValue(
                        LaunchConfiguration("rtabmap_wm_nodes"), value_type=str
                    ),
                    "Mem/IncrementalMemory": ParameterValue(slam_mapping, value_type=str),
                    "Mem/InitWMWithAllNodes": ParameterValue(slam_localization, value_type=str),
                    "RGBD/NeighborLinkRefining": "true",
                    "RGBD/ProximityBySpace": "true",
                    "Reg/Force3DoF": "true",
                    # Build the occupancy grid from the scan alone: a single RGB camera
                    # contributes no depth, so there is nothing else to fill it with.
                    "Grid/Sensor": "0",
                    "Grid/RangeMax": "3.0",
                    "Grid/CellSize": "0.05",
                }],
                remappings=[
                    ("rgb/image", "/camera/front/image_raw"),
                    ("rgb/camera_info", camera_info_topic),
                    ("odom", "/odom"),
                    ("scan", "/scan"),
                    ("map", rtabmap_map_topic),
                    ("grid_map", "/rtabmap/grid_map"),
                ],
                condition=IfCondition(visual_slam),
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_share, "launch", "navigation_launch.py"])),
                launch_arguments={"params_file": params_file, "use_sim_time": sim}.items(),
            ),
            TimerAction(
                period=3.0,
                condition=IfCondition(amcl),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "topic", "pub", "--once", "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
                            "{header: {frame_id: map}, pose: {pose: {position: {x: -4.0, y: -2.5}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]}}",
                        ]
                    )
                ],
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                parameters=[{
                    "address": rosbridge_address,
                    "port": ParameterValue(rosbridge_port, value_type=int),
                }],
                condition=IfCondition(start_rosbridge),
                additional_env={"ROS_DOMAIN_ID": rosbridge_domain},
                output="screen",
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                condition=IfCondition(start_rosbridge),
                additional_env={"ROS_DOMAIN_ID": rosbridge_domain},
                output="screen",
            ),
            ExecuteProcess(
                cmd=["zenoh-bridge-ros2dds", "-c", PathJoinSubstitution([package, "config", "zenoh_bridge.json5"])],
                condition=IfCondition(start_rmf),
                output="screen",
            ),
            Node(
                package="rmf_traffic_ros2",
                executable="rmf_traffic_schedule",
                name="rmf_traffic_schedule_primary",
                condition=IfCondition(start_rmf),
                additional_env={"ROS_DOMAIN_ID": rmf_domain},
                output="screen",
            ),
            Node(
                package="rmf_traffic_ros2",
                executable="rmf_traffic_blockade",
                condition=IfCondition(start_rmf),
                additional_env={"ROS_DOMAIN_ID": rmf_domain},
                output="screen",
            ),
            Node(
                package="rmf_task_ros2",
                executable="rmf_task_dispatcher",
                parameters=[{"bidding_time_window": 2.0}],
                condition=IfCondition(start_rmf),
                additional_env={"ROS_DOMAIN_ID": rmf_domain},
                output="screen",
            ),
            Node(
                package="free_fleet_adapter",
                executable="fleet_adapter.py",
                arguments=[
                    "-c", PathJoinSubstitution([package, "config", "fleet_config.yaml"]),
                    "-n", PathJoinSubstitution([package, "maps", "nav_graph.yaml"]),
                ],
                condition=IfCondition(start_rmf),
                additional_env={"ROS_DOMAIN_ID": rmf_domain},
                output="screen",
            ),
        ]
    )
