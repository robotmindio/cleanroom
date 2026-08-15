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
    rosbridge_address = LaunchConfiguration("rosbridge_address")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    rosbridge_domain = LaunchConfiguration("rosbridge_domain")
    localization = LaunchConfiguration("localization")
    slam_mode = LaunchConfiguration("slam_mode")
    publish_camera = LaunchConfiguration("publish_camera")
    camera_info_url = LaunchConfiguration("camera_info_url")
    rtabmap_database = LaunchConfiguration("rtabmap_database")
    sim = PythonExpression(["'", mode, "' == 'sim'"])
    real = PythonExpression(["'", mode, "' == 'real'"])
    amcl = PythonExpression(["'", localization, "' == 'amcl'"])
    visual_slam = PythonExpression(["'", localization, "' == 'visual_slam'"])
    slam_localization = PythonExpression(["'", slam_mode, "' == 'localization'"])
    slam_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])
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
            DeclareLaunchArgument("remote_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("start_rmf", default_value="true"),
            DeclareLaunchArgument("rmf_domain", default_value="55"),
            DeclareLaunchArgument("start_rosbridge", default_value="true"),
            DeclareLaunchArgument("rosbridge_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("rosbridge_domain", default_value="0"),
            DeclareLaunchArgument("localization", default_value="visual_slam", choices=["amcl", "visual_slam"]),
            DeclareLaunchArgument("slam_mode", default_value="mapping", choices=["mapping", "localization"]),
            DeclareLaunchArgument("publish_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_info_url", default_value="file://${ROS_HOME}/camera_info/lekiwi_front.yaml"
            ),
            DeclareLaunchArgument(
                "rtabmap_database",
                default_value=[EnvironmentVariable("HOME"), "/.ros/lekiwi_rtabmap.db"],
            ),
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
                launch_arguments={"gz_args": ["-r ", PathJoinSubstitution([package, "worlds", "cleanroom.sdf"])]}.items(),
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
            Node(
                package="lekiwi_rmf",
                executable="lekiwi_driver",
                parameters=[{
                    "remote_ip": remote_ip,
                    "publish_camera": ParameterValue(publish_camera, value_type=bool),
                    "require_camera_calibration": ParameterValue(visual_slam, value_type=bool),
                    "camera_info_url": camera_info_url,
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
                condition=IfCondition(visual_slam),
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                parameters=[{"autostart": True, "node_names": ["map_server"], "use_sim_time": ParameterValue(sim, value_type=bool)}],
                condition=IfCondition(visual_slam),
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
                    "subscribe_scan": False,
                    "subscribe_odom_info": False,
                    "approx_sync": True,
                    "publish_tf": True,
                    "qos_image": 2,
                    "qos_camera_info": 2,
                    "qos_odom": 1,
                    "Mem/IncrementalMemory": ParameterValue(slam_mapping, value_type=str),
                    "Mem/InitWMWithAllNodes": ParameterValue(slam_localization, value_type=str),
                    "RGBD/NeighborLinkRefining": "true",
                    "RGBD/ProximityBySpace": "true",
                    "Reg/Force3DoF": "true",
                }],
                remappings=[
                    ("rgb/image", "/camera/front/image_raw"),
                    ("rgb/camera_info", "/camera/front/camera_info"),
                    ("odom", "/odom"),
                    # RTAB-Map also publishes an occupancy grid on /map. Monocular RGB gives it
                    # no depth, so its grid is near-empty -- and it would overwrite the checked-in
                    # PGM in nav2's static layer, leaving the planner a metre-wide world.
                    ("map", "/rtabmap/map"),
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
                            "{header: {frame_id: map}, pose: {pose: {position: {x: -4.0, y: -2.5}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]}",
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
