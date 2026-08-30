import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from lekiwi_rmf.launch_validation import validate_context


LD06_SERIAL_PORTS = (
    # CP2102's usual Linux interface suffix. This is the device presently
    # attached to this robot (ID_SERIAL_SHORT=0001).
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    # Kept for an earlier udev naming variant seen on the same adapter family.
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0",
)


def _lidar_serial_present():
    # /dev/ttyUSB0 is only a kernel-assigned slot: it can just as easily be a
    # console cable or another USB-UART.  Auto mode is deliberately conservative.
    return any(os.path.exists(path) for path in LD06_SERIAL_PORTS)


def _lidar_default_port():
    """Select the stable serial name that actually exists at launch time."""
    return next((path for path in LD06_SERIAL_PORTS if os.path.exists(path)), LD06_SERIAL_PORTS[0])


def _after_success(stage, actions):
    """Start dependent launch actions only when a readiness gate succeeded.

    A gate normally waits forever for an unavailable dependency.  This explicit
    exit-status check also keeps an invalid parameter, import error, or other
    gate crash from being treated as readiness by ``OnProcessExit``.
    """
    def on_exit(event, context):
        # SIGINT makes the lightweight gates leave their spin loops cleanly.
        # Do not mistake that clean exit for readiness and start RTAB-Map/Nav2
        # after launch has already begun tearing the stack down.
        if context.is_shutdown or event.returncode == 130:
            return []
        if event.returncode == 0:
            return actions
        return [LogInfo(msg=f"ERROR: {stage} readiness gate exited with {event.returncode}; dependents remain stopped")]

    return on_exit


def _mapping_guard_exit(event, context):
    """Turn a quota exit into an orderly close of RTAB-Map's SQLite files."""
    if context.is_shutdown or event.returncode in (0, 130, -2, -15):
        return []
    if event.returncode == 75:
        return [EmitEvent(event=Shutdown(reason="RTAB-Map mapping session quota reached"))]
    return [EmitEvent(event=Shutdown(reason=f"RTAB-Map mapping guard failed ({event.returncode})"))]


def generate_launch_description():
    package = FindPackageShare("lekiwi_rmf")
    camera_supervisor = PathJoinSubstitution([package, "scripts", "camera-supervisor.sh"])
    mode = LaunchConfiguration("mode")
    remote_ip = LaunchConfiguration("remote_ip")
    curve_client_secret = LaunchConfiguration("curve_client_secret_key_file")
    curve_server_public = LaunchConfiguration("curve_server_public_key_file")
    auto_arm_on_startup = LaunchConfiguration("auto_arm_on_startup")
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
    wrist_camera_info_url = LaunchConfiguration("wrist_camera_info_url")
    camera_device = LaunchConfiguration("camera_device")
    publish_astra = LaunchConfiguration("publish_astra")
    astra_serial = LaunchConfiguration("astra_serial")
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
    astra_here = PythonExpression([
        camera_here, " and ", real, " and '", publish_astra, "' == 'true'"
    ])
    # Navigation must retain a working RGB-plus-scan path when the optional
    # Astra loses USB power or its UVC interface fails. Astra depth remains
    # available to consumers such as MoveIt when it is healthy, but it is not
    # a bringup dependency.
    slam_rgb_topic = "/camera/front/image_raw"
    slam_camera_info_topic = "/camera/front/camera_info"
    wrist_here = PythonExpression([camera_here, " and '", wrist_device, "' != 'none'"])
    remote_camera = PythonExpression([camera_on, " and ", real, " and '", camera_source, "' == 'remote'"])
    # The canonical camera topics, wherever the frames were read: a local v4l2_camera
    # publishes here directly, and in remote mode the relays below reconstruct them
    # from the compressed stream off the device machine.
    camera_info_topic = "/camera/front/camera_info"
    slam_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    static_map = LaunchConfiguration("static_map")
    canned_map = PythonExpression([visual_slam, " and '", static_map, "' == 'true'"])
    # Whoever owns /map owns what Nav2 plans against: the map server when a floor plan is
    # supplied, RTAB-Map's own grid when the robot is left to draw one.
    rtabmap_map_topic = PythonExpression([
        "'/rtabmap/map' if '", static_map, "' == 'true' else '/map'"])
    lidar_detected = _lidar_serial_present()
    # Whoever owns /scan, owns what Nav2 dodges: a real LD06 on its RobotSkin base, the
    # front camera's floor-geometry trick, or nobody at all. In sim Gazebo
    # always publishes /scan itself and none of this runs.
    laser_source = LaunchConfiguration("laser_source")
    lidar_source = LaunchConfiguration("lidar_source")
    lidar_port = LaunchConfiguration("lidar_port")
    # Gazebo supplies /scan in sim.  RTAB-Map must subscribe to it there too;
    # otherwise mono RGB plus odometry has no range data from which to make a grid.
    lidar_on = PythonExpression(["'", laser_source, "' != 'none'"])
    camera_laser = PythonExpression([
        "('", laser_source, "' == 'camera' or ('", laser_source, "' == 'auto' and not ",
        repr(lidar_detected), ")) and ", real])
    ld06 = PythonExpression([
        "('", laser_source, "' == 'ld06' or ('", laser_source, "' == 'auto' and ",
        repr(lidar_detected), ")) and '", lidar_source, "' == 'local' and ", real])
    remote_ld06 = PythonExpression([
        "'", laser_source, "' == 'ld06' and '", lidar_source, "' == 'remote' and ", real])
    map_file = PathJoinSubstitution([package, "maps", "cleanroom.yaml"])
    selected_map = LaunchConfiguration("selected_map", default=map_file)
    selected_nav_graph = LaunchConfiguration(
        "selected_nav_graph",
        default=PathJoinSubstitution([package, "maps", "nav_graph.yaml"]),
    )
    selected_fleet_config = LaunchConfiguration(
        "selected_fleet_config",
        default=PathJoinSubstitution([package, "config", "fleet_config.yaml"]),
    )
    nav2_share = FindPackageShare("nav2_bringup")
    # Never inherit the upstream TurtleBot/DiffDrive tuning.  This is installed
    # with the package so a launch from an overlay and a source checkout agree.
    params_file = PathJoinSubstitution([package, "config", "nav2_params.yaml"])
    ekf_params_file = PathJoinSubstitution([package, "config", "ekf.yaml"])
    safety_params_file = PathJoinSubstitution([
        package,
        "config",
        PythonExpression(["'safety_simulation.yaml' if ", sim, " else 'safety_production.yaml'"]),
    ])
    safety_acceptance_file = PathJoinSubstitution([package, "config", "safety_acceptance.yaml"])
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([package, "urdf", "lekiwi.urdf.xacro"]), " sim:=", sim]),
        value_type=str,
    )

    # Dependency gates replace fixed launch delays. Each one exits only after a
    # real message/action server is available, then starts its dependent stage.
    # A failed camera, driver, or mapper therefore leaves downstream motion and
    # fleet components stopped instead of launching a noisy degraded stack.
    camera_ready_gate = Node(
        package="lekiwi_rmf", executable="readiness_gate", name="wait_for_camera",
        parameters=[{"kind": "topic", "topic": slam_rgb_topic, "topic_type": "image"}],
        condition=IfCondition(visual_slam), output="screen",
    )
    odom_ready_gate = Node(
        package="lekiwi_rmf", executable="readiness_gate", name="wait_for_odom",
        parameters=[{"kind": "topic", "topic": "/odom", "topic_type": "odom"}], output="screen",
    )
    map_ready_gate = Node(
        package="lekiwi_rmf", executable="readiness_gate", name="wait_for_map",
        parameters=[{"kind": "topic", "topic": "/map", "topic_type": "map"}], output="screen",
    )
    nav_ready_gate = Node(
        package="lekiwi_rmf", executable="readiness_gate", name="wait_for_nav2",
        parameters=[{"kind": "navigate_to_pose_action", "action": "/navigate_to_pose"}],
        condition=IfCondition(start_rmf), output="screen",
    )
    arm_ready_gate = Node(
        package="lekiwi_rmf", executable="readiness_gate", name="wait_for_arm_controller",
        parameters=[{
            "kind": "follow_joint_trajectory_action",
            "action": "/arm_controller/follow_joint_trajectory",
        }],
        condition=IfCondition(start_moveit),
        output="screen",
    )
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([package, "launch", "moveit.launch.py"])),
        launch_arguments={"sim": sim}.items(),
    )
    rtabmap_node = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap",
        parameters=[{
            "use_sim_time": ParameterValue(sim, value_type=bool),
            "frame_id": "base_footprint", "map_frame_id": "map", "odom_frame_id": "odom",
            "database_path": rtabmap_database, "subscribe_rgb": True,
            "subscribe_depth": False,
            "subscribe_rgbd": False, "subscribe_scan": ParameterValue(lidar_on, value_type=bool),
            "subscribe_odom_info": False, "approx_sync": True, "publish_tf": True,
            "qos_image": 1, "qos_camera_info": 1, "qos_scan": 1, "qos_odom": 1,
            "Rtabmap/MemoryThr": ParameterValue(LaunchConfiguration("rtabmap_wm_nodes"), value_type=str),
            "Mem/IncrementalMemory": ParameterValue(slam_mapping, value_type=str),
            "Mem/InitWMWithAllNodes": ParameterValue(slam_localization, value_type=str),
            "RGBD/NeighborLinkRefining": "true", "RGBD/ProximityBySpace": "true",
            "Reg/Force3DoF": "true", "Grid/Sensor": "0", "Grid/RangeMax": "3.0",
            "Grid/CellSize": "0.05", "sync_queue_size": 20, "topic_queue_size": 20,
        }],
        remappings=[
            ("rgb/image", slam_rgb_topic), ("rgb/camera_info", slam_camera_info_topic),
            ("depth/image", "/camera/astra/depth/image_raw"),
            ("depth/camera_info", "/camera/astra/depth/camera_info"),
            ("odom", "/odom"), ("scan", "/scan"), ("map", rtabmap_map_topic),
            ("grid_map", "/rtabmap/grid_map"),
        ],
        condition=IfCondition(visual_slam), output="screen",
    )
    mapping_guard = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindPackagePrefix("lekiwi_rmf"), "lib", "lekiwi_rmf", "rtabmap-session-guard.py"]),
            rtabmap_database,
            "--maximum-bytes", LaunchConfiguration("rtabmap_mapping_max_bytes"),
            "--maximum-seconds", LaunchConfiguration("rtabmap_mapping_max_seconds"),
        ],
        condition=IfCondition(PythonExpression([visual_slam, " and ", slam_mapping])),
        output="screen",
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_share, "launch", "navigation_launch.py"])),
        launch_arguments={"params_file": params_file, "use_sim_time": sim}.items(),
    )
    initial_pose = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once", "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
            "{header: {frame_id: map}, pose: {pose: {position: {x: -4.0, y: -2.5}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]}}",
        ], condition=IfCondition(amcl),
    )
    free_fleet_adapter = Node(
        package="free_fleet_adapter", executable="fleet_adapter.py",
        arguments=["-c", selected_fleet_config, "-n", selected_nav_graph],
        additional_env={"ROS_DOMAIN_ID": rmf_domain}, output="screen",
    )
    rmf_owner_guard = Node(
        package="lekiwi_rmf", executable="rmf_owner_guard", name="rmf_owner_guard",
        parameters=[{
            "fleet_config": selected_fleet_config,
            # Allow DDS discovery to settle before this launch becomes the
            # owner. The guard never stops or adopts a participant it sees.
            "settle_seconds": 1.0,
        }],
        additional_env={"ROS_DOMAIN_ID": rmf_domain}, output="screen",
    )
    rmf_actions = [
        ExecuteProcess(
            cmd=["zenoh-bridge-ros2dds", "-c", PathJoinSubstitution([package, "config", "zenoh_bridge.json5"])],
            output="screen",
        ),
        Node(package="rmf_traffic_ros2", executable="rmf_traffic_schedule", name="rmf_traffic_schedule_primary", additional_env={"ROS_DOMAIN_ID": rmf_domain}, output="screen"),
        Node(package="rmf_traffic_ros2", executable="rmf_traffic_blockade", additional_env={"ROS_DOMAIN_ID": rmf_domain}, output="screen"),
        Node(package="rmf_task_ros2", executable="rmf_task_dispatcher", parameters=[{"bidding_time_window": 2.0}], additional_env={"ROS_DOMAIN_ID": rmf_domain}, output="screen"),
        rmf_owner_guard,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"]),
            # RViz is the normal visualization for this stack. Running Gazebo's server
            # only also works from CI and a machine reached over SSH without an X/GLX
            # display. Pass headless:=false to open Gazebo's own GUI.
            DeclareLaunchArgument("headless", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("remote_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("curve_client_secret_key_file", default_value=""),
            DeclareLaunchArgument("curve_server_public_key_file", default_value=""),
            # Fleet bridging makes the ROS graph discoverable off-host. Keep it
            # opt-in; a local robot can navigate without an external route.
            DeclareLaunchArgument("start_rmf", default_value="false"),
            # The fleet adapter must read this robot's map->base_footprint TF
            # and Nav2 actions. Those live on the primary ROS graph (domain 0),
            # so domain 55 isolates RMF from the robot and prevents initialization.
            DeclareLaunchArgument("rmf_domain", default_value="0"),
            # Rosbridge is opt-in and loopback-bound by default. It has no built-in
            # authentication; do not expose it beyond a protected proxy/firewall.
            DeclareLaunchArgument("start_rosbridge", default_value="false"),
            # MoveIt is optional for mobile navigation and is too expensive to
            # co-run with RTAB-Map on the 4 GB robot computer. Enable it only
            # for an arm task, preferably from the workstation.
            DeclareLaunchArgument("start_moveit", default_value="false"),
            DeclareLaunchArgument("rosbridge_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("rosbridge_domain", default_value="0"),
            DeclareLaunchArgument("localization", default_value="visual_slam", choices=["amcl", "visual_slam"]),
            # Simulation starts a disposable mapping session. A real service
            # starts localization-only so an unattended boot cannot mutate an
            # operational map without an explicit tracked mapping request.
            DeclareLaunchArgument(
                "slam_mode",
                default_value=PythonExpression(["'mapping' if ", sim, " else 'localization'"]),
                choices=["mapping", "localization"],
            ),
            DeclareLaunchArgument("publish_camera", default_value="true"),
            DeclareLaunchArgument(
                "auto_arm_on_startup", default_value="true", choices=["true", "false"]
            ),
            # The Astra Pro is an additional third camera. Existing front and
            # wrist V4L2 cameras continue to publish unchanged.
            DeclareLaunchArgument("publish_astra", default_value="true", choices=["true", "false"]),
            # A serial is deliberately read from this tracked deployment file,
            # not an environment variable or a one-off launch command. An
            # empty value fails before the Astra node can pick an arbitrary
            # compatible camera.
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution([package, "config", "hardware.yaml"]),
            ),
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
                "wrist_camera_info_url",
                default_value=["file://", EnvironmentVariable("HOME"), "/.ros/camera_info/lekiwi_wrist.yaml"],
            ),
            DeclareLaunchArgument(
                "rtabmap_database",
                # Simulation must not reopen a physical mapping session. Apart
                # from polluting a real map, a partially written hardware DB
                # can make RTAB-Map spend startup restoring stale words before
                # it produces the first simulated grid.
                default_value=PythonExpression([
                    "('", EnvironmentVariable("HOME"), "/.ros/lekiwi_rtabmap_sim.db' "
                    "if '", mode, "' == 'sim' else '",
                    EnvironmentVariable("HOME"), "/.ros/lekiwi_rtabmap.db')",
                ]),
            ),
            # RTAB-Map keeps its working memory in RAM and, unbounded, grows without end:
            # an hour of mapping at 1 Hz took 6.6 GB and starved the rest of the machine.
            # Past this many nodes the oldest move to the database and come back only when
            # the robot returns near them, so loop closure still works. Raise it on a
            # machine with memory to spare -- larger working memory closes loops sooner.
            DeclareLaunchArgument("rtabmap_wm_nodes", default_value="300"),
            DeclareLaunchArgument("rtabmap_mapping_max_bytes", default_value="536870912"),
            DeclareLaunchArgument("rtabmap_mapping_max_seconds", default_value="14400"),
            DeclareLaunchArgument(
                "map_bundle",
                default_value=PathJoinSubstitution([
                    package, "maps", "bundles", "cleanroom-development.yaml"
                ]),
            ),
            # The checked-in PGM is a floor plan of a room that does not exist. Left false,
            # nothing serves it and RTAB-Map draws the map itself from what the robot sees.
            DeclareLaunchArgument("static_map", default_value="false"),
            # What publishes /scan on the real robot: the camera-derived obstacle scan
            # (default, needs no extra hardware), an LDROBOT LD06 on its RobotSkin base, or none.
            # In sim this is ignored -- Gazebo publishes /scan from its own lidar model.
            DeclareLaunchArgument(
                "laser_source", default_value="auto", choices=["auto", "camera", "ld06", "none"]
            ),
            DeclareLaunchArgument(
                "lidar_source", default_value="local", choices=["local", "remote"]
            ),
            # Prefer a /dev/serial/by-id/... path for the same reason as camera_device.
            DeclareLaunchArgument(
                "lidar_port",
                default_value=_lidar_default_port(),
            ),
            # Camera-as-laser obstacle detection, and the geometry it stands on.
            # Measured with the checkerboard on the floor, not taken from the URDF: the
            # camera sits 9.3 cm up and all but level, which is why it sees a chair leg at
            # 20 cm. Re-measure after touching the mount.
            DeclareLaunchArgument("camera_height", default_value="0.093"),
            DeclareLaunchArgument("camera_pitch", default_value="0.031"),
            DeclareLaunchArgument("camera_offset_x", default_value="0.03"),
            DeclareLaunchArgument("camera_offset_y", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            # Evaluate cross-argument invariants before the first node, process,
            # or included launch description is allowed to start.
            OpaqueFunction(function=validate_context),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": ParameterValue(sim, value_type=bool)}],
            ),
            # Simulated joint states come from Gazebo physics below. A generic
            # joint_state_publisher would publish zeros concurrently and make
            # robot_state_publisher / MoveIt alternate between fake and actual
            # arm positions.
            # Gazebo resolves the CAD's ``model://lekiwi_rmf/...`` URIs from
            # resource-path roots, not from ament's package index. The parent
            # of this package share is the root that contains ``lekiwi_rmf``.
            # Without it every visual fails to load and GPU lidar receives an
            # invalid empty scene in headless mode.
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                    os.pathsep,
                    PathJoinSubstitution([package, ".."]),
                ],
            ),
            # The simulation model references our native watchdog by library
            # name. Keep its path in the tracked launch path so a simulated
            # robot cannot silently start without the actuator failsafe.
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=[
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                    os.pathsep,
                    PathJoinSubstitution([FindPackagePrefix("lekiwi_rmf"), "lib", "lekiwi_rmf"]),
                ],
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
                # robot_state_publisher keeps the canonical URDF. Gazebo gets
                # its deterministic SDF conversion with explicit anisotropic
                # omni-roller friction, which modern URDF conversion otherwise
                # drops and would leave three mutually constrained wheels.
                arguments=[
                    "-name", "lekiwi_1",
                    "-string", Command(["python3 -m lekiwi_rmf.sim_sdf"]),
                    "-x", "-4", "-y", "-2.5",
                ],
                condition=IfCondition(sim),
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/sim/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                    "/sim/sim_base_left_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                    "/sim/sim_base_back_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                    "/sim/sim_base_right_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                    "/sim/arm/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
                    "/sim/arm/trajectory_heartbeat@std_msgs/msg/Bool]gz.msgs.Boolean",
                    # gz publishes the image on <topic> itself and derives the info topic from
                    # the parent namespace, so <topic>/camera/front gives /camera/camera_info.
                    "/camera/front@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                remappings=[
                    ("/sim/joint_states", "/joint_states"),
                    ("/camera/front", "/camera/front/image_raw"),
                    ("/camera/camera_info", "/camera/front/camera_info"),
                    # Preserve the acquisition stamp while adding seeded
                    # transport latency/dropout before MoveIt sees the cloud.
                    ("/camera/depth/points", "/camera/depth/points_raw"),
                ],
                condition=IfCondition(sim),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "lekiwi_rmf.sim_omni_controller", "--ros-args", "-p", "use_sim_time:=true"],
                condition=IfCondition(sim),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "lekiwi_rmf.sim_sensor_delay", "--ros-args", "-p", "use_sim_time:=true"],
                condition=IfCondition(sim),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "lekiwi_rmf.sim_arm_controller", "--ros-args", "-p", "use_sim_time:=true"],
                condition=IfCondition(sim),
                output="screen",
            ),
            # The Astra Pro is read through its OpenNI/UVC ROS driver, rather than relayed
            # through the LeRobot host. Registered RGB-D supplies RTAB-Map and the MoveIt
            # octomap on their canonical topics; its frame IDs are pinned in astra_pro.yaml.
            Node(
                package="astra_camera", executable="astra_camera_node", name="astra_pro",
                parameters=[PathJoinSubstitution([package, "config", "astra_pro.yaml"]), {
                    "serial_number": astra_serial,
                }],
                remappings=[
                    # astra_camera publishes these at the root namespace. Keep
                    # the source names accurate: remapping /camera/... here
                    # silently creates no canonical RGB-D topics.
                    ("/color/image_raw", "/camera/astra/color/image_raw"),
                    ("/color/camera_info", "/camera/astra/color/camera_info"),
                    ("/depth/image_raw", "/camera/astra/depth/image_raw"),
                    ("/depth/camera_info", "/camera/astra/depth/camera_info"),
                    ("/depth/points", "/camera/depth/points"),
                ],
                condition=IfCondition(astra_here), output="screen",
            ),
            # A V4L2 front camera is read straight off the device rather than relayed through the
            # LeRobot host: the host aborts the whole robot -- motor control included --
            # when a camera frame arrives half a second late, which a USB webcam does. Off
            # the critical path, a stalled camera costs frames instead of the robot.
            ExecuteProcess(
                cmd=[
                    camera_supervisor,
                    "--device", camera_device, "--name", "front_camera",
                    "--namespace", "/camera/front", "--camera-name", "lekiwi_front",
                    # RTAB-Map's detection rate is 1 Hz and the safety scan runs
                    # at 5 Hz; 320x240 preserves calibrated geometry while keeping
                    # the local RGB pipeline inside the Pi's CPU/RAM budget.
                    "--frame", "front_camera_optical_frame", "--size", "[320, 240]",
                    "--camera-info-url", camera_info_url,
                ],
                # A namespace rather than remappings: image_transport builds its extra
                # transport topics from the unremapped base name, so remapping image_raw
                # leaves compressed advertised at /image_raw/compressed.
                condition=IfCondition(PythonExpression([camera_here, " and ", real])),
                output="screen",
            ),
            # The wrist camera is for watching the gripper, not for SLAM. Its CAD-backed
            # mount now has a dedicated optical frame; it must never inherit the generic
            # tool frame because that makes RViz show the image in the wrong pose.
            ExecuteProcess(
                cmd=[
                    camera_supervisor,
                    "--device", wrist_device, "--name", "wrist_camera",
                    "--namespace", "/camera/wrist", "--camera-name", "lekiwi_wrist",
                    "--frame", "wrist_camera_optical_frame", "--size", "[352, 288]",
                    "--camera-info-url", wrist_camera_info_url,
                ],
                    # v4l2_camera has no JPEG decoder -- ask it for MJPG and it aborts on the
                    # first frame, so this is uncompressed and has to stay small: both cameras
                    # share one USB 2.0 hub, and a second 640x480 YUYV feed starves the front
                    # camera into solid green frames. This camera's smallest native YUYV mode
                    # is 352x288; requesting 160x120 can succeed at format negotiation but
                    # then produce no frames on this UVC device.
                condition=IfCondition(PythonExpression([wrist_here, " and ", real])),
                output="screen",
            ),
            # Remote topology: the cameras are read by v4l2_camera on the machine they
            # are plugged into (launch/pi_cameras.launch.py, usually via the
            # lekiwi-cameras service) and only compressed frames cross the network.
            # This relay re-creates what a local v4l2_camera would have published --
            # raw images and CameraInfo on the canonical topics -- so nothing
            # downstream can tell the topologies apart. Frames keep their original
            # stamps: RTAB-Map syncs approximately (approx_sync), which ordinary
            # NTP-synced clocks comfortably satisfy.
            Node(
                package="lekiwi_rmf",
                executable="camera_relay",
                name="camera_relay",
                condition=IfCondition(remote_camera),
                output="screen",
            ),
            # laser_source:=camera: there is no depth sensor, but the floor is flat,
            # which makes every floor pixel a known distance, and the first pixel that
            # stops looking like floor is an obstacle. Nav2's obstacle layer reads /scan
            # and needs nothing else. The geometry below was measured with the
            # checkerboard; `free_space.py --ros-args -p calibrate:=true` prints it again,
            # and wrong numbers put phantom walls in the costmap.
            Node(
                package="lekiwi_rmf", executable="free_space.py", name="free_space",
                parameters=[{
                    "camera_height": ParameterValue(LaunchConfiguration("camera_height"), value_type=float),
                    "camera_pitch": ParameterValue(LaunchConfiguration("camera_pitch"), value_type=float),
                    "camera_offset_x": ParameterValue(LaunchConfiguration("camera_offset_x"), value_type=float),
                    "camera_offset_y": ParameterValue(LaunchConfiguration("camera_offset_y"), value_type=float),
                    "camera_yaw": ParameterValue(LaunchConfiguration("camera_yaw"), value_type=float),
                    "camera_roll": ParameterValue(LaunchConfiguration("camera_roll"), value_type=float),
                }],
                remappings=[("image", "/camera/front/image_raw"), ("camera_info", camera_info_topic), ("scan", "/scan")],
                condition=IfCondition(camera_laser), output="screen",
            ),
            # laser_source:=ld06: a real LDROBOT LD06 on the RobotSkin base. frame_id is the
            # URDF's `laser` link, so robot_state_publisher already
            # provides its pose -- the stock upstream launch adds a static TF that
            # would fight it. The LD06's 12 m range dwarfs the camera trick; keep
            # both off Nav2 at once by never enabling them together.
            Node(
                package="ldlidar_stl_ros2",
                executable="ldlidar_stl_ros2_node",
                name="ld06_lidar",
                parameters=[{
                    "product_name": "LDLiDAR_LD06",
                    "topic_name": "/scan",
                    "frame_id": "laser",
                    "port_name": lidar_port,
                    "port_baudrate": 230400,
                }],
                condition=IfCondition(ld06),
                output="screen",
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="remote_ld06_relay",
                # Keep the relay alive if this compute host boots before the
                # Pi's lidar publisher. Without the known type, topic_tools
                # exits while trying to infer it from a not-yet-advertised topic.
                arguments=["/pi/lidar/scan", "/scan", "sensor_msgs/msg/LaserScan"],
                condition=IfCondition(remote_ld06),
                output="screen",
            ),
            Node(
                package="lekiwi_rmf",
                executable="lekiwi_driver",
                parameters=[{
                    "remote_ip": remote_ip,
                    "curve_client_secret_key_file": curve_client_secret,
                    "curve_server_public_key_file": curve_server_public,
                    # Wheel odometry always starts in its local frame. AMCL or
                    # RTAB-Map owns map->odom and the global initial pose.
                    "initial_x": 0.0,
                    "initial_y": 0.0,
                    "xy_velocity_scale": ParameterValue(xy_velocity_scale, value_type=float),
                    "yaw_velocity_scale": ParameterValue(yaw_velocity_scale, value_type=float),
                    "permission_timeout": 0.5,
                    # The driver still requires current, explicit supervisor
                    # permission and fresh host telemetry before energizing
                    # servos; this only removes the manual arm RPC at startup.
                    "auto_arm_on_startup": ParameterValue(
                        auto_arm_on_startup, value_type=bool
                    ),
                    "odom_topic": "/wheel/odometry",
                    "publish_odom_tf": False,
                }],
                condition=IfCondition(real),
                remappings=[("safety/state", "safety/driver_state")],
                # A ZMQ connect() timing out against a host that is not yet
                # publishing (e.g. right after the device side restarts) is a
                # transient race, not a permanent failure -- respawn instead
                # of leaving the stack running with /safety/arm gone forever.
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            ),
            arm_ready_gate,
            RegisterEventHandler(OnProcessExit(
                target_action=arm_ready_gate,
                on_exit=_after_success("arm controller", [moveit_launch]),
            )),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_params_file],
                remappings=[("odometry/filtered", "/odom")],
                condition=IfCondition(real),
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            ),
            # Unlike the one-shot readiness gates, this authority continuously
            # withdraws both base and arm permission when a required input is
            # missing, stale, or unhealthy. Real mode selects the production
            # profile, which intentionally remains default-deny until the
            # tracked hardware safety inputs are installed and configured.
            Node(
                package="lekiwi_rmf",
                executable="safety_supervisor",
                name="safety_supervisor",
                parameters=[safety_params_file, {
                    "acceptance_file": safety_acceptance_file,
                    # A validated physical record is accepted only when its
                    # measured stopping distance still fits this exact tracked
                    # Nav2 footprint and collision-monitor StopZone.
                    "nav2_params_file": params_file,
                }],
                # This node is the one continuously-enforced source of motion
                # permission; a crash here must not leave the driver believing
                # its last-received permission is still current for good. It
                # already fails safe (driver.py's permission leases expire
                # and auto-disarm without fresh Bool messages), so restarting
                # it is strictly a recovery, never a new risk.
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            ),
            Node(
                package="lekiwi_rmf",
                executable="arm_workspace_monitor",
                name="arm_workspace_monitor",
                parameters=[
                    safety_params_file,
                    {"use_sim_time": ParameterValue(sim, value_type=bool)},
                ],
                condition=IfCondition(start_moveit),
                output="screen",
            ),
            # Join Nav2's smoothed stream and the manually requested stream
            # before collision monitoring. The mux is intentionally live before
            # Nav2 lifecycle activation, so an early controller command cannot
            # bypass the guard while a node is still coming up.
            Node(
                package="lekiwi_rmf",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                parameters=[{"permission_timeout": 0.5}],
                output="screen",
            ),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_share, "launch", "localization_launch.py"])),
                launch_arguments={"map": selected_map, "params_file": params_file, "use_sim_time": sim}.items(),
                condition=IfCondition(amcl),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[{"yaml_filename": selected_map, "use_sim_time": ParameterValue(sim, value_type=bool)}],
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
            # RTAB-Map starts only after an actual camera frame. Nav2 then
            # waits for odometry and the resulting map; fixed delays made both
            # components race slow cameras and telemetry reconnects.
            camera_ready_gate,
            RegisterEventHandler(OnProcessExit(
                target_action=camera_ready_gate,
                on_exit=_after_success("camera", [rtabmap_node, mapping_guard]),
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=mapping_guard,
                on_exit=_mapping_guard_exit,
            )),
            odom_ready_gate,
            RegisterEventHandler(OnProcessExit(
                target_action=odom_ready_gate,
                on_exit=_after_success("odometry", [map_ready_gate]),
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=map_ready_gate,
                on_exit=_after_success("map", [initial_pose, navigation_launch, nav_ready_gate]),
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=nav_ready_gate,
                on_exit=_after_success("Nav2", rmf_actions),
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=rmf_owner_guard,
                on_exit=_after_success("RMF ownership", [free_fleet_adapter]),
            )),
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
        ]
    )
