"""Bring up the LeKiwi stack for one deployment profile: sim, wired, or split.

Arguments are validated as a whole and the started topology is resolved once
(``lekiwi_rmf.launch_validation``); simulation, real sensors and visual SLAM
live in their own included launch files.
"""

from pathlib import Path
import math
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.geometry import polygon, polygon_boundary_distance
from lekiwi_rmf.launch_gates import gated
from lekiwi_rmf.launch_validation import (
    CHOICES, PROFILES, launch_topology, lidar_default_port, permission_timeout, validate_context)
from lekiwi_rmf.motion_guards import load_base_speed_limits, load_base_test_profile
from lekiwi_rmf.odometry import BASE_XY_SCALE, BASE_YAW_SCALE

# The fleet adapter must read this robot's map->base_footprint TF and Nav2
# actions. Those live on the primary ROS graph (domain 0); another domain
# isolates RMF from the robot and prevents initialization.
RMF_DOMAIN = "0"


def _profile_defaults(context):
    defaults = PROFILES[LaunchConfiguration("profile").perform(context)]
    # Retain explicit topology values from previously installed service arguments.
    return [SetLaunchConfiguration(key, value) for key, value in defaults.items()
            if key not in context.launch_configurations]


def _navigation_params(source_file, bounded_test, linear_limit, angular_limit, qualification=None):
    if not bounded_test:
        return source_file
    if qualification:
        # Pure-axis trials need braking reserve along travel; rotation sweeps
        # the circumscribed footprint. Keep every production Nav2 speed.
        source = Path(get_package_share_directory('lekiwi_rmf')) / 'config/nav2_params.yaml'
        nav2 = yaml.safe_load(source.read_text())
        footprint = polygon(nav2['local_costmap']['local_costmap']['ros__parameters']['footprint'], 'footprint')
        xmin, xmax = min(p[0] for p in footprint), max(p[0] for p in footprint)
        ymin, ymax = min(p[1] for p in footprint), max(p[1] for p in footprint)
        monitor = nav2['collision_monitor']['ros__parameters']
        padding = polygon_boundary_distance(footprint, polygon(monitor['StopZone']['points'], 'StopZone'))
        radius = max(math.hypot(x, y) for x, y in footprint) + padding
        def rectangle(left, right, bottom, top):
            return str([[right, top], [right, bottom], [left, bottom], [left, top]])
        for zone, extra in [('StopZone', 0.0), ('SlowdownZone', .07)]:
            settings = monitor[zone]
            settings.pop('points')
            settings.update(type='velocity_polygon', holonomic=True,
                velocity_polygons=['rotation', 'return_forward', 'return_reverse', 'return_left', 'return_right',
                    'forward', 'reverse', 'left', 'right', 'fallback'])
            limits = dict(linear_min=0.0, linear_max=sys.float_info.max,
                          theta_min=-sys.float_info.max, theta_max=sys.float_info.max)
            settings['rotation'] = {**limits, 'linear_max': 0.0,
                'points': rectangle(-radius-extra, radius+extra, -radius-extra, radius+extra)}
            for prefix, cone, margin, side, back, velocities in [
                ('return_', math.pi/4, qualification['return_stopping_margin_m']+extra,
                    max(padding,qualification['return_lateral_margin_m'])+extra,
                    max(padding,qualification['return_rotation_margin_m'])+extra,
                    {**limits,'linear_max': math.nextafter(qualification['return_linear_speed_m_s'],math.inf)}),
                ('', 1e-6, qualification['translation_stopping_margin_m']+extra,
                    padding+(qualification['translation_stopping_margin_m']+extra)*1e-6, padding,
                    {**limits,'theta_min':0.0,'theta_max':0.0}),
            ]:
                for direction, angle, bounds in [
                    ('forward',0.0,(xmin-back,xmax+margin,ymin-side,ymax+side)),
                    ('reverse',math.pi,(xmin-margin,xmax+back,ymin-side,ymax+side)),
                    ('left',math.pi/2,(xmin-side,xmax+side,ymin-back,ymax+margin)),
                    ('right',-math.pi/2,(xmin-side,xmax+side,ymin-margin,ymax+back)),
                ]:
                    start,end=angle-cone,angle+cone
                    settings[prefix+direction]={**velocities,
                        'direction_start_angle':math.atan2(math.sin(start),math.cos(start)),
                        'direction_end_angle':math.atan2(math.sin(end),math.cos(end)),
                        'points':rectangle(*bounds)}
            margin = qualification['maximum_stopping_distance_m'] + extra
            settings['fallback'] = {**limits,
                'points': rectangle(xmin-margin, xmax+margin, ymin-margin, ymax+margin)}
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as output:
            yaml.safe_dump(nav2, output)
            return output.name
    # Match the driver's attended-test limits so MPPI predicts actual movement.
    prefix = "controller_server.ros__parameters."
    return RewrittenYaml(source_file=source_file, param_rewrites={
        prefix + "FollowPath.vx_max": linear_limit,
        prefix + "FollowPath.vx_min": ["-", linear_limit],
        prefix + "FollowPath.vy_max": linear_limit,
        prefix + "FollowPath.wz_max": angular_limit,
        prefix + "FollowPath.vx_std": "0.012",
        prefix + "FollowPath.vy_std": "0.012",
        prefix + "FollowPath.wz_std": "0.04",
        prefix + "progress_checker.required_movement_radius": "0.02",
    }, convert_types=True)


def _include(path, condition=True, **arguments):
    if not condition:
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())]


def _readiness_gate(name, **parameters):
    return Node(package="lekiwi_rmf", executable="readiness_gate", name=name,
                parameters=[parameters], output="screen")


def _stack(context):
    topology = launch_topology(context)
    sim, real = topology.sim, topology.real
    package = FindPackageShare("lekiwi_rmf")
    config = lambda name: PathJoinSubstitution([package, "config", name])  # noqa: E731
    launch_file = lambda name: PathJoinSubstitution([package, "launch", name])  # noqa: E731
    nav2_launch = lambda name: PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", name])  # noqa: E731
    flag = lambda name: LaunchConfiguration(name).perform(context) == "true"  # noqa: E731
    bounded_base_test = flag("bounded_base_test")
    test_stage = LaunchConfiguration('base_test_stage').perform(context)
    if test_stage and not bounded_base_test:
        raise ValueError('base_test_stage requires bounded_base_test:=true')
    qualification = load_base_test_profile(
        Path(get_package_share_directory('lekiwi_rmf')) / 'config/nav2_params.yaml', test_stage
    ) if test_stage else None
    # hold (domestic robot) stays armed through failures; strict disarms on every
    # failure and waits for an operator safety/arm.
    strict_policy = LaunchConfiguration("safety_policy").perform(context) == "strict"
    arm_calibration_file = LaunchConfiguration("arm_calibration_file")
    selected_map = LaunchConfiguration(
        "selected_map", default=PathJoinSubstitution([package, "maps", "cleanroom.yaml"]))
    selected_nav_graph = LaunchConfiguration(
        "selected_nav_graph", default=PathJoinSubstitution([package, "maps", "nav_graph.yaml"]))
    selected_fleet_config = LaunchConfiguration(
        "selected_fleet_config", default=config("fleet_config.yaml"))
    # Never inherit the upstream TurtleBot/DiffDrive tuning.  This is installed
    # with the package so a launch from an overlay and a source checkout agree.
    params_file = _navigation_params(
        config("nav2_params.yaml"), bounded_base_test,
        LaunchConfiguration("base_test_linear_limit"), LaunchConfiguration("base_test_angular_limit"),
        qualification,
    )
    # Simulation layers its few differences over the physical robot's profile.
    safety_params_files = [config("safety_production.yaml"), *([config("safety_simulation.yaml")] if sim else [])]
    lease = permission_timeout(Path(get_package_share_directory("lekiwi_rmf")) / "config")
    use_sim_time = str(sim)

    # Start safety once feedback is genuinely flowing. It does not depend on a
    # map: RTAB-Map may wait for relocalization after an odometry reset, while
    # Nav2 must remain gated until a usable map exists.
    safety_supervisor = Node(
        package="lekiwi_rmf",
        executable="safety_supervisor",
        name="safety_supervisor",
        parameters=[*safety_params_files, {
            "use_sim_time": sim,
            "bounded_base_test": bounded_base_test,
            # Simulation keeps its qualified enforcement; real mode is strict only
            # for larger robots that opt in with safety_policy:=strict.
            "strict": sim or strict_policy,
            "acceptance_file": config("safety_acceptance.yaml"),
            "scan_self_mask_file": ParameterValue(
                config("lidar_self_mask_simulation.yaml" if sim else "lidar_self_mask.yaml"), value_type=str),
            # A validated physical record is accepted only when its measured stopping
            # distance still fits this exact tracked Nav2 footprint and StopZone.
            "nav2_params_file": params_file,
            **({"stow_joint_positions": [qualification['stow_joint_positions'][name] for name in ARM_JOINTS]}
               if qualification and qualification.get('stow_joint_positions') else {}),
        }],
        # This node is the one continuously-enforced source of motion permission; a
        # crash must not leave the driver believing its last lease is current.
        respawn=True,
        respawn_delay=2.0,
        output="screen",
    )
    rmf_owner_guard = Node(
        package="lekiwi_rmf", executable="rmf_owner_guard", name="rmf_owner_guard",
        parameters=[{
            "fleet_config": selected_fleet_config,
            # Allow DDS discovery to settle before this launch becomes the
            # owner. The guard never stops or adopts a participant it sees.
            "settle_seconds": 1.0,
        }],
        additional_env={"ROS_DOMAIN_ID": RMF_DOMAIN}, output="screen",
    )
    rmf_actions = [
        ExecuteProcess(cmd=["zenoh-bridge-ros2dds", "-c", config("zenoh_bridge.json5")], output="screen"),
        Node(package="rmf_traffic_ros2", executable="rmf_traffic_schedule", name="rmf_traffic_schedule_primary", additional_env={"ROS_DOMAIN_ID": RMF_DOMAIN}, output="screen"),
        Node(package="rmf_traffic_ros2", executable="rmf_traffic_blockade", additional_env={"ROS_DOMAIN_ID": RMF_DOMAIN}, output="screen"),
        Node(package="rmf_task_ros2", executable="rmf_task_dispatcher", parameters=[{"bidding_time_window": 2.0}], additional_env={"ROS_DOMAIN_ID": RMF_DOMAIN}, output="screen"),
        *gated(rmf_owner_guard, "RMF ownership", [Node(
            package="free_fleet_adapter", executable="fleet_adapter.py",
            arguments=["-c", selected_fleet_config, "-n", selected_nav_graph],
            additional_env={"ROS_DOMAIN_ID": RMF_DOMAIN}, output="screen",
        )]),
    ]
    navigation = [
        *([ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "--once", "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
                "{header: {frame_id: map}, pose: {pose: {position: {x: -4.0, y: -2.5}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07]}}",
            ],
        )] if topology.amcl else []),
        *_include(nav2_launch("navigation_launch.py"), params_file=params_file, use_sim_time=use_sim_time),
        *(gated(_readiness_gate("wait_for_nav2", kind="navigate_to_pose_action", action="/navigate_to_pose"),
                "Nav2", rmf_actions) if topology.start_rmf else []),
    ]

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(Command([
                    "xacro ", PathJoinSubstitution([package, "urdf", "lekiwi.urdf.xacro"]), f" sim:={sim}",
                ]), value_type=str),
                "use_sim_time": sim,
            }],
        ),
        *_include(launch_file("sim.launch.py"), sim),
        *_include(launch_file("sensors.launch.py"), real),
    ]
    if real:
        actions.append(Node(
            package="lekiwi_rmf",
            executable="lekiwi_driver",
            parameters=[{
                "remote_ip": LaunchConfiguration("remote_ip"),
                "nav2_params_file": config("nav2_params.yaml"),
                "bounded_base_test": bounded_base_test,
                **({'base_test_stage': test_stage} if test_stage else {}),
                "base_test_linear_limit": ParameterValue(LaunchConfiguration("base_test_linear_limit"), value_type=float),
                "base_test_angular_limit": ParameterValue(LaunchConfiguration("base_test_angular_limit"), value_type=float),
                "arm_calibration_file": arm_calibration_file,
                "curve_client_secret_key_file": LaunchConfiguration("curve_client_secret_key_file"),
                "curve_server_public_key_file": LaunchConfiguration("curve_server_public_key_file"),
                # Wheel odometry always starts in its local frame. AMCL or
                # RTAB-Map owns map->odom and the global initial pose.
                "initial_x": 0.0,
                "initial_y": 0.0,
                "xy_velocity_scale": ParameterValue(LaunchConfiguration("xy_velocity_scale"), value_type=float),
                "yaw_velocity_scale": ParameterValue(LaunchConfiguration("yaw_velocity_scale"), value_type=float),
                "permission_timeout": lease,
                # hold arms itself after the first healthy telemetry and current
                # supervisor permission; strict leaves the startup arm to the operator.
                "auto_arm_on_startup": False,
                "disarm_on_failure": strict_policy,
                "odom_topic": "/wheel/odometry",
                "publish_odom_tf": False,
            }],
            remappings=[("safety/state", "safety/driver_state")],
            # A ZMQ connect() timeout can be transient, but an offline
            # robot host must not churn a driver process every few seconds.
            # ponytail: fixed 60s backoff; use exponential backoff if host outages become frequent (#4).
            respawn=True,
            respawn_delay=60.0,
            output="screen",
        ))
        actions.append(Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[config("ekf.yaml")],
            remappings=[("odometry/filtered", "/odom")],
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ))
    if topology.start_moveit:
        actions.extend([
            *gated(
                _readiness_gate("wait_for_arm_controller", kind="follow_joint_trajectory_action",
                                action="/arm_controller/follow_joint_trajectory"),
                "arm controller",
                _include(launch_file("moveit.launch.py"), sim="true" if sim else "false",
                         arm_calibration_file=arm_calibration_file)),
            Node(
                package="lekiwi_rmf",
                executable="arm_workspace_monitor",
                name="arm_workspace_monitor",
                parameters=[*safety_params_files, {"use_sim_time": sim}],
                output="screen",
            ),
            Node(
                package="lekiwi_rmf",
                executable="moveit_cloud_gate",
                name="moveit_cloud_gate",
                parameters=[{"use_sim_time": sim}],
                output="screen",
            ),
        ])
    actions.extend([
        # Join Nav2's smoothed stream and the manually requested stream
        # before collision monitoring. The mux is intentionally live before
        # Nav2 lifecycle activation, so an early controller command cannot
        # bypass the guard while a node is still coming up.
        Node(
            package="lekiwi_rmf",
            executable="cmd_vel_mux",
            name="cmd_vel_mux",
            parameters=[{"permission_timeout": lease}],
            output="screen",
        ),
        *_include(nav2_launch("localization_launch.py"), topology.amcl,
                  map=selected_map, params_file=params_file, use_sim_time=use_sim_time),
    ])
    if topology.visual_slam and topology.static_map:
        actions.extend([
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[{"yaml_filename": selected_map, "use_sim_time": sim}],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                parameters=[{"autostart": True, "node_names": ["map_server"], "service_timeout": 10, "use_sim_time": sim}],
                output="screen",
            ),
        ])
    actions.extend([
        *_include(launch_file("slam.launch.py"), topology.visual_slam, navigation_params_file=params_file),
        *gated(
            _readiness_gate("wait_for_stable_joint_states", kind="joint_states", topic="/joint_states",
                            joint_names=list(ARM_JOINTS), minimum_joint_samples=20),
            "joint states", [safety_supervisor]),
        # Nav2 waits for odometry and the resulting map; fixed delays made
        # both components race slow sensors and telemetry reconnects.
        *gated(
            _readiness_gate("wait_for_odom", kind="topic", topic="/odom", topic_type="odom"),
            "odometry",
            gated(_readiness_gate("wait_for_map", kind="topic", topic="/map", topic_type="map"),
                  "map", navigation)),
    ])
    if topology.start_foxglove:
        actions.append(Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "address": LaunchConfiguration("foxglove_address"),
                "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
                "use_sim_time": sim,
                "capabilities": ["connectionGraph", "assets"],
                "publish_client_count": True,
            }],
            output="screen",
        ))
    if topology.start_rosbridge:
        rosbridge_domain = {"ROS_DOMAIN_ID": LaunchConfiguration("rosbridge_domain")}
        actions.extend([
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                parameters=[{
                    "address": LaunchConfiguration("rosbridge_address"),
                    "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
                }],
                additional_env=rosbridge_domain,
                output="screen",
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                additional_env=rosbridge_domain,
                output="screen",
            ),
        ])
    return actions


def generate_launch_description():
    package = FindPackageShare("lekiwi_rmf")
    config = Path(get_package_share_directory("lekiwi_rmf")) / "config"
    test_linear, test_angular = load_base_speed_limits(config / "nav2_params.yaml")
    # robot_explorer's tracked quota is the default for the overridable arguments.
    mapping_quota = yaml.safe_load((config / "exploration.yaml").read_text())["robot_explorer"]["ros__parameters"]
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="sim", choices=list(PROFILES)),
            OpaqueFunction(function=_profile_defaults),
            # RViz is the normal visualization for this stack. Running Gazebo's server
            # only also works from CI and a machine reached over SSH without an X/GLX
            # display. Pass headless:=false to open Gazebo's own GUI.
            DeclareLaunchArgument("headless", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("remote_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("bounded_base_test", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("base_test_stage", default_value=""),
            DeclareLaunchArgument("base_test_linear_limit", default_value=str(test_linear)),
            DeclareLaunchArgument("base_test_angular_limit", default_value=str(test_angular)),
            DeclareLaunchArgument("curve_client_secret_key_file", default_value=""),
            DeclareLaunchArgument("curve_server_public_key_file", default_value=""),
            # Fleet bridging makes the ROS graph discoverable off-host. Keep it
            # opt-in; a local robot can navigate without an external route.
            DeclareLaunchArgument("start_rmf", default_value="false"),
            # Rosbridge is opt-in and loopback-bound by default. It has no built-in
            # authentication; do not expose it beyond a protected proxy/firewall.
            DeclareLaunchArgument("start_rosbridge", default_value="false"),
            DeclareLaunchArgument("start_foxglove", default_value="true"),
            # MoveIt is optional for mobile navigation and is too expensive to
            # co-run with RTAB-Map on the 4 GB robot computer. Enable it only
            # for an arm task, preferably from the workstation.
            DeclareLaunchArgument("start_moveit", default_value="false"),
            DeclareLaunchArgument(
                "arm_calibration_file",
                default_value=PathJoinSubstitution([
                    EnvironmentVariable("HOME"), ".ros", "lekiwi_arm_calibration.json",
                ]),
            ),
            DeclareLaunchArgument("rosbridge_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("rosbridge_domain", default_value="0"),
            DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("localization", default_value="visual_slam", choices=list(CHOICES["localization"])),
            # A domestic robot keeps extending its map as it runs. The session
            # guard freezes it (switches to localization) at the quota, and
            # slam_mode:=localization pins a finished map.
            DeclareLaunchArgument(
                "slam_mode",
                default_value="mapping",
                choices=list(CHOICES["slam_mode"]),
            ),
            DeclareLaunchArgument("publish_camera", default_value="true"),
            # hold: the robot stays armed; a failure stops the base, freezes the arm with
            # torque on, and the driver re-arms itself. strict (larger robots): every failure
            # disarms, cuts torque, latches TORQUE_FAULT if the cut is unconfirmed, and waits
            # for an explicit safety/arm, including at startup; the supervisor denies motion
            # until the production profile is satisfied.
            DeclareLaunchArgument(
                "safety_policy", default_value="hold", choices=list(CHOICES["safety_policy"])
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
            # "none" leaves the wrist camera out. Both cameras hang off one USB 2.0 hub and
            # neither can be compressed here, so the wrist runs small -- see the node below.
            DeclareLaunchArgument("wrist_camera_device", default_value="none"),
            # LeRobot's kinematics assume base_radius=0.125 m. Measure your own robot --
            # wheel-centre to wheel-centre, divided by sqrt(3), gives the real radius --
            # and set yaw_velocity_scale to 0.125 / that. Wheels 24 cm apart give 0.90.
            DeclareLaunchArgument("xy_velocity_scale", default_value=str(BASE_XY_SCALE)),
            DeclareLaunchArgument("yaw_velocity_scale", default_value=str(BASE_YAW_SCALE)),
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
                    "if '", LaunchConfiguration("mode"), "' == 'sim' else '",
                    EnvironmentVariable("HOME"), "/.ros/lekiwi_rtabmap.db')",
                ]),
            ),
            # RTAB-Map keeps its working memory in RAM and, unbounded, grows without end:
            # an hour of mapping at 1 Hz took 6.6 GB and starved the rest of the machine.
            # Past this many nodes the oldest move to the database and come back only when
            # the robot returns near them, so loop closure still works. Raise it on a
            # machine with memory to spare -- larger working memory closes loops sooner.
            DeclareLaunchArgument("rtabmap_wm_nodes", default_value="300"),
            DeclareLaunchArgument("rtabmap_mapping_max_bytes", default_value=str(mapping_quota["mapping_max_bytes"])),
            DeclareLaunchArgument("rtabmap_mapping_max_seconds", default_value=str(mapping_quota["mapping_max_seconds"]).removesuffix(".0")),
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
                "laser_source", default_value="auto", choices=list(CHOICES["laser_source"])
            ),
            # Prefer a /dev/serial/by-id/... path for the same reason as camera_device.
            DeclareLaunchArgument(
                "lidar_port",
                default_value=lidar_default_port(),
            ),
            # Camera-as-laser obstacle detection, and the geometry it stands on.
            # Measured with the checkerboard on the floor, not taken from the URDF: the
            # camera sits 9.3 cm up and all but level, which is why it sees a chair leg at
            # 20 cm. Re-measure after touching the mount.
            DeclareLaunchArgument("camera_height", default_value="0.093"),
            DeclareLaunchArgument("camera_pitch", default_value="0.031"),
            # Evaluate cross-argument invariants and resolve the topology before
            # the first node, process, or included launch description starts.
            OpaqueFunction(function=validate_context),
            OpaqueFunction(function=_stack),
        ]
    )
