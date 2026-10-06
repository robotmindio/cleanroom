"""Gazebo physics, bridges and simulated actuators; included by bringup in sim mode.

Simulated joint states come from Gazebo physics. A generic
joint_state_publisher would publish zeros concurrently and make
robot_state_publisher / MoveIt alternate between fake and actual arm positions.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _sim_process(module):
    return ExecuteProcess(
        cmd=["python3", "-m", f"lekiwi_rmf.{module}", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )


def generate_launch_description():
    package = FindPackageShare("lekiwi_rmf")
    return LaunchDescription([
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
                IfElseSubstitution(LaunchConfiguration("headless"),
                                   if_value="-r -s --headless-rendering ", else_value="-r "),
                PathJoinSubstitution([package, "worlds", "cleanroom.sdf"]),
            ]}.items(),
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
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/sim/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/sim/sim_base_left_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                "/sim/sim_base_back_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
                "/sim/sim_base_right_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            ],
            remappings=[("/sim/joint_states", "/joint_states")],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="sim_arm_position_bridge",
            arguments=[
                "/sim/arm/joint_positions@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory"
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sim_lidar_bridge",
            arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            parameters=[{"override_frame_id": "laser"}],
            remappings=[("/scan", "/sim/scan_raw")],
            output="screen",
        ),
        # The simulated scan is body-masked exactly like a real LD06's.
        Node(
            package="lekiwi_rmf",
            executable="scan_self_filter",
            name="scan_self_filter",
            parameters=[
                PathJoinSubstitution([package, "config", "lidar_self_mask_simulation.yaml"]),
                {"input_topic": "/sim/scan_raw"},
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sim_camera_bridge",
            # gz derives CameraInfo from the parent namespace. All three
            # streams originate at the same optical frame.
            arguments=[
                "/camera/front@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
            parameters=[{"override_frame_id": "front_camera_optical_frame"}],
            remappings=[
                ("/camera/front", "/camera/front/image_raw"),
                ("/camera/camera_info", "/camera/front/camera_info"),
                # Preserve the acquisition stamp while adding seeded
                # transport latency/dropout before MoveIt sees the cloud.
                ("/camera/depth/points", "/camera/depth/points_raw"),
            ],
            output="screen",
        ),
        _sim_process("sim_omni_controller"),
        _sim_process("sim_sensor_delay"),
        _sim_process("sim_arm_controller"),
    ])
