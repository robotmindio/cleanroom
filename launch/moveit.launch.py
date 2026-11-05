from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

from lekiwi_rmf.moveit_config import moveit_parameters


def generate_launch_description():
    def launch_move_group(context):
        sim = LaunchConfiguration("sim").perform(context)
        parameters = moveit_parameters(
            sim, LaunchConfiguration("arm_calibration_file").perform(context))
        # Keep the calibrated depth scene and bounded planning-scene updates.
        parameters["octomap_resolution"] = 0.1
        parameters["publish_planning_scene"] = True
        parameters["publish_geometry_updates"] = True
        parameters["publish_state_updates"] = True
        parameters["publish_transforms_updates"] = True
        parameters["publish_planning_scene_hz"] = 20.0
        return [Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[parameters],
            respawn=True,
            respawn_delay=2.0,
            sigterm_timeout="15",
            sigkill_timeout="5",
        )]

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "arm_calibration_file",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), ".ros", "lekiwi_arm_calibration.json",
            ]),
        ),
        OpaqueFunction(function=launch_move_group),
    ])
