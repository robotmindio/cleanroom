from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    config = (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_rmf")
        .robot_description(file_path="urdf/lekiwi.urdf.xacro", mappings={"sim": sim})
        .robot_description_semantic(file_path="config/lekiwi.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .sensors_3d(file_path="config/moveit_sensors.yaml")
        .to_moveit_configs()
    )
    parameters = config.to_dict()
    parameters["use_sim_time"] = ParameterValue(sim, value_type=bool)
    # The production safety profile denies arm motion until this updater is fed
    # by a fresh, calibrated depth cloud. Pinning the resolution also keeps the
    # planning-scene representation reproducible across machines.
    parameters["octomap_resolution"] = 0.1
    # The execution-time arm gate requires a live monitored scene, not merely
    # an initial planning snapshot. Publish complete state/geometry updates at
    # a bounded frequency so silence is distinguishable from empty free space.
    parameters["publish_planning_scene"] = True
    parameters["publish_geometry_updates"] = True
    parameters["publish_state_updates"] = True
    parameters["publish_transforms_updates"] = True
    parameters["publish_planning_scene_hz"] = 20.0
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[parameters],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
