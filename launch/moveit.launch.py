from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_rmf")
        .robot_description(file_path="urdf/lekiwi.urdf.xacro", mappings={"sim": "false"})
        .robot_description_semantic(file_path="config/lekiwi.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    parameters = config.to_dict()
    # The robot has no depth sensor, so the octomap monitor has nothing to feed
    # it. Leaving the 'sensors' parameter unset makes move_group log an ERROR on
    # every start; registering an updater whose plugin is '~' is the supported
    # way to keep the monitor explicitly idle. Pinning the resolution silences
    # the "assuming 0.1" warning with the same value.
    parameters["sensors"] = ["none"]
    parameters["none.sensor_plugin"] = "~"
    parameters["octomap_resolution"] = 0.1
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[parameters],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
