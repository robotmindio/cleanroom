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
    return LaunchDescription([
        Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[config.to_dict()]),
    ])
