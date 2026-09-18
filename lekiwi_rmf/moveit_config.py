from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config_builder(sim):
    return (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_rmf")
        .robot_description(file_path="urdf/lekiwi.urdf.xacro", mappings={"sim": sim})
        .robot_description_semantic(file_path="config/lekiwi.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
    )
