import math
from pathlib import Path
import xml.etree.ElementTree as ET

from moveit_configs_utils import MoveItConfigsBuilder

from lekiwi_rmf.arm_trajectory import GRIPPER_LOWER, GRIPPER_UPPER, load_calibration


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


def moveit_parameters(sim, calibration_file):
    """The same model, clock and physical calibration for MoveGroup and RViz."""
    if sim not in ("true", "false"):
        raise ValueError('sim must be true or false')
    parameters = moveit_config_builder(sim).sensors_3d(
        file_path="config/moveit_sensors.yaml").to_moveit_configs().to_dict()
    parameters["collision_detector"] = "lekiwi_rmf/RestFCL"
    parameters["use_sim_time"] = sim == "true"
    if sim == "false":
        # The guarded driver pauses its execution clock when permission is lost.
        parameters.setdefault("trajectory_execution", {})["execution_duration_monitoring"] = False
        apply_gripper_calibration(parameters, calibration_file)
    return parameters


def apply_gripper_calibration(parameters, calibration_file):
    """Keep MoveIt's gripper limits and endpoints aligned with calibration."""
    calibration = Path(calibration_file).expanduser()
    if not calibration.is_file():
        return
    zero_positions, directions = load_calibration(calibration)
    endpoints = (
        directions["arm_gripper"] * (GRIPPER_LOWER - zero_positions["arm_gripper"]),
        directions["arm_gripper"] * (GRIPPER_UPPER - zero_positions["arm_gripper"]),
    )
    configured = parameters["robot_description_planning"]["joint_limits"]["arm_gripper"]
    lower = max(float(configured["min_position"]), min(endpoints))
    upper = min(float(configured["max_position"]), max(endpoints))
    if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
        raise ValueError(f"arm gripper calibration has no reachable MoveIt range: {calibration}")
    if not lower <= 0.0 <= upper:
        raise ValueError(f"arm gripper calibrated closed position is outside MoveIt's range: {calibration}")
    configured["min_position"], configured["max_position"] = lower, upper

    semantic = ET.fromstring(parameters["robot_description_semantic"])
    # Gripper calibration records zero at physical jaw contact. The driver's
    # raw servo minimum sits beyond contact under load and cannot be reached.
    for name, value in (("closed", 0.0), ("open", upper)):
        state = semantic.find(
            f"group_state[@name='{name}'][@group='gripper']/joint[@name='arm_gripper']"
        )
        if state is None:
            raise ValueError(f"MoveIt SRDF is missing gripper state {name!r}")
        state.set("value", format(value, ".12g"))
    parameters["robot_description_semantic"] = ET.tostring(semantic, encoding="unicode")
