"""Shared ROS/Gazebo actuator topic contract for the simulator."""

WHEEL_NAMES = (
    "sim_base_left_wheel",
    "sim_base_back_wheel",
    "sim_base_right_wheel",
)
WHEEL_JOINTS = tuple(f"{name}_joint" for name in WHEEL_NAMES)
WHEEL_COMMAND_TOPICS = tuple(f"/sim/{name}/cmd_vel" for name in WHEEL_NAMES)
ARM_TRAJECTORY_TOPIC = "/sim/arm/joint_trajectory"
ARM_TRAJECTORY_HEARTBEAT_TOPIC = "/sim/arm/trajectory_heartbeat"

SIM_ACTUATOR_BRIDGE_ARGUMENTS = (
    *(f"{topic}@std_msgs/msg/Float64]gz.msgs.Double" for topic in WHEEL_COMMAND_TOPICS),
    f"{ARM_TRAJECTORY_TOPIC}@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
    f"{ARM_TRAJECTORY_HEARTBEAT_TOPIC}@std_msgs/msg/Bool]gz.msgs.Boolean",
)
