"""Shared ROS/Gazebo actuator topic contract for the simulator."""

WHEEL_NAMES = (
    "sim_base_left_wheel",
    "sim_base_back_wheel",
    "sim_base_right_wheel",
)
WHEEL_JOINTS = tuple(f"{name}_joint" for name in WHEEL_NAMES)
WHEEL_COMMAND_TOPICS = tuple(f"/sim/{name}/cmd_vel" for name in WHEEL_NAMES)
