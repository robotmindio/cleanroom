import math


ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
JOINT_LIMITS = {
    "arm_shoulder_pan": (-1.92, 1.92),
    "arm_shoulder_lift": (-1.75, 1.75),
    "arm_elbow_flex": (-1.75, 1.75),
    "arm_wrist_flex": (-1.75, 1.75),
    "arm_gripper": (0.0, 1.57),
}


def action_positions(names, positions):
    if len(names) != len(positions) or not names or len(set(names)) != len(names):
        raise ValueError("trajectory joints and positions must be non-empty and one-to-one")
    if unknown := set(names) - set(ARM_JOINTS):
        raise ValueError(f"unsupported arm joints: {', '.join(sorted(unknown))}")
    if not all(math.isfinite(position) for position in positions):
        raise ValueError("trajectory positions must be finite")
    for name, position in zip(names, positions):
        lower, upper = JOINT_LIMITS.get(name, (-math.inf, math.inf))
        if not lower <= position <= upper:
            raise ValueError("trajectory position exceeds joint limits")
    return {
        name: position / (math.pi / 2) * 100 if name == "arm_gripper" else math.degrees(position)
        for name, position in zip(names, positions)
    }
