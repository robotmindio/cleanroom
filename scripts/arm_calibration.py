#!/usr/bin/env python3
"""Capture the LeRobot joint values for the URDF's upright zero pose.

Start the real stack with no ~/.ros/lekiwi_arm_calibration.json, put the supported arm
in the upright URDF pose, then run this once. Restart the stack after it writes the file.
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lekiwi_rmf.arm_trajectory import ARM_JOINTS


class ArmCalibration(Node):
    def __init__(self):
        super().__init__("arm_calibration")
        self.positions = None
        self.create_subscription(JointState, "joint_states", self.on_joint_state, 10)

    def on_joint_state(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in ARM_JOINTS):
            self.positions = {name: values[name] for name in ARM_JOINTS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=os.path.expanduser("~/.ros/lekiwi_arm_calibration.json"))
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    if os.path.exists(args.output):
        raise RuntimeError(f"{args.output} already exists; move it aside before recalibrating")

    rclpy.init()
    node = ArmCalibration()
    try:
        deadline = time.monotonic() + args.timeout
        while node.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.positions is None:
            raise RuntimeError("did not receive all arm joints on /joint_states")
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        temporary = f"{args.output}.tmp"
        with open(temporary, "w") as output:
            json.dump({"zero_positions": node.positions, "directions": dict.fromkeys(ARM_JOINTS, 1)}, output, indent=2)
            output.write("\n")
        os.replace(temporary, args.output)
        print(f"Saved {args.output}; restart scripts/up.sh to apply it.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
