#!/usr/bin/env python3
"""Capture fresh raw joint values for the SO-101 new-calibration zero pose.

Use the generated SO-101 zero-pose reference, not the legacy folded pose.
The raw topic is independent of the driver's loaded pose offsets. Support the
disarmed arm in the reference pose, then run this once. Restart the driver
after saving to apply the new calibration.
"""
import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, load_calibration


class ArmCalibration(Node):
    def __init__(self):
        super().__init__("arm_calibration")
        self.positions = None
        self.create_subscription(JointState, "arm/raw_joint_states", self.on_joint_state, 10)

    def on_joint_state(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not 0 <= age <= 1.0 or len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if len(values) == len(message.name) and all(
            name in values and math.isfinite(values[name]) for name in ARM_JOINTS
        ):
            self.positions = {name: values[name] for name in ARM_JOINTS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=os.path.expanduser("~/.ros/lekiwi_arm_calibration.json"))
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--directions-from", help="preserve verified joint directions from the previous calibration")
    args = parser.parse_args()
    if os.path.exists(args.output):
        raise RuntimeError(f"{args.output} already exists; move it aside before recalibrating")
    directions = (load_calibration(args.directions_from)[1] if args.directions_from
                  else dict.fromkeys(ARM_JOINTS, 1))

    rclpy.init()
    node = ArmCalibration()
    try:
        deadline = time.monotonic() + args.timeout
        while node.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.positions is None:
            raise RuntimeError("did not receive fresh, finite arm joints on /arm/raw_joint_states")
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        temporary = f"{args.output}.tmp"
        with open(temporary, "w") as output:
            json.dump({"model": "so101_new_calib", "zero_positions": node.positions,
                       "directions": directions}, output, indent=2)
            output.write("\n")
        os.replace(temporary, args.output)
        print(f"Saved {args.output}; restart scripts/up.sh to apply it.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
