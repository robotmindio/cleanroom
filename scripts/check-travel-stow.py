#!/usr/bin/env python3
"""Exit 0 only when fresh /joint_states put the arm in the SRDF travel_stow pose.

Anything that cuts arm torque runs this first: elsewhere the unpowered arm falls.
Compares the arm group's travel_stow joints (config/lekiwi.srdf) within 0.02 rad,
using a message received after start whose stamp is at most --max-age old. The
gripper group is not required; it cannot fall onto anything. Never commands motion.
"""

import argparse
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

SEMANTIC = Path(__file__).resolve().parents[1] / "config" / "lekiwi.srdf"
TOLERANCE = 0.02


def travel_stow(path=SEMANTIC):
    states = ET.parse(path).getroot().findall("group_state[@name='travel_stow'][@group='arm']")
    if len(states) != 1:
        raise ValueError("config/lekiwi.srdf must define exactly one arm travel_stow")
    return {joint.get("name"): float(joint.get("value")) for joint in states[0].findall("joint")}


def stow_errors(stow, names, positions, tolerance=TOLERANCE):
    """Joints outside tolerance (or missing), as {name: error or None}."""
    measured = dict(zip(names, positions))
    errors = {}
    for name, target in stow.items():
        if name not in measured:
            errors[name] = None
        elif abs(measured[name] - target) > tolerance:
            errors[name] = measured[name] - target
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--max-age", type=float, default=3.0)
    args = parser.parse_args()
    stow = travel_stow()
    rclpy.init()
    node = rclpy.create_node("travel_stow_check")
    received = []
    node.create_subscription(JointState, "/joint_states", received.append, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            for message in received:
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                if node.get_clock().now().nanoseconds * 1e-9 - stamp > args.max_age:
                    continue
                errors = stow_errors(stow, message.name, message.position)
                if not errors:
                    print("arm is in travel_stow")
                    return 0
                print("arm is NOT in travel_stow: " + ", ".join(
                    f"{name} {'missing' if error is None else f'{error:+.3f} rad'}" for name, error in errors.items()),
                    file=sys.stderr)
                return 1
            received.clear()
        print(f"no fresh /joint_states within {args.timeout:.0f} s", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
