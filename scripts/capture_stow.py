#!/usr/bin/env python3
"""Record the held arm's stow pose into both safety configuration files.

Position the arm in its reviewed, collision-free stow pose (armed and holding, or
supported), then run this on a machine that sees the driver's /joint_states. It
averages the six arm joints, refuses if they move, and writes the same values to
config/safety_production.yaml (stow_joint_positions) and
config/safety_acceptance.yaml (accepted_stow_joint_positions), which the
supervisor requires to match. It never commands motion. Review and commit the diff.
"""

import argparse
import re
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRODUCTION = ROOT / "config" / "safety_production.yaml"
ACCEPTANCE = ROOT / "config" / "safety_acceptance.yaml"
STOW_JOINTS = (
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll", "arm_gripper",
)
# Radians. Well under the supervisor's 0.08 stow tolerance.
MAX_SPREAD = 0.01
DECIMALS = 4


def stow_from_samples(samples):
    """Mean of each joint over ``samples`` ({joint: position} dicts), rounded.

    Raises ValueError when a joint is missing or moved more than MAX_SPREAD.
    """
    stow = {}
    for joint in STOW_JOINTS:
        values = [sample[joint] for sample in samples if joint in sample]
        if len(values) != len(samples) or not values:
            raise ValueError(f"{joint} is missing from the joint states")
        spread = max(values) - min(values)
        if spread > MAX_SPREAD:
            raise ValueError(f"{joint} moved {spread:.3f} rad while recording; hold the arm still")
        stow[joint] = round(statistics.fmean(values), DECIMALS)
    return stow


def write_stow(production_text, acceptance_text, stow):
    """Both files' text with ``stow`` in place of their stow values; comments are kept."""
    positions = ", ".join(repr(stow[joint]) for joint in STOW_JOINTS)
    production, count = re.subn(
        r"^(\s*stow_joint_positions:\s*)\[[^\]]*\]", rf"\g<1>[{positions}]",
        production_text, flags=re.MULTILINE,
    )
    if count != 1:
        raise ValueError("safety_production.yaml needs exactly one stow_joint_positions list")
    acceptance = acceptance_text
    for joint in STOW_JOINTS:
        acceptance, count = re.subn(
            rf"^(\s+{joint}:\s*)\S+$", rf"\g<1>{stow[joint]!r}",
            acceptance, count=1, flags=re.MULTILINE,
        )
        if count != 1:
            raise ValueError(f"safety_acceptance.yaml has no accepted stow entry for {joint}")
    return production, acceptance


def record(samples_wanted, timeout):
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("capture_stow")
    samples = []
    node.create_subscription(
        JointState, "/joint_states",
        lambda message: samples.append(dict(zip(message.name, message.position))), 10,
    )
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while len(samples) < samples_wanted and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.try_shutdown()
    if len(samples) < samples_wanted:
        sys.exit(f"received {len(samples)} of {samples_wanted} /joint_states messages in {timeout:.0f} s")
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    try:
        stow = stow_from_samples(record(args.samples, args.timeout))
        production, acceptance = write_stow(
            PRODUCTION.read_text(encoding="utf-8"), ACCEPTANCE.read_text(encoding="utf-8"), stow
        )
    except ValueError as error:
        sys.exit(str(error))
    PRODUCTION.write_text(production, encoding="utf-8")
    ACCEPTANCE.write_text(acceptance, encoding="utf-8")
    for joint in STOW_JOINTS:
        print(f"{joint}: {stow[joint]}")
    print("Wrote both stow entries; review the collision check, then commit them together.")


if __name__ == "__main__":
    main()
