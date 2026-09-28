#!/usr/bin/env python3
"""Send one small, explicit physical joint jog through the arm controller.

This is intentionally a command-line tool rather than automatic RViz-slider
execution: a slider emits many intermediate values and must never cause motion
without an operator's explicit confirmation.
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lekiwi_rmf.arm_trajectory import JOINT_LIMITS


ALIASES = {
    "shoulder_pan": "arm_shoulder_pan",
    "shoulder_lift": "arm_shoulder_lift",
    "elbow_flex": "arm_elbow_flex",
    "wrist_flex": "arm_wrist_flex",
    "wrist_roll": "arm_wrist_roll",
    "gripper": "arm_gripper",
}


class ArmJogger(rclpy.node.Node):
    def __init__(self):
        super().__init__("arm_jog")
        self.positions = {}
        self.received_at = 0.0
        self.subscription = self.create_subscription(JointState, "/joint_states", self._joint_state, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

    def _joint_state(self, message):
        self.positions.update(zip(message.name, message.position))
        self.received_at = time.monotonic()

    def current_position(self, joint, timeout=3.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if joint in self.positions and time.monotonic() - self.received_at < 1.0:
                return self.positions[joint]
        raise RuntimeError("no fresh joint state from the real robot")


def wait_for_result(node, handle, result_future, timeout):
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    if result_future.done():
        return result_future.result()

    cancel_future = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=3.0)
    if not cancel_future.done():
        raise RuntimeError("jog timed out; controller did not confirm a cancellation request")
    response = cancel_future.result()
    if not response.goals_canceling:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=2.0)
        if result_future.done():
            return result_future.result()
        raise RuntimeError("jog timed out; controller neither canceled nor completed the goal")

    rclpy.spin_until_future_complete(node, result_future, timeout_sec=2.0)
    if not result_future.done():
        raise RuntimeError("jog timed out; cancellation was acknowledged but no final result arrived")
    raise RuntimeError("jog timed out; controller acknowledged cancellation")


def result_is_successful(result):
    return (
        result.status == GoalStatus.STATUS_SUCCEEDED
        and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move exactly one real arm joint by a small delta through the safety-limited controller."
    )
    parser.add_argument("joint", choices=sorted(ALIASES), help="joint to move")
    parser.add_argument("delta", type=float, help="signed distance in radians (normally +0.1 or -0.1)")
    parser.add_argument("--duration", type=float, default=1.0, help="move duration in seconds (default: 1.0)")
    parser.add_argument("--yes", action="store_true", help="confirm this physical movement")
    args = parser.parse_args()
    if not math.isfinite(args.delta) or not 0.0 < abs(args.delta) <= 0.1:
        parser.error("delta must be finite, nonzero, and no larger than 0.1 rad")
    if not math.isfinite(args.duration) or not 0.5 <= args.duration <= 5.0:
        parser.error("--duration must be between 0.5 and 5.0 seconds")
    return args


def main():
    args = parse_args()
    joint = ALIASES[args.joint]
    rclpy.init()
    node = ArmJogger()
    try:
        current = node.current_position(joint)
        target = current + args.delta
        lower, upper = JOINT_LIMITS.get(joint, (-math.inf, math.inf))
        if not lower <= target <= upper:
            raise RuntimeError(
                f"refusing: target {target:.3f} rad exceeds {joint} limit [{lower:.2f}, {upper:.2f}]"
            )
        print(f"{joint}: {current:.3f} -> {target:.3f} rad in {args.duration:.1f}s")
        if not args.yes:
            answer = input("This moves the physical robot. Type yes to continue: ").strip().lower()
            if answer != "yes":
                print("Cancelled; no command sent.")
                return 0
        if not node.client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("arm controller action is unavailable; start the stack (scripts/up.sh or lekiwi-stack.service) and check that the driver is armed")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=[joint],
            points=[JointTrajectoryPoint(
                positions=[target], time_from_start=Duration(sec=int(args.duration), nanosec=int((args.duration % 1) * 1e9))
            )],
        )
        future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("controller rejected jog (the arm may be disarmed)")
        result_future = handle.get_result_async()
        try:
            result = wait_for_result(node, handle, result_future, args.duration + 7.0)
        except RuntimeError as error:
            raise RuntimeError(f"jog failed: {error}") from error
        if not result_is_successful(result):
            detail = result.result.error_string or (
                f"action status {result.status}, error code {result.result.error_code}"
            )
            raise RuntimeError(f"jog failed: {detail}")
        print("Jog completed.")
        return 0
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f"arm jog: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
