"""Active acceptance client for the renderer-free Gazebo physics test."""

from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PhysicsSmoke(Node):
    def __init__(self) -> None:
        super().__init__("sim_physics_smoke")
        self.odom: Odometry | None = None
        self.joints: dict[str, float] = {}
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(JointState, "/joint_states", self._joints, 20)
        self.command = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.permission = self.create_publisher(
            Bool, "/safety/arm_motion_permitted", latched
        )
        self.arm = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

    def _odom(self, message: Odometry) -> None:
        self.odom = message

    def _joints(self, message: JointState) -> None:
        self.joints.update(zip(message.name, message.position))

    def pump(self, duration: float, command: Twist | None = None) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.permission.publish(Bool(data=True))
            if command is not None:
                self.command.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.permission.publish(Bool(data=True))
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.odom is not None
                and "arm_shoulder_pan" in self.joints
                and self.arm.wait_for_server(timeout_sec=0.0)
            ):
                return
        raise RuntimeError("simulation did not produce odometry, joints and arm action")

    def run(self) -> None:
        self.wait_ready()
        initial_x = self.odom.pose.pose.position.x
        initial_y = self.odom.pose.pose.position.y
        drive = Twist()
        drive.linear.x = 0.10
        self.pump(2.5, drive)
        self.pump(1.0, Twist())
        dx = self.odom.pose.pose.position.x - initial_x
        dy = self.odom.pose.pose.position.y - initial_y
        if not 0.10 <= dx <= 0.35 or abs(dy) > 0.04:
            raise RuntimeError(f"encoder odometry motion outside acceptance: dx={dx:.3f}, dy={dy:.3f}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=["arm_shoulder_pan", "arm_gripper"]
        )
        point = JointTrajectoryPoint(positions=[0.25, 0.30])
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        goal.goal_tolerance = [
            JointTolerance(name="arm_shoulder_pan", position=0.08),
            JointTolerance(name="arm_gripper", position=0.08),
        ]
        goal.goal_time_tolerance.sec = 2
        send = self.arm.send_goal_async(goal)
        while not send.done():
            self.pump(0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("simulated arm rejected a valid trajectory")
        result = handle.get_result_async()
        deadline = time.monotonic() + 8.0
        while not result.done() and time.monotonic() < deadline:
            self.pump(0.05)
        if not result.done():
            raise RuntimeError("simulated arm trajectory timed out")
        response = result.result()
        if (
            response is None
            or response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError("simulated arm trajectory did not succeed")
        final = float(self.joints["arm_shoulder_pan"])
        gripper = float(self.joints["arm_gripper"])
        if not math.isclose(final, 0.25, abs_tol=0.08):
            raise RuntimeError(f"simulated shoulder feedback missed target: {final:.3f}")
        if not math.isclose(gripper, 0.30, abs_tol=0.08):
            raise RuntimeError(f"simulated gripper feedback missed target: {gripper:.3f}")
        print(
            "simulation physics smoke passed: "
            f"odom dx={dx:.3f}, dy={dy:.3f}, shoulder={final:.3f}, "
            f"gripper={gripper:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicsSmoke()
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
