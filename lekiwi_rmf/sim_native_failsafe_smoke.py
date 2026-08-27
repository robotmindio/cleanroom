"""Renderer-free fault-injection acceptance for the native Gazebo watchdog."""

from __future__ import annotations

import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node


WHEEL_TOPICS = (
    "/sim/sim_base_left_wheel/cmd_vel",
    "/sim/sim_base_back_wheel/cmd_vel",
    "/sim/sim_base_right_wheel/cmd_vel",
)


class NativeFailsafeSmoke(Node):
    def __init__(self) -> None:
        super().__init__("sim_native_failsafe_smoke")
        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._joints, 20)
        self.wheels = [self.create_publisher(Float64, topic, 10) for topic in WHEEL_TOPICS]
        self.arm = self.create_publisher(JointTrajectory, "/sim/arm/joint_trajectory", 10)
        self.heartbeat = self.create_publisher(Bool, "/sim/arm/trajectory_heartbeat", 10)

    def _joints(self, message: JointState) -> None:
        self.joints.update(zip(message.name, message.position))

    def pump(self, seconds: float, *, heartbeat: bool = False) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if heartbeat:
                self.heartbeat.publish(Bool(data=True))
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 15.0
        required = {"sim_base_left_wheel_joint", "arm_shoulder_pan"}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if required <= self.joints.keys():
                return
        raise RuntimeError("Gazebo joint feedback did not become available")

    def run(self) -> None:
        self.wait_ready()

        # Publish once, then remove the entire ROS-side command source. The
        # native JointController otherwise retains a nonzero Float64 forever.
        for publisher, value in zip(self.wheels, (3.0, 0.0, -3.0)):
            publisher.publish(Float64(data=value))
        self.pump(0.45)
        wheel_after_command = self.joints["sim_base_left_wheel_joint"]
        self.pump(0.70)
        wheel_after_timeout = self.joints["sim_base_left_wheel_joint"]
        self.pump(0.50)
        wheel_settled = self.joints["sim_base_left_wheel_joint"]
        if abs(wheel_settled - wheel_after_timeout) > 0.10:
            raise RuntimeError(
                "native wheel controller continued after public command loss: "
                f"{wheel_after_timeout:.3f} -> {wheel_settled:.3f}"
            )
        if abs(wheel_after_command) < 0.05:
            raise RuntimeError("watchdog test did not observe initial wheel actuation")

        initial_pan = self.joints["arm_shoulder_pan"]
        # A large bounded displacement stays visibly in flight for longer
        # than the 250 ms watchdog window even with Gazebo's stiff position
        # gains; a small target can settle before failure injection is due.
        target_pan = initial_pan + 2.00
        trajectory = JointTrajectory(joint_names=["arm_shoulder_pan"])
        point = JointTrajectoryPoint(positions=[target_pan])
        point.time_from_start.sec = 3
        trajectory.points = [point]
        self.arm.publish(trajectory)
        # The adapter/bridge is present for only 0.20 s, then disappears.
        self.pump(0.20, heartbeat=True)
        self.pump(1.00)
        held_pan = self.joints["arm_shoulder_pan"]
        self.pump(0.60)
        settled_pan = self.joints["arm_shoulder_pan"]
        if abs(settled_pan - held_pan) > 0.06:
            raise RuntimeError(
                "native arm trajectory continued after heartbeat loss: "
                f"{held_pan:.3f} -> {settled_pan:.3f}"
            )
        if not initial_pan + 0.02 < held_pan < target_pan - 0.25:
            raise RuntimeError(
                "arm watchdog did not interrupt the in-flight native trajectory: "
                f"initial={initial_pan:.3f}, held={held_pan:.3f}, target={target_pan:.3f}"
            )
        print("simulation native failsafe smoke passed")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NativeFailsafeSmoke()
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
