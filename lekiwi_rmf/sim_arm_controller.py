"""FollowJointTrajectory facade for Gazebo's native joint controller.

The native Gazebo controller consumes ``trajectory_msgs/JointTrajectory`` but
MoveIt requires the standard action contract.  This adapter validates every
segment with the same library as the hardware driver, forwards the command,
and evaluates feedback and tolerances against physics-produced joint states.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lekiwi_rmf.arm_trajectory import (
    ARM_JOINTS,
    position_tolerances,
    prepare_trajectory,
    sample_trajectory,
)


def duration_seconds(duration) -> float:
    if duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000:
        raise ValueError("duration is malformed")
    value = float(duration.sec) + float(duration.nanosec) / 1e9
    if not math.isfinite(value):
        raise ValueError("duration must be finite")
    return value


def stamp_nanoseconds(stamp) -> int:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("trajectory header timestamp is malformed")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def trajectory_rows(trajectory: JointTrajectory) -> list[tuple]:
    return [
        (
            duration_seconds(point.time_from_start),
            tuple(point.positions),
            tuple(point.velocities),
            tuple(point.accelerations),
            tuple(point.effort),
        )
        for point in trajectory.points
    ]


def permission_is_fresh(
    permitted: bool,
    received_at: float | None,
    now: float,
    timeout: float,
) -> bool:
    """Return whether a positive safety heartbeat still owns its short lease."""
    return bool(
        permitted
        and received_at is not None
        and 0.0 <= now - received_at <= timeout
    )


class SimArmController(Node):
    def __init__(self) -> None:
        super().__init__("sim_arm_controller")
        self.declare_parameter("state_timeout", 0.25)
        self.declare_parameter("permission_timeout", 0.5)
        self.declare_parameter("default_goal_tolerance", 0.05)
        self.declare_parameter("default_goal_time_tolerance", 1.0)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.permission_timeout = float(self.get_parameter("permission_timeout").value)
        self.default_goal_tolerance = float(self.get_parameter("default_goal_tolerance").value)
        self.default_goal_time = float(self.get_parameter("default_goal_time_tolerance").value)
        if self.permission_timeout <= 0.0:
            raise ValueError("permission_timeout must be positive")
        self._positions: dict[str, float] = {}
        self._position_stamps_ns: dict[str, int] = {}
        self._state_lock = threading.Lock()
        self._arm_permitted = False
        self._permission_received_at: float | None = None
        self._permission_lock = threading.RLock()
        self._reservation = threading.Lock()
        self._goal_reserved = False
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, "/sim/arm/joint_trajectory", 10
        )
        # The native Gazebo watchdog owns the final actuator topics. It only
        # permits an autonomous native trajectory to continue while this
        # action facade is alive and actively supervising it.
        self._trajectory_heartbeat = self.create_publisher(
            Bool, "/sim/arm/trajectory_heartbeat", 10
        )
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, "/safety/arm_motion_permitted", self._permission, latched
        )
        self._action = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup(),
        )

    def _joint_state(self, message: JointState) -> None:
        values = {
            name: float(position)
            for name, position in zip(message.name, message.position)
            if name in ARM_JOINTS and math.isfinite(position)
        }
        if values:
            received_at_ns = self.get_clock().now().nanoseconds
            with self._state_lock:
                self._positions.update(values)
                self._position_stamps_ns.update(dict.fromkeys(values, received_at_ns))

    def _permission(self, message: Bool) -> None:
        with self._permission_lock:
            self._arm_permitted = bool(message.data)
            self._permission_received_at = time.monotonic()
            if not self._arm_permitted:
                with self._reservation:
                    active_names = getattr(self, "_active_names", ())
                if active_names:
                    self._hold(active_names)

    def _permission_fresh(self) -> bool:
        with self._permission_lock:
            return permission_is_fresh(
                self._arm_permitted,
                self._permission_received_at,
                time.monotonic(),
                self.permission_timeout,
            )

    def _goal(self, request: FollowJointTrajectory.Goal) -> GoalResponse:
        names = tuple(request.trajectory.joint_names)
        if (
            not names
            or len(set(names)) != len(names)
            or set(names) - set(ARM_JOINTS)
            or not request.trajectory.points
        ):
            return GoalResponse.REJECT
        with self._reservation:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _fresh_positions(self, names: tuple[str, ...]) -> dict[str, float] | None:
        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            if any(
                name not in self._positions
                or name not in self._position_stamps_ns
                or not 0 <= now_ns - self._position_stamps_ns[name] <= self.state_timeout * 1e9
                for name in names
            ):
                return None
            return {name: self._positions[name] for name in names}

    @staticmethod
    def _result(code: int, text: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = text
        return result

    def _hold(self, names: tuple[str, ...]) -> None:
        # A feedback timeout is itself a stop condition, so requiring fresh
        # feedback here would leave Gazebo's previously published trajectory
        # active. The last finite position is safer than allowing that motion
        # to continue open-loop.
        with self._state_lock:
            if any(name not in self._positions for name in names):
                return
            positions = {name: self._positions[name] for name in names}
        command = JointTrajectory()
        command.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in names]
        point.time_from_start.nanosec = 100_000_000
        command.points = [point]
        self._trajectory_publisher.publish(command)

    def _feedback(
        self,
        names: tuple[str, ...],
        desired_positions: dict[str, float],
        desired_velocities: dict[str, float],
        desired_accelerations: dict[str, float],
        actual: dict[str, float],
    ) -> FollowJointTrajectory.Feedback:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(names)
        feedback.desired.positions = [desired_positions[name] for name in names]
        feedback.desired.velocities = [desired_velocities[name] for name in names]
        feedback.desired.accelerations = [desired_accelerations[name] for name in names]
        feedback.actual.positions = [actual[name] for name in names]
        feedback.error.positions = [
            desired_positions[name] - actual[name] for name in names
        ]
        return feedback

    def _execute(self, goal_handle):
        names = tuple(goal_handle.request.trajectory.joint_names)
        with self._reservation:
            self._active_names = names
        try:
            start = self._fresh_positions(names)
            if start is None:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "simulated joint feedback is incomplete or stale",
                )
            if not self._permission_fresh():
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "safety supervisor denies simulated arm motion",
                )
            try:
                points = prepare_trajectory(names, trajectory_rows(goal_handle.request.trajectory), start)
                path_tolerances = position_tolerances(
                    names, goal_handle.request.path_tolerance, {}
                )
                goal_tolerances = position_tolerances(
                    names,
                    goal_handle.request.goal_tolerance,
                    dict.fromkeys(names, self.default_goal_tolerance),
                )
                goal_time = duration_seconds(goal_handle.request.goal_time_tolerance)
                start_ns = stamp_nanoseconds(goal_handle.request.trajectory.header.stamp)
            except ValueError as error:
                goal_handle.abort()
                return self._result(FollowJointTrajectory.Result.INVALID_GOAL, str(error))

            now_ns = self.get_clock().now().nanoseconds
            if start_ns and start_ns < now_ns:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP,
                    "trajectory header stamp is in the past",
                )
            while start_ns and self.get_clock().now().nanoseconds < start_ns:
                if goal_handle.is_cancel_requested or not self._permission_fresh():
                    self._hold(names)
                    goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
                    return self._result(FollowJointTrajectory.Result.INVALID_GOAL, "trajectory canceled before start")
                time.sleep(0.01)

            command = goal_handle.request.trajectory
            command.header.stamp.sec = 0
            command.header.stamp.nanosec = 0
            with self._permission_lock:
                if goal_handle.is_cancel_requested or not self._permission_fresh():
                    self._hold(names)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.INVALID_GOAL,
                        "trajectory canceled or safety permission withdrawn before dispatch",
                    )
                self._trajectory_publisher.publish(command)
                self._trajectory_heartbeat.publish(Bool(data=True))
            execution_start_ns = self.get_clock().now().nanoseconds
            final_time = points[-1].time
            if goal_time <= 0.0:
                goal_time = self.default_goal_time

            while True:
                self._trajectory_heartbeat.publish(Bool(data=True))
                now_ns = self.get_clock().now().nanoseconds
                elapsed = max(0.0, (now_ns - execution_start_ns) / 1e9)
                actual = self._fresh_positions(names)
                if goal_handle.is_cancel_requested:
                    self._hold(names)
                    goal_handle.canceled()
                    return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory canceled and held")
                if not self._permission_fresh() or actual is None:
                    self._hold(names)
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.INVALID_GOAL,
                        "safety permission expired/withdrawn or joint feedback stale",
                    )
                desired, velocity, acceleration = sample_trajectory(names, start, points, elapsed)
                goal_handle.publish_feedback(
                    self._feedback(names, desired, velocity, acceleration, actual)
                )
                if elapsed < final_time:
                    violated = [
                        name
                        for name, tolerance in path_tolerances.items()
                        if abs(desired[name] - actual[name]) > tolerance
                    ]
                    if violated:
                        self._hold(names)
                        goal_handle.abort()
                        return self._result(
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                            f"path tolerance violated: {', '.join(violated)}",
                        )
                else:
                    outside = [
                        name
                        for name, tolerance in goal_tolerances.items()
                        if abs(points[-1].positions[name] - actual[name]) > tolerance
                    ]
                    if not outside:
                        goal_handle.succeed()
                        return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "")
                    if elapsed > final_time + goal_time:
                        self._hold(names)
                        goal_handle.abort()
                        return self._result(
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                            f"goal tolerance violated: {', '.join(outside)}",
                        )
                time.sleep(0.02)
        finally:
            with self._reservation:
                self._active_names = ()
                self._goal_reserved = False

    def destroy_node(self):
        self._action.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimArmController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
