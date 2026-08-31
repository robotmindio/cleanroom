"""Three-wheel omni base controller and encoder odometry for Gazebo.

Gazebo owns the wheel contacts and joint dynamics.  This node only performs
the same kinematic transform as the physical LeKiwi firmware, applies tracked
velocity / acceleration / jerk limits, and derives odometry from simulated
wheel positions.  It intentionally never publishes ground-truth model pose as
``/odom``: localization sees the same encoder limitations as hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from lekiwi_rmf.sim_topics import WHEEL_COMMAND_TOPICS, WHEEL_JOINTS


WHEEL_RADIUS = 0.050
BASE_RADIUS = 0.125
SQRT3 = math.sqrt(3.0)


def body_to_wheels(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    """Map REP-103 body velocity to left, back and right wheel rad/s."""
    return (
        (-SQRT3 * 0.5 * x + 0.5 * y + BASE_RADIUS * yaw) / WHEEL_RADIUS,
        (-y + BASE_RADIUS * yaw) / WHEEL_RADIUS,
        (SQRT3 * 0.5 * x + 0.5 * y + BASE_RADIUS * yaw) / WHEEL_RADIUS,
    )


def wheels_to_body(left: float, back: float, right: float) -> tuple[float, float, float]:
    """Inverse kinematics for wheel angular rates or angular increments."""
    return (
        WHEEL_RADIUS * (right - left) / SQRT3,
        WHEEL_RADIUS * (left - 2.0 * back + right) / 3.0,
        WHEEL_RADIUS * (left + back + right) / (3.0 * BASE_RADIUS),
    )


def saturate_wheels(wheels: tuple[float, float, float], limit: float) -> tuple[float, float, float]:
    """Preserve the requested chassis direction while respecting motor speed."""
    peak = max(abs(value) for value in wheels)
    if peak <= limit:
        return wheels
    scale = limit / peak
    return tuple(value * scale for value in wheels)


@dataclass
class MotionLimiter:
    """Deterministic component-wise acceleration and jerk limiter."""

    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def update(
        self,
        desired: tuple[float, float, float],
        dt: float,
        acceleration_limits: tuple[float, float, float],
        jerk_limits: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        if not 0.0 < dt <= 0.25:
            self.acceleration = (0.0, 0.0, 0.0)
            return self.velocity
        next_velocity, next_acceleration = [], []
        for target, current, previous_accel, accel_limit, jerk_limit in zip(
            desired, self.velocity, self.acceleration, acceleration_limits, jerk_limits
        ):
            # A finite convergence time avoids demanding an acceleration step
            # of error/dt every cycle, which would defeat the jerk limit as the
            # target is reached.
            requested_accel = max(-accel_limit, min(accel_limit, (target - current) * 4.0))
            accel_delta = max(
                -jerk_limit * dt,
                min(jerk_limit * dt, requested_accel - previous_accel),
            )
            acceleration = previous_accel + accel_delta
            velocity = current + acceleration * dt
            next_velocity.append(velocity)
            next_acceleration.append(acceleration)
        self.velocity = tuple(next_velocity)
        self.acceleration = tuple(next_acceleration)
        return self.velocity


class SimOmniController(Node):
    def __init__(self) -> None:
        super().__init__("sim_omni_controller")
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("maximum_linear_velocity", 0.30)
        self.declare_parameter("maximum_angular_velocity", 1.57)
        self.declare_parameter("linear_acceleration", 0.60)
        self.declare_parameter("angular_acceleration", 3.14)
        self.declare_parameter("linear_jerk", 2.0)
        self.declare_parameter("angular_jerk", 10.0)
        self.declare_parameter("maximum_wheel_velocity", 12.0)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_linear = float(self.get_parameter("maximum_linear_velocity").value)
        self.max_angular = float(self.get_parameter("maximum_angular_velocity").value)
        self.acceleration_limits = (
            float(self.get_parameter("linear_acceleration").value),
            float(self.get_parameter("linear_acceleration").value),
            float(self.get_parameter("angular_acceleration").value),
        )
        self.jerk_limits = (
            float(self.get_parameter("linear_jerk").value),
            float(self.get_parameter("linear_jerk").value),
            float(self.get_parameter("angular_jerk").value),
        )
        self.max_wheel_velocity = float(self.get_parameter("maximum_wheel_velocity").value)
        limits = (
            self.command_timeout,
            self.max_linear,
            self.max_angular,
            *self.acceleration_limits,
            *self.jerk_limits,
            self.max_wheel_velocity,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError("simulated drivetrain timeouts and motion limits must be finite and positive")

        self._desired = (0.0, 0.0, 0.0)
        self._last_command_ns: int | None = None
        self._last_control_ns: int | None = None
        self._limiter = MotionLimiter()
        self._wheel_publishers = [self.create_publisher(Float64, topic, 10) for topic in WHEEL_COMMAND_TOPICS]
        self.create_subscription(Twist, "/cmd_vel_safe", self._command, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self._tf = TransformBroadcaster(self)
        self._last_wheels: tuple[float, float, float] | None = None
        self._last_wheel_stamp_ns: int | None = None
        self._x = self._y = self._yaw = 0.0
        self.create_timer(0.02, self._control)

    def _command(self, message: Twist) -> None:
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("discarding non-finite simulated base command")
            return
        self._desired = (
            max(-self.max_linear, min(self.max_linear, values[0])),
            max(-self.max_linear, min(self.max_linear, values[1])),
            max(-self.max_angular, min(self.max_angular, values[2])),
        )
        self._last_command_ns = self.get_clock().now().nanoseconds

    def _control(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_control_ns is None:
            self._last_control_ns = now_ns
            return
        dt = (now_ns - self._last_control_ns) / 1e9
        self._last_control_ns = now_ns
        desired = self._desired
        if self._last_command_ns is None or (now_ns - self._last_command_ns) / 1e9 > self.command_timeout:
            desired = (0.0, 0.0, 0.0)
        limited = self._limiter.update(desired, dt, self.acceleration_limits, self.jerk_limits)
        wheels = saturate_wheels(body_to_wheels(*limited), self.max_wheel_velocity)
        for publisher, velocity in zip(self._wheel_publishers, wheels):
            publisher.publish(Float64(data=velocity))

    @staticmethod
    def _message_stamp_ns(message: JointState) -> int:
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in WHEEL_JOINTS):
            return
        wheels = tuple(float(positions[name]) for name in WHEEL_JOINTS)
        if not all(math.isfinite(value) for value in wheels):
            return
        stamp_ns = self._message_stamp_ns(message) or self.get_clock().now().nanoseconds
        if self._last_wheels is None or self._last_wheel_stamp_ns is None:
            self._last_wheels, self._last_wheel_stamp_ns = wheels, stamp_ns
            return
        dt = (stamp_ns - self._last_wheel_stamp_ns) / 1e9
        if not 0.0 < dt <= 0.5:
            self._last_wheels, self._last_wheel_stamp_ns = wheels, stamp_ns
            return
        increments = tuple(current - previous for current, previous in zip(wheels, self._last_wheels))
        # Reject simulator resets and joint teleports rather than integrating a
        # discontinuity into localization.
        if any(abs(value) > self.max_wheel_velocity * dt * 1.5 + 0.05 for value in increments):
            self._last_wheels, self._last_wheel_stamp_ns = wheels, stamp_ns
            return
        dx_body, dy_body, dyaw = wheels_to_body(*increments)
        heading = self._yaw + 0.5 * dyaw
        self._x += math.cos(heading) * dx_body - math.sin(heading) * dy_body
        self._y += math.sin(heading) * dx_body + math.cos(heading) * dy_body
        self._yaw = math.atan2(math.sin(self._yaw + dyaw), math.cos(self._yaw + dyaw))
        velocities = wheels_to_body(*(value / dt for value in increments))
        self._last_wheels, self._last_wheel_stamp_ns = wheels, stamp_ns
        self._publish_odometry(message.header.stamp, velocities)

    def _publish_odometry(self, stamp, velocity: tuple[float, float, float]) -> None:
        half = 0.5 * self._yaw
        qz, qw = math.sin(half), math.cos(half)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.04
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.03
        odom.twist.covariance[35] = 0.05
        odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e6
        self._odom_publisher.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimOmniController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # An orderly shutdown explicitly removes wheel demand; the independent
        # command watchdog remains the protection for crashes and bridge loss.
        if rclpy.ok():
            for publisher in node._wheel_publishers:
                publisher.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
