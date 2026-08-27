#!/usr/bin/env python3
"""Arbitrate velocity sources before Nav2's collision monitor.

The hardware driver deliberately consumes only ``/cmd_vel_safe``.  This node
is the one accepted source of unfiltered velocity: it chooses a short-lived
manual command over Nav2's smoothed command and publishes the result to the
collision monitor.  It never preserves a stale command and it rejects invalid
numeric values rather than allowing a lower layer to reinterpret them.

It is intentionally small: obstacle checking, braking, and footprint
projection remain Nav2 collision monitor's responsibility.  Keeping source
selection separate from that safety mechanism makes it possible for both
manual and autonomous commands to use exactly the same final collision path.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


@dataclass(frozen=True)
class _Command:
    message: Twist
    received_at: int


def _finite_twist(message: Twist) -> bool:
    """Return whether every Twist field is finite.

    Checking all six fields is deliberate.  Consumers should not assume that
    unused axes are zero: a malformed remote client can populate any field.
    """

    return all(math.isfinite(value) for value in (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    ))


def _zero_twist() -> Twist:
    return Twist()


class CmdVelMux(Node):
    """Give a fresh manual command priority over a fresh Nav2 command."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")
        self.declare_parameter("manual_topic", "/cmd_vel_manual")
        self.declare_parameter("navigation_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel_muxed")
        self.declare_parameter("manual_timeout", 0.25)
        self.declare_parameter("navigation_timeout", 0.50)
        self.declare_parameter("publish_frequency", 20.0)
        self.declare_parameter("permission_topic", "/safety/base_motion_permitted")
        # Bool has no source timestamp.  A transient-local sample is only a
        # restart convenience, never a permission lease: stop accepting motion
        # when the supervisor stops refreshing its decision.
        self.declare_parameter("permission_timeout", 0.5)

        self._manual_timeout_ns = self._positive_seconds("manual_timeout")
        self._navigation_timeout_ns = self._positive_seconds("navigation_timeout")
        self._permission_timeout_ns = self._positive_seconds("permission_timeout")
        publish_frequency = float(self.get_parameter("publish_frequency").value)
        if not math.isfinite(publish_frequency) or publish_frequency <= 0.0:
            raise ValueError("publish_frequency must be finite and positive")

        self._manual: Optional[_Command] = None
        self._navigation: Optional[_Command] = None
        self._last_source = "none"
        # Permission is deliberately default-deny. A transient-local supervisor
        # sample restores the latest decision when this node restarts.
        self._motion_permitted = False
        self._permission_received_at_ns: Optional[int] = None
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("manual_topic").value),
            self._manual_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("navigation_topic").value),
            self._navigation_callback,
            10,
        )
        permission_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool,
            str(self.get_parameter("permission_topic").value),
            self._permission_callback,
            permission_qos,
        )
        self.create_timer(1.0 / publish_frequency, self._publish_selected)

    def _positive_seconds(self, name: str) -> int:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return int(value * 1_000_000_000)

    def _manual_callback(self, message: Twist) -> None:
        if self._accept(message, "manual"):
            self._manual = _Command(message, time.monotonic_ns())

    def _navigation_callback(self, message: Twist) -> None:
        if self._accept(message, "navigation"):
            self._navigation = _Command(message, time.monotonic_ns())

    def _permission_callback(self, message: Bool) -> None:
        self._motion_permitted = bool(message.data)
        self._permission_received_at_ns = time.monotonic_ns()

    @staticmethod
    def _permission_is_fresh(
        received_at_ns: Optional[int], timeout_ns: int, now_ns: Optional[int] = None
    ) -> bool:
        """Require a recent decision even when DDS replays a latched sample."""
        if received_at_ns is None:
            return False
        current = time.monotonic_ns() if now_ns is None else now_ns
        age = current - received_at_ns
        return 0 <= age <= timeout_ns

    def _accept(self, message: Twist, source: str) -> bool:
        if _finite_twist(message):
            return True
        self.get_logger().warning(
            f"discarded non-finite {source} Twist", throttle_duration_sec=1.0
        )
        return False

    @staticmethod
    def _fresh(command: Optional[_Command], now: int, timeout_ns: int) -> bool:
        if command is None:
            return False
        age = now - command.received_at
        return 0 <= age <= timeout_ns

    def selected_command(
        self, now: Optional[int] = None, permission_now: Optional[int] = None
    ) -> tuple[Twist, str]:
        """Return a current command; stale inputs always become an explicit stop."""

        current_time = time.monotonic_ns() if now is None else now
        permission_fresh = self._permission_is_fresh(
            self._permission_received_at_ns, self._permission_timeout_ns, permission_now
        )
        if not self._motion_permitted or not permission_fresh:
            # Drop the cached true state as well as the selected command.  A
            # later supervisor callback may deliberately refresh it.
            self._motion_permitted = False
            return _zero_twist(), "interlock"
        if self._fresh(self._manual, current_time, self._manual_timeout_ns):
            assert self._manual is not None
            return self._manual.message, "manual"
        if self._fresh(self._navigation, current_time, self._navigation_timeout_ns):
            assert self._navigation is not None
            return self._navigation.message, "navigation"
        return _zero_twist(), "none"

    def _publish_selected(self) -> None:
        message, source = self.selected_command()
        if source != self._last_source:
            self.get_logger().info(f"velocity source switched to {source}")
            self._last_source = source
        try:
            self._publisher.publish(message)
        except RuntimeError:
            # ROS invalidates publishers before a spinning executor observes
            # SIGINT. A timer already in flight must not turn an otherwise
            # clean stack shutdown into a process failure.
            if rclpy.ok():
                raise


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[CmdVelMux] = None
    try:
        node = CmdVelMux()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
