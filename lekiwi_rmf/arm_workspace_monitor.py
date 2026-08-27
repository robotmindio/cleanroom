#!/usr/bin/env python3
"""Fail-closed execution-time collision gate backed by MoveIt's live scene."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetStateValidity
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from lekiwi_rmf.arm_trajectory import ARM_JOINTS


@dataclass
class ArmWorkspaceState:
    """Receive-time leases for the inputs and latest MoveIt verdict."""

    joint_received_ns: int | None = None
    scene_received_ns: int | None = None
    checked_ns: int | None = None
    collision_free: bool = False
    detail: str = "not checked"

    def decision(
        self, now_ns: int, joint_timeout_ns: int, scene_timeout_ns: int,
        validity_timeout_ns: int,
    ) -> tuple[bool, str]:
        for label, stamp, timeout in (
            ("joint state", self.joint_received_ns, joint_timeout_ns),
            ("planning scene", self.scene_received_ns, scene_timeout_ns),
            ("state-validity result", self.checked_ns, validity_timeout_ns),
        ):
            if stamp is None:
                return False, f"{label} missing"
            age = now_ns - stamp
            if age < 0 or age > timeout:
                return False, f"{label} stale"
        if not self.collision_free:
            return False, self.detail or "MoveIt reports collision"
        return True, "fresh MoveIt state-validity check is collision-free"


def complete_joint_snapshot(message: JointState, expected_names) -> JointState | None:
    """Return a canonical complete finite snapshot; never merge partial samples."""
    if (
        len(message.name) != len(set(message.name))
        or len(message.position) != len(message.name)
    ):
        return None
    positions = dict(zip(message.name, message.position))
    if any(
        name not in positions or not math.isfinite(float(positions[name]))
        for name in expected_names
    ):
        return None
    stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
    if stamp_ns <= 0:
        return None
    snapshot = JointState()
    snapshot.header = message.header
    snapshot.name = list(expected_names)
    snapshot.position = [float(positions[name]) for name in expected_names]
    return snapshot


def _positive_seconds(node: Node, name: str) -> tuple[float, int]:
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value, int(value * 1_000_000_000)


class ArmWorkspaceMonitor(Node):
    def __init__(self) -> None:
        super().__init__("arm_workspace_monitor")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("joint_names", list(ARM_JOINTS))
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("planning_scene_topic", "/monitored_planning_scene")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("output_topic", "/safety/arm_workspace_clear")
        self.declare_parameter("check_frequency", 30.0)
        self.declare_parameter("joint_timeout", 0.25)
        self.declare_parameter("planning_scene_timeout", 0.50)
        self.declare_parameter("validity_timeout", 0.15)
        self.declare_parameter("service_timeout", 0.10)

        self._group_name = str(self.get_parameter("group_name").value)
        self._joint_names = tuple(self.get_parameter("joint_names").value)
        if (
            not self._group_name.strip()
            or not self._joint_names
            or len(set(self._joint_names)) != len(self._joint_names)
            or any(not isinstance(name, str) or not name for name in self._joint_names)
        ):
            raise ValueError("MoveIt group and joint_names must be non-empty and unique")
        frequency, _unused = _positive_seconds(self, "check_frequency")
        self._joint_timeout, self._joint_timeout_ns = _positive_seconds(self, "joint_timeout")
        self._scene_timeout, self._scene_timeout_ns = _positive_seconds(
            self, "planning_scene_timeout"
        )
        self._validity_timeout, self._validity_timeout_ns = _positive_seconds(
            self, "validity_timeout"
        )
        self._service_timeout, self._service_timeout_ns = _positive_seconds(
            self, "service_timeout"
        )
        if 1.0 / frequency >= self._validity_timeout:
            raise ValueError("check_frequency must refresh before validity_timeout")

        self._state = ArmWorkspaceState()
        self._joint_snapshot: JointState | None = None
        self._scene_generation = 0
        self._pending = None
        self._pending_sent_ns: int | None = None
        self._pending_scene_generation: int | None = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scene_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._permission_pub = self.create_publisher(
            Bool, str(self.get_parameter("output_topic").value), latched
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state, 10,
        )
        self.create_subscription(
            PlanningScene, str(self.get_parameter("planning_scene_topic").value),
            self._on_planning_scene, scene_qos,
        )
        self._client = self.create_client(
            GetStateValidity, str(self.get_parameter("state_validity_service").value)
        )
        self.create_timer(1.0 / frequency, self._tick)
        self._publish(False, "monitor starting")

    @staticmethod
    def _monotonic_ns() -> int:
        return time.monotonic_ns()

    def _on_joint_state(self, message: JointState) -> None:
        snapshot = complete_joint_snapshot(message, self._joint_names)
        source_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        source_age_ns = self.get_clock().now().nanoseconds - source_ns
        if (
            snapshot is None
            or source_age_ns < 0
            or source_age_ns > self._joint_timeout_ns
        ):
            self._joint_snapshot = None
            self._state.joint_received_ns = None
            self._state.collision_free = False
            self._state.detail = (
                "joint state is incomplete, duplicate, non-finite, stampless, or stale"
            )
            return
        self._joint_snapshot = snapshot
        self._state.joint_received_ns = self._monotonic_ns()

    def _on_planning_scene(self, message: PlanningScene) -> None:
        # Robot-state-only scene diffs are not evidence that the depth updater
        # is alive. Advance the lease only for a recently stamped octomap, so a
        # live move_group with a dead perception plugin remains fail-closed.
        stamp = message.world.octomap.octomap.header.stamp
        source_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        source_age_ns = self.get_clock().now().nanoseconds - source_ns
        if (
            source_ns <= 0
            or source_age_ns < 0
            or source_age_ns > self._scene_timeout_ns
        ):
            return
        self._scene_generation += 1
        self._state.scene_received_ns = self._monotonic_ns()

    def _preconditions(self, now_ns: int) -> tuple[bool, str]:
        if self._joint_snapshot is None or self._state.joint_received_ns is None:
            return False, "complete joint state missing"
        joint_age = now_ns - self._state.joint_received_ns
        if joint_age < 0 or joint_age > self._joint_timeout_ns:
            return False, "joint state stale"
        if self._state.scene_received_ns is None:
            return False, "monitored planning scene missing"
        scene_age = now_ns - self._state.scene_received_ns
        if scene_age < 0 or scene_age > self._scene_timeout_ns:
            return False, "monitored planning scene stale"
        if not self._client.service_is_ready():
            return False, "MoveIt state-validity service unavailable"
        return True, ""

    def _expire_pending(self, now_ns: int) -> None:
        if (
            self._pending is None
            or self._pending_sent_ns is None
            or now_ns - self._pending_sent_ns <= self._service_timeout_ns
        ):
            return
        future = self._pending
        self._pending = None
        self._pending_sent_ns = None
        self._pending_scene_generation = None
        try:
            self._client.remove_pending_request(future)
        except (AttributeError, KeyError):
            pass
        self._state.collision_free = False
        self._state.detail = "MoveIt state-validity request timed out"

    def _request_check(self, now_ns: int) -> None:
        request = GetStateValidity.Request()
        request.group_name = self._group_name
        request.robot_state.joint_state = self._joint_snapshot
        request.robot_state.is_diff = False
        future = self._client.call_async(request)
        self._pending = future
        self._pending_sent_ns = now_ns
        self._pending_scene_generation = self._scene_generation
        future.add_done_callback(self._on_check_complete)

    def _on_check_complete(self, future) -> None:
        if future is not self._pending:
            return
        sent_scene_generation = self._pending_scene_generation
        self._pending = None
        self._pending_sent_ns = None
        self._pending_scene_generation = None
        now_ns = self._monotonic_ns()
        try:
            response = future.result()
        except Exception as error:
            self._state.collision_free = False
            self._state.detail = f"MoveIt state-validity request failed: {error}"
            return
        if response is None:
            self._state.collision_free = False
            self._state.detail = "MoveIt state-validity response is missing"
            return
        if response.valid is not True:
            contacts = sorted({
                f"{contact.contact_body_1}/{contact.contact_body_2}"
                for contact in response.contacts
            })
            self._state.checked_ns = now_ns
            self._state.collision_free = False
            self._state.detail = (
                "MoveIt reports collision" + (f": {', '.join(contacts)}" if contacts else "")
            )
            self._publish(False, self._state.detail)
            return
        if sent_scene_generation != self._scene_generation:
            # A newer scene arrived while the service evaluated the old one.
            # Keep only a still-fresh prior verdict and immediately recheck.
            self._state.detail = "planning scene changed during state-validity check"
            return
        self._state.checked_ns = now_ns
        self._state.collision_free = True
        self._state.detail = "collision-free"

    def _tick(self) -> None:
        now_ns = self._monotonic_ns()
        self._expire_pending(now_ns)
        preconditions_ok, detail = self._preconditions(now_ns)
        if not preconditions_ok:
            self._state.collision_free = False
            self._state.detail = detail
        elif self._pending is None:
            self._request_check(now_ns)
        clear, decision_detail = self._state.decision(
            now_ns, self._joint_timeout_ns, self._scene_timeout_ns,
            self._validity_timeout_ns,
        )
        self._publish(clear, decision_detail)

    def _publish(self, clear: bool, detail: str) -> None:
        permission = Bool()
        permission.data = bool(clear)
        self._permission_pub.publish(permission)

        status = DiagnosticStatus()
        status.name = "lekiwi/arm_workspace_monitor"
        status.hardware_id = "lekiwi"
        status.level = DiagnosticStatus.OK if clear else DiagnosticStatus.ERROR
        status.message = "CLEAR" if clear else "BLOCKED"
        status.values = [
            KeyValue(key="arm_workspace_clear", value=str(bool(clear)).lower()),
            KeyValue(key="detail", value=detail),
            KeyValue(key="planning_scene_generation", value=str(self._scene_generation)),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ArmWorkspaceMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
