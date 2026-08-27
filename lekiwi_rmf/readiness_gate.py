#!/usr/bin/env python3
"""Exit successfully only after a required ROS dependency is genuinely ready.

Launch files use this short-lived node with ``OnProcessExit`` instead of relying
on machine-specific sleep timers.  It has no timeout by design: a dependency
that never becomes healthy must keep its consumers from starting rather than
starting a degraded control stack after an arbitrary delay.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


TOPIC_TYPES = {
    "image": Image,
    "odom": Odometry,
    "map": OccupancyGrid,
}


def topic_qos(topic_type: str) -> QoSProfile:
    """Use the producer's delivery contract for a readiness subscription.

    RTAB-Map publishes ``/map`` as a transient-local OccupancyGrid.  A volatile
    gate that starts after its first grid is published does not receive that
    cached sample and can block Nav2 indefinitely even though a valid map is
    available.  Images and odometry are deliberately live-only: a historical
    sample would not prove their producers are currently healthy.
    """
    if topic_type == "map":
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class ReadinessGate(Node):
    """Wait for one message or a Nav2 action server, then terminate."""

    def __init__(self) -> None:
        super().__init__("readiness_gate")
        self.declare_parameter("kind", "topic")
        self.declare_parameter("topic", "")
        self.declare_parameter("topic_type", "")
        self.declare_parameter("action", "")
        self.declare_parameter("lifecycle_node", "/bt_navigator")
        self._ready = False
        kind = str(self.get_parameter("kind").value)

        if kind == "topic":
            topic = str(self.get_parameter("topic").value)
            topic_type = str(self.get_parameter("topic_type").value)
            if not topic or topic_type not in TOPIC_TYPES:
                raise ValueError("topic readiness requires topic and a supported topic_type")
            self.create_subscription(TOPIC_TYPES[topic_type], topic, self._on_message, topic_qos(topic_type))
            self.get_logger().info(f"waiting for {topic_type} message on {topic}")
        elif kind in ("navigate_to_pose_action", "follow_joint_trajectory_action"):
            action = str(self.get_parameter("action").value)
            if not action:
                raise ValueError("action readiness requires action")
            action_type = (
                NavigateToPose if kind == "navigate_to_pose_action"
                else FollowJointTrajectory
            )
            self._action_client = ActionClient(self, action_type, action)
            self._lifecycle_client = None
            self._lifecycle_future = None
            if kind == "navigate_to_pose_action":
                lifecycle_node = str(self.get_parameter("lifecycle_node").value).rstrip("/")
                if not lifecycle_node:
                    raise ValueError("Nav2 readiness requires a lifecycle_node")
                self._lifecycle_client = self.create_client(
                    GetState, f"{lifecycle_node}/get_state"
                )
            self._timer = self.create_timer(0.2, self._check_action)
            self.get_logger().info(f"waiting for active action server {action}")
        else:
            raise ValueError(f"unsupported readiness kind: {kind}")

    def _on_message(self, message: Image | Odometry | OccupancyGrid) -> None:
        if isinstance(message, Image):
            self._ready = (
                message.width > 0 and message.height > 0 and message.step > 0
                and bool(message.encoding) and bool(message.data)
            )
        elif isinstance(message, OccupancyGrid):
            expected = message.info.width * message.info.height
            self._ready = (
                message.info.resolution > 0.0 and expected > 0
                and len(message.data) == expected
                and any(value >= 0 for value in message.data)
            )
        else:
            values = (
                message.pose.pose.position.x, message.pose.pose.position.y,
                message.pose.pose.orientation.z, message.pose.pose.orientation.w,
                message.twist.twist.linear.x, message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            )
            self._ready = bool(message.child_frame_id) and all(
                math.isfinite(value) for value in values
            )

    def _check_action(self) -> None:
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            return
        if self._lifecycle_client is None:
            self._ready = True
            return
        if self._lifecycle_future is None:
            if self._lifecycle_client.wait_for_service(timeout_sec=0.0):
                self._lifecycle_future = self._lifecycle_client.call_async(
                    GetState.Request()
                )
            return
        if not self._lifecycle_future.done():
            return
        try:
            response = self._lifecycle_future.result()
            self._ready = (
                response is not None
                and response.current_state.id == State.PRIMARY_STATE_ACTIVE
            )
        except Exception:
            self._ready = False
        finally:
            if not self._ready:
                self._lifecycle_future = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ReadinessGate] = None
    interrupted = False
    try:
        node = ReadinessGate()
        while rclpy.ok() and not node._ready:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A gate returning zero means its dependency is ready. SIGINT is not
        # readiness: make the launch OnProcessExit success handler leave all
        # downstream actions stopped even if shutdown events are reordered.
        interrupted = True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if interrupted:
        raise SystemExit(130)


if __name__ == "__main__":
    main()
