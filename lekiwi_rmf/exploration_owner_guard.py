#!/usr/bin/env python3
"""Cancel Explorer-owned Nav2 goals whose independent wall-time lease expires."""

import time
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from unique_identifier_msgs.msg import UUID

# An identifiable namespace lets a restarted guard recover ownership from Nav2.
PREFIX = b'lekiwiex'
LEASE_SEC = 1.0
TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


def owned_goal_id():
    return UUID(uuid=list(PREFIX + uuid.uuid4().bytes[8:]))


class ExplorationOwnerGuard(Node):
    def __init__(self, **kwargs):
        super().__init__('exploration_owner_guard', **kwargs)
        self._seen = {}
        self._active = set()
        self._cancels = {}
        self._started = time.monotonic()
        self._cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._ack = self.create_publisher(UUID, '/robot/explore/navigation_guarded', 10)
        self._ready = self.create_publisher(Bool, '/robot/explore/navigation_guard_ready', 1)
        self.create_subscription(UUID, '/robot/explore/navigation_owner', self._heartbeat, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status, latched)
        self.create_timer(0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _expired(self):
        now = time.monotonic()
        return {key for key in self._active if now - self._seen.get(key, 0.0) > LEASE_SEC}

    def _available(self):
        # Allow latched Nav2 status discovery before acknowledging a new owner.
        return (time.monotonic() - self._started >= LEASE_SEC
                and self._cancel.service_is_ready() and not self._expired())

    def _heartbeat(self, message):
        key = bytes(message.uuid)
        if not key.startswith(PREFIX):
            return
        # An expired active lease cannot be revived before its terminal status.
        if key in self._expired():
            return
        self._seen[key] = time.monotonic()
        if self._available():
            self._ack.publish(message)

    def _status(self, message):
        for item in message.status_list:
            key = bytes(item.goal_info.goal_id.uuid)
            if not key.startswith(PREFIX):
                continue
            if item.status in TERMINAL:
                self._active.discard(key)
                self._seen.pop(key, None)
                self._cancels.pop(key, None)
            else:
                self._active.add(key)

    def _tick(self):
        self._ready.publish(Bool(data=self._available()))
        for key in self._expired():
            future, sent = self._cancels.get(key, (None, 0.0))
            if future is not None and not future.done():
                if time.monotonic() - sent < LEASE_SEC:
                    continue
                future.cancel()
            if not self._cancel.service_is_ready():
                continue
            if time.monotonic() - sent < LEASE_SEC:
                continue
            request = CancelGoal.Request()
            request.goal_info.goal_id = UUID(uuid=list(key))
            future = self._cancel.call_async(request)
            future.add_done_callback(self._cancel_reply)
            self._cancels[key] = (future, time.monotonic())
            self.get_logger().warning(f'canceling orphaned exploration navigation {key.hex()}', throttle_duration_sec=5)
        # Unsent/terminal goals need no retained heartbeat; late acceptance is
        # still recognized by its UUID namespace when Nav2 reports its status.
        now = time.monotonic()
        self._seen = {key: seen for key, seen in self._seen.items()
                      if key in self._active or now - seen <= LEASE_SEC}

    def _cancel_reply(self, future):
        if future.cancelled():
            return
        try:
            reply = future.result()
            if reply.return_code:
                self.get_logger().warning(f'orphan cancellation returned code {reply.return_code}', throttle_duration_sec=5)
        except Exception as error:
            self.get_logger().error(f'orphan cancellation failed: {error}')
        # A cancel reply alone is not proof of a terminal goal. Keep watching status.


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationOwnerGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
