#!/usr/bin/env python3
"""Cancel Explorer-owned Nav2 goals whose independent wall-time lease expires."""

import time
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from unique_identifier_msgs.msg import UUID

from lekiwi_rmf.navigation_observer import NavigationObserver, server_instance, status_instance

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
        self._instances = {}
        self._results = {}
        self._forgotten = {}
        self._children = NavigationObserver(self)
        self._started = time.monotonic()
        self._cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._result = self.create_client(NavigateToPose.Impl.GetResultService, '/navigate_to_pose/_action/get_result')
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
                and self._cancel.service_is_ready() and not self._expired()
                and (bool(self._active) or not self._children.busy()))

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
                self._instances.pop(key, None)
                self._forgotten.pop(key, None)
            else:
                self._active.add(key)
                self._instances.setdefault(key, status_instance(self, '/navigate_to_pose'))

    def _tick(self):
        self._ready.publish(Bool(data=self._available()))
        for key in self._expired():
            forgotten, since = self._forgotten.get(key, ('', time.monotonic()))
            if (forgotten and time.monotonic() - since >= 3.0
                    and server_instance(self, '/navigate_to_pose') == forgotten and not self._children.busy()):
                self._active.discard(key)
                self._seen.pop(key, None)
                self._cancels.pop(key, None)
                self._instances.pop(key, None)
                self._forgotten.pop(key, None)
                continue
            future, sent = self._cancels.get(key, (None, 0.0))
            if future is not None and not future.done():
                if time.monotonic() - sent < LEASE_SEC:
                    continue
                self._cancel.remove_pending_request(future)
                future.cancel()
            if not self._cancel.service_is_ready():
                continue
            if time.monotonic() - sent < LEASE_SEC:
                continue
            request = CancelGoal.Request()
            request.goal_info.goal_id = UUID(uuid=list(key))
            instance = server_instance(self, '/navigate_to_pose')
            future = self._cancel.call_async(request)
            future.add_done_callback(lambda done, goal=key, server=instance: self._cancel_reply(done, goal, server))
            self._cancels[key] = (future, time.monotonic())
            self.get_logger().warning(f'canceling orphaned exploration navigation {key.hex()}', throttle_duration_sec=5)
        # Unsent/terminal goals need no retained heartbeat; late acceptance is
        # still recognized by its UUID namespace when Nav2 reports its status.
        now = time.monotonic()
        self._seen = {key: seen for key, seen in self._seen.items()
                      if key in self._active or now - seen <= LEASE_SEC}

    def _confirm_forgotten(self, key, instance):
        previous, sent = self._results.get(key, (None, 0.0))
        if previous is not None and not previous.done():
            if time.monotonic() - sent < LEASE_SEC:
                return
            self._result.remove_pending_request(previous)
            previous.cancel()
        if not self._result.service_is_ready():
            return
        request = NavigateToPose.Impl.GetResultService.Request(goal_id=UUID(uuid=list(key)))
        future = self._result.call_async(request)
        self._results[key] = (future, time.monotonic())
        future.add_done_callback(lambda done: self._forgotten_result(done, key, instance))

    def _forgotten_result(self, future, key, instance):
        if future.cancelled():
            return
        try:
            if (future.result().status == GoalStatus.STATUS_UNKNOWN
                    and key in self._expired() and self._instances.get(key) != instance
                    and server_instance(self, '/navigate_to_pose') == instance):
                self._forgotten.setdefault(key, (instance, time.monotonic()))
        except Exception as error:
            self.get_logger().error(f'orphan result is unconfirmed: {error}')

    def _cancel_reply(self, future, key, instance):
        if future.cancelled():
            return
        try:
            reply = future.result()
            if (reply.return_code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID and instance
                    and self._instances.get(key) and self._instances[key] != instance
                    and server_instance(self, '/navigate_to_pose') == instance):
                self._confirm_forgotten(key, instance)
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
