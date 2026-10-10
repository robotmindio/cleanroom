#!/usr/bin/env python3
"""Read-only deployment qualification of navigation and current robot inputs."""

import argparse
import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class StackReadiness(Node):
    def __init__(self, **kwargs):
        super().__init__('stack_readiness', **kwargs)
        self.inputs = {}
        self.states = {}
        self.requests = {}
        self._lifecycle_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                        for name in ('controller_server', 'planner_server', 'bt_navigator')}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        for topic in ('base_motion_permitted', 'arm_stowed'):
            self.create_subscription(Bool, f'/safety/{topic}',
                                     lambda msg, name=topic: self._record(name, msg.data), latched)
        self.create_subscription(Bool, '/robot/explore/navigation_guard_ready',
                                 lambda msg: self._record('guard', msg.data), 1)
        self.create_subscription(Info, '/info', self._slam, 1)
        self.create_subscription(String, '/safety/driver_state',
                                 lambda msg: self._record('driver_armed', msg.data == 'ARMED'), latched)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostics, 10)

    def _record(self, name, value):
        self.inputs[name] = (value, time.monotonic())

    def _map(self, message):
        n = message.info.width * message.info.height
        self._record('map', math.isfinite(message.info.resolution) and message.info.resolution > 0
                     and n > 0 and len(message.data) == n and any(v >= 0 for v in message.data))

    def _slam(self, message):
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(message.header.stamp)).nanoseconds / 1e9
        self._record('slam', -0.3 <= age <= 4.0)

    def _diagnostics(self, message):
        for status in message.status:
            if status.name == 'lekiwi/exploration':
                values = {item.key: item.value for item in status.values}
                self._record('explorer', values.get('prerequisite_reason') == '')

    def reason(self, *, armed, exploration):
        now = time.monotonic()
        for name, client in self._lifecycle_clients.items():
            future, sent = self.requests.get(name, (None, 0.0))
            if future is not None:
                if future.done():
                    try:
                        self.states[name] = (future.result().current_state.id, now)
                    except Exception as error:
                        self.states.pop(name, None)
                        self.get_logger().warning(f'{name} readiness failed: {error}')
                    self.requests.pop(name, None)
                elif now - sent >= 3:
                    future.cancel()
                    self.requests.pop(name, None)
            if name not in self.requests and client.service_is_ready():
                self.requests[name] = (client.call_async(GetState.Request()), now)
        for name in self._lifecycle_clients:
            state, received = self.states.get(name, (None, 0.0))
            if state != State.PRIMARY_STATE_ACTIVE or now - received > 3:
                return f'{name} is not confirmed active'
        if not self.inputs.get('map', (False,))[0]:
            return 'no valid occupancy map'
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            t, q = transform.transform.translation, transform.transform.rotation
            if not -0.3 <= age <= 1 or not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
                return 'map-to-robot transform is stale or non-finite'
        except TransformException:
            return 'map-to-robot transform unavailable'
        required = [('slam', 4.0), ('guard', 1.0), ('explorer', 1.0)] if exploration else []
        if armed:
            required += [('driver_armed', 1.0), ('base_motion_permitted', 1.0), ('arm_stowed', 1.0)]
        for name, timeout in required:
            value, received = self.inputs.get(name, (False, 0.0))
            if not value or now - received > timeout:
                return f'{name} missing, stale or denied'
        return ''


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--armed', action='store_true')
    parser.add_argument('--exploration', action='store_true')
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rclpy.init(args=[])
    node = StackReadiness()
    ready = False
    reason = 'waiting for ROS discovery'
    until = time.monotonic() + options.timeout
    try:
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
            reason = node.reason(armed=options.armed, exploration=options.exploration)
            if not reason:
                ready = True
                break
        print('Robot operationally ready' if ready else f'Robot not ready: {reason}', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ready else 1)


if __name__ == '__main__':
    main()
