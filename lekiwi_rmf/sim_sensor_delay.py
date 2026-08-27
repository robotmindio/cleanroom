"""Deterministic depth-cloud delay and dropout model for simulation."""

from __future__ import annotations

import heapq
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class SimSensorDelay(Node):
    def __init__(self) -> None:
        super().__init__("sim_depth_delay")
        self.declare_parameter("delay", 0.075)
        self.declare_parameter("jitter", 0.015)
        self.declare_parameter("dropout_probability", 0.01)
        self.declare_parameter("random_seed", 42017)
        self.delay = float(self.get_parameter("delay").value)
        self.jitter = float(self.get_parameter("jitter").value)
        self.dropout = float(self.get_parameter("dropout_probability").value)
        if self.delay < 0.0 or self.jitter < 0.0 or not 0.0 <= self.dropout <= 1.0:
            raise ValueError("delay/jitter must be non-negative and dropout_probability must be in [0, 1]")
        self._random = random.Random(int(self.get_parameter("random_seed").value))
        self._sequence = 0
        self._queue: list[tuple[int, int, PointCloud2]] = []
        self._publisher = self.create_publisher(
            PointCloud2, "/camera/depth/points", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/camera/depth/points_raw",
            self._enqueue,
            qos_profile_sensor_data,
        )
        self.create_timer(0.005, self._release)

    def _enqueue(self, message: PointCloud2) -> None:
        if self._random.random() < self.dropout:
            return
        latency = max(0.0, self._random.gauss(self.delay, self.jitter))
        release_ns = self.get_clock().now().nanoseconds + int(latency * 1e9)
        heapq.heappush(self._queue, (release_ns, self._sequence, message))
        self._sequence += 1
        # A paused or overloaded simulation must degrade by dropping old data,
        # never by releasing an arbitrarily stale burst later.
        while len(self._queue) > 8:
            heapq.heappop(self._queue)

    def _release(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        while self._queue and self._queue[0][0] <= now_ns:
            _release_ns, _sequence, message = heapq.heappop(self._queue)
            self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSensorDelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
