"""Exercise the real Nav2 monitor in isolated DDS; no hardware subscribers."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


ROOT = Path(__file__).parents[1]


@pytest.fixture
def monitor(tmp_path):
    assert os.environ.get("ROS_DOMAIN_ID", "0") != "0", "run on an isolated DDS domain"
    context = Context()
    rclpy.init(context=context)
    node = Node("collision_monitor_fixture", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "odom", "base_footprint"
    transform.transform.rotation.w = 1.0
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)
    command_pub = node.create_publisher(Twist, "/cmd_vel_muxed", 10)
    scan_pub = node.create_publisher(LaserScan, "/scan", 10)
    received = []
    node.create_subscription(Twist, "/cmd_vel_safe", received.append, 10)
    state = {"points": [], "velocity": (0.0, 0.0, 0.0), "scan": True}

    def publish():
        if state["scan"]:
            scan = LaserScan()
            scan.header.frame_id = "base_footprint"
            scan.header.stamp = node.get_clock().now().to_msg()
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
            scan.angle_max = math.pi - scan.angle_increment
            scan.range_min, scan.range_max = 0.01, 10.0
            scan.ranges = [math.inf] * 720
            for x, y in state["points"]:
                index = round((math.atan2(y, x) - scan.angle_min) / scan.angle_increment) % 720
                scan.ranges[index] = math.hypot(x, y)
            scan_pub.publish(scan)
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = state["velocity"]
        command_pub.publish(command)

    node.create_timer(0.03, publish)
    lifecycle = node.create_client(ChangeState, "/collision_monitor/change_state")
    executable = Path(get_package_prefix("nav2_collision_monitor")) / "lib/nav2_collision_monitor/collision_monitor"
    log_path = tmp_path / "monitor.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            str(executable), "--ros-args", "--params-file", str(ROOT / "config/nav2_params.yaml"),
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            assert lifecycle.wait_for_service(timeout_sec=8), log_path.read_text()
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                future = lifecycle.call_async(ChangeState.Request(transition=Transition(id=transition)))
                executor.spin_until_future_complete(future, timeout_sec=8)
                assert future.done() and future.result().success, log_path.read_text()

            def request(points, velocity, *, scan=True, seconds=0.7):
                state.update(points=points, velocity=velocity, scan=scan)
                received.clear()
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.03)
                assert received, log_path.read_text()
                last = received[-1]
                return last.linear.x, last.linear.y, last.angular.z

            yield request
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)


@pytest.mark.parametrize("velocity", [(0.25, 0.0, 0.0), (-0.25, 0.0, 0.0)])
def test_straight_travel_passes_with_one_mm_beside_the_actual_body(monitor, velocity):
    assert monitor([(0.0, 0.221), (0.0, -0.221)], velocity) == pytest.approx(velocity)


@pytest.mark.parametrize("points, velocity", [
    ([(0.34, 0.0)], (0.25, 0.0, 0.0)),
    ([(-0.32, 0.0)], (-0.25, 0.0, 0.0)),
    ([(0.0, 0.32)], (0.0, 0.25, 0.0)),
    ([(0.0, -0.32)], (0.0, -0.25, 0.0)),
    ([(0.34, 0.32)], (0.15, 0.15, 0.1)),
    ([(0.0, 0.221)], (0.0, 0.0, 0.4)),
    ([(0.0, -0.221)], (0.0, 0.0, -0.4)),
])
def test_obstacles_in_the_swept_body_reduce_every_component(monitor, points, velocity):
    safe = monitor(points, velocity)
    ratios = [actual / requested for actual, requested in zip(safe, velocity) if requested]
    assert 0 <= ratios[0] < 1
    assert ratios == pytest.approx([ratios[0]] * len(ratios))


def test_overlap_stops_and_clearing_the_body_restores_motion(monitor):
    velocity = (0.25, 0.0, 0.0)
    assert monitor([(0.10, 0.0)], velocity) == pytest.approx((0.0, 0.0, 0.0))
    assert monitor([], velocity) == pytest.approx(velocity)


def test_stale_scan_still_stops(monitor):
    velocity = (0.25, 0.0, 0.0)
    assert monitor([], velocity) == pytest.approx(velocity)
    assert monitor([], velocity, scan=False, seconds=1.1) == pytest.approx((0.0, 0.0, 0.0))
