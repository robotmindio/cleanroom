"""Check the installed native planner against the actual exploration footprint."""

import math
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from launch import LaunchContext
from launch_ros.parameter_descriptions import ParameterFile
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
import yaml

from lekiwi_rmf.exploration import footprint_is_free, load_navigation_footprint

ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("scene", ["unknown_corner", "close_corridor"])
@pytest.mark.parametrize("planner_id", ["ExploreKnown", "GridBased"])
def test_native_routes_fit_the_body_without_extra_wall_clearance(tmp_path, scene, planner_id):
    context = Context()
    rclpy.init(context=context)
    node = Node("exploration_planner_fixture", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    broadcaster = TransformBroadcaster(node)
    publisher = node.create_publisher(
        OccupancyGrid, "/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid = np.zeros((80, 80), dtype=np.int16)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    position = (0.0, 0.0)
    if scene == "unknown_corner":
        # NavFn rounds this actual pose to (0.05, 0.05), putting the body's
        # corner into unknown cells that the actual starting body clears.
        position = (0.026, 0.026)
        grid[45:, 45:] = -1
        goal.pose.position.x, goal.pose.position.y = -0.4, 0.4
        yaw = 3 * math.pi / 4
    else:
        grid[[34, 46], :] = 100
        goal.pose.position.x = 0.8
        yaw = 0.0
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = message.info.height = 80
    message.info.resolution = 0.05
    message.info.origin.position.x = message.info.origin.position.y = -2.0
    message.info.origin.orientation.w = 1.0
    message.data = grid.ravel().tolist()

    def publish():
        stamp = node.get_clock().now().to_msg()
        message.header.stamp = stamp
        publisher.publish(message)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_footprint"
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = position
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    node.create_timer(0.05, publish)
    parameter_file = ParameterFile(str(ROOT / "config/nav2_params.yaml"), allow_substs=True)
    params = yaml.safe_load(Path(parameter_file.evaluate(LaunchContext())).read_text())
    params["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"] = ["static_layer", "inflation_layer"]
    params_path = tmp_path / "planner.yaml"
    params_path.write_text(yaml.safe_dump(params))
    log_path = tmp_path / "planner.log"
    process = None
    client = None

    def response(future):
        until = time.monotonic() + 8
        while not future.done() and time.monotonic() < until:
            executor.spin_once(timeout_sec=0.05)
        assert future.done(), log_path.read_text()
        return future.result()

    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                str(Path(get_package_prefix("nav2_planner")) / "lib/nav2_planner/planner_server"),
                "--ros-args", "--params-file", str(params_path),
            ], stdout=log, stderr=subprocess.STDOUT)
            change = node.create_client(ChangeState, "/planner_server/change_state")
            until = time.monotonic() + 8
            while not change.service_is_ready() and time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            assert change.service_is_ready(), log_path.read_text()
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                assert response(change.call_async(ChangeState.Request(
                    transition=Transition(id=transition)))).success, log_path.read_text()
            client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
            until = time.monotonic() + 8
            while not client.server_is_ready() and time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            assert client.server_is_ready(), log_path.read_text()
            # Let the first latched map reach the newly activated costmap.
            until = time.monotonic() + 0.5
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            goal.header.stamp = node.get_clock().now().to_msg()
            handle = response(client.send_goal_async(ComputePathToPose.Goal(goal=goal, planner_id=planner_id)))
            assert handle.accepted
            result = response(handle.get_result_async())
            assert result.status == GoalStatus.STATUS_SUCCEEDED and not result.result.error_code, log_path.read_text()
            path = result.result.path.poses
            footprint, _, _ = load_navigation_footprint(ROOT / "config/nav2_params.yaml")
            assert path and all(footprint_is_free(
                grid, 0.05, (-2.0, -2.0, 0.0), (pose.pose.position.x, pose.pose.position.y),
                2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w), footprint, 20,
            ) for pose in path), [(pose.pose.position, pose.pose.orientation) for pose in path]
            assert math.hypot(path[-1].pose.position.x - goal.pose.position.x,
                              path[-1].pose.position.y - goal.pose.position.y) < 0.03
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if client is not None:
            client.destroy()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        parameter_file.cleanup()
