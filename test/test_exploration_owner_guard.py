"""Independent ownership enforcement on an isolated graph, without motor commands."""

from concurrent.futures import Future
from pathlib import Path
import subprocess
import threading
import json
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from lekiwi_rmf.explore_client import ExploreClient
from test_exploration_ros import RobotPeers, response, wait


@pytest.mark.parametrize('restart_guard', [False, True])
@pytest.mark.parametrize('graceful', [False, True])
def test_restarted_explorer_resumes_original_bounds_and_preserves_unrelated_navigation(tmp_path, restart_guard, graceful):
    context = Context()
    rclpy.init(context=context)
    peers = RobotPeers(context)
    client_node = Node('orphan_test', context=context)
    ready = []
    client_node.create_subscription(Bool, '/robot/explore/navigation_guard_ready',
                                    lambda msg: ready.append(msg.data), 1)
    operator = ExploreClient(context=context)
    interrupted = threading.Event()
    codes = []
    operator_thread = threading.Thread(target=lambda: codes.append(operator.run(interrupted)))
    navigate = ActionClient(client_node, NavigateToPose, '/navigate_to_pose')
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (peers, client_node):
        executor.add_node(node)
    finished = Future()
    thread = threading.Thread(target=executor.spin_until_future_complete, args=(finished,), daemon=True)
    thread.start()
    log_path = tmp_path / 'explorer.log'
    with log_path.open('w') as log:
        guard = subprocess.Popen(['/usr/bin/python3', '-m', 'lekiwi_rmf.exploration_owner_guard'],
                                 stdout=log, stderr=subprocess.STDOUT, cwd=Path(__file__).parents[1])
        process = subprocess.Popen(['/usr/bin/python3', '-m', 'lekiwi_rmf.exploration_node',
                                    '--ros-args', '-p', f'database_path:={tmp_path / "map.db"}'],
                                   stdout=log, stderr=subprocess.STDOUT, cwd=Path(__file__).parents[1])
        try:
            wait(lambda: ready and ready[-1], timeout=10)
            operator_thread.start()
            wait(lambda: peers.nav_active, timeout=10)
            checkpoint = tmp_path / 'map.db.exploration.json'
            original = json.loads(checkpoint.read_text())
            if graceful:
                process.terminate()
            else:
                process.kill()
            process.wait(timeout=5)
            if restart_guard:
                guard.kill()
                guard.wait(timeout=5)
                guard = subprocess.Popen(['/usr/bin/python3', '-m', 'lekiwi_rmf.exploration_owner_guard'],
                                         stdout=log, stderr=subprocess.STDOUT, cwd=Path(__file__).parents[1])
            try:
                wait(lambda: peers.nav_canceled == 1 and not peers.nav_active, timeout=8)
            except AssertionError:
                raise AssertionError(log_path.read_text())
            wait(lambda: ready[-1], timeout=5)
            peers.position = (0.2, 0.1)
            process = subprocess.Popen(['/usr/bin/python3', '-m', 'lekiwi_rmf.exploration_node',
                                        '--ros-args', '-p', f'database_path:={tmp_path / "map.db"}'],
                                       stdout=log, stderr=subprocess.STDOUT, cwd=Path(__file__).parents[1])
            try:
                wait(lambda: peers.nav_active and peers.nav_count == 2, timeout=10)
            except AssertionError:
                raise AssertionError(log_path.read_text())
            resumed = json.loads(checkpoint.read_text())
            for key in ('id', 'center', 'radius', 'deadline', 'previous_mapping'):
                assert resumed[key] == original[key]
            interrupted.set()
            operator_thread.join(timeout=10)
            assert not operator_thread.is_alive(), log_path.read_text()
            assert codes == [130], log_path.read_text()
            assert peers.nav_canceled == 2 and not checkpoint.exists()
            # A normal client UUID has no Explorer ownership namespace.
            other = response(navigate.send_goal_async(NavigateToPose.Goal(
                pose=PoseStamped(), behavior_tree='explore_nav_to_pose.xml')))
            assert other.accepted
            wait(lambda: peers.nav_active)
            time.sleep(1.5)
            assert peers.nav_active and peers.nav_canceled == 2
            response(other.cancel_goal_async())
            assert response(other.get_result_async()).status == GoalStatus.STATUS_CANCELED
        finally:
            interrupted.set()
            if operator_thread.is_alive():
                operator_thread.join(timeout=15)
            assert not operator_thread.is_alive()
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
            guard.kill()
            guard.wait(timeout=5)
            peers.nav_mode = 'fail'
            wait(lambda: not peers.nav_active)
            finished.set_result(True)
            thread.join(timeout=5)
            assert not thread.is_alive()
            assert executor.shutdown(timeout_sec=5)
            operator.destroy_node()
            navigate.destroy()
            for node in (client_node, peers):
                node.destroy_node()
            context.shutdown()
