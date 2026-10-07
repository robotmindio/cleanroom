"""Load and execute the tracked exploration tree in the real Jazzy BT navigator."""

from pathlib import Path
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration_node import RobotExplorer
from test_exploration_ros import RobotPeers, response, wait


ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("cancel", [False, True])
def test_real_nav2_tree_uses_known_space_planner_and_cancels_controller(tmp_path, cancel):
    context = Context()
    rclpy.init(context=context)
    peers = RobotPeers(context, native_navigation=True)
    peers.nav_mode = "hold" if cancel else "success"

    def follow(goal):
        peers.nav_active = True
        while peers.nav_mode == "hold" and not goal.is_cancel_requested:
            time.sleep(0.03)
        if goal.is_cancel_requested:
            peers.nav_canceled += 1
            goal.canceled()
        else:
            p = goal.request.path.poses[-1].pose.position
            peers.position = (p.x, p.y)
            peers.grid.data = [0] * 6400
            goal.succeed()
        peers.nav_active = False
        return FollowPath.Result()

    controller = ActionServer(peers, FollowPath, "/follow_path", follow,
                              cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                              callback_group=peers.group)
    explorer = RobotExplorer(context=context, parameter_overrides=[
        Parameter("database_path", value=str(tmp_path / "map.db")),
        Parameter("max_duration_sec", value=10.0),
        Parameter("settle_sec", value=0.2),
    ])
    client_node = Node("native_exploration_test", context=context)
    client = ActionClient(client_node, Explore, "/robot/explore")
    lifecycle = client_node.create_client(ChangeState, "/bt_navigator/change_state")
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (peers, explorer, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    executable = Path(get_package_prefix("nav2_bt_navigator")) / "lib/nav2_bt_navigator/bt_navigator"
    log_path = tmp_path / "bt-navigator.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            str(executable), "--ros-args", "--params-file", str(ROOT / "config/nav2_params.yaml"),
            "-p", "navigators:=[navigate_to_pose]",
            # The tracked 20 ms reply budget suits Nav2's C++ servers; these
            # Python fakes share a busy executor on loaded CI runners.
            "-p", "default_server_timeout:=500",
            "-p", f"default_nav_to_pose_bt_xml:={ROOT / 'config/explore_nav_to_pose.xml'}",
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            wait(lifecycle.service_is_ready)
            wait(lambda: explorer._mapping is False and explorer._map is not None
                 and explorer._tf.can_transform("map", "base_footprint", rclpy.time.Time()))
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                reply = response(lifecycle.call_async(ChangeState.Request(
                    transition=Transition(id=transition))))
                assert reply.success, log_path.read_text()
            wait(lambda: client.server_is_ready() and explorer._navigation.server_is_ready()
                 and explorer._planner.server_is_ready())
            handle = response(client.send_goal_async(Explore.Goal(max_radius_m=1.8)))
            assert handle.accepted, log_path.read_text()
            if cancel:
                wait(lambda: peers.nav_active)
                response(handle.cancel_goal_async())
            result = response(handle.get_result_async(), timeout=10)
            expected = GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_SUCCEEDED
            assert result.status == expected, (result.result.message, log_path.read_text())
            assert len(peers.planner_ids) >= 2 and set(peers.planner_ids) == {"ExploreKnown"}
            assert not peers.nav_active
            assert peers.nav_canceled == int(cancel)
        finally:
            peers.nav_mode = "fail"
            peers.permitted = False
            explorer._shutdown_requested.set()
            wait(lambda: not explorer._busy, timeout=6)
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            controller.destroy()
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            for node in (client_node, explorer, peers):
                node.destroy_node()
            context.shutdown()
