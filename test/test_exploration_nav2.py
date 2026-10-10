"""Load and execute the tracked exploration tree in the real Jazzy BT navigator."""

from pathlib import Path
import signal
import subprocess
import threading
from concurrent.futures import Future
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import BackUp, ComputePathToPose, FollowPath, Spin, Wait
from nav2_msgs.srv import ClearEntireCostmap
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration_node import RobotExplorer
from lekiwi_rmf.exploration_owner_guard import ExplorationOwnerGuard
from test_exploration_ros import RobotPeers, response, wait


ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("failure", [
    "none", "cancel", "planner", "controller", "backup", "cancel_recovery", "exhausted", "fatal",
])
def test_real_nav2_tree_recovers_with_the_same_target_and_cancels_its_actions(tmp_path, failure):
    context = Context()
    rclpy.init(context=context)

    class Peers(RobotPeers):
        def plan(self, goal):
            # Let Explorer's preflight pass, then fail the native tree's first plan.
            if failure == "planner" and len(self.planner_ids) == 1:
                self.planner_ids.append(goal.request.planner_id)
                goal.abort()
                return ComputePathToPose.Result(error_code=ComputePathToPose.Result.NO_VALID_PATH)
            return super().plan(goal)

    peers = Peers(context, native_navigation=True)
    guard = ExplorationOwnerGuard(context=context)
    peers.nav_mode = "hold" if failure == "cancel" else "success"
    failed_controls = {"controller": 1, "backup": 8, "cancel_recovery": 6,
                       "exhausted": 14, "fatal": 1}.get(failure, 0)
    follow_goals, clears, recoveries = [], [], []
    recovery = {"active": None, "canceled": 0}

    def clear(_request, reply, which):
        clears.append(which)
        return reply

    for which in ("local", "global"):
        peers.create_service(ClearEntireCostmap, f"/{which}_costmap/clear_entirely_{which}_costmap",
                             lambda request, reply, which=which: clear(request, reply, which),
                             callback_group=peers.group)

    def recover(goal, kind, action):
        recoveries.append(kind)
        recovery["active"] = kind
        while failure == "cancel_recovery" and kind == "wait" and not goal.is_cancel_requested:
            time.sleep(0.03)
        if goal.is_cancel_requested:
            recovery["canceled"] += 1
            goal.canceled()
        else:
            goal.succeed()
        recovery["active"] = None
        return action.Result()

    behaviors = [ActionServer(peers, action, f"/{kind}",
                              lambda goal, kind=kind, action=action: recover(goal, kind, action),
                              cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                              callback_group=peers.group)
                 for kind, action in (("spin", Spin), ("wait", Wait), ("backup", BackUp))]

    def follow(goal):
        peers.nav_active = True
        p = goal.request.path.poses[-1].pose.position
        follow_goals.append((p.x, p.y))
        if len(follow_goals) <= failed_controls:
            peers.nav_active = False
            goal.abort()
            code = FollowPath.Result.INVALID_CONTROLLER if failure == "fatal" else FollowPath.Result.FAILED_TO_MAKE_PROGRESS
            return FollowPath.Result(error_code=code)
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
        Parameter("navigation_tree", value=str(ROOT / "config/explore_nav_to_pose.xml")),
        Parameter("max_duration_sec", value=30.0),
        # This test covers the native tree against instrumented Python peers.
        # Production one-second input leases are exercised by the ROS fault tests.
        Parameter("data_timeout_sec", value=3.0),
        Parameter("settle_sec", value=0.2),
    ])
    client_node = Node("native_exploration_test", context=context)
    client = ActionClient(client_node, Explore, "/robot/explore")
    lifecycle = client_node.create_client(ChangeState, "/bt_navigator/change_state")
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (peers, guard, explorer, client_node):
        executor.add_node(node)
    finished = Future()
    thread = threading.Thread(target=executor.spin_until_future_complete, args=(finished,), daemon=True)
    thread.start()
    executable = Path(get_package_prefix("nav2_bt_navigator")) / "lib/nav2_bt_navigator/bt_navigator"
    log_path = tmp_path / "bt-navigator.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            str(executable), "--ros-args", "--params-file", str(ROOT / "config/nav2_params.yaml"),
            "-p", "navigators:=[navigate_to_pose]",
            # Python peers under coverage need longer than the production 20 ms
            # goal-response budget; task RPC/cleanup deadlines remain bounded.
            "-p", "default_server_timeout:=1000",

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
            wait(lambda: explorer._inputs.get("navigation_guard", (False,))[0])
            wait(lambda: "collision_monitor" in explorer._inputs)
            handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
            assert handle.accepted, log_path.read_text()
            if failure == "cancel":
                wait(lambda: peers.nav_active)
                response(handle.cancel_goal_async())
            elif failure == "cancel_recovery":
                wait(lambda: recovery["active"] == "wait", timeout=15)
                response(handle.cancel_goal_async())
            result = response(handle.get_result_async(), timeout=20)
            cancel = failure in ("cancel", "cancel_recovery")
            failed = failure == "fatal"
            expected = (GoalStatus.STATUS_CANCELED if cancel else
                        GoalStatus.STATUS_ABORTED if failed else GoalStatus.STATUS_SUCCEEDED)
            assert result.status == expected, (result.result.message, log_path.read_text())
            assert len(peers.planner_ids) >= 2 and set(peers.planner_ids) == {"ExploreKnown"}
            wait(lambda: not peers.nav_active)
            assert peers.nav_canceled == int(failure == "cancel")
            assert recovery["active"] is None
            assert recovery["canceled"] == int(failure == "cancel_recovery")
            assert len(set(follow_goals)) == 1 + int(failure == "exhausted")
            assert result.result.unreachable_targets == 0
            assert result.result.visited_targets == int(not cancel and not failed)
            if failure == "planner":
                assert "global" in clears
            if failure == "controller":
                assert clears == ["local"] and len(follow_goals) == 2
            if failure == "backup":
                assert recoveries == ["spin", "wait", "backup"]
                assert len(follow_goals) == failed_controls + 1
            if failure == "exhausted":
                assert recoveries == ["spin", "wait", "backup", "spin"]
                assert len(follow_goals) == failed_controls + 1
                assert len(set(follow_goals[:-1])) == 1
            if failure == "fatal":
                assert result.result.message == "Nav2 controller plugin is invalid"
                assert not clears and not recoveries and len(follow_goals) == 1
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
            finished.set_result(True)
            thread.join(timeout=5)
            assert not thread.is_alive()
            assert executor.shutdown(timeout_sec=5)
            controller.destroy()
            for behavior in behaviors:
                behavior.destroy()
            client.destroy()
            for node in (client_node, explorer, guard, peers):
                node.destroy_node()
            context.shutdown()
