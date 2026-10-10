"""Real ROS action/service exchanges on an isolated DDS domain, without motors."""

from concurrent.futures import Future
from itertools import groupby
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration_node import RobotExplorer
from lekiwi_rmf.exploration_owner_guard import ExplorationOwnerGuard


def wait(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("ROS condition did not complete before its deadline")


def response(future, timeout=5):
    wait(future.done, timeout)
    return future.result()


class RobotPeers(Node):
    def __init__(self, context, *, native_navigation=False):
        super().__init__("rtabmap", context=context)
        self.declare_parameter("Mem/IncrementalMemory", "false")
        self.mapping_requests = []
        self.permitted = True
        self.publish_slam = True
        self.slam_delay = 0.0
        self.nav_mode = "hold"
        self.nav_active = False
        self.nav_canceled = 0
        self.nav_count = 0
        self.planner_ids = []
        self.acceptance_delay = 0.0
        self.position = (0.0, 0.0)
        self.group = ReentrantCallbackGroup()
        self.create_service(Empty, "/rtabmap/set_mode_mapping", lambda req, res: self.set_mode(True, res),
                            callback_group=self.group)
        self.create_service(Empty, "/rtabmap/set_mode_localization", lambda req, res: self.set_mode(False, res),
                            callback_group=self.group)
        self.planner = ActionServer(self, ComputePathToPose, "/compute_path_to_pose", self.plan,
                                    callback_group=self.group)
        self.navigator = None if native_navigation else ActionServer(self, NavigateToPose, "/navigate_to_pose", self.navigate,
                                      goal_callback=self.accept_navigation,
                                      cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                                      callback_group=self.group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.base_pub = self.create_publisher(Bool, "/safety/base_motion_permitted", latched)
        self.stow_pub = self.create_publisher(Bool, "/safety/arm_stowed", latched)
        self.info_pub = self.create_publisher(Info, "/info", 1)
        self.tf_pub = TransformBroadcaster(self)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "map"
        self.grid.info.width = self.grid.info.height = 80
        self.grid.info.resolution = 0.05
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -2.0
        self.grid.info.origin.orientation.w = 1.0
        self.grid.data = [0 if index % 80 < 60 else -1 for index in range(6400)]
        self.create_timer(0.05, self.publish)

    def set_mode(self, mapping, result):
        self.mapping_requests.append(mapping)
        self.set_parameters([Parameter("Mem/IncrementalMemory", value=str(mapping).lower())])
        return result

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.grid.header.stamp = stamp
        self.map_pub.publish(self.grid)
        self.base_pub.publish(Bool(data=self.permitted))
        self.stow_pub.publish(Bool(data=True))
        if self.publish_slam:
            message = Info()
            message.header.stamp = (self.get_clock().now() - Duration(seconds=self.slam_delay)).to_msg()
            self.info_pub.publish(message)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_footprint"
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = self.position
        transform.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(transform)

    def plan(self, goal):
        self.planner_ids.append(goal.request.planner_id)
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x, start.pose.position.y = self.position
        start.pose.orientation.w = 1.0
        route = Path()
        route.header.frame_id = "map"
        route.poses = [start, goal.request.goal]
        goal.succeed()
        return ComputePathToPose.Result(path=route)

    def navigate(self, goal):
        assert goal.request.behavior_tree.endswith("explore_nav_to_pose.xml")
        self.nav_count += 1
        self.nav_active = True
        while self.nav_mode == "hold" and not goal.is_cancel_requested:
            current = PoseStamped()
            current.header.frame_id = "map"
            current.pose.position.x, current.pose.position.y = self.position
            goal.publish_feedback(NavigateToPose.Feedback(current_pose=current))
            time.sleep(0.03)
        if goal.is_cancel_requested:
            self.nav_canceled += 1
            goal.canceled()
        elif self.nav_mode == "success":
            p = goal.request.pose.pose.position
            self.position = (p.x, p.y)
            self.grid.data = [0] * 6400  # The new observation completes this map.
            goal.succeed()
        else:
            goal.abort()
        self.nav_active = False
        return NavigateToPose.Result()

    def accept_navigation(self, _goal):
        time.sleep(self.acceptance_delay)
        return GoalResponse.ACCEPT

    def destroy_node(self):
        if self.navigator is not None:
            self.navigator.destroy()
        self.planner.destroy()
        return super().destroy_node()


@pytest.fixture
def graph(tmp_path):
    context = Context()
    rclpy.init(context=context)
    peers = RobotPeers(context)
    guard = ExplorationOwnerGuard(context=context)
    explorer = RobotExplorer(context=context, parameter_overrides=[
        Parameter("database_path", value=str(tmp_path / "map.db")),
        Parameter("mapping_max_bytes", value=4096),
        Parameter("mapping_max_seconds", value=10.0),
        Parameter("max_duration_sec", value=10.0),
        Parameter("service_timeout_sec", value=2.0),
        Parameter("settle_sec", value=0.1),
    ])
    client_node = Node("exploration_test", context=context)
    client = ActionClient(client_node, Explore, "/robot/explore")
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (peers, guard, explorer, client_node):
        executor.add_node(node)
    finished = Future()
    thread = threading.Thread(target=executor.spin_until_future_complete, args=(finished,), daemon=True)
    thread.start()
    try:
        wait(lambda: client.server_is_ready() and explorer._mapping is False and explorer._map is not None)
        wait(lambda: "slam" in explorer._inputs and explorer._navigation.server_is_ready()
             and explorer._planner.server_is_ready() and explorer._tf.can_transform("map", "base_footprint", rclpy.time.Time()))
        wait(lambda: explorer._inputs.get("navigation_guard", (False,))[0])
        yield peers, explorer, client
    finally:
        peers.nav_mode = "fail"
        peers.permitted = False
        explorer._shutdown_requested.set()
        try:
            wait(lambda: not explorer._busy, timeout=6)
            wait(lambda: not peers.nav_active)
        finally:
            finished.set_result(True)
            thread.join(timeout=5)
            assert not thread.is_alive()
            assert executor.shutdown(timeout_sec=5)
            client.destroy()
            for node in (client_node, explorer, guard, peers):
                node.destroy_node()
            context.shutdown()


def start(graph, duration=8, revisit=False):
    peers, _, client = graph
    handle = response(client.send_goal_async(Explore.Goal(
        revisit_known=revisit, max_duration_sec=float(duration), max_radius_m=2.5)))
    assert handle.accepted
    wait(lambda: peers.nav_active)
    return handle


def test_cancel_stops_navigation_restores_mode_and_rejects_concurrent_goal(graph):
    peers, explorer, client = graph
    handle = start(graph)
    parameters = AsyncParameterClient(peers, "/robot_explorer", callback_group=peers.group)
    wait(parameters.services_are_ready)
    values = response(parameters.get_parameters(["slam_timeout_sec"]))
    assert values.values[0].double_value == 4.0
    assert peers.mapping_requests == [True]
    other = response(client.send_goal_async(Explore.Goal()))
    assert not other.accepted
    response(handle.cancel_goal_async())
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert peers.nav_canceled == 1 and not peers.nav_active
    assert peers.mapping_requests[-1] is False and explorer._mapping is False


def test_success_confirms_visited_target_and_retains_database(graph):
    peers, explorer, client = graph
    explorer.database.write_bytes(b"retained-map")
    peers.nav_mode = "success"
    feedback = []
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5),
                                              feedback_callback=feedback.append))
    assert handle.accepted
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.complete
    assert result.result.visited_targets == 1 and result.result.mapped_area_m2 > 0
    assert feedback and explorer.database.read_bytes() == b"retained-map"
    assert peers.mapping_requests == [True, False]


@pytest.mark.parametrize("fault", ["database", "duration", "mode"])
def test_quota_duration_or_mode_change_ends_and_cancels_the_owned_navigation_goal(graph, fault):
    peers, explorer, _ = graph
    handle = start(graph, duration=2 if fault == "duration" else 8)
    if fault == "database":
        explorer.database.write_bytes(b"x" * 4096)
    elif fault == "mode":
        peers.set_mode(False, Empty.Response())
    result = response(handle.get_result_async(), timeout=6)
    assert result.status == GoalStatus.STATUS_ABORTED and not result.result.complete
    assert peers.nav_canceled == 1 and not peers.nav_active
    assert explorer._mapping is False


@pytest.mark.parametrize("fault", ["permission", "slam"])
def test_recoverable_fault_stops_the_robot_then_resumes_instead_of_aborting(graph, fault):
    peers, explorer, client = graph
    # The tracked 4 s SLAM budget would consume most of the task's duration on a slow runner.
    explorer.config["slam_timeout_sec"] = 1.0
    feedback = []
    handle = response(client.send_goal_async(
        Explore.Goal(max_duration_sec=10.0, max_radius_m=2.5),
        feedback_callback=lambda message: feedback.append(message.feedback.stage)))
    assert handle.accepted
    wait(lambda: peers.nav_active)
    if fault == "permission":
        peers.permitted = False
    else:
        peers.publish_slam = False
    wait(lambda: peers.nav_canceled == 1 and not peers.nav_active)
    wait(lambda: any(stage.startswith("paused: ") for stage in feedback))
    time.sleep(0.5)
    assert explorer._busy and not peers.nav_active and peers.nav_count == 1
    if fault == "permission":
        peers.permitted = True
    else:
        peers.publish_slam = True
    wait(lambda: peers.nav_active and peers.nav_count == 2)
    response(handle.cancel_goal_async())
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not peers.nav_active and explorer._mapping is False


def test_a_mapped_cell_under_the_robot_neither_blocks_a_goal_nor_pauses_it(graph):
    peers, explorer, client = graph
    peers.grid.data[40 * 80 + 44] = 100  # Stale obstacle under the front corner, before the goal.
    feedback = []
    handle = response(client.send_goal_async(
        Explore.Goal(max_duration_sec=8.0, max_radius_m=2.5),
        feedback_callback=lambda message: feedback.append(message.feedback.stage)))
    assert handle.accepted
    wait(lambda: peers.nav_active)
    time.sleep(0.5)
    assert peers.nav_active and peers.nav_canceled == 0
    assert not any(stage.startswith("paused: ") for stage in feedback)
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED


def test_a_timed_out_mode_query_keeps_the_last_reading_until_it_ages_out():
    from rclpy.task import Future
    # The reply callback alone, without a ROS graph: a cancelled query must not erase the reading.
    explorer = SimpleNamespace(_lock=threading.Lock(), _mode_query_at=0.0, _mapping=False, _mode_at=12.0)
    timed_out = Future()
    timed_out.cancel()
    RobotExplorer._mode_response(explorer, timed_out, time.monotonic())
    assert explorer._mapping is False and explorer._mode_at == 12.0


@pytest.mark.parametrize("rpc", ["mode", "freeze"])
def test_timed_out_monitor_rpc_is_retried_and_confirmed(graph, rpc):
    from rclpy.task import Future as RosFuture

    peers, explorer, _ = graph
    explorer._timer.cancel()
    wait(lambda: explorer._mode_future.done())
    now = time.monotonic()
    stalled = RosFuture()
    setattr(explorer, f"_{rpc}_future", stalled)
    setattr(explorer, f"_{rpc}_requested_at", now - explorer.config["service_timeout_sec"] - 1)
    if rpc == "mode":
        explorer._mode_at = 0.0
        stalled.add_done_callback(lambda future: explorer._mode_response(future, now - 4))
    else:
        peers.set_mode(True, Empty.Response())
        explorer._mapping = True
        explorer._mapping_started = now - explorer.config["mapping_max_seconds"] - 1
        # Keep the parallel mode query pending while exercising the quota RPC.
        explorer._mode_future = RosFuture()
        explorer._mode_requested_at = now
        stalled.add_done_callback(explorer._check_freeze)

    explorer._monitor()
    assert stalled.cancelled() and not stalled.done()  # Real rclpy cancellation semantics.
    retried = getattr(explorer, f"_{rpc}_future")
    assert retried is not stalled
    wait(retried.done)
    if rpc == "mode":
        wait(lambda: explorer._mapping is False and explorer._mode_at >= now)
    else:
        assert peers.mapping_requests == [True, False]
        assert retried.result() is not None


def test_close_wall_exploration_uses_the_actual_body_instead_of_a_corner_circle(graph):
    peers, explorer, client = graph
    peers.position = (0.025, 0.025)
    peers.grid.data = [100 if index // 80 in (34, 46) else (-1 if index % 80 >= 60 else 0)
                       for index in range(6400)]
    peers.nav_mode = "success"
    wait(lambda: explorer._map[0][34, 40] == 100
         and abs(explorer._pose().pose.position.y - 0.025) < 1e-6)
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    assert handle.accepted
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.complete
    assert result.result.visited_targets == peers.nav_count == 1 and not peers.nav_active


def test_idle_guard_catches_direct_mapping_service_and_fresh_sessions(graph):
    peers, explorer, _ = graph
    explorer.config["mapping_max_seconds"] = 0.6
    peers.set_mode(True, Empty.Response())
    # A fresh session begins only after the previous freeze response completes.
    wait(lambda: True in peers.mapping_requests and peers.mapping_requests[-1] is False
         and explorer._mapping is False and explorer._freeze_future.done())
    assert not peers.nav_active and peers.nav_count == 0
    peers.set_mode(True, Empty.Response())
    wait(lambda: explorer._mapping is True)
    wait(lambda: peers.mapping_requests[-1] is False and explorer._mapping is False)
    # Quota enforcement may repeat the idempotent freeze before its mode query replies.
    assert [mode for mode, _ in groupby(peers.mapping_requests)] == [True, False, True, False]


def test_mode_rpc_failures_and_old_replies_do_not_reset_the_session_budget(graph):
    _, explorer, _ = graph
    explorer._timer.cancel()
    wait(lambda: explorer._mode_future.done())
    def reply(mapping):
        future = Future()
        future.set_result(SimpleNamespace(values=[SimpleNamespace(type=1, bool_value=mapping)]))
        return future
    sent = time.monotonic()
    explorer._mode_response(reply(True), sent)
    started = explorer._mapping_started
    explorer._mode_response(reply(False), sent - 1)
    assert explorer._mapping is True and explorer._mapping_started == started
    failure = Future()
    failure.set_exception(RuntimeError("temporary RPC loss"))
    explorer._mode_response(failure, sent + 1)
    assert explorer._mapping is None and explorer._mapping_started == started
    explorer._mode_response(reply(False), sent + 0.5)
    assert explorer._mapping is None and explorer._mapping_started == started
    explorer._mode_response(reply(True), sent + 2)
    assert explorer._mapping is True and explorer._mapping_started == started
    explorer._mode_response(reply(False), sent + 3)
    assert explorer._mapping_started is None


def test_invalid_or_denied_requests_have_no_mapping_or_navigation_side_effects(graph):
    peers, explorer, client = graph
    for value in (float("nan"), -1.0, 11.0):
        handle = response(client.send_goal_async(Explore.Goal(max_duration_sec=value)))
        assert not handle.accepted
    peers.permitted = False
    wait(lambda: not explorer._inputs["base_motion_permitted"][0])
    assert not response(client.send_goal_async(Explore.Goal())).accepted
    assert peers.mapping_requests == [] and peers.nav_count == 0


def test_disabled_exploration_keeps_the_mapping_monitor_without_navigation_geometry(tmp_path):
    context = Context()
    rclpy.init(context=context)
    explorer = RobotExplorer(context=context, parameter_overrides=[
        Parameter("allow_exploration", value=False),
        Parameter("database_path", value=str(tmp_path / "map.db")),
        Parameter("navigation_params_file", value=str(tmp_path / "unused-nav2.yaml")),
    ])
    try:
        explorer._monitor()
        assert explorer._timer is not None
        assert explorer._accept(Explore.Goal()) == GoalResponse.REJECT
    finally:
        explorer.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("delay,accepted", [(2.5, True), (4.5, False)])
def test_camera_stamped_slam_has_a_separate_bounded_freshness_budget(graph, delay, accepted):
    peers, explorer, client = graph
    peers.slam_delay = delay
    wait(lambda: (explorer.get_clock().now() - rclpy.time.Time.from_msg(
        explorer._inputs["slam"][0].stamp)).nanoseconds / 1e9 >= delay)
    assert explorer.config["data_timeout_sec"] == 1.0
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    assert handle.accepted is accepted
    if accepted:
        wait(lambda: peers.nav_active)
        response(handle.cancel_goal_async())
        assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
    else:
        assert peers.nav_count == 0 and peers.mapping_requests == []


def test_late_nav2_acceptance_is_canceled_and_ownership_is_retained(graph):
    peers, explorer, client = graph
    # Cleanup gives up after two 2 s service deadlines; accepting well after
    # that keeps the retained-ownership window open on a slow runner.
    peers.acceptance_delay = 8.0
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5)))
    assert handle.accepted
    result = response(handle.get_result_async(), timeout=10)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert "navigation stop unconfirmed" in result.result.message
    assert explorer._nav_uncertain
    assert not response(client.send_goal_async(Explore.Goal())).accepted
    wait(lambda: peers.nav_canceled == 1 and not explorer._nav_uncertain, timeout=10)
    assert not peers.nav_active


def test_shutdown_request_cancels_before_ros_context_is_closed(graph):
    peers, explorer, _ = graph
    handle = start(graph)
    explorer._shutdown_requested.set()
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_ABORTED
    assert peers.nav_canceled == 1 and not peers.nav_active
