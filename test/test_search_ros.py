"""Active search through the real explorer, owner guard and action contracts; fake Nav2 and perception."""

from concurrent.futures import Future
import hashlib
import json
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Header, String
from tf2_ros import StaticTransformBroadcaster

from lekiwi_rmf.action import Search
from lekiwi_rmf.exploration_node import RobotExplorer
from lekiwi_rmf.exploration_owner_guard import ExplorationOwnerGuard
from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.srv import QueryObservations
from test_exploration_ros import RobotPeers, response, wait

MODEL = 'b' * 64
FALLBACK = 'f' * 64
PROFILE_REVISION = 'e' * 64
FX, WIDTH, HEIGHT = 554.3, 640, 480
MOUNT = (0.1, 0.0, 0.2)


class SearchPeers(RobotPeers):
    """Nav2 that honours the goal heading, plus odometry for the travel budget."""

    def __init__(self, context):
        super().__init__(context)
        self.yaw = 0.0
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.02, self.publish_odom)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.grid.header.stamp = stamp
        self.map_pub.publish(self.grid)
        self.base_pub.publish(Bool(data=self.permitted))
        self.stow_pub.publish(Bool(data=True))
        self.collision_pub.publish(self.collision)
        self.info_pub.publish(Info(header=Header(stamp=stamp)))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = self.position
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.tf_pub.sendTransform(transform)

    def publish_odom(self):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = self.position
        self.odom.publish(message)

    def navigate(self, goal):
        if self.nav_mode == 'success':
            q = goal.request.pose.pose.orientation
            self.yaw = 2 * math.atan2(q.z, q.w)
        grid = list(self.grid.data)
        result = super().navigate(goal)
        self.grid.data = grid  # This map is already complete; navigation must not rewrite it.
        return result


class Perception(Node):
    """Profile, intrinsics and an observation store fed by a simulated detector seeing ``target``."""

    def __init__(self, context, peers, labels=('chair', 'book', 'cup')):
        super().__init__('fake_perception', context=context)
        self.peers, self.target, self.stationary_while_moving = peers, None, False
        self.model = MODEL  # The recognizer serving frames: the primary, or the fallback while it is down.
        self.records = []
        self.lock = threading.Lock()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        binding = camera_bindings(['front'])['front']
        self.create_publisher(String, '/robot/perception/profile', latched).publish(String(data=json.dumps({
            'schema': 1, 'cameras': {'front': binding['sha256']}, 'recognizers': [
                {'role': 'primary', 'model_sha256': MODEL, 'model_name': 'fake_open', 'profile': {'revision': PROFILE_REVISION},
                 'labels': list(labels), 'open_vocabulary': True},
                {'role': 'fallback', 'model_sha256': FALLBACK, 'model_name': 'fake_closed',
                 'profile': {'revision': PROFILE_REVISION}, 'labels': ['chair'], 'open_vocabulary': False}]})))
        self.info = self.create_publisher(CameraInfo, '/camera/front/camera_info', 10)
        self.create_service(QueryObservations, '/robot/observations/query', self.query)
        self.static = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_footprint', 'front_camera_optical_frame'
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = MOUNT
        mount.transform.rotation.x, mount.transform.rotation.y = -0.5, 0.5
        mount.transform.rotation.z, mount.transform.rotation.w = -0.5, 0.5
        self.static.sendTransform(mount)
        self.create_timer(0.2, self.capture)

    def capture(self):
        info = CameraInfo(width=WIDTH, height=HEIGHT)
        info.header.frame_id = 'front_camera_optical_frame'
        info.k = [FX, 0.0, WIDTH / 2, 0.0, FX, HEIGHT / 2, 0.0, 0.0, 1.0]
        self.info.publish(info)
        stamp = (self.get_clock().now() - Duration(seconds=0.15)).to_msg()
        x, y, yaw = (*self.peers.position, self.peers.yaw)
        cx, cy = x + MOUNT[0] * math.cos(yaw), y + MOUNT[0] * math.sin(yaw)
        detections = []
        if self.target is not None:
            label, (tx, ty) = self.target
            bearing = math.remainder(math.atan2(ty - cy, tx - cx) - yaw, 2 * math.pi)
            u = WIDTH / 2 - FX * math.tan(bearing) if abs(bearing) < 0.5 else -1
            if 15 <= u <= WIDTH - 15 and math.dist((cx, cy), (tx, ty)) <= 1.8:
                detections.append({'label': label, 'score': 0.9, 'bbox': [u - 15, 300.0, 30.0, 40.0]})
        with self.lock:
            sequence = len(self.records) + 1
            identity = hashlib.sha256(f'{sequence}:{stamp.sec}:{stamp.nanosec}'.encode()).hexdigest()
            self.records.append({
                'schema': 1, 'id': identity, 'sequence': sequence, 'camera_id': 'front',
                'frame_id': 'front_camera_optical_frame', 'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                'model_sha256': self.model, 'profile': {'revision': PROFILE_REVISION},
                'detections': detections, 'grounding': 'image_only',
                'stationary': self.stationary_while_moving or not self.peers.nav_active})

    def query(self, request, result):
        values = json.loads(request.request_json)
        with self.lock:
            after = values.get('after_sequence', 0)
            records = [r for r in self.records if r['sequence'] > after][:values.get('limit', 20)]
            latest = len(self.records)
        result.success = True
        result.response_json = json.dumps({
            'schema': 1, 'epoch': 'e' * 32, 'observations': records, 'pins': {}, 'cursor_gap': False,
            'next_sequence': records[-1]['sequence'] if records else after, 'latest_sequence': latest})
        return result


@pytest.fixture
def search_graph(tmp_path, request):
    labels = getattr(request, 'param', ('chair', 'book', 'cup'))
    context = Context()
    rclpy.init(context=context)
    peers = SearchPeers(context)
    peers.grid.data = [0] * 6400  # A fully mapped 4 m x 4 m room.
    perception = Perception(context, peers, labels)
    guard = ExplorationOwnerGuard(context=context)
    explorer = RobotExplorer(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'map.db')),
        Parameter('allow_search', value=True),
        Parameter('service_timeout_sec', value=2.0), Parameter('settle_sec', value=0.1),
        Parameter('search_dwell_sec', value=3.0), Parameter('search_capture_log', value=500),
    ])
    client_node = Node('search_test', context=context)
    client = ActionClient(client_node, Search, '/robot/search')
    executor = MultiThreadedExecutor(num_threads=8, context=context)
    for node in (peers, perception, guard, explorer, client_node):
        executor.add_node(node)
    finished = Future()
    thread = threading.Thread(target=executor.spin_until_future_complete, args=(finished,), daemon=True)
    thread.start()
    try:
        wait(lambda: client.server_is_ready() and explorer._mapping is False and explorer._map is not None)
        wait(lambda: explorer._inputs.get('navigation_guard', (False,))[0] and 'collision_monitor' in explorer._inputs)
        wait(lambda: explorer._evidence.fresh(1.0) and explorer._evidence.mounts()
             and explorer._odometry.status()['ready'], timeout=15)
        yield peers, perception, explorer, client
    finally:
        peers.nav_mode = 'fail'
        peers.permitted = False
        explorer._shutdown_requested.set()
        try:
            wait(lambda: not explorer._busy, timeout=8)
            wait(lambda: not peers.nav_active)
        finally:
            finished.set_result(True)
            thread.join(timeout=5)
            assert executor.shutdown(timeout_sec=5)
            client.destroy()
            for node in (client_node, explorer, guard, perception, peers):
                node.destroy_node()
            context.shutdown()


def run(client, goal, timeout=60, feedback=None):
    handle = response(client.send_goal_async(goal, feedback_callback=feedback))
    assert handle.accepted
    return handle, response(handle.get_result_async(), timeout=timeout)


def test_search_observes_outside_the_initial_view_and_reports_evidence(search_graph):
    """T-37: the label behind the robot is found by choosing new views, not a fixed list."""
    peers, perception, explorer, client = search_graph
    peers.nav_mode = 'success'
    perception.target = ('book', (-1.2, 0.3))
    stages = []
    _, outcome = run(client, Search.Goal(query='Book', region_radius_m=1.8),
                     feedback=lambda message: stages.append(message.feedback.stage))
    result = outcome.result
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED, result.message
    assert result.complete and result.found and result.outcome == 'found'
    assert result.evidence_ids and peers.nav_count >= 1
    report = json.loads(result.report_json)
    assert report['found']['evidence_ids'] == list(result.evidence_ids)
    assert report['perception']['label_supported']
    assert report['negative_evidence']['model_sha256s'] == [MODEL]
    assert report['negative_evidence']['observation_count'] > 0
    assert report['plan']['revision_count'] >= 1 and 'heuristic' in report['plan']['utility']
    assert any(stage.startswith('searching: viewpoint') for stage in stages)
    # Every goal went through the guarded exploration behaviour tree.
    assert all(abs(pose.pose.position.x) < 2 for pose in peers.navigation_goals)


def test_absent_label_ends_at_stated_coverage_without_claiming_absence(search_graph):
    peers, _, explorer, client = search_graph
    peers.nav_mode = 'success'
    _, outcome = run(client, Search.Goal(query='cup', region_radius_m=1.2, target_coverage=0.6), timeout=90)
    result = outcome.result
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED, result.message
    assert result.outcome == 'coverage_reached' and not result.found and not result.evidence_ids
    assert result.coverage_fraction >= 0.6 and result.viewpoints_visited >= 1
    report = json.loads(result.report_json)
    assert 'not found in the inspected zones' in report['statement']
    assert 'not evidence of absence there' in report['statement']
    assert report['coverage']['unobserved_area_m2'] >= 0 and report['negative_evidence']['recent_observation_ids']
    assert report['negative_evidence']['criteria']['require_stationary'] is True


@pytest.mark.parametrize('search_graph', [('chair',)], indirect=True)
def test_label_outside_the_catalogue_is_incapable_and_never_moves(search_graph):
    peers, _, _, client = search_graph
    _, outcome = run(client, Search.Goal(query='cup'))
    assert outcome.status == GoalStatus.STATUS_ABORTED
    assert outcome.result.outcome == 'incapable' and peers.nav_count == 0
    assert 'nothing is claimed' in json.loads(outcome.result.report_json)['statement']


def test_evidence_en_route_cancels_and_replans_the_viewpoint(search_graph):
    """New coverage that refutes the planned view cancels its Nav2 goal before arrival."""
    peers, perception, explorer, client = search_graph
    perception.stationary_while_moving = True  # The camera observes cleanly while the base is driving.
    handle = response(client.send_goal_async(Search.Goal(query='cup', region_radius_m=1.5)))
    assert handle.accepted
    wait(lambda: peers.nav_active, timeout=15)
    planned = peers.navigation_goals[-1].pose
    # Drive smoothly to the planned view; its captures then cover the planned gain before arrival.
    # A teleport would jump odometry, pause the search and cancel for that reason instead.
    peers.yaw = 2 * math.atan2(planned.orientation.z, planned.orientation.w)
    start = peers.position
    steps = max(1, math.ceil(math.dist(start, (planned.position.x, planned.position.y)) / 0.02))
    for step in range(1, steps + 1):
        peers.position = (start[0] + (planned.position.x - start[0]) * step / steps,
                          start[1] + (planned.position.y - start[1]) * step / steps)
        time.sleep(0.02)
    wait(lambda: peers.nav_canceled >= 1, timeout=15)
    wait(lambda: peers.nav_count >= 2, timeout=15)
    response(handle.cancel_goal_async())
    outcome = response(handle.get_result_async(), timeout=15)
    assert outcome.status == GoalStatus.STATUS_CANCELED and outcome.result.outcome == 'canceled'
    revisions = json.loads(outcome.result.report_json)['plan']['recent_revisions']
    replanned = [entry for entry in revisions if entry['reason'] == 'new observations already covered the planned view']
    assert replanned and replanned[0]['evidence_ids']
    assert not peers.nav_active


def test_travel_budget_stops_the_owned_goal_with_an_honest_report(search_graph):
    peers, _, _, client = search_graph
    handle = response(client.send_goal_async(Search.Goal(query='cup', max_travel_m=0.3)))
    assert handle.accepted
    wait(lambda: peers.nav_active, timeout=15)
    for _ in range(30):  # Drive smoothly at 1 m/s so odometry accounts every metre.
        peers.position = (peers.position[0] + 0.02, peers.position[1])
        time.sleep(0.02)
    outcome = response(handle.get_result_async(), timeout=15)
    assert outcome.status == GoalStatus.STATUS_ABORTED
    assert outcome.result.outcome == 'budget_exhausted' and outcome.result.travel_m >= 0.3
    assert peers.nav_canceled >= 1 and not peers.nav_active
    assert 'not found in the inspected zones' in json.loads(outcome.result.report_json)['statement']


def test_search_shares_the_explorer_busy_lock_and_validates_limits(search_graph):
    peers, _, explorer, client = search_graph
    for goal in (Search.Goal(query=''), Search.Goal(query='cup', max_travel_m=1e6),
                 Search.Goal(query='cup', target_coverage=1.5), Search.Goal(query='cup', region_radius_m=float('nan'))):
        assert not response(client.send_goal_async(goal)).accepted
    handle = response(client.send_goal_async(Search.Goal(query='cup')))
    assert handle.accepted
    wait(lambda: peers.nav_active, timeout=15)
    assert not response(client.send_goal_async(Search.Goal(query='cup'))).accepted
    from lekiwi_rmf.action import Explore
    explore = ActionClient(peers, Explore, '/robot/explore')
    try:
        wait(explore.server_is_ready)
        assert not response(explore.send_goal_async(Explore.Goal())).accepted
    finally:
        explore.destroy()
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async(), timeout=15).status == GoalStatus.STATUS_CANCELED
    assert not peers.nav_active


def test_fallback_frames_never_cover_a_primary_only_label(search_graph):
    """While only the closed-set fallback serves, a primary-only label gains no evidence of absence."""
    peers, perception, _, client = search_graph
    perception.model = FALLBACK
    peers.nav_mode = 'success'
    _, outcome = run(client, Search.Goal(query='cup', region_radius_m=1.2, target_coverage=0.6, max_duration_sec=8.0))
    result = outcome.result
    assert outcome.status == GoalStatus.STATUS_ABORTED, result.message
    assert result.outcome in ('budget_exhausted', 'inaccessible') and result.coverage_fraction < 0.6
    report = json.loads(result.report_json)
    # Only primary frames retained from before the switch count; every fallback frame was ignored.
    assert set(report['negative_evidence']['observations_by_model']) <= {MODEL}
    assert report['negative_evidence']['ignored_unable_model'] >= 10
    assert [(r['role'], r['label_supported']) for r in report['perception']['recognizers']] == [
        ('primary', True), ('fallback', False)]
