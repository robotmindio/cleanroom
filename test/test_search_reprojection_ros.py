"""Search evidence follows SLAM revisions: anchored captures are reprojected, unanchored ones leave (T-26)."""

import hashlib
from array import array
from dataclasses import replace
import json
import math
import threading
from types import SimpleNamespace

import pytest
from octomap_msgs.msg import Octomap

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time as RosTime
import rclpy
from geometry_msgs.msg import TransformStamped
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer

from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.spatial_anchors import SCHEMA as SPATIAL_SCHEMA
from lekiwi_rmf.search_evidence import SearchEvidence
from lekiwi_rmf.spatial_memory_node import SpatialMemory
from lekiwi_rmf.srv import QueryObservations
from test_spatial_memory_ros import graph, rtabmap_database, stop, wait
from test_coverage_search import capture

MODEL = 'ab' * 32
PROFILE_REVISION = 'e' * 64


def test_native_mapper_publishes_known_free_3d_geometry_with_the_graph_stamp(tmp_path, monkeypatch):
    """Exercise the configured producer, not a hand-encoded OctoMap message."""
    import signal
    import subprocess
    import time
    from pathlib import Path

    import numpy as np
    import yaml
    from ament_index_python.packages import get_package_prefix
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from tf2_ros import StaticTransformBroadcaster

    from launch_snapshot import find_node, resolve_launch
    from lekiwi_rmf.search_octomap import SearchOctomap
    from test_launch_validation import search_configuration

    search_configuration(monkeypatch, True)
    parameters = find_node(resolve_launch(profile='sim')[0], name='rtabmap')['parameters']
    parameters.update(use_sim_time=False, database_path=str(tmp_path / 'map.db'),
                      subscribe_scan_cloud=False, **{'Reg/Strategy': '0'})
    config = tmp_path / 'mapper.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    context = Context()
    rclpy.init(context=context)
    assert context.get_domain_id() != 0, 'synthetic geometry must never enter the physical domain'
    node = Node('depth_geometry_producer_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    image = node.create_publisher(Image, '/geometry_probe/rgb', 10)
    depth = node.create_publisher(Image, '/geometry_probe/depth', 10)
    info = node.create_publisher(CameraInfo, '/geometry_probe/info', 10)
    maps, graphs = [], []
    node.create_subscription(Octomap, '/geometry_probe/octomap_binary', maps.append, 10)
    node.create_subscription(MapGraph, '/geometry_probe/mapGraph', graphs.append, 10)
    broadcaster = StaticTransformBroadcaster(node)
    camera = TransformStamped()
    camera.header.frame_id, camera.child_frame_id = 'base_footprint', 'probe_optical'
    camera.transform.translation.x = camera.transform.translation.y = .025
    camera.transform.translation.z = .325
    camera.transform.rotation.x, camera.transform.rotation.y = -.5, .5
    camera.transform.rotation.z, camera.transform.rotation.w = -.5, .5
    odom = TransformStamped()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
    odom.transform.rotation.w = 1.
    broadcaster.sendTransform([camera, odom])
    bridge = CvBridge()
    pixels = np.random.default_rng(4).integers(0, 256, (240, 320, 3), dtype=np.uint8)
    rgb = bridge.cv2_to_imgmsg(pixels, 'bgr8')
    wall = bridge.cv2_to_imgmsg(np.full((240, 320), 2., np.float32), '32FC1')
    calibration = CameraInfo(width=320, height=240, distortion_model='plumb_bob', d=[0.] * 5)
    calibration.k = [300., 0., 160., 0., 300., 120., 0., 0., 1.]
    calibration.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    calibration.p = [300., 0., 160., 0., 0., 300., 120., 0., 0., 0., 1., 0.]
    binary = Path(get_package_prefix('rtabmap_slam')) / 'lib/rtabmap_slam/rtabmap'
    with (tmp_path / 'mapper.log').open('w+') as log:
        process = subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:=/geometry_probe',
                                    '--params-file', str(config), '-r', 'rgb/image:=/geometry_probe/rgb',
                                    '-r', 'depth/image:=/geometry_probe/depth',
                                    '-r', 'rgb/camera_info:=/geometry_probe/info'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and not (len(maps) >= 2 and len(graphs) >= 2):
                stamp = node.get_clock().now().to_msg()
                for message in (rgb, wall, calibration):
                    message.header.stamp, message.header.frame_id = stamp, 'probe_optical'
                image.publish(rgb)
                depth.publish(wall)
                info.publish(calibration)
                executor.spin_once(timeout_sec=.1)
            log.flush()
            assert len(maps) >= 2 and graphs, (tmp_path / 'mapper.log').read_text()[-6000:]
            assert maps[-1].header.stamp != maps[0].header.stamp
            assert any(m.header.stamp == g.header.stamp for m in maps for g in graphs)
            scene = SearchOctomap.from_binary(maps[-1].data, maps[-1].resolution)
            (tmp_path / 'scene.bin').write_bytes(memoryview(maps[-1].data).cast('B'))
            assert maps[-1].binary and maps[-1].header.frame_id == 'map'
            # Verify free voxels above the lidar plane; sparse native rays need not clear a whole column.
            assert any(scene.ray_free((x, .025, z), (x + .005, .025, z))
                       for x in (.225, .525, .975, 1.475) for z in (.225, .325, .425))
            assert not scene.ray_free((.075, .025, .325), (3.025, .025, .325))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            executor.shutdown(timeout_sec=5)
            node.destroy_node()
            context.try_shutdown()


class Store(Node):
    """Observation store fake: records with or without a capture-time map pose."""

    def __init__(self, context):
        super().__init__('fake_store', context=context)
        self.records, self.lock = [], threading.Lock()
        self.binding = camera_bindings(['front'])['front']
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(String, '/robot/perception/profile', latched).publish(String(data=json.dumps({
            'schema': 1, 'cameras': {'front': self.binding['sha256']}, 'recognizers': [
                {'role': 'primary', 'model_sha256': MODEL, 'model_name': 'fake', 'profile': {'revision': PROFILE_REVISION},
                 'labels': ['cup'], 'open_vocabulary': True}]})))
        self.info = self.create_publisher(CameraInfo, self.binding['camera_info_topic'], 10)
        self.create_service(QueryObservations, '/robot/observations/query', self.query)
        self.create_timer(0.2, self.publish_info)

    def publish_info(self):
        info = CameraInfo(width=640, height=480)
        info.header.frame_id = self.binding['optical_frame']
        info.k = [554.3, 0.0, 320.0, 0.0, 554.3, 240.0, 0.0, 0.0, 1.0]
        self.info.publish(info)

    def add(self, map_pose, stamp, query=('cup', 'mug')):
        with self.lock:
            sequence = len(self.records) + 1
            self.records.append({
                'schema': 1, 'id': hashlib.sha256(str(sequence).encode()).hexdigest(), 'sequence': sequence,
                'camera_id': 'front', 'frame_id': self.binding['optical_frame'],
                'stamp': {'sec': stamp, 'nanosec': 0}, 'model_sha256': MODEL, 'profile': {'revision': PROFILE_REVISION}, 'stationary': True,
                'grounding': 'camera_position', 'map_pose': map_pose,
                'vocabulary': 'open', 'query': {'labels': list(query)},
                'detections': [{'label': 'cup', 'score': 0.9, 'bbox': [300.0, 200.0, 40.0, 40.0], 'position': {
                    'mode': 'camera', 'xyz': [0.0, 0.0, 1.0], 'frame_id': self.binding['optical_frame'],
                    'uncertainty_m': 0.02}}]})

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


def test_loop_closure_reprojects_the_capture_log(tmp_path):
    context = Context()
    rclpy.init(context=context)
    database = tmp_path / 'rtabmap.db'
    rtabmap_database(database)
    spatial = SpatialMemory(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'spatial.db')),
        Parameter('rtabmap_database', value=str(database)), Parameter('site_ref', value='home'),
        Parameter('graph_topic', value='/test/mapGraph'), Parameter('info_topic', value='/test/info')])
    store = Store(context)
    host = Node('search_host', context=context)
    transforms = Buffer()
    # TF for the record without a map pose: it is placed while TF holds its time, and only then.
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'map', store.binding['optical_frame']
    tf.transform.translation.x = 1.0
    tf.transform.rotation.x, tf.transform.rotation.y = -.5, .5
    tf.transform.rotation.z, tf.transform.rotation.w = -.5, .5
    transforms.set_transform_static(tf, 'test')
    evidence = SearchEvidence(host, transforms, ReentrantCallbackGroup(), 100)
    graphs = host.create_publisher(MapGraph, '/test/mapGraph', 1)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    for node in (spatial, store, host):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait(lambda: graphs.get_subscription_count() == 1 and evidence.profile is not None)
        now = host.get_clock().now().to_msg().sec
        graphs.publish(graph(now - 5))
        wait(lambda: evidence._target is not None)
        first = dict(evidence._target)
        # The cursor starts at the store's head, so captures are added after the first poll.
        # A page consumed before camera_info arrives is correctly rejected, not retried.
        wait(lambda: evidence._cursor is not None and 'front' in evidence._intrinsics
             and evidence._spatial.service_is_ready())
        camera = {'t': [2.1, 0.0, 0.3], 'q': [-0.5, 0.5, -0.5, 0.5]}
        store.add({'site_ref': 'home', 'map_ref': first['map_ref'], 'revision': first['revision'],
                   'localization_session': 'x', 'pose': camera}, now - 4)
        store.add(None, now - 3)  # An older record without a capture-time map pose.
        wait(lambda: len(evidence.since(0)) == 2)
        anchored, unanchored = (capture for _, capture in evidence.since(0))
        assert math.dist((anchored.pose.x, anchored.pose.y), (2.1, 0.0)) < 1e-6
        assert math.dist(anchored.detections[0].map_xyz, (3.1, 0.0, 0.3)) < 1e-6
        assert evidence.rejected.get('unanchored_capture') == 1 and unanchored.pose.x == 1.0
        # Both pose paths keep the labels the frame's own query asked for.
        assert anchored.query_labels == unanchored.query_labels == ('cup', 'mug')
        generation = evidence.generation

        # Loop closure: node 2 moves 0.1 m in y; the anchored capture follows, the unanchored one leaves.
        graphs.publish(graph(now - 2, shift_y=0.05))
        wait(lambda: evidence.generation > generation)
        _, revision, log = evidence.snapshot()
        assert revision != first['revision'] and len(log) == 1
        moved = log[0][1]
        assert moved.observation_id == anchored.observation_id and moved.query_labels == ('cup', 'mug')
        assert math.dist((moved.pose.x, moved.pose.y), (2.1, 0.1)) < 1e-6
        assert math.dist(moved.detections[0].map_xyz, (3.1, 0.1, 0.3)) < 1e-6
        assert evidence.rejected['unanchored_after_revision'] == 1
    finally:
        stop(executor, thread, (spatial, store, host), context)


@pytest.fixture
def geometry_log(monkeypatch):
    context = Context()
    rclpy.init(context=context)
    node = Node('search_geometry_contract', context=context)
    now = [100.]
    evidence = SearchEvidence(node, Buffer(), ReentrantCallbackGroup(), 8)
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: RosTime(nanoseconds=round(now[0] * 1e9))))
    try:
        yield evidence, now
    finally:
        node.destroy_node()
        context.try_shutdown()


def geometry_revision(evidence, stamp, revision='r1'):
    evidence._on_revision(String(data=json.dumps({
        'schema': SPATIAL_SCHEMA, 'map_ref': 'test-map', 'revision': revision, 'graph_stamp_unix_sec': stamp})))
    evidence._poll()


def octomap_message(stamp, payload=b'\x55\x55', **overrides):
    message = Octomap(binary=True, id='OcTree', resolution=.05, data=array('b', payload))
    message.header.frame_id = 'map'
    message.header.stamp = RosTime(nanoseconds=round(stamp * 1e9)).to_msg()
    for key, value in overrides.items():
        setattr(message, key, value)
    return message


@pytest.mark.parametrize('map_first', [False, True])
def test_3d_map_and_graph_bind_in_either_order(geometry_log, map_first):
    evidence, _ = geometry_log
    geometry_revision(evidence, 99.9)
    if map_first:
        evidence._on_geometry(octomap_message(100.))
        assert evidence.geometry_for(capture(stamp=100.1)) is None
        geometry_revision(evidence, 100.)
    else:
        geometry_revision(evidence, 100.)
        evidence._on_geometry(octomap_message(100.))
    assert evidence.geometry_for(capture(stamp=100.1)) is not None


@pytest.mark.parametrize('fault', ['old', 'future', 'zero_stamp', 'frame', 'full', 'type', 'truncated', 'resolution'])
def test_invalid_or_stale_3d_map_never_admits_negative_geometry(geometry_log, fault):
    evidence, _ = geometry_log
    geometry_revision(evidence, 100.)
    message = octomap_message(100.)
    if fault in ('old', 'future', 'zero_stamp'):
        message = octomap_message({'old': 98., 'future': 100.2, 'zero_stamp': 0.}[fault])
    elif fault == 'frame':
        message.header.frame_id = 'odom'
    elif fault == 'full':
        message.binary = False
    elif fault == 'type':
        message.id = 'UnsupportedTree'
    elif fault == 'truncated':
        message.data = array('b', b'\x55')
    else:
        message.resolution = math.nan
    evidence._on_geometry(message)
    assert evidence.geometry_for(capture(stamp=100.1)) is None
    assert evidence.rejected['invalid_3d_geometry'] == 1


def test_future_scene_is_not_retroactively_used_for_an_older_image(geometry_log):
    evidence, _ = geometry_log
    geometry_revision(evidence, 100.02)
    evidence._on_geometry(octomap_message(100.02))
    assert evidence.geometry_for(capture(stamp=100.01)) is None
    assert evidence.geometry_for(capture(identifier='b' * 64, stamp=100.03)) is not None
    assert evidence.geometry_for(capture(identifier='c' * 64, stamp=102.)) is None


def test_new_revision_rejects_same_stamp_replay_but_preserves_capture_time_proof(geometry_log):
    evidence, now = geometry_log
    geometry_revision(evidence, 100.)
    evidence._on_geometry(octomap_message(100.))
    original = capture(stamp=100.01)
    assert evidence.geometry_for(original) is not None
    geometry_revision(evidence, 99.)  # An older notification cannot weaken the replay boundary.
    geometry_revision(evidence, 100., 'r2')
    evidence._on_geometry(octomap_message(100.))
    assert evidence.geometry_for(capture(identifier='b' * 64, stamp=100.02)) is None
    moved = replace(original, pose=replace(original.pose, y=.5))
    assert evidence.geometry_for(moved).reference_pose == (0., 0., .2, 0., 0., 0.)
    now[0] = 100.2
    geometry_revision(evidence, 100.2, 'r2')
    evidence._on_geometry(octomap_message(100.2))
    assert evidence.geometry_for(capture(identifier='c' * 64, stamp=100.21)) is not None


def test_evicted_capture_geometry_cannot_be_replaced_with_a_newer_map(geometry_log, monkeypatch):
    from lekiwi_rmf import search_evidence
    evidence, now = geometry_log
    monkeypatch.setattr(search_evidence, 'GEOMETRY_CACHE_BYTES', 2)
    geometry_revision(evidence, 100.)
    evidence._on_geometry(octomap_message(100.))
    original = capture(stamp=100.01)
    assert evidence.geometry_for(original) is not None
    now[0] = 100.2
    geometry_revision(evidence, 100.2)
    evidence._on_geometry(octomap_message(100.2, resolution=.025))
    assert evidence._geometry_bytes <= 2
    assert evidence.geometry_for(original) is None
