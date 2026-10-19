"""Search evidence follows SLAM revisions: anchored captures are reprojected, unanchored ones leave (T-26)."""

import hashlib
import json
import math
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
import rclpy
from geometry_msgs.msg import TransformStamped
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer

from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.search_evidence import SearchEvidence
from lekiwi_rmf.spatial_memory_node import SpatialMemory
from lekiwi_rmf.srv import QueryObservations
from test_spatial_memory_ros import graph, rtabmap_database, stop, wait

MODEL = 'ab' * 32


class Store(Node):
    """Observation store fake: records with or without a capture-time map pose."""

    def __init__(self, context):
        super().__init__('fake_store', context=context)
        self.records, self.lock = [], threading.Lock()
        self.binding = camera_bindings(['front'])['front']
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(String, '/robot/perception/profile', latched).publish(String(data=json.dumps({
            'schema': 1, 'cameras': {'front': self.binding['sha256']}, 'recognizers': [
                {'role': 'primary', 'model_sha256': MODEL, 'model_name': 'fake', 'confidence_threshold': .5,
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
                'stamp': {'sec': stamp, 'nanosec': 0}, 'model_sha256': MODEL, 'stationary': True,
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
    tf.transform.translation.x, tf.transform.rotation.w = 1.0, 1.0
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
        wait(lambda: evidence._cursor is not None)
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
