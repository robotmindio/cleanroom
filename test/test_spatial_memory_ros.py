"""RTAB-Map graphs become revisions; observations carry capture-time map poses that reproject (T-26, T-28)."""

import json
import math
import sqlite3
import threading
import time

from geometry_msgs.msg import Pose, TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
import rclpy
from rtabmap_msgs.msg import Info, MapGraph
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import pytest
from vision_msgs.msg import Detection2DArray
import yaml

from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.object_observation_node import ObjectObservations
from lekiwi_rmf.object_recognition import recognition_identity
from lekiwi_rmf.spatial_memory_node import SpatialMemory, parse_request
from lekiwi_rmf.srv import QueryObservations, QuerySpatial
from ros_test_utils import release_graph
from test_object_observations import capture, operating_point
from lekiwi_rmf.msg import PerceptionOperatingPoint

MODEL_SHA256 = recognition_identity('yolox')[0]


def wait(predicate, timeout=30):
    # Generous: the coverage job runs every test in one loaded process.
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('spatial graph operation timed out')
        time.sleep(.02)


def graph(stamp, shift_y=0.0, ids=range(1, 6)):
    message = MapGraph()
    message.header.stamp.sec = stamp
    message.header.frame_id = 'map'
    for node in ids:
        pose = Pose()
        pose.position.x, pose.position.y = float(node), shift_y * node
        pose.orientation.w = 1.0
        message.poses_id.append(node)
        message.poses.append(pose)
    return message


def rtabmap_database(path):
    connection = sqlite3.connect(path)
    connection.execute('CREATE TABLE Node (id INTEGER PRIMARY KEY, time_enter DATE)')
    connection.execute("INSERT INTO Node VALUES (1, '2026-10-19 09:00:00')")
    connection.commit()
    connection.close()


def test_request_validation():
    assert parse_request('{"op": "current"}') == {'op': 'current'}
    assert parse_request('{"op": "pose"}') == {'op': 'pose'}
    for text in ('[]', '{"op": "drop"}', '{"op": "current", "x": 1}', '{"op": "pose", "frame": "odom"}', '{"op": "revision", "limit": 0}',
                 '{"op": "project", "items": []}', '{"op": "project", "items": [{"map_pose": {}}], "target": {}}'):
        try:
            parse_request(text)
        except ValueError:
            continue
        raise AssertionError(f'accepted {text}')


def test_graphs_become_revisions_and_observations_reproject(tmp_path):
    context = Context()
    rclpy.init(context=context)
    database = tmp_path / 'rtabmap.db'
    rtabmap_database(database)
    spatial = SpatialMemory(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'spatial.db')),
        Parameter('rtabmap_database', value=str(database)), Parameter('site_ref', value='home'),
        Parameter('graph_topic', value='/test/mapGraph'), Parameter('info_topic', value='/test/info')])
    bindings = tmp_path / 'cameras.yaml'
    bindings.write_text(yaml.safe_dump({'astra_color': {
        'mount': 'base', 'image_topic': '/camera/test/image', 'optical_frame': 'camera_optical',
        'camera_info_topic': '/camera/test/info', 'depth_topic': '', 'calibration_file': '',
        'input_quality': {'dark_luma': 40., 'bright_luma': 225., 'blur_variance': 30.}}}))
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')),
        Parameter('camera_ids', value=['astra_color']),
        Parameter('camera_profiles_file', value=str(bindings)), Parameter('max_source_age_sec', value=1.0)])
    peer = Node('spatial_peer', context=context)
    graphs = peer.create_publisher(MapGraph, '/test/mapGraph', 1)
    infos = peer.create_publisher(Info, '/test/info', 10)
    revisions = []
    peer.create_subscription(String, '/robot/spatial/revision', lambda msg: revisions.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    images = peer.create_publisher(Image, '/robot/perception/astra_color/source_image', 1)
    detections = peer.create_publisher(Detection2DArray, '/robot/perception/astra_color/detections', 1)
    points = peer.create_publisher(PerceptionOperatingPoint, '/robot/perception/astra_color/operating_point', 1)
    spatial_query = peer.create_client(QuerySpatial, '/robot/spatial/query')
    observation_query = peer.create_client(QueryObservations, '/robot/observations/query')
    camera = TransformStamped()
    camera.header.frame_id, camera.child_frame_id = 'map', 'camera_optical'
    camera.transform.translation.x, camera.transform.translation.z = 2.1, 0.3
    camera.transform.rotation.x, camera.transform.rotation.y = -0.5, 0.5
    camera.transform.rotation.z, camera.transform.rotation.w = -0.5, 0.5
    StaticTransformBroadcaster(peer).sendTransform(camera)
    executor = SingleThreadedExecutor(context=context)
    for node in (peer, spatial, collector):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(client, request):
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    def spatial_call(value):
        response = call(spatial_query, QuerySpatial.Request(request_json=json.dumps(value)))
        assert response.success, response.detail
        return json.loads(response.response_json)

    try:
        wait(lambda: spatial_query.service_is_ready() and graphs.get_subscription_count() == 1
             and images.get_subscription_count() == 1)
        assert not spatial_call({'op': 'current'})['ready']
        now = peer.get_clock().now().to_msg().sec
        graphs.publish(graph(now - 5))
        wait(lambda: revisions)
        first = revisions[-1]
        assert first['cause'] == 'initial' and first['site_ref'] == 'home' and first['anchor_count'] == 5
        assert first['session']['map_ref'] == first['map_ref']
        current = spatial_call({'op': 'current'})
        assert current['ready'] and current['revision']['revision'] == first['revision']

        # A capture after the revision stores its capture-time camera pose and revision.
        wait(lambda: len(collector._spatial) == 1)
        image, found = capture(count=1)
        image.header.stamp = peer.get_clock().now().to_msg()
        found.header = image.header
        for item in found.detections:
            item.header = image.header
        detections.publish(found)
        points.publish(operating_point(image, count=1))
        images.publish(image)
        records = []

        def stored():
            result = json.loads(call(observation_query, QueryObservations.Request(request_json='{}')).response_json)
            records[:] = result['observations']
            return bool(records)
        wait(stored)
        map_pose = records[0]['map_pose']
        assert map_pose['revision'] == first['revision'] and map_pose['site_ref'] == 'home'
        assert map_pose['child_frame_id'] == 'camera_optical' and map_pose['source'] == 'tf_at_capture'
        assert math.dist(map_pose['pose']['t'], (2.1, 0.0, 0.3)) < 1e-6

        anchored = spatial_call({'op': 'anchor', 'items': [
            {'id': records[0]['id'], 'map_pose': map_pose, 'points': [[0, 0, 1.0]], 'uncertainties_m': [0.02]}]})
        item = anchored['items'][0]
        assert item['ok'] and item['anchor_ref'].endswith(':2') and item['revision'] == first['revision']

        # A loop closure moves the graph; the stored observation is reprojected, not rewritten.
        infos.publish(Info(loop_closure_id=4))
        time.sleep(.1)
        graphs.publish(graph(now - 4, shift_y=0.05))
        wait(lambda: revisions[-1]['revision'] != first['revision'])
        closed = revisions[-1]
        assert closed['cause'] == 'loop_closure' and closed['parent_revision'] == first['revision']
        projected = spatial_call({'op': 'project', 'items': [{'id': records[0]['id'], 'map_pose': map_pose,
                                                               'points': [[0, 0, 1.0]]}]})['items'][0]
        assert projected['ok'] and projected['revision'] == closed['revision']
        assert projected['points'][0][1] == pytest.approx(0.1, abs=1e-6) and projected['shift_m'][0] == pytest.approx(0.1, abs=1e-6)
        stored()
        assert records[0]['map_pose'] == map_pose

        page = spatial_call({'op': 'revision', 'limit': 2})
        assert page['revision'] == closed['revision'] and len(page['anchors']) == 2 and page['truncated']
        rest = spatial_call({'op': 'revision', 'revision': closed['revision'], 'after_anchor': page['next_anchor'],
                             'limit': 10})
        assert len(rest['anchors']) == 3 and not rest['truncated']
        foreign = spatial_call({'op': 'project', 'items': [{'map_pose': {**map_pose, 'site_ref': 'other'}}]})
        assert foreign['items'][0]['code'] == 'site_mismatch'
        maps = spatial_call({'op': 'maps'})
        assert [entry['map_ref'] for entry in maps['maps']] == [first['map_ref']]
    finally:
        stop(executor, thread, (peer, spatial, collector), context)


def test_stored_observations_become_object_entries_and_the_robot_pose_is_answered(tmp_path):
    context = Context()
    rclpy.init(context=context)
    database = tmp_path / 'rtabmap.db'
    rtabmap_database(database)
    spatial = SpatialMemory(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'spatial.db')),
        Parameter('rtabmap_database', value=str(database)), Parameter('site_ref', value='home'),
        Parameter('graph_topic', value='/test/mapGraph'), Parameter('info_topic', value='/test/info'),
        Parameter('objects_tick_sec', value=0.1)])
    binding = camera_bindings(['front'])['front']
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')),
        Parameter('camera_ids', value=['front']), Parameter('max_source_age_sec', value=1.0)])
    peer = Node('spatial_peer', context=context)
    graphs = peer.create_publisher(MapGraph, '/test/mapGraph', 1)
    revisions = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    peer.create_subscription(String, '/robot/spatial/revision', lambda msg: revisions.append(json.loads(msg.data)), latched)
    images = peer.create_publisher(Image, '/robot/perception/front/source_image', 1)
    detections = peer.create_publisher(Detection2DArray, '/robot/perception/front/detections', 1)
    points = peer.create_publisher(PerceptionOperatingPoint, '/robot/perception/front/operating_point', 1)
    infos = peer.create_publisher(CameraInfo, binding['camera_info_topic'], 10)
    profiles = peer.create_publisher(String, '/robot/perception/profile', latched)
    spatial_query = peer.create_client(QuerySpatial, '/robot/spatial/query')
    object_query = peer.create_client(QuerySpatial, '/robot/objects/query')
    camera = TransformStamped()
    camera.header.frame_id, camera.child_frame_id = 'map', binding['optical_frame']
    camera.transform.translation.x, camera.transform.translation.z = 2.1, 0.3
    camera.transform.rotation.x, camera.transform.rotation.y = -0.5, 0.5
    camera.transform.rotation.z, camera.transform.rotation.w = -0.5, 0.5
    StaticTransformBroadcaster(peer).sendTransform(camera)
    base = TransformStamped()
    base.header.frame_id, base.child_frame_id = 'map', 'base_footprint'
    base.transform.translation.x, base.transform.rotation.w = 2.1, 1.0
    broadcaster, localized = TransformBroadcaster(peer), threading.Event()

    def publish():
        infos.publish(CameraInfo(header=Header(frame_id=binding['optical_frame']), width=60, height=40,
                                 k=[50., 0., 30., 0., 50., 20., 0., 0., 1.]))
        if localized.is_set():
            base.header.stamp = peer.get_clock().now().to_msg()
            broadcaster.sendTransform(base)
    ticker = peer.create_timer(0.05, publish)
    executor = SingleThreadedExecutor(context=context)
    for node in (peer, spatial, collector):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(client, value):
        future = client.call_async(QuerySpatial.Request(request_json=json.dumps(value)))
        wait(future.done)
        response = future.result()
        return response.success, json.loads(response.response_json) if response.success else response.detail

    try:
        wait(lambda: spatial_query.service_is_ready() and object_query.service_is_ready()
             and graphs.get_subscription_count() == 1 and images.get_subscription_count() == 1)
        # Before any graph the robot is nowhere it can name; with a graph but no map transform, still nowhere.
        assert call(spatial_query, {'op': 'pose'}) == (
            False, 'localization unavailable: waiting for an optimized RTAB-Map graph')
        assert call(object_query, {'region': {'x': 0, 'y': 0, 'radius_m': 1}})[1].startswith('localization unavailable')
        assert call(object_query, {'bogus': 1})[0] is False
        now = peer.get_clock().now().to_msg().sec
        graphs.publish(graph(now - 5))
        wait(lambda: revisions and len(collector._spatial) == 1)
        ok, detail = call(spatial_query, {'op': 'pose'})
        assert not ok and 'no transform from map to base_footprint' in detail
        localized.set()
        wait(lambda: call(spatial_query, {'op': 'pose'})[0])
        pose = call(spatial_query, {'op': 'pose'})[1]
        assert pose['site_ref'] == 'home' and pose['map_ref'] == revisions[-1]['map_ref']
        assert pose['revision'] == revisions[-1]['revision'] and pose['anchor_ref'].endswith(':2')
        assert pose['pose']['t'] == [2.1, 0.0, 0.0] and pose['child_frame_id'] == 'base_footprint'
        assert 0 <= peer.get_clock().now().nanoseconds / 1e9 - pose['stamp_unix_sec'] <= 1.0
        assert pose['localization_session'] == revisions[-1]['session']['session']

        # The detector's operating point names the cameras; a stored capture then becomes an entry.
        profiles.publish(String(data=json.dumps({'schema': 1, 'cameras': {'front': binding['sha256']}, 'recognizers': [
            {'role': 'primary', 'model_sha256': MODEL_SHA256, 'model_name': 'yolox', 'open_vocabulary': False,
             'labels': ['cup'], 'profile': {'revision': 'b' * 64}}]})))
        assert call(object_query, {})[1]['objects'] == []
        image, found = capture(count=1)
        image.header.frame_id = binding['optical_frame']
        image.header.stamp = peer.get_clock().now().to_msg()
        found.header = image.header
        for item in found.detections:
            item.header = image.header
        detections.publish(found)
        points.publish(operating_point(image, count=1))
        images.publish(image)
        wait(lambda: call(object_query, {'label': 'cup'})[1]['objects'])
        result = call(object_query, {'label': 'cup'})[1]
        entry = result['objects'][0]
        # No depth and no 3D map: the honest estimate is a bearing, and the observation keeps no position.
        assert entry['method'] == 'bearing_only' and entry['position'] is None and entry['qualified'] is False
        assert entry['bearings'][0]['origin'] == [2.1, 0.0] and entry['state'] == 'present'
        assert entry['map_ref'] == pose['map_ref'] and entry['revision'] == pose['revision']
        assert result['localized'] and result['ingestion']['enabled'] and result['ingestion']['observation_sequence'] == 1
        assert result['ingestion']['counts']['created'] == 1 and not entry['observations'][0]['evidence_expired']
        stored = collector._store.query('{}')['observations'][0]
        assert stored['id'] == entry['observations'][0]['id'] and stored['position'] is None
        assert 'position' not in stored['detections'][0]
        # The cursor is persisted with the entry: reading on creates nothing more.
        time.sleep(0.5)
        later = call(object_query, {})[1]['objects']
        assert len(later) == 1 and later[0]['sightings'] == 1
        assert spatial._objects.cursor()[:2] == (collector._store.epoch, 1)
    finally:
        ticker.cancel()  # Nothing may still be publishing while the graph is torn down.
        stop(executor, thread, (peer, spatial, collector), context)


def stop(executor, thread, nodes, context):
    """Leave nothing on the domain for the next test in the same process."""
    release_graph(executor=executor, thread=thread, nodes=nodes, context=context)


def test_teardown_releases_everything_after_a_failing_step():
    """A failing teardown step must not leave later nodes or the context alive for the next test."""
    released = []

    class Part:
        def __init__(self, name, fails=False):
            self.name, self.fails = name, fails

        def destroy(self):
            released.append(self.name)
            if self.fails:
                raise RuntimeError(f'{self.name} failed')

        destroy_node = destroy

    class Executor:
        def shutdown(self, timeout_sec):
            released.append('executor')
            return False  # Reported, but teardown continues.

    class Context:
        def try_shutdown(self):
            released.append('context')

    with pytest.raises(AssertionError, match='executor did not shut down'):
        release_graph(executor=Executor(), waitables=(Part('client', fails=True),),
                      nodes=(Part('explorer', fails=True), Part('peers')), context=Context())
    assert released == ['executor', 'client', 'explorer', 'peers', 'context']
