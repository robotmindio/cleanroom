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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
import pytest
from vision_msgs.msg import Detection2DArray
import yaml

from lekiwi_rmf.object_observation_node import ObjectObservations
from lekiwi_rmf.object_recognition import recognition_identity
from lekiwi_rmf.spatial_memory_node import SpatialMemory, parse_request
from lekiwi_rmf.srv import QueryObservations, QuerySpatial
from test_object_observations import capture

MODEL_SHA256 = recognition_identity('yolox')[0]


def wait(predicate, timeout=5):
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
    for text in ('[]', '{"op": "drop"}', '{"op": "current", "x": 1}', '{"op": "revision", "limit": 0}',
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
        'camera_info_topic': '/camera/test/info', 'depth_topic': '', 'calibration_file': ''}}))
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')),
        Parameter('model_sha256', value=MODEL_SHA256), Parameter('camera_ids', value=['astra_color']),
        Parameter('camera_profiles_file', value=str(bindings)), Parameter('max_source_age_sec', value=1.0)])
    peer = Node('spatial_peer', context=context)
    graphs = peer.create_publisher(MapGraph, '/test/mapGraph', 1)
    infos = peer.create_publisher(Info, '/test/info', 10)
    revisions = []
    peer.create_subscription(String, '/robot/spatial/revision', lambda msg: revisions.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    images = peer.create_publisher(Image, '/robot/perception/astra_color/source_image', 1)
    detections = peer.create_publisher(Detection2DArray, '/robot/perception/astra_color/detections', 1)
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
        executor.shutdown()
        for node in (peer, spatial, collector):
            node.destroy_node()
        rclpy.shutdown(context=context)
        thread.join(timeout=5)

