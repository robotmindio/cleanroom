"""Actual ROS message synchronization and evidence services on an isolated graph."""

import base64
import json
import threading
import time

from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
import rclpy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from lekiwi_rmf.object_recognition import recognition_identity

from lekiwi_rmf.object_observation_node import ObjectObservations
from lekiwi_rmf.srv import GetObservationEvidence, PinObservationEvidence, QueryObservations
from test_object_observations import capture, operating_point
from lekiwi_rmf.msg import PerceptionOperatingPoint


MODEL_SHA256 = recognition_identity('yolox')[0]

def wait(predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('observation graph operation timed out')
        time.sleep(.02)


def test_synchronized_capture_query_evidence_and_stale_stream(tmp_path):
    context = Context()
    rclpy.init(context=context)
    import yaml
    bindings = tmp_path / 'cameras.yaml'
    bindings.write_text(yaml.safe_dump({'astra_color': {
        'mount': 'base', 'image_topic': '/camera/test/image', 'optical_frame': 'camera_optical',
        'camera_info_topic': '/camera/test/info', 'depth_topic': '', 'calibration_file': ''}}))
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')),
        Parameter('model_sha256', value=MODEL_SHA256), Parameter('camera_ids', value=['astra_color']),
        Parameter('camera_profiles_file', value=str(bindings)), Parameter('max_source_age_sec', value=1.0)])
    peer = Node('observation_peer', context=context)
    images = peer.create_publisher(Image, '/robot/perception/astra_color/source_image', 1)
    detections = peer.create_publisher(Detection2DArray, '/robot/perception/astra_color/detections', 1)
    points = peer.create_publisher(PerceptionOperatingPoint, '/robot/perception/astra_color/operating_point', 1)
    query = peer.create_client(QueryObservations, '/robot/observations/query')
    evidence = peer.create_client(GetObservationEvidence, '/robot/observations/evidence')
    pin = peer.create_client(PinObservationEvidence, '/robot/observations/pin')
    status = peer.create_client(Trigger, '/robot/observations/status')
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    executor.add_node(collector)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(client, request):
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    try:
        wait(lambda: query.service_is_ready() and images.get_subscription_count() == 1 and detections.get_subscription_count() == 1)
        assert not call(status, Trigger.Request()).success
        image, found = capture(count=2)
        image.header.stamp = peer.get_clock().now().to_msg()
        found.header = image.header
        for item in found.detections:
            item.header = image.header
        detections.publish(found)
        points.publish(operating_point(image, count=2))
        time.sleep(.05)
        assert not json.loads(call(query, QueryObservations.Request(request_json='{}')).response_json)['observations']
        images.publish(image)
        wait(lambda: call(status, Trigger.Request()).success)
        response = call(query, QueryObservations.Request(request_json='{}'))
        result = json.loads(response.response_json)
        assert response.success and len(result['observations']) == 1
        record = result['observations'][0]
        assert record['clock_domain'] == 'ros_system_time'
        assert record['ros_domain_id'] == context.get_domain_id()
        assert len(record['detections']) == 2 and record['position'] is None
        # The evidence is the exact artifact the recognizer consumed, so it replays byte-exact.
        assert record['evidence']['model_input'] is True and record['profile']['backend'] == 'yolox'
        # Without depth, each detection says why it has no position and what would supply one.
        assert [item['reason'] for item in record['abstentions']] == ['geometry_unavailable'] * 2
        assert record['detections'][0]['geometry_unavailable'] == 'no_depth_calibration'
        saved = call(evidence, GetObservationEvidence.Request(observation_id=record['id']))
        assert saved.success and len(base64.b64decode(saved.data_base64)) == record['evidence']['size']
        assert saved.sha256 == record['evidence']['sha256']
        assert call(pin, PinObservationEvidence.Request(request_json=json.dumps({
            'request_id': 'goal-ros-test', 'observation_ids': [record['id']]}))).success
        receipt = call(query, QueryObservations.Request(request_json=json.dumps({
            'observation_ids': [record['id']], 'pin_request_id': 'goal-ros-test'})))
        assert record['id'] in json.loads(receipt.response_json)['pins']
        assert not call(query, QueryObservations.Request(request_json='{"limit":true}')).success
        assert not call(evidence, GetObservationEvidence.Request(observation_id='../secret')).success
        wait(lambda: not call(status, Trigger.Request()).success)
        # Stale evidence remains queryable as history, never as a ready live stream.
        assert not json.loads(call(query, QueryObservations.Request(request_json='{}')).response_json)['stream_ready']
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        collector.destroy_node()
        peer.destroy_node()
        context.try_shutdown()


def test_multiple_camera_readiness_cannot_mask_wrong_or_missing_source(tmp_path):
    import yaml
    bindings = tmp_path / 'cameras.yaml'
    bindings.write_text(yaml.safe_dump({name: {
        'mount': 'arm' if name == 'wrist' else 'base', 'image_topic': f'/camera/{name}/image', 'optical_frame': name + '_optical',
        'camera_info_topic': f'/camera/{name}/info', 'depth_topic': '', 'calibration_file': ''}
        for name in ['front', 'wrist']}))
    context = Context()
    rclpy.init(context=context)
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'multiple.db')),
        Parameter('model_sha256', value=MODEL_SHA256), Parameter('camera_ids', value=['front', 'wrist']),
        Parameter('camera_profiles_file', value=str(bindings))])
    def ingest(camera, frame):
        image, found = capture()
        image.header.frame_id = frame
        image.header.stamp = collector.get_clock().now().to_msg()
        found.header = image.header
        for item in found.detections:
            item.header = image.header
        collector._capture(image, found, camera, collector._roles[''], operating_point(image, count=1))
    try:
        ingest('front', 'front_optical')
        state = collector._readiness()
        assert state['front']['ready'] and not state['wrist']['ready']
        ingest('wrist', 'front_optical')
        assert not collector._readiness()['wrist']['ready']
        ingest('wrist', 'wrist_optical')
        assert collector._status(None, Trigger.Response()).success
        records = collector._store.query('{}')['observations']
        assert {record['camera_id'] for record in records} == {'front', 'wrist'}
        assert all(not record['stationary'] for record in records)
        collector._streams['wrist']['last'] -= 3
        assert collector._readiness()['wrist']['reason'] == 'observation stream stale'
        assert not collector._status(None, Trigger.Response()).success
        assert collector._readiness()['front']['ready']
    finally:
        collector.destroy_node()
        context.try_shutdown()


def test_depth_cache_retains_capture_until_inference_freshness_deadline(tmp_path):
    from builtin_interfaces.msg import Time as Stamp
    from rclpy.time import Time
    from sensor_msgs.msg import Image

    context = Context()
    rclpy.init(context=context)
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'depth-cache.db')),
        Parameter('model_sha256', value=MODEL_SHA256), Parameter('camera_ids', value=['astra_color']),
        Parameter('max_source_age_sec', value=3.)])
    try:
        cache = collector._streams['astra_color']['depth']
        for index in range(91):
            image = Image()
            image.header.stamp = Stamp(sec=100 + index // 30, nanosec=(index % 30) * 1000000000 // 30)
            cache.add(image)
        captured = Time.from_msg(Stamp(sec=100))
        assert cache.getInterval(captured, captured)
        assert len(cache.cache_msgs) <= 94
    finally:
        collector.destroy_node()
        context.try_shutdown()
