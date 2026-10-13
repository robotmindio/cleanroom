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

from lekiwi_rmf.object_model import MODEL_SHA256
from lekiwi_rmf.object_observation_node import ObjectObservations
from lekiwi_rmf.srv import GetObservationEvidence, QueryObservations
from test_object_observations import capture


def wait(predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('observation graph operation timed out')
        time.sleep(.02)


def test_synchronized_capture_query_evidence_and_stale_stream(tmp_path):
    context = Context()
    rclpy.init(context=context)
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')),
        Parameter('model_sha256', value=MODEL_SHA256), Parameter('max_source_age_sec', value=1.0)])
    peer = Node('observation_peer', context=context)
    images = peer.create_publisher(Image, 'image', 1)
    detections = peer.create_publisher(Detection2DArray, 'detections', 1)
    query = peer.create_client(QueryObservations, '/robot/observations/query')
    evidence = peer.create_client(GetObservationEvidence, '/robot/observations/evidence')
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
        saved = call(evidence, GetObservationEvidence.Request(observation_id=record['id']))
        assert saved.success and len(base64.b64decode(saved.data_base64)) == record['evidence']['size']
        assert saved.sha256 == record['evidence']['sha256']
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
