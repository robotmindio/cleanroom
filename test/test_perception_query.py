"""The open-vocabulary query set: bounds, ownership, per-frame binding and observation identity."""

import json
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
import yaml
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import LabelInfo

from lekiwi_rmf.object_observations import observation
from lekiwi_rmf.object_recognition import ProviderUnavailable, recognition_identity
from lekiwi_rmf.perception_query import MAX_LABELS, make_query, normalize_label, recorded_query
from lekiwi_rmf.srv import QueryObservations, SetPerceptionQuery
from test_object_observations import capture
from test_remote_perception import label_info

REMOTE = 'grounding_dino_tiny_gpu'
DEFAULT = make_query(['a cup', 'a mug'])


def test_labels_are_normalized_deduplicated_and_ordered():
    query = make_query(['  Phillips\tScrewdriver ', 'phillips screwdriver', 'Taza', "kid's cup", 'usb-c cable'])
    assert query.labels == ('phillips screwdriver', 'taza', "kid's cup", 'usb-c cable')
    assert query.prompt == "phillips screwdriver. taza. kid's cup. usb-c cable."
    assert make_query(['Taza']).revision == make_query(['taza ']).revision
    # Phrase order is part of the prompt Grounding DINO sees, so it is part of the revision.
    assert make_query(['a', 'b']).revision != make_query(['b', 'a']).revision
    assert normalize_label('ＣＵＰ') == 'cup'


@pytest.mark.parametrize('labels', [
    [], [''], ['   '], ['a cup. a mug'], ['cup,mug'], ['cup\x00mug'], ['cup/mug'], ['x' * 49], ['x' * 500], [7], 'cup',
    [f'label {index}' for index in range(MAX_LABELS + 1)],
    # Six 46-character labels keep each label in bounds but build a 287-character prompt.
    [f'{index:02d}' + 'z' * 44 for index in range(6)]])
def test_queries_outside_their_bounds_are_rejected(labels):
    with pytest.raises(ValueError):
        make_query(labels)


def test_a_recorded_query_must_already_be_canonical():
    assert recorded_query(['a cup', 'a mug']) == DEFAULT
    for labels in (['A cup'], ['a cup', 'a cup'], ['a  cup']):
        with pytest.raises(ValueError):
            recorded_query(labels)


def test_the_same_capture_under_another_query_is_another_inference():
    model = recognition_identity(REMOTE, .5)[0]
    cups, _ = observation(*capture(), model, 'astra_color', DEFAULT)
    spoons, _ = observation(*capture(), model, 'astra_color', make_query(['spoon']))
    closed, _ = observation(*capture(), model, 'astra_color')
    assert cups['capture_id'] == spoons['capture_id'] == closed['capture_id']
    assert len({cups['id'], spoons['id'], closed['id']}) == 3
    assert cups['query'] == DEFAULT.document() and cups['vocabulary'] == 'open'
    assert spoons['query']['labels'] == ['spoon'] and spoons['query']['prompt'] == 'spoon.'
    # Closed-vocabulary identity keeps its established derivation; it carries no query.
    assert closed['vocabulary'] == 'closed' and 'query' not in closed
    import hashlib
    assert closed['id'] == hashlib.sha256(f"{closed['capture_id']}:{model}".encode()).hexdigest()


def wait(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('timed out')
        time.sleep(.02)


class Harness:
    """A detector with fake recognizers, optionally a collector, and a peer on one isolated context."""

    def __init__(self, monkeypatch, tmp_path, primary, fallback=None, collector=False):
        from lekiwi_rmf import object_detector
        from lekiwi_rmf.object_observation_node import ObjectObservations

        bindings = tmp_path / 'cameras.yaml'
        bindings.write_text(yaml.safe_dump({'astra_color': {
            'mount': 'base', 'image_topic': '/camera/test/image', 'optical_frame': 'camera_optical',
            'camera_info_topic': '/camera/test/info', 'depth_topic': '', 'calibration_file': ''}}))
        recognizers = {REMOTE: primary, 'rtdetr_v2_r18': fallback}
        monkeypatch.setattr(object_detector, 'Recognizer', lambda name, *args: recognizers[name])
        self.context = Context()
        rclpy.init(context=self.context)
        shared = [Parameter('model_name', value=REMOTE), Parameter('camera_ids', value=['astra_color']),
                  Parameter('camera_profiles_file', value=str(bindings))]
        if fallback is not None:
            shared.append(Parameter('fallback_model_name', value='rtdetr_v2_r18'))
        self.detector = object_detector.ObjectDetector(context=self.context, parameter_overrides=[
            *shared, Parameter('max_rate_hz', value=10.)])
        self.nodes = [self.detector]
        self.collector = None
        if collector:
            self.collector = ObjectObservations(context=self.context, parameter_overrides=[
                *shared, Parameter('database_path', value=str(tmp_path / 'observations.db'))])
            self.nodes.append(self.collector)
        self.peer = Node('query_peer', context=self.context)
        self.nodes.append(self.peer)
        self.source = self.peer.create_publisher(Image, '/camera/test/image', 1)
        self.queries, self.profiles = [], []
        self.peer.create_subscription(LabelInfo, '/robot/perception/astra_color/query', self.queries.append, 10)
        self.peer.create_subscription(String, '/robot/perception/profile', lambda message: self.profiles.append(
            json.loads(message.data)), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = self.peer.create_client(SetPerceptionQuery, '/robot/perception/set_query')
        self.observations = self.peer.create_client(QueryObservations, '/robot/observations/query')
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        for node in self.nodes:
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait(lambda: self.source.get_subscription_count() >= 1 and self.client.service_is_ready())

    def set_query(self, labels, expected=''):
        future = self.client.call_async(SetPerceptionQuery.Request(labels=labels, expected_revision=expected))
        wait(future.done)
        return future.result()

    def send(self):
        image, _ = capture()
        image.header.frame_id = 'camera_optical'
        image.header.stamp = self.peer.get_clock().now().to_msg()
        self.source.publish(image)
        return image.header

    def records(self):
        # Through the collector's service: its store connection belongs to the executor's threads.
        future = self.observations.call_async(QueryObservations.Request(request_json='{"limit":100}'))
        wait(future.done)
        return json.loads(future.result().response_json)['observations']

    def close(self):
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(5)
        for node in self.nodes:
            node.destroy_node()
        self.context.try_shutdown()


def open_recognizer(predict):
    return SimpleNamespace(predict=predict, sha256='a' * 64, default_query=DEFAULT, labels=None)


def closed_recognizer(predict):
    return SimpleNamespace(predict=predict, sha256='b' * 64, default_query=None, labels=['cup', 'spoon'])


def test_query_service_validates_applies_and_restores_with_optimistic_revisions(monkeypatch, tmp_path):
    harness = Harness(monkeypatch, tmp_path, open_recognizer(lambda image, query: []),
                      closed_recognizer(lambda image: []))
    try:
        wait(lambda: harness.profiles)
        assert harness.profiles[-1]['vocabulary'] == 'open' and harness.profiles[-1]['labels'] == ['a cup', 'a mug']
        assert harness.profiles[-1]['fallback'] == {'model_sha256': 'b' * 64, 'model_name': 'rtdetr_v2_r18',
                                                    'vocabulary': 'closed', 'labels': ['cup', 'spoon']}
        applied = harness.set_query(['Screwdriver', 'cable tie'])
        assert applied.success and applied.labels == ['screwdriver', 'cable tie']
        assert applied.previous_revision == DEFAULT.revision and applied.previous_labels == ['a cup', 'a mug']
        wait(lambda: harness.profiles[-1]['query']['revision'] == applied.revision)
        assert harness.profiles[-1]['labels'] == ['screwdriver', 'cable tie']

        rejected = harness.set_query(['a cup. a mug'])
        assert not rejected.success and 'letters' in rejected.detail
        assert rejected.revision == applied.revision and rejected.labels == ['screwdriver', 'cable tie']
        too_many = harness.set_query([f'thing {index}' for index in range(MAX_LABELS + 1)])
        assert not too_many.success and harness.detector._query.revision == applied.revision

        # A stale restore must not clobber a newer query set by someone else.
        stale = harness.set_query([], expected=DEFAULT.revision)
        assert not stale.success and 'changed' in stale.detail
        restored = harness.set_query([], expected=applied.revision)
        assert restored.success and restored.revision == DEFAULT.revision
        wait(lambda: harness.profiles[-1]['query']['revision'] == DEFAULT.revision)
    finally:
        harness.close()


def test_closed_primary_rejects_queries(monkeypatch, tmp_path):
    from lekiwi_rmf import object_detector

    monkeypatch.setattr(object_detector, 'Recognizer', lambda *args: closed_recognizer(list))
    context = Context()
    rclpy.init(context=context)
    node = object_detector.ObjectDetector(context=context)
    try:
        response = node._set_query(SetPerceptionQuery.Request(labels=['spoon']), SetPerceptionQuery.Response())
        assert not response.success and 'closed vocabulary' in response.detail and response.revision == ''
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_open_fallback_is_refused(monkeypatch):
    from lekiwi_rmf import object_detector

    monkeypatch.setattr(object_detector, 'Recognizer', lambda name, *args: open_recognizer(list))
    context = Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match='closed-vocabulary'):
            object_detector.ObjectDetector(context=context, parameter_overrides=[
                Parameter('model_name', value=REMOTE), Parameter('fallback_model_name', value='rtdetr_v2_r18')])
    finally:
        context.try_shutdown()


def test_a_query_change_during_inference_keeps_that_frame_under_its_own_query(monkeypatch, tmp_path):
    entered, release, seen = threading.Event(), threading.Event(), []

    def predict(image, query):
        seen.append(query)
        if len(seen) == 1:
            entered.set()
            release.wait(5)
        return [[10, 10, 20, 20, .9, query.labels[0]]]

    harness = Harness(monkeypatch, tmp_path, open_recognizer(predict), collector=True)
    try:
        wait(harness.observations.service_is_ready)
        first = harness.send()
        assert entered.wait(5)
        changed = harness.set_query(['spoon'])
        assert changed.success
        release.set()
        wait(lambda: len(harness.records()) == 1)
        record, = harness.records()
        assert record['stamp'] == {'sec': first.stamp.sec, 'nanosec': first.stamp.nanosec}
        assert record['query'] == DEFAULT.document() and record['detections'][0]['label'] == 'a cup'
        time.sleep(.05)
        harness.send()
        wait(lambda: len(harness.records()) == 2)
        latest = harness.records()[-1]
        assert latest['query']['labels'] == ['spoon'] and latest['detections'][0]['label'] == 'spoon'
        assert seen[0] == DEFAULT and seen[-1] == make_query(['spoon'])
        assert harness.detector._stats['astra_color']['query_revision'] == changed.revision
    finally:
        release.set()
        harness.close()


def test_provider_down_serves_closed_fallback_without_claiming_the_query(monkeypatch, tmp_path):
    def unavailable(image, query):
        raise ProviderUnavailable('perception provider unreachable: refused')

    harness = Harness(monkeypatch, tmp_path, open_recognizer(unavailable),
                      closed_recognizer(lambda image: [[10, 10, 20, 20, .9, 'spoon']]), collector=True)
    try:
        assert harness.set_query(['spoon']).success
        wait(harness.observations.service_is_ready)
        deadline = time.monotonic() + 10
        while not harness.records():
            harness.send()
            time.sleep(.1)
            assert time.monotonic() < deadline
        record = harness.records()[0]
        assert record['model_sha256'] == recognition_identity('rtdetr_v2_r18', .5)[0]
        assert record['vocabulary'] == 'closed' and 'query' not in record
        assert record['detections'][0]['label'] == 'spoon'
        assert not harness.queries  # No query is published for frames the fallback served.
        assert harness.detector._stats['astra_color']['served_by'] == 'fallback'
    finally:
        harness.close()


def test_collector_rejects_a_frame_query_outside_the_operating_point(monkeypatch, tmp_path):
    from lekiwi_rmf.object_observation_node import ObjectObservations

    yaml_path = tmp_path / 'cameras.yaml'
    yaml_path.write_text(yaml.safe_dump({'astra_color': {
        'mount': 'base', 'image_topic': '/camera/test/image', 'optical_frame': 'camera_optical',
        'camera_info_topic': '/camera/test/info', 'depth_topic': '', 'calibration_file': ''}}))
    context = Context()
    rclpy.init(context=context)
    node = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')), Parameter('model_name', value=REMOTE),
        Parameter('camera_ids', value=['astra_color']), Parameter('camera_profiles_file', value=str(yaml_path))])
    try:
        image, _ = capture()
        assert node._frame_query(image, label_info(image.header, DEFAULT)) == DEFAULT
        other, _ = capture(stamp=2)
        for info, reason in [(label_info(other.header, DEFAULT), 'header'),
                             (label_info(image.header, DEFAULT, threshold=.4), 'threshold'),
                             (label_info(image.header, SimpleNamespace(labels=('A Cup',))), 'canonical')]:
            with pytest.raises(ValueError, match=reason):
                node._frame_query(image, info)
        shuffled = label_info(image.header, DEFAULT)
        shuffled.class_map[0].class_id = 5
        with pytest.raises(ValueError, match='order'):
            node._frame_query(image, shuffled)
    finally:
        node.destroy_node()
        context.try_shutdown()
