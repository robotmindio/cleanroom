"""The open-vocabulary query set: bounds, ownership, per-frame binding and observation identity."""

import json
import threading
import time

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

from lekiwi_rmf.msg import PerceptionOperatingPoint
from lekiwi_rmf.object_observations import observation
from lekiwi_rmf.object_recognition import ProviderUnavailable, recognition_identity
from lekiwi_rmf.perception_profiles import default_profile
from lekiwi_rmf.perception_query import MAX_LABELS, make_query, normalize_label, recorded_query
from lekiwi_rmf.srv import ApplyPerceptionProfile, QueryObservations, SetPerceptionQuery
from test_object_observations import capture, fake_recognizer, operating_point

REMOTE = 'grounding_dino_tiny_gpu'
# A closed fallback whose identity needs no torch metadata, so these tests run without the model runtime.
FALLBACK = 'yolox'
DEFAULT = make_query(['a cup', 'a mug'])
OPEN_PROFILE = default_profile('remote_grounding_dino')


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
    model = recognition_identity(REMOTE)[0]
    cups, _ = observation(*capture(), model, 'astra_color', DEFAULT, profile=OPEN_PROFILE)
    spoons, _ = observation(*capture(), model, 'astra_color', make_query(['spoon']), profile=OPEN_PROFILE)
    closed, _ = observation(*capture(), model, 'astra_color', profile=OPEN_PROFILE)
    assert cups['capture_id'] == spoons['capture_id'] == closed['capture_id']
    assert len({cups['id'], spoons['id'], closed['id']}) == 3
    assert cups['query'] == DEFAULT.document() and cups['vocabulary'] == 'open'
    assert spoons['query']['labels'] == ['spoon'] and spoons['query']['prompt'] == 'spoon.'
    # A closed-vocabulary inference is identified by model and profile; it carries no query.
    assert closed['vocabulary'] == 'closed' and 'query' not in closed
    import hashlib
    assert closed['id'] == hashlib.sha256(f"{closed['capture_id']}:{model}:{OPEN_PROFILE.revision}".encode()).hexdigest()


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
        recognizers = {REMOTE: primary, FALLBACK: fallback}
        monkeypatch.setattr(object_detector, 'Recognizer', lambda name, *args: recognizers[name])
        self.context = Context()
        rclpy.init(context=self.context)
        shared = [Parameter('model_name', value=REMOTE), Parameter('camera_ids', value=['astra_color']),
                  Parameter('camera_profiles_file', value=str(bindings))]
        if fallback is not None:
            shared.append(Parameter('fallback_model_name', value=FALLBACK))
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
        self.peer.create_subscription(PerceptionOperatingPoint, '/robot/perception/astra_color/operating_point',
                                      self.queries.append, 10)
        self.peer.create_subscription(String, '/robot/perception/profile', lambda message: self.profiles.append(
            json.loads(message.data)), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = self.peer.create_client(SetPerceptionQuery, '/robot/perception/set_query')
        self.profile_client = self.peer.create_client(ApplyPerceptionProfile, '/robot/perception/apply_profile')
        self.observations = self.peer.create_client(QueryObservations, '/robot/observations/query')
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        for node in self.nodes:
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait(lambda: self.source.get_subscription_count() >= 1 and self.client.service_is_ready())

    def set_query(self, labels, expected='', **fields):
        future = self.client.call_async(SetPerceptionQuery.Request(labels=labels, expected_revision=expected, **fields))
        wait(future.done)
        return future.result()

    def apply_profile(self, **fields):
        future = self.profile_client.call_async(ApplyPerceptionProfile.Request(**fields))
        wait(future.done, 15)
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
    """`predict(image, query, profile, artifact)`, as the detector calls an open recognizer."""
    return fake_recognizer(predict, 'remote_grounding_dino', sha256=recognition_identity(REMOTE)[0], name=REMOTE,
                           default_query=DEFAULT, labels=None, open_vocabulary=True, supports_concepts=True)


def closed_recognizer(predict):
    return fake_recognizer(predict, 'yolox', sha256=recognition_identity(FALLBACK)[0], name=FALLBACK,
                           labels=['cup', 'spoon'])


def test_query_service_validates_applies_and_restores_with_optimistic_revisions(monkeypatch, tmp_path):
    harness = Harness(monkeypatch, tmp_path, open_recognizer(lambda *_: []), closed_recognizer(lambda *_: []))
    try:
        wait(lambda: harness.profiles)
        primary, fallback = harness.profiles[-1]['recognizers']
        assert primary['open_vocabulary'] and primary['labels'] == ['a cup', 'a mug']
        assert primary['query'] == DEFAULT.document()
        assert (fallback['model_sha256'], fallback['labels'], fallback['query']) == (
            recognition_identity(FALLBACK)[0], ['cup', 'spoon'], None)
        applied = harness.set_query(['Screwdriver', 'cable tie'])
        assert applied.success and applied.labels == ['screwdriver', 'cable tie']
        assert applied.previous_revision == DEFAULT.revision and applied.previous_labels == ['a cup', 'a mug']
        wait(lambda: harness.profiles[-1]['recognizers'][0]['query']['revision'] == applied.revision)
        # Search and Fiber treat the queried labels as the ones the primary can report now.
        assert harness.profiles[-1]['recognizers'][0]['labels'] == ['screwdriver', 'cable tie']

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
        wait(lambda: harness.profiles[-1]['recognizers'][0]['query']['revision'] == DEFAULT.revision)
    finally:
        harness.close()


def test_closed_primary_rejects_queries(monkeypatch, tmp_path):
    from lekiwi_rmf import object_detector

    monkeypatch.setattr(object_detector, 'Recognizer', lambda *args: closed_recognizer(lambda *_: []))
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

    monkeypatch.setattr(object_detector, 'Recognizer', lambda name, *args: open_recognizer(lambda *_: []))
    context = Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match='closed-vocabulary'):
            object_detector.ObjectDetector(context=context, parameter_overrides=[
                Parameter('model_name', value=REMOTE), Parameter('fallback_model_name', value=FALLBACK)])
    finally:
        context.try_shutdown()


def test_a_query_change_during_inference_keeps_that_frame_under_its_own_query(monkeypatch, tmp_path):
    entered, release, seen = threading.Event(), threading.Event(), []

    def predict(image, query, *_):
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
    def unavailable(image, query, *_):
        raise ProviderUnavailable('perception provider unreachable: refused')

    harness = Harness(monkeypatch, tmp_path, open_recognizer(unavailable),
                      closed_recognizer(lambda *_: [[10, 10, 20, 20, .9, 'spoon']]), collector=True)
    try:
        assert harness.set_query(['spoon']).success
        wait(harness.observations.service_is_ready)
        deadline = time.monotonic() + 10
        while not harness.records():
            harness.send()
            time.sleep(.1)
            assert time.monotonic() < deadline
        record = harness.records()[0]
        assert record['model_sha256'] == recognition_identity(FALLBACK)[0]
        assert record['vocabulary'] == 'closed' and 'query' not in record
        assert record['detections'][0]['label'] == 'spoon'
        assert record['degraded']['reason'] == 'provider_unavailable'
        assert not harness.queries  # No primary operating point is published for frames the fallback served.
        assert harness.detector._stats['astra_color']['served_by'] == 'fallback'
    finally:
        harness.close()


def test_collector_rejects_a_frame_operating_point_it_cannot_prove(monkeypatch, tmp_path):
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
    model, role = recognition_identity(REMOTE)[0], node._roles['']
    try:
        image, _ = capture()
        query, profile = node._operating_point(image, operating_point(image, model, OPEN_PROFILE, DEFAULT), role)[:2]
        assert query == DEFAULT and profile == OPEN_PROFILE
        other, _ = capture(stamp=2)
        tampered = OPEN_PROFILE.document() | {'revision': '0' * 64}
        for point, reason in [
                (operating_point(other, model, OPEN_PROFILE, DEFAULT), 'header'),
                (operating_point(image, 'b' * 64, OPEN_PROFILE, DEFAULT), 'another recognizer'),
                (operating_point(image, model, OPEN_PROFILE, DEFAULT, profile_json=json.dumps(tampered)), 'revision'),
                (operating_point(image, model, default_profile('yolox'), DEFAULT), 'proposal profile of this'),
                (operating_point(image, model, OPEN_PROFILE, None), 'vocabulary'),
                (operating_point(image, model, OPEN_PROFILE, DEFAULT,
                                 query_json=json.dumps({'labels': ['A Cup']})), 'canonical'),
                (operating_point(image, model, OPEN_PROFILE, DEFAULT, input_sha256=''), 'digest')]:
            with pytest.raises(ValueError, match=reason):
                node._operating_point(image, point, role)
    finally:
        node.destroy_node()
        context.try_shutdown()
