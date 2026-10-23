"""Remote GPU perception provider adapter against a fake HTTP provider."""

import base64
import hashlib
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from lekiwi_rmf import object_recognition
from lekiwi_rmf.msg import PerceptionOperatingPoint
from lekiwi_rmf.object_observations import artifact_pixels, capture_artifact
from lekiwi_rmf.latency_budget import LatencyBudget
from lekiwi_rmf.object_recognition import (
    ProviderUnavailable, Recognizer, model_spec, provider_endpoint, provider_timings,
    recognition_identity)
from lekiwi_rmf.perception_profiles import default_profile, make_profile
from lekiwi_rmf.perception_query import make_query
from test_object_observations import capture, fake_recognizer, operating_point

REMOTE = 'grounding_dino_tiny_gpu'
SPEC = model_spec(REMOTE)
DEFAULT = make_query(SPEC['default_query'])
PROFILE = default_profile('remote_grounding_dino')


class FakeProvider:
    """Echoes the admitted operating point unless a test perturbs the response."""

    def __init__(self):
        self.requests, self.respond = [], self.admitted
        provider = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):  # noqa: N802 - stdlib handler name
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                provider.requests.append((self.path, body))
                status, payload = provider.respond(body)
                data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def admitted(self, body, **changes):
        data = base64.b64decode(body['image'])
        pixels = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        phrases = body['labels'] + [prompt for concept in body.get('concepts', []) for prompt in concept['prompts']]
        profile = body['profile']
        document = {
            'schema': 1, 'run_id': 'r' * 32, 'state': 'succeeded', 'inference_id': 'i' * 64,
            'model': body['model'], 'model_sha256': SPEC['provider_model_sha256'],
            'effective': {'labels': body['labels'], 'prompt': '. '.join(phrases) + '.',
                          'box_threshold': profile['box_threshold'], 'text_threshold': profile['text_threshold']},
            'profile': {'family': 'proposal', 'revision': 'p' * 64, 'parameters': profile},
            'proposals': {'floor': profile['proposal_floor'], 'retained': 4,
                          'discarded': {'below_box_threshold': 1, 'nms': 1, 'max_detections': 0, 'concept_mismatch': 0}},
            'image': {'width': pixels.shape[1], 'height': pixels.shape[0], 'format': 'image/jpeg',
                      'sha256': hashlib.sha256(data).hexdigest()},
            # The provider applied the profile's prompt-wide NMS: the overlapping synonym is an alternative.
            'detections': [{'box': [10.0, 12.0, 30.0, 40.0], 'score': .9, 'label': 'a cup',
                            'alternatives': [{'label': 'a mug', 'score': .7}]},
                           {'box': [50.0, 5.0, 60.0, 20.0], 'score': .6, 'label': 'a mug'}],
            'timings_ms': {'decode': 1., 'queue': 0., 'inference': 500., 'total': 501.}}
        document.update(changes)
        return 200, document

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def provider(monkeypatch):
    fake = FakeProvider()
    monkeypatch.setattr(object_recognition, 'provider_endpoint', lambda name: (fake.url, 1.0))
    yield fake
    fake.close()


def frame():
    pixels = np.zeros((48, 64, 3), np.uint8)
    pixels[..., 0], pixels[..., 2] = np.arange(64, dtype=np.uint8), 200
    return pixels


def predict(recognizer, query, profile=None):
    """As the detector calls it: the decoded evidence artifact and the artifact bytes themselves."""
    artifact = capture_artifact(frame())
    return recognizer.predict(artifact_pixels(artifact), query, profile, artifact), artifact


def test_remote_detections_use_source_pixels_the_profile_and_the_evidence_artifact(provider):
    recognizer = Recognizer(REMOTE, None)
    rows, artifact = predict(recognizer, DEFAULT)
    assert rows == [[10.0, 12.0, 20.0, 28.0, .9, 'a cup', {'alternatives': [{'label': 'a mug', 'score': .7}]}],
                    [50.0, 5.0, 10.0, 15.0, .6, 'a mug', {}]]
    assert rows.run == {'run_id': 'r' * 32, 'inference_id': 'i' * 64, 'proposals': provider.admitted(
        provider.requests[0][1])[1]['proposals']}
    path, body = provider.requests[0]
    assert path == '/v1/infer'
    assert body['model'] == 'grounding-dino-tiny' and body['labels'] == ['a cup', 'a mug']
    # The robot sends its complete profile, so the provider's own active profile never decides its frames.
    assert body['profile'] == PROFILE.parameters and body['deadline_ms'] == 1000
    assert 'box_threshold' not in body and 'concepts' not in body
    # The provider sees exactly the stored evidence bytes.
    assert base64.b64decode(body['image']) == artifact


def test_remote_provider_receives_the_requested_query_for_any_label(provider):
    recognizer = Recognizer(REMOTE, None)
    provider.respond = lambda body: provider.admitted(body, detections=[
        {'box': [1.0, 2.0, 11.0, 22.0], 'score': .8, 'label': 'phillips screwdriver'}])
    query = make_query(['Phillips  Screwdriver', 'cable tie'])
    assert predict(recognizer, query)[0] == [[1.0, 2.0, 10.0, 20.0, .8, 'phillips screwdriver', {}]]
    body = provider.requests[-1][1]
    assert body['labels'] == ['phillips screwdriver', 'cable tie']


def test_a_stricter_profile_reaches_the_provider_and_must_be_echoed(provider):
    recognizer = Recognizer(REMOTE, None)
    strict = make_profile('remote_grounding_dino', PROFILE.parameters | {'box_threshold': .6, 'max_detections': 5})
    predict(recognizer, DEFAULT, strict)
    assert provider.requests[-1][1]['profile']['box_threshold'] == .6
    # A provider that applied any other parameters did not run this frame's profile (T-10).
    provider.respond = lambda body: provider.admitted(body, profile={'family': 'proposal', 'revision': 'p' * 64,
                                                                     'parameters': PROFILE.parameters})
    with pytest.raises(ProviderUnavailable, match='operating point'):
        predict(recognizer, DEFAULT, strict)


def test_open_recognizers_need_a_query_and_closed_ones_refuse_one(provider):
    with pytest.raises(ValueError, match='needs a query'):
        predict(Recognizer(REMOTE, None), None)
    with pytest.raises(ValueError, match='profile for yolox'):
        predict(Recognizer(REMOTE, None), DEFAULT, default_profile('yolox'))
    assert not provider.requests


@pytest.mark.parametrize('perturb', [
    lambda provider, body: provider.admitted(body, model_sha256='0' * 64),
    lambda provider, body: provider.admitted(body, model='owlv2'),
    lambda provider, body: provider.admitted(body, schema=2),
    lambda provider, body: provider.admitted(body, effective={**provider.admitted(body)[1]['effective'], 'text_threshold': .3}),
    lambda provider, body: provider.admitted(body, profile=None),
    lambda provider, body: provider.admitted(body, state='cancelled'),
    lambda provider, body: provider.admitted(body, detections=[{'box': [1, 2, 3, 4], 'score': .9, 'label': 'a cup',
                                                                'alternatives': 'a mug'}]),
    # A concept match the query never asked for is not evidence of it.
    lambda provider, body: provider.admitted(body, detections=[{'box': [1, 2, 3, 4], 'score': .9, 'label': 'lata',
                                                                'concept': {'name': 'lata', 'revision': 'c' * 64,
                                                                            'similarity': .9, 'negative_similarity': 0}}]),
    # A provider that answered another query (cached, stale or rewritten) does not answer this frame's.
    lambda provider, body: provider.admitted(body, effective={**provider.admitted(body)[1]['effective'], 'labels': ['a cup']}),
    lambda provider, body: provider.admitted(body, effective={**provider.admitted(body)[1]['effective'], 'prompt': 'a cup.'}),
    lambda provider, body: provider.admitted(body, image={**provider.admitted(body)[1]['image'], 'sha256': '0' * 64}),
    lambda provider, body: provider.admitted(body, image={**provider.admitted(body)[1]['image'], 'width': 63}),
    lambda provider, body: provider.admitted(body, detections=[{'box': [1, 2, 3], 'score': .9, 'label': 'a cup'}]),
    lambda provider, body: provider.admitted(body, detections=[{'box': [1, 2, 3, 4], 'score': '.9', 'label': 'a cup'}]),
    lambda provider, body: provider.admitted(body, detections=[{'box': [float('nan'), 0, 1, 1], 'score': .9, 'label': 'a cup'}]),
    lambda provider, body: (200, b'not json'),
    lambda provider, body: (200, b'[]'),
    lambda provider, body: (200, b' ' * (1024 * 1024 + 1)),
    lambda provider, body: (503, {'error': {'code': 'provider_unavailable'}}),
    lambda provider, body: (429, {'error': {'code': 'provider_busy'}}),
])
def test_responses_outside_the_admitted_operating_point_are_unavailable(provider, perturb):
    provider.respond = lambda body: perturb(provider, body)
    with pytest.raises(ProviderUnavailable):
        predict(Recognizer(REMOTE, None), DEFAULT)


def test_unreachable_and_slow_providers_are_unavailable(monkeypatch, provider):
    with socket.socket() as closed:
        closed.bind(('127.0.0.1', 0))
        port = closed.getsockname()[1]
    monkeypatch.setattr(object_recognition, 'provider_endpoint', lambda name: (f'http://127.0.0.1:{port}', .5))
    with pytest.raises(ProviderUnavailable, match='unreachable'):
        predict(Recognizer(REMOTE, None), DEFAULT)
    monkeypatch.setattr(object_recognition, 'provider_endpoint', lambda name: (provider.url, .2))
    provider.respond = lambda body: time.sleep(.5) or provider.admitted(body)
    with pytest.raises(ProviderUnavailable, match='unreachable'):
        predict(Recognizer(REMOTE, None), DEFAULT)


def test_remote_identity_pins_the_provider_hash_but_not_its_endpoint(monkeypatch):
    identity, document = recognition_identity(REMOTE)
    assert document['device'] == 'remote' and set(document['runtime']) == {'opencv', 'numpy'}
    assert document['artifacts']['provider_model_sha256'] == SPEC['provider_model_sha256']
    assert 'tailb78697' not in json.dumps(document) and ':30610' not in json.dumps(document)
    # Thresholds belong to the profile; the identity pins what the model consumes and how.
    assert 'confidence_threshold' not in document and document['input'] == 'evidence_jpeg_q80'
    # The query is per frame, not part of the model identity: the default only seeds it.
    assert document['labels'] == 'query' and 'default_query' not in document['artifacts']
    monkeypatch.setattr(object_recognition, 'model_spec', lambda name: {**SPEC, 'default_query': ['a spoon']})
    assert recognition_identity(REMOTE)[0] == identity
    changed = {**SPEC, 'provider_model_sha256': '0' * 64}
    monkeypatch.setattr(object_recognition, 'model_spec', lambda name: changed)
    assert recognition_identity(REMOTE)[0] != identity


def test_tracked_provider_endpoint_is_valid():
    url, timeout = provider_endpoint(SPEC['provider'])
    assert url.startswith('http://') and 0 < timeout <= 10
    assert len(SPEC['provider_model_sha256']) == 64 and int(SPEC['provider_model_sha256'], 16) >= 0


@pytest.mark.parametrize('endpoint', [
    {'url': 'ftp://host:1', 'timeout_sec': 1}, {'url': 'http://host:1/v1', 'timeout_sec': 1},
    {'url': 'http://user:secret@host:1', 'timeout_sec': 1}, {'url': 'http://host:1', 'timeout_sec': 0},
    {'url': 'http://host:1', 'timeout_sec': 11}, {'url': 'http://host:1'}, {'url': 7, 'timeout_sec': 1},
])
def test_provider_endpoints_fail_closed(monkeypatch, tmp_path, endpoint):
    (tmp_path / 'perception_providers.yaml').write_text(json.dumps({'gpu': endpoint}))
    monkeypatch.setattr(object_recognition, 'config_directory', lambda: tmp_path)
    with pytest.raises(ValueError):
        provider_endpoint('gpu')


def test_runtime_follows_every_selected_recognizer():
    assert not object_recognition.needs_private_runtime(REMOTE, 'yolox')
    assert object_recognition.needs_private_runtime(REMOTE, 'rtdetr_v2_r18')
    assert object_recognition.selected_models({'model_name': REMOTE, 'fallback_model_name': ''}) == [REMOTE]


def wait(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('timed out')
        time.sleep(.02)


def test_detector_serves_fallback_frames_on_fallback_topics_and_retries_primary_later(monkeypatch):
    from lekiwi_rmf import object_detector

    calls = {'primary': 0, 'fallback': 0}

    def primary(image, query, *_):
        calls['primary'] += 1
        raise ProviderUnavailable('perception provider unreachable: refused')

    def fallback(image, query, *_):
        calls['fallback'] += 1
        return [[10, 10, 20, 20, .9, 'cup']]

    recognizers = {'grounding_dino_tiny_gpu': fake_recognizer(
                       primary, 'remote_grounding_dino', name='grounding_dino_tiny_gpu', labels=None,
                       open_vocabulary=True, default_query=DEFAULT, supports_concepts=True),
                   'rtdetr_v2_r18': fake_recognizer(fallback, 'rtdetr_v2', sha256='b' * 64, name='rtdetr_v2_r18',
                                                    labels=['cup', 'chair'])}
    monkeypatch.setattr(object_detector, 'Recognizer', lambda name, *args: recognizers[name])
    context = Context()
    rclpy.init(context=context)
    node = object_detector.ObjectDetector(context=context, parameter_overrides=[
        Parameter('model_name', value='grounding_dino_tiny_gpu'), Parameter('fallback_model_name', value='rtdetr_v2_r18'),
        Parameter('max_rate_hz', value=10.)])
    peer = Node('fallback_peer', context=context)
    profiles = []
    peer.create_subscription(String, '/robot/perception/profile', lambda msg: profiles.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    source = peer.create_publisher(Image, '/camera/front/image_raw', 1)
    primary_out, fallback_out, points = [], [], []
    peer.create_subscription(Detection2DArray, '/robot/perception/front/detections', primary_out.append, 10)
    peer.create_subscription(Detection2DArray, '/robot/perception/front/fallback/detections', fallback_out.append, 10)
    peer.create_subscription(PerceptionOperatingPoint, '/robot/perception/front/fallback/operating_point', points.append, 10)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def send():
        image, _ = capture()
        image.header.frame_id = 'front_camera_optical_frame'
        image.header.stamp = peer.get_clock().now().to_msg()
        source.publish(image)

    try:
        wait(lambda: source.get_subscription_count() == 1 and profiles)
        # Fallback frames carry their own identity and closed catalogue; they never vouch for primary-only labels.
        assert [(r['role'], r['model_sha256'], r['labels'], r['open_vocabulary']) for r in profiles[-1]['recognizers']] == [
            ('primary', 'a' * 64, ['a cup', 'a mug'], True), ('fallback', 'b' * 64, ['cup', 'chair'], False)]
        deadline = time.monotonic() + 20
        while len(fallback_out) < 3:
            send()
            time.sleep(.05)
            assert time.monotonic() < deadline
        assert not primary_out
        # One timeout per retry window, not one per frame.
        assert calls['primary'] == 1 and calls['fallback'] >= 3
        stats = node._stats['front']
        assert stats['served_by'] == 'fallback' and stats['fallback_published'] >= 3
        assert 'unreachable' in stats['primary_reason']
        # Each fallback frame says it was served degraded, under the fallback's own profile and no query.
        wait(lambda: points)
        assert 'unreachable' in points[-1].degraded_reason and not points[-1].query_json
        assert json.loads(points[-1].profile_json)['backend'] == 'rtdetr_v2'
        recognizers['grounding_dino_tiny_gpu'].predict = lambda image, query, *_: [[1, 1, 5, 5, .8, 'a cup']]
        node._primary_retry_at = 0.
        while not primary_out:
            send()
            time.sleep(.05)
            assert time.monotonic() < deadline
        assert node._stats['front']['primary_reason'] == ''
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(5)
        node.destroy_node()
        peer.destroy_node()
        context.try_shutdown()


def test_detector_without_fallback_rejects_unavailable_frames(monkeypatch):
    from lekiwi_rmf import object_detector

    def unavailable(image, query, *_):
        raise ProviderUnavailable('perception provider returned HTTP 503')

    monkeypatch.setattr(object_detector, 'Recognizer', lambda *args: fake_recognizer(
        unavailable, 'remote_grounding_dino', name='x', labels=None, open_vocabulary=True, default_query=DEFAULT))
    context = Context()
    rclpy.init(context=context)
    node = object_detector.ObjectDetector(context=context)
    try:
        assert node._fallback is None and set(node._outputs['front']) == {'primary'}
        image, _ = capture()
        image.header.frame_id = 'front_camera_optical_frame'
        image.header.stamp = node.get_clock().now().to_msg()
        node._frames['front'] = image
        node._infer()
        assert node._stats['front']['rejected'] == 1 and 'HTTP 503' in node._stats['front']['reason']
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_fallback_must_differ_from_primary(monkeypatch):
    from lekiwi_rmf import object_detector

    monkeypatch.setattr(object_detector, 'Recognizer', lambda *args: fake_recognizer(lambda *_: [], name='x'))
    context = Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match='must differ'):
            object_detector.ObjectDetector(context=context, parameter_overrides=[
                Parameter('model_name', value='yolox'), Parameter('fallback_model_name', value='yolox')])
    finally:
        context.try_shutdown()


def test_observations_carry_the_identity_of_the_recognizer_that_served_the_frame(tmp_path):
    import yaml
    from rclpy.executors import SingleThreadedExecutor
    from lekiwi_rmf.object_observation_node import ObjectObservations

    bindings = tmp_path / 'cameras.yaml'
    bindings.write_text(yaml.safe_dump({'astra_color': {
        'mount': 'base', 'image_topic': '/camera/test/image', 'optical_frame': 'camera_optical',
        'camera_info_topic': '/camera/test/info', 'depth_topic': '', 'calibration_file': ''}}))
    primary = recognition_identity(REMOTE)[0]
    fallback = recognition_identity('yolox')[0]
    context = Context()
    rclpy.init(context=context)
    collector = ObjectObservations(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'observations.db')),
        Parameter('model_name', value=REMOTE), Parameter('fallback_model_name', value='yolox'),
        Parameter('camera_ids', value=['astra_color']), Parameter('camera_profiles_file', value=str(bindings))])
    peer = Node('identity_peer', context=context)
    publishers = {suffix: (peer.create_publisher(Image, f'/robot/perception/astra_color{suffix}/source_image', 1),
                           peer.create_publisher(Detection2DArray, f'/robot/perception/astra_color{suffix}/detections', 1),
                           peer.create_publisher(PerceptionOperatingPoint,
                                                 f'/robot/perception/astra_color{suffix}/operating_point', 1))
                  for suffix in ('', '/fallback')}
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    executor.add_node(collector)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def publish(suffix):
        image, found = capture(count=1)
        image.header.stamp = peer.get_clock().now().to_msg()
        found.header = image.header
        for item in found.detections:
            item.header = image.header
        images, detections, points = publishers[suffix]
        if suffix:
            points.publish(operating_point(image, fallback, default_profile('yolox'), count=1,
                                           degraded_reason='perception provider unreachable: refused'))
        else:
            points.publish(operating_point(image, primary, PROFILE, DEFAULT, count=1,
                                           run_json=json.dumps({'run_id': 'r' * 32, 'inference_id': 'i' * 64})))
        detections.publish(found)
        images.publish(image)

    try:
        wait(lambda: all(pub.get_subscription_count() == 1 for group in publishers.values() for pub in group))
        publish('/fallback')
        wait(lambda: len(collector._store.query('{}')['observations']) == 1)
        time.sleep(.01)
        publish('')
        wait(lambda: len(collector._store.query('{}')['observations']) == 2)
        records = collector._store.query('{}')['observations']
        assert sorted(record['model_sha256'] for record in records) == sorted([primary, fallback])
        assert len({record['capture_id'] for record in records}) == 2
        served = {record['model_sha256']: record for record in records}
        assert served[primary]['vocabulary'] == 'open' and served[primary]['query'] == DEFAULT.document()
        assert served[primary]['profile']['revision'] == PROFILE.revision
        assert served[primary]['provider_run'] == {'run_id': 'r' * 32, 'inference_id': 'i' * 64}
        # The query labels nobody saw are typed abstentions with the information that would answer them: this
        # flat, dark fixture frame is low-quality input, not evidence that the objects are absent.
        assert served[primary]['input_quality']['flags'] == ['dark', 'blurred']
        missing = {item['label']: item for item in served[primary]['abstentions'] if item['label']}
        assert set(missing) >= {'a cup', 'a mug'} and missing['a mug']['reason'] == 'low_quality_input'
        assert missing['a mug']['next_requirement'] == 'more_light_or_another_camera'
        assert served[fallback]['degraded']['reason'] == 'provider_unavailable'
        assert served[fallback]['abstentions'][0]['next_requirement'] == 'retry_primary_provider'
        assert served[fallback]['vocabulary'] == 'closed' and 'query' not in served[fallback]
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        collector.destroy_node()
        peer.destroy_node()
        context.try_shutdown()


def test_provider_timings_are_read_only_when_complete_and_consistent(provider):
    recognizer = Recognizer(REMOTE, None)
    predict(recognizer, DEFAULT)
    round_trip, timings = recognizer.last_remote
    assert timings == pytest.approx({'decode': .001, 'queue': 0., 'inference': .5, 'total': .501}) and round_trip > 0
    for bad in [None, {}, {'timings_ms': {'decode': 1, 'queue': 0, 'inference': 5}},
                {'timings_ms': {'decode': 1, 'queue': 0, 'inference': -5, 'total': 9}},
                {'timings_ms': {'decode': 1, 'queue': 0, 'inference': 50, 'total': 9}},
                {'timings_ms': {'decode': 1, 'queue': 0, 'inference': 5, 'total': float('nan')}}]:
        assert provider_timings(bad) is None
    # A response without timings is still admitted on its operating point; only the split is unknown.
    provider.respond = lambda body: provider.admitted(body, timings_ms=None)
    assert predict(recognizer, DEFAULT)[0] and recognizer.last_remote[1] is None


def test_remote_answers_split_into_queue_inference_and_network_stages():
    from lekiwi_rmf.object_detector import ObjectDetector

    def stages(remote, queued=.2, duration=.7):
        state = SimpleNamespace(_detector=SimpleNamespace(last_remote=remote), _fallback=None,
                                _budget=LatencyBudget('perception', 2.))
        ObjectDetector._record_stages(state, 'primary', queued, duration)
        return {stage: value['p50_sec'] for stage, value in state._budget.summary()['stages'].items()}

    split = stages((.7, {'decode': .01, 'queue': .09, 'inference': .5, 'total': .6}))
    assert split == pytest.approx({'sensor_age': None, 'queue': .3, 'inference': .5, 'network': .1})
    # Without usable provider timings the whole call is inference and the network is not guessed.
    for remote in (None, (.4, None), (.4, {'decode': 0., 'queue': 0., 'inference': .5, 'total': .5})):
        assert stages(remote) == {'sensor_age': None, 'queue': .2, 'inference': .7, 'network': None}
