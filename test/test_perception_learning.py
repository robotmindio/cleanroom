"""Learned concepts, parts, replay of stored captures and the catalogue/contract suite (T-05, T-06, T-13, T-15).

These are contract tests with controlled backend outputs (spec 7.6): they prove that every label, part, concept
and profile crosses the system intact. They do not measure recognition accuracy, which needs real models
and labeled images and is reported separately.
"""

import base64
import hashlib
import json

import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from lekiwi_rmf import object_recognition, perception_concepts
from lekiwi_rmf.object_detector import detect
from lekiwi_rmf.object_observations import ObservationStore, artifact_pixels, capture_artifact, observation
from lekiwi_rmf.object_recognition import COCO_LABELS, ProviderUnavailable, Recognizer
from lekiwi_rmf.perception_concepts import calibrate, learn_concept, load_concept, reference, validate_concept
from lekiwi_rmf.perception_profiles import default_profile, make_profile
from lekiwi_rmf.perception_query import make_query, recorded_query
from lekiwi_rmf.perception_replay import replay
from test_object_observations import bare_recognizer, fake_recognizer
from test_perception_query import DEFAULT, Harness, open_recognizer
from test_remote_perception import PROFILE as REMOTE_PROFILE, REMOTE, FakeProvider

DIMENSIONS = 4


def unit(*values):
    vector = np.asarray(values, float)
    return [float(value) for value in vector / np.linalg.norm(vector)]


def scene(seed=0, size=(64, 96)):
    pixels = np.random.default_rng(seed).integers(40, 220, (*size, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode('.png', pixels)
    return encoded.tobytes()


def view(polarity, split, seed, region=(8, 8, 32, 32)):
    image = scene(seed)
    return {'image': image, 'artifact_sha256': hashlib.sha256(image).hexdigest(), 'region': list(region),
            'polarity': polarity, 'split': split, 'source': f'knowledge:region/{seed}'}


class Encoder:
    """Embeds a crop by its seed's polarity: positives near +x, negatives near +y; returns /v1/embed documents."""

    def __init__(self, vectors):
        self.vectors, self.calls = list(vectors), []

    def __call__(self, model, crops):
        self.calls.append((model, len(crops)))
        batch, self.vectors = self.vectors[:len(crops)], self.vectors[len(crops):]
        return {'model': model, 'model_sha256': 'e' * 64, 'modality': 'image', 'normalized': True,
                'dimensions': DIMENSIONS, 'embeddings': batch}


def teaching():
    views = ([view('positive', 'teach', seed) for seed in range(3)] + [view('negative', 'teach', 10)]
             + [view('positive', 'heldout', seed) for seed in (20, 21)] + [view('negative', 'heldout', 30)])
    vectors = [unit(1, .1 * i, 0, 0) for i in range(3)] + [unit(0, 1, 0, 0)] + [unit(1, .2, .1, 0), unit(1, 0, .3, 0)] \
        + [unit(.2, 1, 0, 0)]
    return {'name': 'Lata', 'prompts': ['a soda can', 'an aluminium can'], 'encoder': 'siglip2-base-patch16-224',
            'views': views}, Encoder(vectors)


def test_a_new_category_is_learned_from_byte_exact_views_without_retraining(tmp_path):
    manifest, encoder = teaching()
    record = learn_concept(manifest, encoder, store=tmp_path, now=1.)
    concept = record['concept']
    assert concept['name'] == 'lata' and concept['prompts'] == ['a soda can', 'an aluminium can']
    assert concept['encoder'] == {'model': 'siglip2-base-patch16-224', 'model_sha256': 'e' * 64}
    assert len(concept['exemplars']) == 3 and len(concept['negatives']) == 1
    # The match threshold admits both held-out positives and the held-out negative stays out.
    assert record['evaluation']['admitted_positives'] == 2 and record['evaluation']['false_admissions'] == 0
    assert record['views'][0]['source'] == 'knowledge:region/0' and 'detector weights unchanged' in record['method']
    assert encoder.calls == [('siglip2-base-patch16-224', 7)]
    assert load_concept(concept['revision'], tmp_path) == concept
    assert reference(concept).prompts == ('a soda can', 'an aluminium can')
    with pytest.raises(ValueError, match='revision'):
        validate_concept({**concept, 'match': {'min_similarity': .1, 'min_margin': 0.}})


@pytest.mark.parametrize('change,reason', [
    (lambda m: m['views'][0].update(artifact_sha256='0' * 64), 'artifact digest'),
    (lambda m: m.update(views=[v for v in m['views'] if v['split'] == 'teach']), 'held-out'),
    (lambda m: m['views'][0].update(region=[0, 0, 4, 4]), 'smaller than 8'),
])
def test_teaching_refuses_views_that_are_not_evidence_or_cannot_be_evaluated(tmp_path, change, reason):
    manifest, encoder = teaching()
    change(manifest)
    with pytest.raises(ValueError, match=reason):
        learn_concept(manifest, encoder, store=tmp_path)
    assert not list(tmp_path.iterdir())


def test_inseparable_views_call_for_a_dataset_and_training_instead_of_a_threshold():
    exemplars = [unit(1, 0, 0, 0)]
    match, evidence = calibrate([unit(1, 1, 0, 0)], [unit(1, .05, 0, 0)], exemplars, [])
    assert match is None and evidence['admitted_positives'] == 0
    match, _ = calibrate([unit(1, .1, 0, 0)], [unit(0, 1, 0, 0)], exemplars, [])
    assert match['min_margin'] == 0. and -1 < match['min_similarity'] <= 1


@pytest.fixture
def concept(tmp_path, monkeypatch):
    manifest, encoder = teaching()
    monkeypatch.setattr(perception_concepts, 'STORE', tmp_path / 'concepts')
    return learn_concept(manifest, encoder)['concept']


@pytest.mark.parametrize('labels', [['a cup'], []])
def test_a_query_names_learned_concepts_and_the_provider_proves_each_match(concept, monkeypatch, labels):
    fake = FakeProvider()
    monkeypatch.setattr(object_recognition, 'provider_endpoint', lambda name: (fake.url, 1.0))
    try:
        query = make_query(labels, concepts=[reference(concept)])
        assert query.prompt == ('a cup. ' if labels else '') + 'a soda can. an aluminium can.'
        assert query.reportable == (('a cup', 'lata') if labels else ('lata',))
        assert recorded_query(query.document()) == query
        assert query.document()['concepts'] == [{'name': 'lata', 'revision': concept['revision'],
                                                  'prompts': ['a soda can', 'an aluminium can']}]
        assert query.revision != make_query(['a cup']).revision
        match = {'name': 'lata', 'revision': concept['revision'], 'similarity': .93, 'negative_similarity': .1}
        fake.respond = lambda body: fake.admitted(body, detections=[
            {'box': [1.0, 2.0, 21.0, 32.0], 'score': .7, 'label': 'lata', 'concept': match}])
        artifact = capture_artifact(np.full((48, 64, 3), 90, np.uint8))
        rows = Recognizer(REMOTE, None).predict(artifact_pixels(artifact), query, REMOTE_PROFILE, artifact)
        assert rows == [[1.0, 2.0, 20.0, 30.0, .7, 'lata', {'concept': match}]]
        # The provider receives the full exemplar document the query revision pins.
        assert fake.requests[-1][1]['concepts'] == [concept]
        assert fake.requests[-1][1]['labels'] == labels
        with pytest.raises(ValueError, match='cannot match learned concepts'):
            bare = bare_recognizer('grounding_dino')
            bare.predict(np.zeros((40, 60, 3), np.uint8), query, default_profile('grounding_dino'))
    finally:
        fake.close()


def test_the_query_service_resolves_concepts_and_refuses_recognizers_without_an_encoder(concept, monkeypatch, tmp_path):
    harness = Harness(monkeypatch, tmp_path, open_recognizer(lambda *_: []))
    try:
        applied = harness.set_query(['a cup'], concept_revisions=[concept['revision']])
        assert applied.success
        assert harness.detector._query.reportable == ('a cup', 'lata')
        only = harness.set_query([], concept_revisions=[concept['revision']], expected=applied.revision)
        assert only.success and only.labels == []
        assert harness.detector._query.reportable == ('lata',)
        assert harness.detector._query.prompt == 'a soda can. an aluminium can.'
        harness.detector._detector.supports_concepts = False
        refused = harness.set_query([], concept_revisions=[concept['revision']], expected=only.revision)
        assert not refused.success and 'cannot match learned concepts' in refused.detail
        unknown = harness.set_query(['a cup'], concept_revisions=['f' * 64])
        assert not unknown.success and 'not in the concept store' in unknown.detail
    finally:
        harness.close()


def test_parts_are_tied_only_to_a_containing_whole_of_the_declared_label():
    """Spec 7.4: a part keeps its own region and a part_of link with evidence; without a referent it stays unassigned."""
    query = make_query(['mug', 'mug handle', 'table'], part_of=['', 'mug', ''])
    assert query.parts == (('mug handle', 'mug'),) and query.document()['parts'] == [{'label': 'mug handle', 'whole': 'mug'}]
    pixels = np.random.default_rng(2).integers(40, 220, (120, 160, 3), dtype=np.uint8)
    rows = [[10, 10, 60, 60, .9, 'mug'], [55, 30, 12, 20, .8, 'mug handle'], [120, 80, 10, 10, .7, 'mug handle'],
            [0, 0, 160, 120, .6, 'table']]
    recognizer = open_recognizer(lambda *_: rows)
    header = Header(stamp=Time(sec=3), frame_id='camera_optical')
    found, extras, _ = detect(recognizer, pixels, header, query, REMOTE_PROFILE)
    image = CvBridge().cv2_to_imgmsg(pixels, 'bgr8')
    image.header = header
    record, _ = observation(image, found, 'a' * 64, 'astra_color', query, profile=REMOTE_PROFILE, extras=extras)
    handle, stray = [item for item in record['detections'] if item['label'] == 'mug handle']
    assert handle['part_of']['label'] == 'mug' and handle['part_of']['containment'] >= .8
    assert record['detections'][handle['part_of']['index']]['label'] == 'mug'
    assert stray['part_of'] is None and stray['part_status'] == 'no_referent'
    assert 'part_of' not in record['detections'][0]
    for labels, part_of in [(['handle'], ['mug']), (['mug', 'handle'], ['', 'handle']), (['a', 'b', 'c'], ['b', 'c', ''])]:
        with pytest.raises(ValueError):
            make_query(labels, part_of=part_of)


def stored_observation(tmp_path, rows, profile=None, query=DEFAULT):
    pixels = np.random.default_rng(4).integers(30, 230, (60, 80, 3), dtype=np.uint8)
    artifact = capture_artifact(pixels)
    recognizer = open_recognizer(lambda image, *_: rows)
    header = Header(stamp=Time(sec=9), frame_id='camera_optical')
    profile = profile or REMOTE_PROFILE
    found, extras, _ = detect(recognizer, artifact_pixels(artifact), header, query, profile, artifact)
    image = CvBridge().cv2_to_imgmsg(pixels, 'bgr8')
    image.header = header
    record, evidence = observation(image, found, recognizer.sha256, 'astra_color', query, profile=profile,
                                   extras=extras, input_sha256=hashlib.sha256(artifact).hexdigest())
    store = ObservationStore(tmp_path / 'store.db')
    store.put(record, evidence)
    stored = store.query(json.dumps({'observation_ids': [record['id']]}))['observations'][0]
    data, _ = store.evidence(record['id'])
    store.close()
    return stored, data, recognizer


def test_a_stored_capture_replays_byte_exact_and_another_operating_point_is_a_new_inference(tmp_path):
    rows = [[10, 10, 20, 20, .9, 'a cup']]
    record, artifact, recognizer = stored_observation(tmp_path, rows)
    assert record['evidence']['model_input'] is True
    seen = []
    recognizer.predict = lambda image, query, profile, data: seen.append(data) or [row for row in rows
                                                                                    if row[4] > profile.get('box_threshold')]
    same = replay(record, artifact, recognizer)
    assert same['id'] == record['id'] and same['same_operating_point'] and same['reproduced'] is True
    assert seen == [artifact]
    strict = make_profile('remote_grounding_dino', REMOTE_PROFILE.parameters | {'box_threshold': .95})
    other = replay(record, artifact, recognizer, profile=strict)
    assert other['id'] != record['id'] and not other['same_operating_point'] and other['reproduced'] is None
    assert other['capture_id'] == record['capture_id'] and other['detections'] == []
    spoons = replay(record, artifact, recognizer, query=make_query(['spoon']))
    assert spoons['id'] not in (record['id'], other['id']) and spoons['query']['labels'] == ['spoon']
    # A model whose output drifted under the same operating point does not reproduce the original.
    rows[0][4] = .8
    assert replay(record, artifact, recognizer)['reproduced'] is False
    with pytest.raises(ValueError, match='byte-exact'):
        replay(record, artifact + b'\0', recognizer)
    assert replay({**record, 'evidence': {**record['evidence'], 'model_input': False}}, artifact, recognizer)['reproduced'] is None


def test_every_catalogue_label_crosses_detector_observation_store_and_query(tmp_path):
    """T-05: every native label (including spaces and the non-consecutive COCO ids) and open labels with
    hyphens, apostrophes and digits survive end to end; no stage filters to `cup`."""
    open_labels = ["kid's cup", 'usb-c cable', 'phillips screwdriver', 'size 10 wrench', 'taza']
    store = ObservationStore(tmp_path / 'store.db')
    stored = set()
    try:
        closed = fake_recognizer(None, labels=list(COCO_LABELS))
        for start in range(0, len(COCO_LABELS), 20):
            batch = COCO_LABELS[start:start + 20]
            closed.predict = lambda *_, batch=batch: [[i * 3, 0, 2, 2, .9, label] for i, label in enumerate(batch)]
            stored |= ingest(store, closed, None, default_profile('yolox'), start)
        query = make_query(open_labels)
        primary = open_recognizer(lambda *_: [[i * 3, 0, 2, 2, .9, label] for i, label in enumerate(query.labels)])
        stored |= ingest(store, primary, query, REMOTE_PROFILE, 100)
    finally:
        store.close()
    assert stored == set(COCO_LABELS) | set(open_labels)


def ingest(store, recognizer, query, profile, stamp):
    pixels = np.random.default_rng(stamp).integers(30, 230, (40, 100, 3), dtype=np.uint8)
    header = Header(stamp=Time(sec=stamp + 1), frame_id='camera_optical')
    found, extras, _ = detect(recognizer, pixels, header, query, profile)
    image = CvBridge().cv2_to_imgmsg(pixels, 'bgr8')
    image.header = header
    record, evidence = observation(image, found, recognizer.sha256, 'astra_color', query, profile=profile, extras=extras)
    store.put(record, evidence)
    result = store.query(json.dumps({'observation_ids': [record['id']]}))['observations'][0]
    return {item['label'] for item in result['detections']}


@pytest.mark.parametrize('width,height', [(320, 240), (640, 480), (1280, 720), (1920, 1080)])
def test_letterboxed_boxes_return_to_source_pixels_at_every_resolution(width, height):
    """T-06: the closed backend sees an RGB, letterboxed 640x640 input and its boxes map back to the source."""
    recognizer = bare_recognizer('yolox')
    received = []
    scale = min(640 / width, 640 / height)

    def infer(padded):
        received.append(padded)
        return np.array([[100 * scale, 50 * scale, 40 * scale, 30 * scale, .9, 41]], np.float32)
    recognizer.model = type('Model', (), {'infer': staticmethod(infer)})()
    pixels = np.zeros((height, width, 3), np.uint8)
    pixels[..., 0] = 255  # Blue in BGR: the model must receive it in the red-last RGB order it was trained on.
    rows = recognizer.predict(pixels)
    assert received[0].shape == (640, 640, 3) and received[0][0, 0, 2] == 255 and received[0][0, 0, 0] == 0
    # Padding sits right/bottom with the YOLOX fill value.
    assert received[0][-1, -1, 0] == 114 or (width, height) == (640, 640)
    x, y, w, h, score, label, _ = rows[0]
    assert label == 'cup' and np.allclose([x, y, w, h], [100, 50, 40, 30], atol=1e-3)


def test_remote_boxes_stay_in_source_pixels_and_need_the_echoed_size(monkeypatch):
    fake = FakeProvider()
    monkeypatch.setattr(object_recognition, 'provider_endpoint', lambda name: (fake.url, 1.0))
    try:
        for size in [(240, 320), (480, 640)]:
            artifact = capture_artifact(np.full((*size, 3), 80, np.uint8))
            rows = Recognizer(REMOTE, None).predict(artifact_pixels(artifact), DEFAULT, REMOTE_PROFILE, artifact)
            assert rows[0][:4] == [10.0, 12.0, 20.0, 28.0]
            sent = cv2.imdecode(np.frombuffer(base64.b64decode(fake.requests[-1][1]['image']), np.uint8), cv2.IMREAD_COLOR)
            assert sent.shape[:2] == size
        fake.respond = lambda body: fake.admitted(body, image={**fake.admitted(body)[1]['image'], 'height': 1})
        with pytest.raises(ProviderUnavailable):
            Recognizer(REMOTE, None).predict(artifact_pixels(artifact), DEFAULT, REMOTE_PROFILE, artifact)
    finally:
        fake.close()


def test_hard_conditions_are_diagnosed_per_frame_and_counted_in_the_corpus(tmp_path):
    """T-15: dark and blurred inputs abstain as low quality; the evaluator reports condition coverage."""
    from test_object_observations import PROFILE
    dark = np.full((40, 60, 3), 10, np.uint8)
    header = Header(stamp=Time(sec=1), frame_id='camera_optical')
    found, extras, _ = detect(open_recognizer(lambda *_: []), dark, header, DEFAULT, REMOTE_PROFILE)
    image = CvBridge().cv2_to_imgmsg(dark, 'bgr8')
    image.header = header
    record, _ = observation(image, found, 'a' * 64, 'astra_color', DEFAULT, profile=REMOTE_PROFILE, extras=extras)
    assert record['input_quality']['flags'] == ['dark', 'blurred']
    assert {item['reason'] for item in record['abstentions']} == {'low_quality_input'}

    import importlib.util
    from pathlib import Path
    source = Path(__file__).parents[1] / 'scripts/evaluate-object-recognition.py'
    spec = importlib.util.spec_from_file_location('evaluate_object_recognition', source)
    evaluator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evaluator)
    scenes = []
    for index, conditions in enumerate([['dark'], ['occlusion', 'transparent'], []]):
        name = f'{index}.png'
        cv2.imwrite(str(tmp_path / name), np.random.default_rng(index).integers(0, 255, (40, 60, 3), dtype=np.uint8))
        scenes.append({'scene_id': name, 'split': 'heldout', 'image': name, 'conditions': conditions,
                       'objects': [{'label': 'cup', 'bbox': [5, 5, 10, 10]}]})
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps(scenes))
    recognizer = fake_recognizer(lambda image, query, profile, artifact: [[5, 5, 10, 10, .9, 'cup']], identity={})
    report = evaluator.evaluate(manifest, recognizer=recognizer)
    assert report['conditions']['dark'] == 1 and report['conditions']['occlusion'] == 1
    assert report['conditions']['overhead'] == 0 and 'overhead' in report['missing_hard_conditions']
    assert report['profile'] == PROFILE.document() and report['scenes'][0]['conditions'] == ['dark']
    assert report['metrics']['cup']['tp'] == 3
    manifest.write_text(json.dumps([{**scenes[0], 'conditions': ['Dark!']}]))
    with pytest.raises(ValueError, match='conditions'):
        evaluator.load_scenes(manifest)
