"""Image contract, quota, corruption and replay cases; no physical acceptance."""

import copy
import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from lekiwi_rmf.object_recognition import recognition_identity, model_spec

from lekiwi_rmf.object_observations import ObservationStore, observation, query_parameters
from lekiwi_rmf.perception_profiles import default_profile
from lekiwi_rmf.perception_query import make_query


MODEL_SHA256 = recognition_identity('yolox')[0]
PROFILE = default_profile('yolox')


def bare_recognizer(backend):
    """A Recognizer without model files, for exercising one backend's pre- and postprocessing."""
    from lekiwi_rmf.object_recognition import OPEN_BACKENDS, Recognizer
    recognizer = object.__new__(Recognizer)
    recognizer.spec, recognizer.backend = {'backend': backend}, backend
    recognizer.open_vocabulary, recognizer.supports_concepts = backend in OPEN_BACKENDS, False
    recognizer.default_profile = default_profile(backend)
    recognizer.name = backend
    return recognizer

def capture(count=1, stamp=1, blue=True):
    pixels = np.zeros((40, 60, 3), np.uint8)
    pixels[:] = (255, 0, 0) if blue else (0, 0, 255)
    image = CvBridge().cv2_to_imgmsg(pixels, 'bgr8')
    image.header = Header(stamp=Time(sec=stamp), frame_id='camera_optical')
    detections = Detection2DArray(header=image.header)
    for _ in range(count):
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id, result.hypothesis.score = 'cup', .9
        item = Detection2D(header=image.header, results=[result])
        item.bbox.center.position.x, item.bbox.center.position.y = 20.0, 20.0
        item.bbox.size_x, item.bbox.size_y = 20.0, 20.0
        detections.detections.append(item)
    return image, detections


def fake_recognizer(predict, backend='yolox', **fields):
    """A recognizer double with the attributes the detector reads; `predict(image, query, profile, artifact)`."""
    values = {'sha256': 'a' * 64, 'name': backend, 'labels': ['cup'], 'open_vocabulary': False, 'default_query': None,
              'supports_concepts': False, 'backend': backend, 'default_profile': default_profile(backend)}
    return SimpleNamespace(predict=predict, **{**values, **fields})


def operating_point(image, model=MODEL_SHA256, profile=PROFILE, query=None, extras=None, **fields):
    """The detector's per-frame operating point for `image`, as the collector receives it."""
    import hashlib
    from lekiwi_rmf.msg import PerceptionOperatingPoint
    from lekiwi_rmf.object_observations import capture_artifact
    pixels = CvBridge().imgmsg_to_cv2(image, 'bgr8')
    count = fields.pop('count', None)
    message = PerceptionOperatingPoint(
        header=image.header, model_sha256=model, profile_json=json.dumps(profile.document()),
        query_json=json.dumps(query.document()) if query is not None else '',
        input_sha256=hashlib.sha256(capture_artifact(pixels)).hexdigest(),
        detections_json=json.dumps(extras if extras is not None else [{}] * (count or 0)))
    for key, value in fields.items():
        setattr(message, key, value)
    return message


def document(count=1, stamp=1):
    return observation(*capture(count, stamp), MODEL_SHA256, 'astra_color', profile=PROFILE)


@pytest.mark.parametrize('count', [0, 1, 2])
def test_contract_preserves_no_match_single_and_ambiguous_candidates(count):
    record, jpeg = document(count)
    assert len(record['detections']) == count
    assert record['grounding'] == 'image_only' and record['position'] is None
    assert record['stamp'] == {'sec': 1, 'nanosec': 0}
    assert record['evidence']['size'] == len(jpeg)
    assert cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape == (40, 60, 3)
    assert all(item['blue_fraction'] == 1 for item in record['detections'])


def test_each_operating_point_on_one_capture_is_a_distinct_observation(tmp_path):
    first, jpeg = observation(*capture(), MODEL_SHA256, 'astra_color', profile=PROFILE)
    second, _ = observation(*capture(), 'b' * 64, 'astra_color', profile=PROFILE)
    stricter = default_profile('yolox').parameters | {'score_threshold': .6}
    from lekiwi_rmf.perception_profiles import make_profile
    third, _ = observation(*capture(), MODEL_SHA256, 'astra_color', profile=make_profile('yolox', stricter))
    # Another profile on the same capture and model is another inference, recorded with its revision.
    assert third['capture_id'] == first['capture_id'] and third['id'] not in (first['id'], second['id'])
    assert third['profile']['parameters']['score_threshold'] == .6 and first['profile']['revision'] == PROFILE.revision
    assert first['capture_id'] == second['capture_id'] and first['id'] != second['id']
    store = ObservationStore(tmp_path / 'store.db')
    try:
        store.put(first, jpeg)
        store.put(second, jpeg)
        assert {item['id'] for item in store.query('{"limit":5}')['observations']} == {first['id'], second['id']}
        with pytest.raises(ValueError, match='different content'):
            store.put({**first, 'detections': []}, jpeg)
    finally:
        store.close()


def test_store_serves_queries_while_another_thread_writes(tmp_path):
    # A node's executor writes while services and tests read; one shared sqlite3 connection must not race.
    store = ObservationStore(tmp_path / 'store.db')
    records = [document(stamp=stamp) for stamp in range(1, 201)]
    errors = []

    def write():
        try:
            for record, jpeg in records:
                store.put(record, jpeg)
        except Exception as error:  # noqa: BLE001 - surfaced by the assertion below
            errors.append(error)

    writer = threading.Thread(target=write)
    try:
        writer.start()
        while writer.is_alive():
            store.query('{"limit":50}')
        writer.join()
        assert not errors
        assert store.query('{"limit":1}')['latest_sequence'] == len(records)
    finally:
        store.close()


def test_operating_point_identity_records_native_label_output():
    _, document = recognition_identity('yolox')
    assert document['labels'] == 'native'
    # Thresholds are the profile's, recorded per frame; the model identity never carries them.
    assert 'confidence_threshold' not in document and document['postprocessing'] == 'profile'


def test_color_measurement_does_not_label_red_as_blue():
    record, _ = observation(*capture(blue=False), MODEL_SHA256, 'astra_color', profile=PROFILE)
    assert record['detections'][0]['blue_fraction'] == 0


@pytest.mark.parametrize('fault', ['stamp', 'frame', 'rotation', 'score', 'box', 'payload', 'count', 'dimensions'])
def test_invalid_boundary_data_is_rejected(fault):
    image, detections = capture()
    if fault == 'stamp':
        detections.header = copy.deepcopy(image.header)
        detections.header.stamp.sec = 2
    elif fault == 'frame':
        image.header.frame_id = ''
    elif fault == 'rotation':
        detections.detections[0].bbox.center.theta = .1
    elif fault == 'score':
        detections.detections[0].results[0].hypothesis.score = float('nan')
    elif fault == 'box':
        detections.detections[0].bbox.size_x = 200.0
    elif fault == 'payload':
        image.data = image.data[:-1]
    elif fault == 'count':
        detections.detections *= 21
    else:
        image.width = 2000
    with pytest.raises(ValueError):
        observation(image, detections, MODEL_SHA256, 'astra_color', profile=PROFILE)


@pytest.mark.parametrize('text', ['[]', '{"limit":true}', '{"limit":101}', '{"after_sequence":-1}',
                                  '{"after_sequence":1}', '{"observation_ids":["../secret"]}',
                                  '{"pose":[0,0]}', '[' * 1100 + '0' + ']' * 1100, ' ' * 4097])
def test_query_rejects_untrusted_input(text):
    with pytest.raises(ValueError):
        query_parameters(text)


def test_snapshot_survives_restart_and_capture_replay(tmp_path):
    path = tmp_path / 'observations.sqlite3'
    store = ObservationStore(path)
    record, jpeg = document()
    sequence = store.put(record, jpeg, now=100)
    assert store.put(record, jpeg, now=101) == sequence
    epoch = store.epoch
    store.close()
    store = ObservationStore(path)
    try:
        response = store.query('{}', now=101)
        assert response['epoch'] == epoch
        assert response['latest_sequence'] == sequence
        assert response['observations'][0]['id'] == record['id']
        assert store.evidence(record['id'], now=101)[0] == jpeg
        changed = {**record, 'width': 59}
        with pytest.raises(ValueError, match='different content'):
            store.put(changed, jpeg, now=101)
        assert path.stat().st_mode & 0o777 == 0o600
    finally:
        store.close()


def test_retention_reports_cursor_gap_and_evidence_expiration(tmp_path):
    store = ObservationStore(tmp_path / 'store.db', max_records=2, retention_sec=10)
    try:
        original, jpeg = document(stamp=1)
        store.put(original, jpeg, now=100)
        for stamp in (2, 3, 4):
            store.put(*document(stamp=stamp), now=100)
        cursor = json.dumps({'after_sequence': 1, 'epoch': store.epoch})
        result = store.query(cursor, now=100)
        assert result['cursor_gap'] and len(result['observations']) == 2
        with pytest.raises(ValueError, match='expired'):
            store.evidence(original['id'], now=100)
        assert store.query(cursor, now=111)['cursor_gap']
        assert not store.query('{}', now=111)['observations']
    finally:
        store.close()


def test_pinned_survivor_does_not_hide_evicted_records_from_cursor(tmp_path):
    store = ObservationStore(tmp_path / 'store.db', max_records=5)
    try:
        first, _ = document(stamp=1)
        store.put(*document(stamp=1), now=100)
        store.pin(json.dumps({'request_id': 'goal', 'observation_ids': [first['id']]}), now=100)
        for stamp in range(2, 6):
            store.put(*document(stamp=stamp), now=100)
        cursor = json.dumps({'after_sequence': 5, 'epoch': store.epoch})
        assert not store.query(cursor, now=100)['cursor_gap']
        for stamp in range(6, 21):
            store.put(*document(stamp=stamp), now=100)
        result = store.query(cursor, now=100)
        assert result['cursor_gap'] and result['observations'][0]['sequence'] > 6
    finally:
        store.close()


def test_full_database_file_evicts_instead_of_failing_every_capture(tmp_path):
    rng = np.random.default_rng(0)
    store = ObservationStore(tmp_path / 'store.db', max_bytes=8 * 1024**2)
    try:
        record, _ = document()
        for index in range(300):
            blob = rng.integers(0, 256, int(rng.integers(45000, 65000)), dtype=np.uint8).tobytes()
            store.put({**record, 'id': f'{index:064x}'}, blob, now=100)
        assert store.query('{"limit":1}', now=100)['latest_sequence'] == 300
        assert (tmp_path / 'store.db').stat().st_size <= 8 * 1024**2 * 5 // 4
    finally:
        store.close()


def test_pins_cannot_reserve_more_than_half_of_storage(tmp_path):
    store = ObservationStore(tmp_path / 'store.db', max_bytes=65536 * 4)
    try:
        record, _ = document()
        ids = [f'{index:064x}' for index in range(4)]
        for identifier in ids:
            store.put({**record, 'id': identifier}, bytes(60000), now=100)
        store.pin(json.dumps({'request_id': 'one', 'observation_ids': ids[:2]}), now=100)
        with pytest.raises(ValueError, match='half of the storage'):
            store.pin(json.dumps({'request_id': 'two', 'observation_ids': ids[2:3]}), now=100)
        store.put({**record, 'id': 'f' * 64}, bytes(60000), now=100)
    finally:
        store.close()


def test_new_database_epoch_rejects_old_cursor(tmp_path):
    first, second = ObservationStore(tmp_path / 'one.db'), ObservationStore(tmp_path / 'two.db')
    try:
        with pytest.raises(ValueError, match='epoch changed'):
            second.query(json.dumps({'epoch': first.epoch, 'after_sequence': 1}))
    finally:
        first.close()
        second.close()


def test_evidence_corruption_and_competing_writer_fail_clearly(tmp_path):
    path = tmp_path / 'store.db'
    store = ObservationStore(path)
    try:
        with pytest.raises(BlockingIOError):
            ObservationStore(path)
        record, jpeg = document()
        store.put(record, jpeg)
        with store._db:
            store._db.execute('UPDATE observations SET evidence=?', (b'corrupt',))
        with pytest.raises(RuntimeError, match='hash mismatch'):
            store.evidence(record['id'])
    finally:
        store.close()


def test_oversized_capture_does_not_evict_existing_evidence(tmp_path):
    store = ObservationStore(tmp_path / 'store.db', max_bytes=65536)
    try:
        record, jpeg = document()
        store.put(record, jpeg)
        with pytest.raises(ValueError, match='quota'):
            store.put({**record, 'id': 'a' * 64}, bytes(65536))
        assert store.evidence(record['id'])[0] == jpeg
    finally:
        store.close()


def test_recognition_publishes_every_native_label_in_source_coordinates():
    from lekiwi_rmf.object_detector import detect

    recognizer = SimpleNamespace(predict=lambda *_: [[32, 32, 64, 64, .9, 'cup'],
                                                    [160, 32, 32, 32, .8, 'cup'],
                                                    [0, 0, 50, 50, .7, 'potted plant']])
    result, extras, run = detect(recognizer, np.zeros((320, 320, 3), np.uint8), Header())
    assert extras == [{}, {}, {}] and run is None
    assert [item.results[0].hypothesis.class_id for item in result.detections] == ['cup', 'cup', 'potted plant']
    assert result.detections[0].bbox.size_x == 64
    assert all(not item.id for item in result.detections)


def test_cluttered_frame_keeps_the_highest_scores_instead_of_dropping_the_capture():
    from lekiwi_rmf.object_detector import detect

    rows = [[index, 0, 10, 10, index / 30, f'class {index}'] for index in range(1, 26)]
    result, _, _ = detect(SimpleNamespace(predict=lambda *_: rows), np.zeros((40, 60, 3), np.uint8), Header())
    scores = [item.results[0].hypothesis.score for item in result.detections]
    assert len(scores) == 20 and scores == sorted(scores, reverse=True) and min(scores) == 6 / 30


@pytest.mark.parametrize('label', ['', 41, 'x' * 129])
def test_recognition_rejects_missing_or_numeric_labels(label):
    from lekiwi_rmf.object_detector import detect

    with pytest.raises(ValueError, match='native label'):
        detect(SimpleNamespace(predict=lambda *_: [[0, 0, 10, 10, .9, label]]), np.zeros((40, 60, 3), np.uint8), Header())


def test_yolox_maps_every_output_class_to_its_native_coco_label():
    from lekiwi_rmf.object_recognition import COCO_LABELS

    recognizer = bare_recognizer('yolox')
    recognizer.model = SimpleNamespace(infer=lambda _: np.array(
        [[0, 0, 64, 64, .9, index] for index in range(80)], dtype=np.float32))
    rows = recognizer.predict(np.zeros((320, 640, 3), np.uint8))
    assert len(COCO_LABELS) == 80 and len(set(COCO_LABELS)) == 80
    assert [row[5] for row in rows] == list(COCO_LABELS) and COCO_LABELS[41] == 'cup'
    assert rows[0][2] == 64 / (640 / 640)
    # The profile's thresholds reach the graph's own class-aware NMS; no second suppression follows it.
    assert (recognizer.model.confThreshold, recognizer.model.nmsThreshold) == (.5, .5)


@pytest.mark.parametrize('backend,labels,kept', [
    ('rtdetr_v2', ['cup', 'bowl'], ['cup', 'bowl']), ('rtdetr_v2', ['cup', 'cup'], ['cup']),
    ('grounding_dino', ['a cup', 'mug'], ['a cup'])])
def test_transformer_nms_keeps_overlapping_classes_but_merges_synonymous_prompts(monkeypatch, backend, labels, kept):
    import contextlib
    import sys
    monkeypatch.setitem(sys.modules, 'PIL', SimpleNamespace(Image=SimpleNamespace(fromarray=lambda pixels: pixels)))
    result = {'boxes': np.array([[0., 0., 20., 20.], [1., 1., 20., 20.]]), 'scores': [.9, .8],
              'labels': [0, 1 if labels[1] != labels[0] else 0], 'text_labels': labels}

    class Inputs(dict):
        input_ids = None

    texts = []

    class Processor:
        def __call__(self, **kwargs):
            texts.append(kwargs.get('text'))
            return Inputs()

        def post_process_object_detection(self, *args, **kwargs):
            return [result]

        post_process_grounded_object_detection = post_process_object_detection

    recognizer = bare_recognizer(backend)
    query = make_query(['a cup', 'mug']) if recognizer.open_vocabulary else None
    recognizer.torch = SimpleNamespace(inference_mode=contextlib.nullcontext)
    recognizer.processor = Processor()
    recognizer.model = type('Model', (), {'__call__': lambda self, **kwargs: None,
                                          'config': SimpleNamespace(id2label=dict(enumerate(labels)))})()
    assert [row[5] for row in recognizer.predict(np.zeros((40, 60, 3), np.uint8), query)] == kept
    # The local Grounding DINO localizes exactly the frame's query.
    assert texts == ['a cup. mug.' if recognizer.open_vocabulary else None]


def test_nms_fallback_preserves_class_aware_suppression(monkeypatch):
    from lekiwi_rmf.opencv_yolox import YoloX

    monkeypatch.delattr(cv2.dnn, 'NMSBoxesBatched', raising=False)
    model = object.__new__(YoloX)
    model.confThreshold, model.nmsThreshold = .5, .5
    model.grids, model.expanded_strides = np.zeros((1, 3, 2)), np.ones((1, 3, 1))
    outputs = np.zeros((1, 3, 85), np.float32)
    outputs[:, :, :4] = [10, 10, np.log(10), np.log(10)]
    outputs[:, :, 4] = 1
    outputs[0, 0, 5], outputs[0, 1, 5], outputs[0, 2, 6] = .9, .8, .9
    result = model.postprocess(outputs)
    assert len(result) == 2 and set(result[:, -1]) == {0, 1}
    outputs[:] = 0
    assert model.postprocess(outputs).size == 0


def test_model_installer_rejects_existing_unpinned_artifact(tmp_path):

    source = Path(__file__).parents[1] / 'scripts/install-object-model.py'
    spec = importlib.util.spec_from_file_location('install_object_model', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / next(iter(model_spec('yolox')['files']))).write_bytes(b'not a model')
    with pytest.raises(ValueError, match='wrong hash'):
        module.install(tmp_path)


@pytest.fixture
def evaluator():
    source = Path(__file__).parents[1] / 'scripts/evaluate-object-recognition.py'
    spec = importlib.util.spec_from_file_location('evaluate_object_recognition', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_evaluation_matches_each_target_once_and_reports_uncertainty(evaluator):
    module = evaluator
    box = [10, 10, 10, 10]
    assert module.counts([box, box], [box]) == (1, 1, 0)
    assert module.counts([], [box]) == (0, 0, 1)
    assert module.counts([box], []) == (0, 1, 0)
    assert module.overlap(box, [30, 30, 10, 10]) == 0
    assert module.interval(0, 0) is None
    low, high = module.interval(20, 20)
    assert low < .9 and high == 1


def test_pin_preserves_evidence_and_reports_receipt(tmp_path):
    store = ObservationStore(tmp_path / 'store.db', max_records=2, retention_sec=10)
    first, jpeg = document(stamp=1)
    second, other = document(stamp=2)
    third, newest = document(stamp=3)
    store.put(first, jpeg, now=100)
    request = json.dumps({'request_id': 'goal-example', 'observation_ids': [first['id']]})
    store.pin(request, now=105)
    store.pin(request, now=106)
    store.put(second, other, now=111)
    store.put(third, newest, now=112)
    receipt = store.query(json.dumps({'observation_ids': [first['id']], 'pin_request_id': 'goal-example'}), now=112)
    assert receipt['pins'] == {first['id']: 115}
    assert store.evidence(first['id'], now=112)[0] == jpeg
    with pytest.raises(ValueError, match='different evidence'):
        store.pin(json.dumps({'request_id': 'goal-example', 'observation_ids': [third['id']]}), now=112)
    with pytest.raises(ValueError, match='expired'):
        store.evidence(first['id'], now=116)
    store.close()


def test_pin_is_atomic_and_cannot_evict_reserved_evidence(tmp_path):
    store = ObservationStore(tmp_path / 'store.db', max_records=1)
    first, jpeg = document()
    store.put(first, jpeg, now=100)
    with pytest.raises(ValueError, match='missing'):
        store.pin(json.dumps({'request_id': 'attempt', 'observation_ids': [first['id'], 'f' * 64]}), now=101)
    assert store.query(json.dumps({'observation_ids': [first['id']], 'pin_request_id': 'attempt'}), now=101)['pins'] == {}
    store.pin(json.dumps({'request_id': 'attempt', 'observation_ids': [first['id']]}), now=101)
    second, jpeg2 = document(stamp=2)
    with pytest.raises(ValueError, match='reserved'):
        store.put(second, jpeg2, now=102)
    assert store.evidence(first['id'], now=102)[0] == jpeg
    store.close()


@pytest.mark.parametrize('split,extension', [('heldout', '.png'), ('development', '.bmp')])
def test_evaluation_rejects_duplicate_content_with_new_scene_ids(tmp_path, evaluator, split, extension):
    pixels = np.full((40, 60, 3), 90, np.uint8)
    cv2.imwrite(str(tmp_path / 'one.png'), pixels)
    cv2.imwrite(str(tmp_path / ('two' + extension)), pixels)
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps([
        {'scene_id': 'first', 'split': 'heldout', 'image': 'one.png', 'cups': []},
        {'scene_id': 'second', 'split': split, 'image': 'two' + extension, 'cups': []}]))
    with pytest.raises(ValueError, match='duplicate image content'):
        evaluator.load_scenes(manifest)


def test_evaluation_validates_development_labels_before_inference(tmp_path, evaluator):
    cv2.imwrite(str(tmp_path / 'one.png'), np.zeros((40, 60, 3), np.uint8))
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps([{'scene_id': 'development', 'split': 'development',
        'image': 'one.png', 'cups': [{'bbox': [50, 0, 20, 10], 'blue': True}]}]))
    with pytest.raises(ValueError, match='exceeds its image'):
        evaluator.load_scenes(manifest)


def test_evaluation_reports_failed_scene_and_keeps_small_corpus_unqualified(tmp_path, evaluator, monkeypatch):
    import hashlib
    pixels = np.zeros((40, 60, 3), np.uint8)
    cv2.imwrite(str(tmp_path / 'one.png'), pixels)
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps([{'scene_id': 'test-fixture', 'split': 'heldout',
        'image': 'one.png', 'cups': [{'bbox': [10, 10, 20, 20], 'blue': True}]}]))
    model = tmp_path / 'fixture.onnx'
    model.write_bytes(b'fixture')
    monkeypatch.setattr(evaluator, 'Recognizer', lambda *args, **kwargs: SimpleNamespace(
        predict=lambda *_: [], sha256=MODEL_SHA256, identity={}, default_query=None, default_profile=PROFILE))
    result = evaluator.evaluate(manifest, model)
    assert not result['passed'] and not result['gates']['heldout_scenes']
    assert result['metrics']['cup']['fn'] == 1 and result['scenes'][0]['scene_id'] == 'test-fixture'
    assert result['scenes'][0]['goals']['cup'] == (0, 0, 1) and result['scenes'][0]['predictions'] == []
    assert result['labels'] == ['cup'] and result['blue_labels'] == ['cup'] and result['schema'] == 3
    assert result['manifest_sha256'] == hashlib.sha256(manifest.read_bytes()).hexdigest()


def test_evaluation_rejects_images_changed_after_preflight(tmp_path, evaluator, monkeypatch):
    image = tmp_path / 'one.png'
    cv2.imwrite(str(image), np.zeros((40, 60, 3), np.uint8))
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps([{'scene_id': 'test-fixture', 'split': 'heldout',
        'image': 'one.png', 'cups': []}]))
    model = tmp_path / 'fixture.onnx'
    model.write_bytes(b'fixture')
    def change_image(*args, **kwargs):
        cv2.imwrite(str(image), np.full((40, 60, 3), 255, np.uint8))
        return SimpleNamespace(predict=lambda *_: [], sha256=MODEL_SHA256, identity={}, default_query=None,
                               default_profile=PROFILE)
    monkeypatch.setattr(evaluator, 'Recognizer', change_image)
    with pytest.raises(ValueError, match='changed after dataset validation'):
        evaluator.evaluate(manifest, model)


def cups(*boxes, blue):
    return [{'label': 'cup', 'bbox': box, 'blue': blue} for box in boxes]


def test_nonblue_mug_miss_is_not_hidden_by_blue_goal_metrics(evaluator):
    expected = cups([10, 10, 20, 20], blue=False)
    result = evaluator.scene_metrics([], expected, .35, ['cup'], ['cup'])
    assert result['goals']['cup'] == (0, 0, 1)
    assert result['goals']['blue cup'] == (0, 0, 0)
    detection = {'label': 'cup', 'bbox': expected[0]['bbox'], 'blue_fraction': .5}
    result = evaluator.scene_metrics([detection], expected, .35, ['cup'], ['cup'])
    assert result['goals']['cup'] == (1, 0, 0)
    assert result['blue_attribute'] == [0, 1, 0, 0]
    assert result['goal_outcomes']['blue cup']['false_verified']


def test_evaluation_rejects_related_views_crossing_splits(tmp_path, evaluator):
    scenes = []
    for index, split in enumerate(['development', 'heldout']):
        name = f'{index}.png'
        cv2.imwrite(str(tmp_path / name), np.full((40, 60, 3), index, np.uint8))
        scenes.append({'scene_id': str(index), 'scene_group': 'same-arrangement',
                       'split': split, 'image': name, 'cups': []})
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps(scenes))
    with pytest.raises(ValueError, match='group crosses'):
        evaluator.load_scenes(manifest)


def test_ambiguous_scene_cannot_become_verified_by_missing_one_cup(evaluator):
    expected = cups([0, 0, 10, 10], [20, 0, 10, 10], blue=True)
    result = evaluator.scene_metrics([{'label': 'cup', 'bbox': expected[0]['bbox'], 'blue_fraction': 1}],
                                     expected, .35, ['cup'], ['cup'])
    assert result['goal_outcomes']['blue cup']['false_verified'] and not result['goal_outcomes']['blue cup']['correct']
    assert result['goals']['cup'] == (1, 0, 1)


def test_ambiguous_nonblue_cup_goal_is_scored_independently(evaluator):
    expected = cups([0, 0, 10, 10], [20, 0, 10, 10], blue=False)
    result = evaluator.scene_metrics([{'label': 'cup', 'bbox': expected[0]['bbox'], 'blue_fraction': 0}],
                                     expected, .35, ['cup'], ['cup'])
    assert result['goal_outcomes']['cup']['false_verified']
    assert not result['goal_outcomes']['blue cup']['false_verified']


def test_classes_are_scored_separately_and_unlabeled_classes_are_not_false_positives(evaluator):
    box = [0, 0, 10, 10]
    objects = [{'label': 'cup', 'bbox': box}, {'label': 'bowl', 'bbox': [20, 0, 10, 10]}]
    detections = [{'label': 'bowl', 'bbox': box, 'blue_fraction': 0},
                  {'label': 'vase', 'bbox': [40, 0, 10, 10], 'blue_fraction': 0}]
    result = evaluator.scene_metrics(detections, objects, .35, ['bowl', 'cup'], [])
    assert result['goals'] == {'bowl': (0, 1, 1), 'cup': (0, 0, 1)}
    assert [item['label'] for item in result['unscored_predictions']] == ['vase']
    assert 'blue cup' not in result['goals']


def test_multiclass_manifest_defines_vocabulary_and_keeps_legacy_cups(tmp_path, evaluator):
    for name, value in [('one.png', 0), ('two.png', 255)]:
        cv2.imwrite(str(tmp_path / name), np.full((40, 60, 3), value, np.uint8))
    manifest = tmp_path / 'scenes.json'
    manifest.write_text(json.dumps([
        {'scene_id': 'one', 'split': 'heldout', 'image': 'one.png', 'cups': [{'bbox': [0, 0, 5, 5], 'blue': True}]},
        {'scene_id': 'two', 'split': 'heldout', 'image': 'two.png',
         'objects': [{'label': 'potted plant', 'bbox': [0, 0, 5, 5]}]}]))
    _, selected, labels, blue_labels = evaluator.load_scenes(manifest)
    assert labels == ['cup', 'potted plant'] and blue_labels == ['cup']
    assert selected[0][0]['objects'] == [{'label': 'cup', 'bbox': [0, 0, 5, 5], 'blue': True}]
    for bad in [{'cups': [], 'objects': []}, {}, {'objects': [{'bbox': [0, 0, 5, 5]}]},
                {'objects': [{'label': 'cup', 'bbox': [0, 0, 5, 5], 'blue': 'yes'}]}]:
        manifest.write_text(json.dumps([{'scene_id': 'one', 'split': 'heldout', 'image': 'one.png', **bad}]))
        with pytest.raises(ValueError):
            evaluator.load_scenes(manifest)
