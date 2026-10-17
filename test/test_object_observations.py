"""Image contract, quota, corruption and replay cases; no physical acceptance."""

import copy
import importlib.util
import json
from pathlib import Path
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


MODEL_SHA256 = recognition_identity('yolox')[0]

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


def document(count=1, stamp=1):
    return observation(*capture(count, stamp), MODEL_SHA256, 'astra_color')


@pytest.mark.parametrize('count', [0, 1, 2])
def test_contract_preserves_no_match_single_and_ambiguous_candidates(count):
    record, jpeg = document(count)
    assert len(record['detections']) == count
    assert record['grounding'] == 'image_only' and record['position'] is None
    assert record['stamp'] == {'sec': 1, 'nanosec': 0}
    assert record['evidence']['size'] == len(jpeg)
    assert cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape == (40, 60, 3)
    assert all(item['blue_fraction'] == 1 for item in record['detections'])


def test_color_measurement_does_not_label_red_as_blue():
    record, _ = observation(*capture(blue=False), MODEL_SHA256, 'astra_color')
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
        observation(image, detections, MODEL_SHA256, 'astra_color')


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


def test_recognition_uses_source_coordinates_and_keeps_different_cups():
    from lekiwi_rmf.object_detector import detect

    recognizer = SimpleNamespace(predict=lambda _: np.array([[32, 32, 64, 64, .9, 41],
                                                           [160, 32, 32, 32, .8, 41],
                                                           [0, 0, 50, 50, .9, 0]]))
    result = detect(recognizer, np.zeros((320, 320, 3), np.uint8), Header())
    assert len(result.detections) == 2
    assert result.detections[0].bbox.size_x == 64
    assert result.detections[0].results[0].hypothesis.class_id == 'cup'
    assert all(not item.id for item in result.detections)


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
    monkeypatch.setattr(evaluator, 'Recognizer', lambda *args: SimpleNamespace(
        predict=lambda image: [], sha256=MODEL_SHA256, identity={}))
    result = evaluator.evaluate(manifest, model)
    assert not result['passed'] and not result['gates']['heldout_scenes']
    assert result['metrics']['cup']['fn'] == 1 and result['scenes'][0]['scene_id'] == 'test-fixture'
    assert result['scenes'][0]['cup'] == (0, 0, 1) and result['scenes'][0]['predictions'] == []
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
        return SimpleNamespace(predict=lambda pixels: [], sha256=MODEL_SHA256, identity={})
    monkeypatch.setattr(evaluator, 'Recognizer', change_image)
    with pytest.raises(ValueError, match='changed after dataset validation'):
        evaluator.evaluate(manifest, model)


def test_nonblue_mug_miss_is_not_hidden_by_blue_goal_metrics(evaluator):
    cups = [{'bbox': [10, 10, 20, 20], 'blue': False}]
    result = evaluator.scene_metrics([], cups, .35)
    assert result['cup'] == (0, 0, 1)
    assert result['blue_goal'] == (0, 0, 0)
    detection = {'bbox': cups[0]['bbox'], 'blue_fraction': .5}
    result = evaluator.scene_metrics([detection], cups, .35)
    assert result['cup'] == (1, 0, 0)
    assert result['blue_attribute'] == [0, 1, 0, 0]
    assert result['goal_outcomes']['blue']['false_verified']


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
    cups = [{'bbox': [0, 0, 10, 10], 'blue': True}, {'bbox': [20, 0, 10, 10], 'blue': True}]
    result = evaluator.scene_metrics([{'bbox': cups[0]['bbox'], 'blue_fraction': 1}], cups, .35)
    assert result['goal_outcomes']['blue']['false_verified'] and not result['goal_outcomes']['blue']['correct']
    assert result['cup'] == (1, 0, 1)


def test_ambiguous_nonblue_cup_goal_is_scored_independently(evaluator):
    cups = [{'bbox': [0, 0, 10, 10], 'blue': False}, {'bbox': [20, 0, 10, 10], 'blue': False}]
    result = evaluator.scene_metrics([{'bbox': cups[0]['bbox'], 'blue_fraction': 0}], cups, .35)
    assert result['goal_outcomes']['cup']['false_verified']
    assert not result['goal_outcomes']['blue']['false_verified']
