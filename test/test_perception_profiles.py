"""Proposal profiles: specification, owned atomic application, calibration studies and reversion (T-09..T-12)."""

import json
from pathlib import Path
import re
import threading
import time

import pytest

from lekiwi_rmf import perception_study
from lekiwi_rmf.object_recognition import ProviderUnavailable, suppress
from lekiwi_rmf.perception_profiles import (
    PARAMETERS, default_profile, make_profile, recorded_profile, specification, tracked_profiles)
from lekiwi_rmf.perception_study import admitted_candidate, paired_bootstrap, run_study, save
from test_perception_query import DEFAULT, Harness, closed_recognizer, open_recognizer, wait


def test_fiber_stationary_profile_matches_the_tracked_camera_and_proposal():
    from lekiwi_rmf.object_cameras import camera_bindings

    root = Path(__file__).resolve().parents[1]
    profile = json.loads((root / 'config/fiber_perception_profile.json').read_text())
    assert profile['camera_sha256'] == camera_bindings(['front'])['front']['sha256']
    assert profile['profile_revision'] == tracked_profiles()[1]['remote_grounding_dino']['near_field'].revision
    assert not profile['enabled'] and not profile['allow_search'] and not profile['views']
    assert profile['max_evidence_age_sec'] == profile['confirmation_window_sec'] == 2

OPEN = default_profile('remote_grounding_dino')


def test_tracked_profiles_cover_every_backend_and_keep_the_established_operating_point():
    default, profiles = tracked_profiles()
    assert default == 'standard' and set(profiles) == set(PARAMETERS)
    assert profiles['yolox'][default].parameters == {'score_threshold': .5, 'nms_iou': .5, 'max_detections': 20}
    assert OPEN.parameters == {'box_threshold': .5, 'text_threshold': .25, 'nms_iou': .5, 'nms_scope': 'prompt',
                               'max_detections': 20, 'shortest_edge': 800, 'proposal_floor': .3}
    # Each parameter is specified: type, range, unit, cost and meaning; nothing is an implicit shared 0.5.
    for backend in PARAMETERS:
        for name, rule in specification(backend)['parameters'].items():
            assert {'type', 'unit', 'cost', 'semantics'} <= set(rule), (backend, name)


def test_revision_is_the_content_not_the_name():
    renamed = make_profile('remote_grounding_dino', OPEN.parameters, 'renamed')
    assert renamed.revision == OPEN.revision and renamed.document()['name'] == 'renamed'
    stricter = make_profile('remote_grounding_dino', OPEN.parameters | {'box_threshold': .6})
    assert stricter.revision != OPEN.revision
    # The same numbers under another backend are another profile.
    closed = default_profile('yolox').parameters
    assert make_profile('rtdetr_v2', closed).revision != make_profile('yolox', closed).revision


@pytest.mark.parametrize('backend,change,reason', [
    ('yolox', {'shortest_edge': 640}, 'fixed 640x640'),
    ('rtdetr_v2', {'text_threshold': .2}, 'phrase grounding'),
    ('grounding_dino', {'proposal_floor': .2}, 'retains no proposals'),
    ('remote_grounding_dino', {'score_threshold': .5}, 'box_threshold and text_threshold'),
    ('yolox', {'gamma': 1}, 'not a yolox profile parameter'),
    ('yolox', {'score_threshold': 0}, 'within (0'),
    ('yolox', {'max_detections': 21}, 'within [1, 20]'),
    ('yolox', {'max_detections': 2.5}, 'integer'),
    ('grounding_dino', {'nms_scope': 'image'}, 'one of label, prompt'),
    ('remote_grounding_dino', {'proposal_floor': .9}, 'must not exceed box_threshold'),
])
def test_unsupported_or_out_of_range_parameters_are_refused_with_their_reason(backend, change, reason):
    """T-10: a parameter the backend cannot honour is rejected with why, never silently ignored."""
    with pytest.raises(ValueError, match=re.escape(reason)):
        make_profile(backend, default_profile(backend).parameters | change)
    incomplete = dict(default_profile(backend).parameters)
    incomplete.pop(next(iter(incomplete)))
    with pytest.raises(ValueError, match='incomplete'):
        make_profile(backend, incomplete)


def test_a_recorded_profile_must_prove_its_revision():
    assert recorded_profile(OPEN.document(), 'remote_grounding_dino') == OPEN
    for document, backend in [(OPEN.document() | {'revision': '0' * 64}, 'remote_grounding_dino'),
                              (OPEN.document(), 'grounding_dino'),
                              (OPEN.document() | {'parameters': OPEN.parameters | {'box_threshold': .7}}, 'remote_grounding_dino')]:
        with pytest.raises(ValueError):
            recorded_profile(document, backend)


def test_suppression_follows_an_identified_scope_and_keeps_the_suppressed_as_evidence():
    """T-07: label scope never deletes another class; prompt scope merges synonyms and records them."""
    rows = [[0, 0, 20, 20, .9, 'cup'], [1, 1, 20, 20, .8, 'mug'], [2, 2, 20, 20, .7, 'cup'], [50, 50, 5, 5, .6, 'mug']]
    by_label = suppress(rows, 'label', .5)
    assert [(row[5], row[4]) for row in by_label] == [('cup', .9), ('mug', .8), ('mug', .6)]
    assert all(row[6] == {} for row in by_label)
    by_prompt = suppress(rows, 'prompt', .5)
    assert [(row[5], row[4]) for row in by_prompt] == [('cup', .9), ('mug', .6)]
    assert by_prompt[0][6] == {'alternatives': [{'label': 'mug', 'score': .8}]}
    assert len(suppress(rows, 'prompt', 1.)) == 4


def test_negative_query_stays_negative_instead_of_forcing_a_detection(monkeypatch, tmp_path):
    """T-09: candidates below the profile are discarded and the frame abstains with a typed reason; an
    untracked, unevaluated lower threshold cannot be applied to force the wanted answer."""
    import numpy as np
    from cv_bridge import CvBridge
    from lekiwi_rmf.object_detector import detect
    from lekiwi_rmf.object_observations import observation
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time

    weak = [[5, 5, 20, 20, .31, 'a mug']]
    recognizer = open_recognizer(lambda image, query, profile, artifact: [
        row for row in weak if row[4] > profile.get('box_threshold')])
    pixels = np.random.default_rng(1).integers(60, 200, (48, 64, 3), dtype=np.uint8)
    header = Header(stamp=Time(sec=5), frame_id='camera_optical')
    found, extras, _ = detect(recognizer, pixels, header, DEFAULT, OPEN)
    image = CvBridge().cv2_to_imgmsg(pixels, 'bgr8')
    image.header = header
    record, _ = observation(image, found, 'a' * 64, 'astra_color', DEFAULT, profile=OPEN, extras=extras)
    assert record['detections'] == [] and record['input_quality']['flags'] == []
    assert [(item['label'], item['reason']) for item in record['abstentions']] == [
        ('a cup', 'no_candidate'), ('a mug', 'no_candidate')]
    assert record['abstentions'][0]['next_requirement'] == 'another_viewpoint_or_memory_query'
    monkeypatch.setattr(perception_study, 'STORE', tmp_path / 'studies')
    harness = Harness(monkeypatch, tmp_path, recognizer)
    try:
        lowered = harness.apply_profile(expected_revision=OPEN.revision, evidence_id='0' * 64,
                                        parameters_json=json.dumps(OPEN.parameters | {'box_threshold': .3}))
        assert not lowered.success and 'no recorded study' in lowered.detail
        assert harness.detector._profiles['primary'] == OPEN
    finally:
        harness.close()


class Gate:
    """A remote-like recognizer double whose first frame blocks until released."""

    def __init__(self):
        self.entered, self.release, self.seen = threading.Event(), threading.Event(), []

    def __call__(self, image, query, profile, artifact):
        self.seen.append((image.shape, profile))
        if image.shape[:2] != (48, 64) and len([s for s in self.seen if s[0][:2] != (48, 64)]) == 1:
            self.entered.set()
            self.release.wait(5)
        return [[10, 10, 20, 20, .9, query.labels[0]]]


def test_profile_applies_atomically_with_expected_revision_and_never_relabels_a_frame_in_flight(monkeypatch, tmp_path):
    """T-10/T-11: the candidate is verified before the swap, the response reads back the active profile, the
    frame in flight keeps its own, and the next frame carries the new revision into its observation."""
    gate = Gate()
    harness = Harness(monkeypatch, tmp_path, open_recognizer(gate), collector=True)
    strict = make_profile('remote_grounding_dino', OPEN.parameters | {'box_threshold': .6}, 'strict')
    _, profiles = tracked_profiles()
    profiles['remote_grounding_dino']['strict'] = strict
    from lekiwi_rmf import object_detector
    monkeypatch.setattr(object_detector, 'tracked_profiles', lambda: ('standard', profiles))
    try:
        wait(harness.observations.service_is_ready)
        first = harness.send()
        assert gate.entered.wait(5)
        missing = harness.apply_profile(profile_name='strict')
        assert not missing.success and 'expected_revision is required' in missing.detail
        stale = harness.apply_profile(profile_name='strict', expected_revision='0' * 64)
        assert not stale.success and 'changed' in stale.detail and stale.revision == OPEN.revision
        unknown = harness.apply_profile(profile_name='lenient', expected_revision=OPEN.revision)
        assert not unknown.success and 'not a tracked' in unknown.detail
        # The candidate's probe waits for the recognizer the frame in flight holds; nothing swaps meanwhile.
        pending = harness.profile_client.call_async(object_detector.ApplyPerceptionProfile.Request(
            profile_name='strict', expected_revision=OPEN.revision))
        time.sleep(.3)
        assert not pending.done() and harness.detector._profiles['primary'] == OPEN
        gate.release.set()
        wait(pending.done, 15)
        applied = pending.result()
        assert applied.success and applied.revision == strict.revision and applied.previous_revision == OPEN.revision
        assert json.loads(applied.profile_json) == strict.document()
        # The probe ran exactly the candidate on a synthetic capture before the swap.
        assert any(shape[:2] == (48, 64) and profile == strict for shape, profile in gate.seen)
        wait(lambda: harness.profiles[-1]['recognizers'][0]['profile']['revision'] == strict.revision)
        wait(lambda: len(harness.records()) == 1)
        record, = harness.records()
        assert record['stamp'] == {'sec': first.stamp.sec, 'nanosec': first.stamp.nanosec}
        assert record['profile'] == OPEN.document()
        time.sleep(.05)
        harness.send()
        wait(lambda: len(harness.records()) == 2)
        latest = harness.records()[-1]
        assert latest['profile']['revision'] == strict.revision and latest['capture_id'] != record['capture_id']
        assert harness.detector._stats['astra_color']['profile_revision'] == strict.revision
        # Reverting restores the exact earlier revision and keeps history.
        reverted = harness.apply_profile(revert_to_revision=OPEN.revision, expected_revision=strict.revision)
        assert reverted.success and reverted.revision == OPEN.revision
        assert [profile.revision for profile in harness.detector._history['primary']] == [OPEN.revision, strict.revision]
        again = harness.apply_profile(revert_to_revision='f' * 64, expected_revision=OPEN.revision)
        assert not again.success and 'history' in again.detail
    finally:
        gate.release.set()
        harness.close()


def test_a_candidate_that_fails_its_probe_leaves_the_previous_profile_active(monkeypatch, tmp_path):
    """T-12: a failed load or health check neither swaps nor announces the candidate."""
    def predict(image, query, profile, artifact):
        if profile.get('box_threshold') == .6:
            raise ProviderUnavailable('perception provider returned HTTP 503 profile_unhealthy')
        return []

    harness = Harness(monkeypatch, tmp_path, open_recognizer(predict), closed_recognizer(lambda *_: []))
    strict = make_profile('remote_grounding_dino', OPEN.parameters | {'box_threshold': .6}, 'strict')
    _, profiles = tracked_profiles()
    profiles['remote_grounding_dino']['strict'] = strict
    from lekiwi_rmf import object_detector
    monkeypatch.setattr(object_detector, 'tracked_profiles', lambda: ('standard', profiles))
    try:
        wait(lambda: harness.profiles)
        announced = len(harness.profiles)
        failed = harness.apply_profile(profile_name='strict', expected_revision=OPEN.revision)
        assert not failed.success and 'profile_unhealthy' in failed.detail
        assert failed.revision == OPEN.revision and harness.detector._profiles['primary'] == OPEN
        time.sleep(.2)
        assert len(harness.profiles) == announced
        assert not harness.detector._history['primary']
        # The fallback has its own profile, applied by role.
        fallback = harness.apply_profile(role='fallback', profile_name='standard',
                                         expected_revision=default_profile('yolox').revision)
        assert not fallback.success and 'already in force' in fallback.detail
        nobody = harness.apply_profile(role='wrist', profile_name='standard', expected_revision=OPEN.revision)
        assert not nobody.success and 'no wrist recognizer' in nobody.detail
    finally:
        harness.close()


def scenes(gains, labels=('cup',)):
    """Evaluator scene results: `gains[group]` true marks scenes the candidate gets right and the current misses."""
    current, candidate = [], []
    for index, better in enumerate(gains):
        base = {'scene_id': str(index), 'scene_group': f'g{index}', 'conditions': ['dark'] if index % 2 else []}
        current.append({**base, 'goals': {label: (0, 0, 1) if better else (1, 0, 0) for label in labels}})
        candidate.append({**base, 'goals': {label: (1, 0, 0) for label in labels}})
    return current, candidate


def fake_evaluator(results, manifest='m' * 64):
    calls = []

    def evaluate(profile, split):
        calls.append((profile.revision, split))
        return {'manifest_sha256': manifest, 'scenes': results(profile, split)}
    return evaluate, calls


def study(results, candidates, **kwargs):
    from types import SimpleNamespace
    evaluate, calls = fake_evaluator(results, **kwargs)
    recognizer = SimpleNamespace(name='grounding_dino_tiny_gpu', sha256='a' * 64)
    return run_study(evaluate, recognizer, OPEN, candidates, manifest_sha256='m' * 64, labels=['cup']), calls


CANDIDATES = [make_profile('remote_grounding_dino', OPEN.parameters | {'box_threshold': value}, f'c{index}')
              for index, value in enumerate((.4, .6))]


def test_study_searches_development_reads_heldout_once_and_selects_a_real_gain():
    current, better = scenes([True] * 30)

    def results(profile, split):
        return better if profile.revision == CANDIDATES[0].revision else current
    record, calls = study(results, CANDIDATES)
    assert record['decision'] == 'selected' and record['candidate']['revision'] == CANDIDATES[0].revision
    assert [split for _, split in calls] == ['development', 'development', 'heldout', 'heldout']
    assert calls[-2:] == [(OPEN.revision, 'heldout'), (CANDIDATES[0].revision, 'heldout')]
    assert record['heldout']['paired']['ci95'][0] > 0 and record['revert']['revision'] == OPEN.revision
    assert record['protocol']['bootstrap'] == {'unit': 'scene_group', 'resamples': 2000, 'seed': 20261019}
    # Per-label and per-condition results are kept, not just one aggregate.
    assert record['heldout']['candidate']['labels']['cup']['recall'] == 1
    assert record['heldout']['current']['conditions']['dark']['scenes'] == 15


def test_study_rejects_a_gain_inside_sampling_variation_and_a_regressing_label():
    current, better = scenes([True] + [False] * 29)
    record, _ = study(lambda profile, split: better if profile.revision == CANDIDATES[0].revision else current,
                      CANDIDATES)
    assert record['decision'] == 'rejected' and 'sampling variation' in record['reasons'][0]
    # A candidate that helps one label but destroys another is refused by the regression rule.
    cups, mugs = scenes([True] * 30, labels=('cup', 'mug'))
    for scene in mugs:
        scene['goals']['mug'] = (0, 0, 1)
    for scene in cups:
        scene['goals']['mug'] = (1, 0, 0)
    from types import SimpleNamespace
    evaluate, _ = fake_evaluator(lambda profile, split: mugs if profile.revision == CANDIDATES[0].revision else cups)
    record = run_study(evaluate, SimpleNamespace(name='x', sha256='a' * 64), OPEN, CANDIDATES,
                       manifest_sha256='m' * 64, labels=['cup', 'mug'])
    assert record['decision'] == 'rejected'
    assert {(item['name'], item['metric']) for item in record['heldout']['regressions']} >= {('mug', 'recall')}


def test_study_refuses_a_corpus_that_changes_and_a_baseline_posing_as_a_candidate():
    current, _ = scenes([False] * 5)
    with pytest.raises(ValueError, match='corpus changed'):
        study(lambda profile, split: current, CANDIDATES, manifest='n' * 64)
    with pytest.raises(ValueError, match='baseline'):
        study(lambda profile, split: current, [OPEN])
    assert paired_bootstrap(current, current, ['cup'])['ci95'] == [0.0, 0.0]


def test_only_a_selected_study_admits_its_exact_candidate(monkeypatch, tmp_path):
    current, better = scenes([True] * 30)
    selected, _ = study(lambda profile, split: better if profile.revision == CANDIDATES[0].revision else current,
                        CANDIDATES)
    rejected, _ = study(lambda profile, split: current, CANDIDATES)
    for record in (selected, rejected):
        save(record, tmp_path)
    admitted = admitted_candidate(selected['id'], 'a' * 64, CANDIDATES[0], tmp_path)
    assert admitted.revision == CANDIDATES[0].revision and admitted.name == f'study-{selected["id"][:12]}'
    for evidence, model, candidate, reason in [
            (rejected['id'], 'a' * 64, CANDIDATES[0], 'rejected'),
            (selected['id'], 'b' * 64, CANDIDATES[0], 'another recognizer'),
            (selected['id'], 'a' * 64, CANDIDATES[1], 'not the candidate'),
            ('f' * 64, 'a' * 64, CANDIDATES[0], 'no recorded study')]:
        with pytest.raises(ValueError, match=reason):
            admitted_candidate(evidence, model, candidate, tmp_path)
    path = tmp_path / f'{selected["id"]}.json'
    tampered = json.loads(path.read_text())
    tampered['candidate']['parameters']['box_threshold'] = .1
    path.write_text(json.dumps(tampered))
    with pytest.raises(ValueError, match='does not match'):
        admitted_candidate(selected['id'], 'a' * 64, CANDIDATES[0], tmp_path)


def test_detector_applies_a_study_selected_candidate_through_its_service(monkeypatch, tmp_path):
    current, better = scenes([True] * 30)
    from types import SimpleNamespace
    from lekiwi_rmf.object_recognition import recognition_identity
    evaluate, _ = fake_evaluator(lambda profile, split: better if profile.revision == CANDIDATES[0].revision else current)
    record = run_study(evaluate, SimpleNamespace(name='grounding_dino_tiny_gpu', sha256=recognition_identity(
        'grounding_dino_tiny_gpu')[0]), OPEN, CANDIDATES, manifest_sha256='m' * 64, labels=['cup'])
    monkeypatch.setattr(perception_study, 'STORE', tmp_path / 'studies')
    save(record)
    harness = Harness(monkeypatch, tmp_path, open_recognizer(lambda *_: []))
    try:
        applied = harness.apply_profile(expected_revision=OPEN.revision, evidence_id=record['id'],
                                        parameters_json=json.dumps(CANDIDATES[0].parameters))
        assert applied.success and applied.revision == CANDIDATES[0].revision
        assert json.loads(applied.profile_json)['name'] == f'study-{record["id"][:12]}'
        both = harness.apply_profile(expected_revision=applied.revision, evidence_id=record['id'],
                                     parameters_json=json.dumps(CANDIDATES[0].parameters), profile_name='standard')
        assert not both.success and 'exactly one' in both.detail
    finally:
        harness.close()
