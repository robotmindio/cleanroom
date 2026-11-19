"""Threshold calibration: exact intervals, scene-group counting, cut fitting and the write gate.

Fixtures here are synthetic test inputs that exercise the procedure; they calibrate nothing.
"""

import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf import threshold_calibration as calibration

ROOT = Path(__file__).resolve().parents[1]


def test_exact_bounds_match_closed_forms():
    # With zero failures the one-sided bounds have closed forms (spec 27.3).
    assert calibration.upper_bound(0, 299) == pytest.approx(1 - .05 ** (1 / 299), abs=1e-9)
    assert calibration.upper_bound(0, 299) < .01 < calibration.upper_bound(0, 298)
    assert calibration.lower_bound(29, 29) == pytest.approx(.05 ** (1 / 29), abs=1e-9)
    assert calibration.lower_bound(29, 29) == pytest.approx(.9018, abs=1e-4)
    assert calibration.lower_bound(0, 10) == 0 and calibration.upper_bound(10, 10) == 1
    assert calibration.lower_bound(0, 0) is None and calibration.interval(3, 0) is None
    low, high = calibration.interval(5, 10)
    # Clopper-Pearson 95% for 5/10.
    assert low == pytest.approx(.18709, abs=1e-4) and high == pytest.approx(.81291, abs=1e-4)
    assert calibration.groups_needed(.9) == 29
    assert calibration.groups_needed(.9, failures=1) == 46
    assert calibration.groups_needed(.99) == 299


def test_a_group_fails_when_any_of_its_trials_is_missed():
    trials = [{'group': 'a', 'detected': True}, {'group': 'a', 'detected': False},
              {'group': 'b', 'detected': True}, {'group': 'b', 'detected': True}]
    result = calibration.tally(trials)
    assert (result['trials'], result['detected'], result['groups'], result['groups_all_detected']) == (4, 3, 2, 1)
    assert result['group_recall'] == .5


def scene(identifier, group, objects, predictions, quality=None, conditions=()):
    item = {'scene_id': identifier, 'scene_group': group, 'expected': objects, 'predictions': predictions,
            'conditions': list(conditions)}
    if quality is not None:
        item['input_quality'] = quality
    return item


def report(tmp_path, name, split, scenes, model='m' * 64, **fields):
    path = tmp_path / f'{name}.json'
    path.write_text(json.dumps({'schema': 3, 'split': split, 'model_sha256': model, 'manifest_sha256': name * 8,
                                'scenes': scenes, **fields}))
    return path


def size_scenes(prefix, groups, small_missed=True):
    """Per group: a large object always detected and a 10 px object missed when `small_missed`."""
    result = []
    for index in range(groups):
        large, small = {'label': 'cup', 'bbox': [0, 0, 60, 40]}, {'label': 'cup', 'bbox': [100, 100, 10, 12]}
        predictions = [dict(large, score=.9)] + ([] if small_missed else [dict(small, score=.9)])
        result.append(scene(f'{prefix}{index}', f'{prefix}g{index}', [large, small], predictions))
    return result


def test_search_cut_is_fitted_on_development_and_confirmed_on_heldout(tmp_path):
    loaded = [calibration.load_report(report(tmp_path, 'dev', 'development', size_scenes('d', 30)), 'front'),
              calibration.load_report(report(tmp_path, 'held', 'heldout', size_scenes('h', 30)), 'front')]
    result = calibration.calibrate_search(loaded, {'search_min_object_px': 24.})
    front = result['cameras']['front']
    assert result['status'] == 'calibrated' and result['proposed_value'] == 40.
    assert front['heldout_trusted']['groups'] == 30 and front['heldout_flagged']['group_recall'] == 0
    assert not result['domain_declarations']['search_min_object_m']['measured']


def test_search_needs_heldout_and_enough_groups(tmp_path):
    development = calibration.load_report(report(tmp_path, 'dev', 'development', size_scenes('d', 30)))
    alone = calibration.calibrate_search([development], {})
    assert alone['cameras']['pooled']['reason'] == 'heldout_report_required'
    heldout = calibration.load_report(report(tmp_path, 'held', 'heldout', size_scenes('h', 5)))
    result = calibration.calibrate_search([development, heldout], {})
    pooled = result['cameras']['pooled']
    assert result['status'] == 'insufficient_corpus' and result['proposed_value'] is None
    assert pooled['additional_groups_needed'] == {'heldout': 24}


def test_search_reports_no_feasible_cut_when_even_large_objects_are_missed(tmp_path):
    def missed(prefix):
        return [scene(f'{prefix}{i}', f'{prefix}{i}', [{'label': 'cup', 'bbox': [0, 0, 50, 50]}], [])
                for i in range(30)]
    loaded = [calibration.load_report(report(tmp_path, 'dev', 'development', missed('d'))),
              calibration.load_report(report(tmp_path, 'held', 'heldout', missed('h')))]
    assert calibration.calibrate_search(loaded, {})['status'] == 'no_feasible_cut'


def test_reports_must_share_one_model(tmp_path):
    loaded = [calibration.load_report(report(tmp_path, 'dev', 'development', size_scenes('d', 1))),
              calibration.load_report(report(tmp_path, 'held', 'heldout', size_scenes('h', 1), model='n' * 64))]
    with pytest.raises(ValueError, match='same model_sha256'):
        calibration.calibrate_search(loaded, {})
    failed = tmp_path / 'failed.json'
    failed.write_text(json.dumps({'schema': 3, 'passed': False, 'error': 'boom'}))
    with pytest.raises(ValueError, match='completed schema-3'):
        calibration.load_report(failed)


CURRENT = calibration.input_quality_limits(ROOT)


def test_tracked_cameras_keep_the_uncalibrated_engineering_limits():
    # cleanroom#98: until a calibration record replaces them, and never with a lower dark limit.
    assert set(CURRENT) == {'front', 'wrist', 'astra_color'}
    assert all(limits == {'dark_luma': 40., 'bright_luma': 225., 'blur_variance': 30.} for limits in CURRENT.values())


def test_input_quality_without_runtime_features_is_insufficient(tmp_path):
    loaded = [calibration.load_report(report(tmp_path, 'dev', 'development', size_scenes('d', 30)))]
    result = calibration.calibrate_input_quality(loaded, CURRENT)
    assert (result['status'], result['reason']) == ('insufficient_corpus', 'report_lacks_input_quality')
    assert result['proposed_value'] is None


def quality_scenes(prefix, groups):
    """Dark frames (luma 10) miss the object; normal frames (luma 120) detect it."""
    result = []
    for index in range(groups):
        cup = {'label': 'cup', 'bbox': [0, 0, 50, 50]}
        for luma, found in ((10., False), (120., True)):
            tags = [tag for tag in calibration.HARD_CONDITIONS if luma < 40 or tag != 'dark']
            result.append(scene(f'{prefix}{index}-{luma}', f'{prefix}{index}-{luma}', [cup],
                                [cup] if found else [], {'mean_luma': luma, 'sharpness': 500.}, tags))
    return result


def test_input_quality_limits_and_condition_gate(tmp_path):
    loaded = [calibration.load_report(report(tmp_path, 'dev', 'development', quality_scenes('d', 30))),
              calibration.load_report(report(tmp_path, 'held', 'heldout', quality_scenes('h', 30)))]
    result = calibration.calibrate_input_quality(loaded, CURRENT)
    assert result['status'] == 'calibrated', result['reason']
    # Reports that name no camera are one pooled fit, proposed for every tracked camera.
    fitted = {'dark_luma': 120., 'bright_luma': 120., 'blur_variance': 500.}
    assert result['proposed_value'] == {camera: fitted for camera in CURRENT}
    assert result['cameras']['pooled']['conditions']['dark']['group_recall'] == 0
    few = [calibration.load_report(report(tmp_path, 'dev2', 'development', quality_scenes('d', 5))),
           calibration.load_report(report(tmp_path, 'held2', 'heldout', quality_scenes('h', 4)))]
    thin = calibration.calibrate_input_quality(few, CURRENT)
    assert thin['status'] == 'insufficient_corpus' and thin['reason'] == 'pooled:hard_conditions_below_minimum'
    assert thin['cameras']['pooled']['condition_groups_missing']['dark'] == 11


def test_input_quality_is_fitted_per_camera_against_its_own_current_limits(tmp_path):
    def loaded(camera, development, heldout, **fields):
        return [calibration.load_report(report(tmp_path, f'{camera}-dev', 'development', development, **fields), camera),
                calibration.load_report(report(tmp_path, f'{camera}-held', 'heldout', heldout, **fields), camera)]
    front = loaded('front', quality_scenes('d', 30), quality_scenes('h', 30))
    wrist = loaded('wrist', quality_scenes('d', 30), quality_scenes('h', 4))
    result = calibration.calibrate_input_quality(front, CURRENT)
    # Only the camera the reports name is proposed; the others keep their limits.
    assert result['status'] == 'calibrated' and set(result['proposed_value']) == {'front'}
    both = calibration.calibrate_input_quality(front + wrist, CURRENT)
    assert both['status'] == 'insufficient_corpus' and both['proposed_value'] is None
    assert both['cameras']['front']['status'] == 'calibrated'
    assert both['cameras']['wrist']['status'] == 'insufficient_corpus'
    # A camera's fit uses its own current limits: one that already trusts only very sharp frames keeps none.
    strict = {**CURRENT, 'front': {**CURRENT['front'], 'blur_variance': 900.}}
    assert calibration.calibrate_input_quality(front, strict)['cameras']['front']['limits']['dark_luma']['status'] \
        == 'insufficient_corpus'
    # The evaluator's own --camera labels a report when no prefix overrides it.
    labelled = calibration.load_report(report(tmp_path, 'own', 'development', quality_scenes('d', 1), camera='wrist'))
    assert {trial['camera'] for trial in calibration.trials(labelled)} == {'wrist'}
    pooled = [calibration.load_report(report(tmp_path, 'pool', 'development', quality_scenes('p', 1)))]
    with pytest.raises(ValueError, match='every report'):
        calibration.calibrate_input_quality(front + pooled, CURRENT)
    with pytest.raises(ValueError, match='different limits'):
        calibration.calibrate_input_quality(pooled, strict)
    with pytest.raises(ValueError, match='untracked cameras'):
        calibration.calibrate_input_quality(loaded('ceiling', quality_scenes('d', 1), quality_scenes('h', 1)), CURRENT)


def test_write_refuses_anything_but_calibrated(tmp_path):
    for status in ('insufficient_corpus', 'no_feasible_cut'):
        with pytest.raises(ValueError, match='refusing'):
            calibration.write_calibration(tmp_path, {'status': status, 'procedure': calibration.SEARCH_PROCEDURE})


def test_write_updates_one_tracked_line_and_records_the_evidence(tmp_path):
    (tmp_path / 'config').mkdir()
    original = (ROOT / 'config/exploration.yaml').read_text()
    (tmp_path / 'config/exploration.yaml').write_text(original)
    result = {'status': 'calibrated', 'procedure': calibration.SEARCH_PROCEDURE, 'proposed_value': 31.,
              'inputs': [{'split': 'heldout', 'manifest_sha256': 'a' * 64, 'sha256': 'b' * 64}]}
    written = calibration.write_calibration(tmp_path, result)
    record = Path(written[0])
    assert record.parent == Path('config/calibration') and record.name.startswith('search-min-object-px-1-')
    updated = (tmp_path / 'config/exploration.yaml').read_text().splitlines()
    before = original.splitlines()
    changed = [i for i, (a, b) in enumerate(zip(before, updated)) if a != b]
    assert len(before) == len(updated) and len(changed) == 1
    assert updated[changed[0]] == f'    search_min_object_px: 31  # calibrated: {record}'
    assert json.loads((tmp_path / record).read_text()) == result


def test_input_quality_write_updates_only_the_fitted_camera_and_keeps_its_binding(tmp_path):
    from lekiwi_rmf.object_cameras import camera_bindings
    (tmp_path / 'config').mkdir()
    original = (ROOT / 'config/object_cameras.yaml').read_text()
    (tmp_path / 'config/object_cameras.yaml').write_text(original)
    result = {'status': 'calibrated', 'procedure': calibration.INPUT_QUALITY_PROCEDURE,
              'proposed_value': {'wrist': {'dark_luma': 45., 'bright_luma': 240., 'blur_variance': 20.}},
              'inputs': [{'split': 'heldout', 'manifest_sha256': None, 'sha256': 'c' * 64}]}
    written = calibration.write_calibration(tmp_path, result)
    assert written[1] == 'config/object_cameras.yaml' and json.loads((tmp_path / written[0]).read_text()) == result
    updated = (tmp_path / 'config/object_cameras.yaml').read_text()
    # The file's leading documentation, including the DEBT link, survives the rewrite.
    comments = [line for line in original.splitlines() if line.startswith('#')]
    assert comments and updated.splitlines()[:len(comments)] == comments
    assert calibration.input_quality_limits(tmp_path) == {**CURRENT, 'wrist': result['proposed_value']['wrist']}
    names = ['front', 'wrist', 'astra_color']
    before, after = camera_bindings(names), camera_bindings(names, tmp_path / 'config/object_cameras.yaml')
    # The limits are no part of the camera identity an admitted Fiber profile pins.
    assert {name: after[name]['sha256'] for name in names} == {name: before[name]['sha256'] for name in names}
    assert after['wrist']['input_quality'] == result['proposed_value']['wrist']
    assert after['front']['input_quality'] == before['front']['input_quality']
    with pytest.raises(ValueError, match='not a camera'):
        calibration.write_calibration(tmp_path, {**result, 'proposed_value': {'ceiling': result['proposed_value']['wrist']}})


def test_cli_writes_nothing_on_an_insufficient_corpus(tmp_path):
    root = tmp_path / 'repo'
    (root / 'config').mkdir(parents=True)
    shutil.copy(ROOT / 'config/exploration.yaml', root / 'config/exploration.yaml')
    development = report(tmp_path, 'dev', 'development', size_scenes('d', 3))
    heldout = report(tmp_path, 'held', 'heldout', size_scenes('h', 3))
    output = tmp_path / 'out.json'
    run = subprocess.run([sys.executable, str(ROOT / 'scripts/calibrate-thresholds.py'), 'search',
                          '--report', f'front={development}', '--report', f'front={heldout}',
                          '--output', str(output), '--write', '--root', str(root)],
                         capture_output=True, text=True, check=False)
    assert run.returncode == 1, run.stderr
    assert json.loads(run.stdout)['written'] == [] and 'nothing written' in run.stderr
    assert json.loads(output.read_text())['status'] == 'insufficient_corpus'
    assert (root / 'config/exploration.yaml').read_text() == (ROOT / 'config/exploration.yaml').read_text()
    assert not (root / 'config/calibration').exists()
