"""Calibration studies: frozen corpus, development search, one held-out comparison and a recorded decision.

A study freezes the manifest, its splits, the objective and the regression rule before it looks at any
candidate. It searches candidates on development only, then evaluates the current profile and the best
candidate once on the held-out split. The candidate is selected only when a paired bootstrap over scene
groups shows a held-out improvement whose lower bound is above zero and no label or condition regresses
beyond the tolerance. The record is content-addressed; the detector admits untracked parameters only
from a selected record (`admitted_candidate`), and reverting restores the exact earlier revision.
"""

import hashlib
import json
import os
from pathlib import Path
import random
import tempfile
import time

from lekiwi_rmf.perception_profiles import make_profile

STORE = Path.home() / '.ros/lekiwi/perception/studies'
RESAMPLES = 2000
SEED = 20261019
MAX_LABEL_REGRESSION = .02
MIN_CONDITION_SCENES = 5


def canonical(document):
    return json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False)


def f1(tp, fp, fn):
    return 2 * tp / (2 * tp + fp + fn) if tp + fp + fn else None


def summarize(scenes, labels):
    """Per-label precision/recall/F1 and their macro mean over labels that have any evidence."""
    per_label = {}
    for label in labels:
        tp, fp, fn = (sum(scene['goals'][label][index] for scene in scenes) for index in range(3))
        per_label[label] = {'tp': tp, 'fp': fp, 'fn': fn, 'precision': tp / (tp + fp) if tp + fp else None,
                            'recall': tp / (tp + fn) if tp + fn else None, 'f1': f1(tp, fp, fn)}
    scores = [value['f1'] for value in per_label.values() if value['f1'] is not None]
    return {'macro_f1': sum(scores) / len(scores) if scores else None, 'labels': per_label}


def by_condition(scenes, labels):
    conditions = sorted({tag for scene in scenes for tag in scene.get('conditions', [])})
    return {tag: {'scenes': sum(tag in scene.get('conditions', []) for scene in scenes),
                  **summarize([scene for scene in scenes if tag in scene.get('conditions', [])], labels)}
            for tag in conditions}


def paired_bootstrap(current, candidate, labels, *, resamples=RESAMPLES, seed=SEED):
    """95% interval of the macro-F1 gain, resampling whole scene groups so correlated views move together."""
    groups = sorted({scene['scene_group'] for scene in current})
    if {scene['scene_group'] for scene in candidate} != set(groups) or not groups:
        raise ValueError('held-out results must cover the same scene groups')
    def grouped(scenes):
        result = {group: [] for group in groups}
        for scene in scenes:
            result[scene['scene_group']].append(scene)
        return result
    left, right = grouped(current), grouped(candidate)
    generator = random.Random(seed)
    deltas = []
    for _ in range(resamples):
        sample = [generator.choice(groups) for _ in groups]
        before = summarize([scene for group in sample for scene in left[group]], labels)['macro_f1']
        after = summarize([scene for group in sample for scene in right[group]], labels)['macro_f1']
        if before is not None and after is not None:
            deltas.append(after - before)
    if not deltas:
        return {'delta': None, 'ci95': None, 'resamples': 0, 'seed': seed}
    deltas.sort()
    point = summarize(candidate, labels)['macro_f1'] - summarize(current, labels)['macro_f1']
    return {'delta': point, 'ci95': [deltas[int(.025 * (len(deltas) - 1))], deltas[int(.975 * (len(deltas) - 1))]],
            'resamples': len(deltas), 'seed': seed}


def regressions(current, candidate, labels, tolerance=MAX_LABEL_REGRESSION):
    """Labels and well-covered conditions whose held-out F1 or recall drops by more than `tolerance`."""
    found = []
    before, after = summarize(current, labels)['labels'], summarize(candidate, labels)['labels']
    for label in labels:
        for metric in ('f1', 'recall'):
            old, new = before[label][metric], after[label][metric]
            if old is not None and (new is None or old - new > tolerance):
                found.append({'scope': 'label', 'name': label, 'metric': metric, 'current': old, 'candidate': new})
    old_conditions, new_conditions = by_condition(current, labels), by_condition(candidate, labels)
    for tag, value in old_conditions.items():
        if value['scenes'] < MIN_CONDITION_SCENES:
            continue
        old, new = value['macro_f1'], new_conditions.get(tag, {}).get('macro_f1')
        if old is not None and (new is None or old - new > tolerance):
            found.append({'scope': 'condition', 'name': tag, 'metric': 'macro_f1', 'current': old, 'candidate': new})
    return found


def run_study(evaluate, recognizer, current, candidates, *, manifest_sha256, labels, objective='macro_f1'):
    """`evaluate(profile, split)` returns the evaluator's report for that split under `profile`.

    The frozen protocol is recorded before any search; development picks one candidate; held-out is read
    exactly once for the current profile and once for that candidate.
    """
    if not candidates or len({candidate.revision for candidate in candidates}) != len(candidates):
        raise ValueError('a study needs distinct candidate profiles')
    if any(candidate.backend != current.backend for candidate in [current, *candidates]):
        raise ValueError('every candidate must belong to the current recognizer backend')
    if current.revision in {candidate.revision for candidate in candidates}:
        raise ValueError('the current profile is the baseline, not a candidate')
    protocol = {'manifest_sha256': manifest_sha256, 'labels': list(labels), 'objective': objective,
                'search_split': 'development', 'comparison_split': 'heldout',
                'selection': 'held-out paired bootstrap lower bound of macro-F1 gain above 0',
                'regression': {'max_drop': MAX_LABEL_REGRESSION, 'metrics': ['f1', 'recall'],
                               'conditions_with_at_least_scenes': MIN_CONDITION_SCENES},
                'bootstrap': {'unit': 'scene_group', 'resamples': RESAMPLES, 'seed': SEED}}
    development = {}
    for candidate in candidates:
        report = evaluate(candidate, 'development')
        if report.get('manifest_sha256') != manifest_sha256:
            raise ValueError('the corpus changed during the study')
        development[candidate.revision] = {'profile': candidate.document(), **summarize(report['scenes'], labels),
                                           'conditions': by_condition(report['scenes'], labels)}
    ranked = sorted(candidates, key=lambda c: -(development[c.revision]['macro_f1'] or -1))
    best = ranked[0]
    held_current, held_best = evaluate(current, 'heldout'), evaluate(best, 'heldout')
    if {held_current.get('manifest_sha256'), held_best.get('manifest_sha256')} != {manifest_sha256}:
        raise ValueError('the corpus changed during the study')
    paired = paired_bootstrap(held_current['scenes'], held_best['scenes'], labels)
    regressed = regressions(held_current['scenes'], held_best['scenes'], labels)
    reasons = []
    if paired['ci95'] is None or paired['ci95'][0] <= 0:
        reasons.append('held-out gain is not distinguishable from sampling variation')
    if regressed:
        reasons.append('the candidate regresses a label or condition beyond the tolerance')
    record = {
        'schema': 1, 'kind': 'perception_profile_study', 'created_at_unix': time.time(), 'protocol': protocol,
        'recognizer': {'model_name': recognizer.name, 'model_sha256': recognizer.sha256, 'backend': current.backend},
        'current': current.document(), 'development': development, 'selected_on_development': best.revision,
        'heldout': {'current': {**summarize(held_current['scenes'], labels),
                                'conditions': by_condition(held_current['scenes'], labels)},
                    'candidate': {**summarize(held_best['scenes'], labels),
                                  'conditions': by_condition(held_best['scenes'], labels)},
                    'paired': paired, 'regressions': regressed},
        'decision': 'rejected' if reasons else 'selected', 'reasons': reasons,
        'candidate': best.document(),
        'revert': {'revision': current.revision, 'how': 'apply_profile revert_to_revision'}}
    record['id'] = hashlib.sha256(canonical(record).encode()).hexdigest()
    return record


def save(record, store=None):
    directory = Path(store or STORE)
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', dir=directory, delete=False) as stream:
        stream.write(canonical(record))
    os.replace(stream.name, directory / f'{record["id"]}.json')
    return directory / f'{record["id"]}.json'


def load(evidence_id, store=None):
    if not isinstance(evidence_id, str) or len(evidence_id) != 64 or any(c not in '0123456789abcdef' for c in evidence_id):
        raise ValueError('evidence_id is a study SHA-256')
    path = Path(store or STORE) / f'{evidence_id}.json'
    if not path.is_file() or path.stat().st_size > 16 * 1024**2:
        raise ValueError('no recorded study has that evidence_id')
    record = json.loads(path.read_text())
    body = {key: value for key, value in record.items() if key != 'id'}
    if record.get('id') != evidence_id or hashlib.sha256(canonical(body).encode()).hexdigest() != evidence_id:
        raise ValueError('the study record does not match its evidence_id')
    return record


def admitted_candidate(evidence_id, model_sha256, candidate, store=None):
    """The candidate as a profile, if and only if a recorded study selected exactly it for this recognizer."""
    record = load(evidence_id, store)
    if record['decision'] != 'selected':
        raise ValueError('that study rejected its candidate: ' + '; '.join(record['reasons']))
    if record['recognizer']['model_sha256'] != model_sha256:
        raise ValueError('that study evaluated another recognizer')
    if record['candidate']['revision'] != candidate.revision:
        raise ValueError('these parameters are not the candidate that study selected')
    return make_profile(candidate.backend, candidate.parameters, f'study-{evidence_id[:12]}')
