"""Fit perception thresholds from labelled evaluator reports, with honest exact intervals.

Two procedures share one rule. A threshold splits trials (labelled objects) into a trusted side,
where a miss is accepted as evidence (search coverage, a `no_candidate` abstention), and a
flagged side. The fitted cut is the most permissive one whose one-sided 95% Clopper-Pearson
lower bound of scene-group recall on the trusted side reaches the target. The unit of
independence is the scene group: a group fails when any of its trusted trials is missed, the
same convention as scripts/evaluate-object-recognition.py. Cuts are chosen on development
reports and confirmed on held-out reports; only a confirmed cut is `calibrated`.

ROS-free, so it runs anywhere the evaluator's JSON reports are available.
"""

from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re

SEARCH_PROCEDURE = 'search-min-object-px/1'
INPUT_QUALITY_PROCEDURE = 'input-quality-limits/1'
CONFIDENCE = .95
DEFAULT_TARGET_RECALL = .90
# cleanroom#98: every hard condition needs this many scene groups before limits are chosen.
HARD_CONDITIONS = ('dark', 'overhead', 'occlusion', 'transparent', 'reflective', 'blur', 'similar_instances')
MIN_CONDITION_GROUPS = 20
# Engineering defaults in force until a calibration replaces them (cleanroom#98).
INPUT_QUALITY_DEFAULTS = {'dark_luma': 40., 'bright_luma': 225., 'blur_variance': 30.}
INPUT_QUALITY_LINE = re.compile(r'^DARK_LUMA, BRIGHT_LUMA, BLUR_VARIANCE = .*$', re.MULTILINE)
SEARCH_PX_LINE = re.compile(r'^(?P<indent>[ \t]*)search_min_object_px:[^\n]*$', re.MULTILINE)


# ── box matching (shared with scripts/evaluate-object-recognition.py) ──────────────────────

def overlap(left, right):
    x, y = max(left[0], right[0]), max(left[1], right[1])
    width = max(0, min(left[0] + left[2], right[0] + right[2]) - x)
    height = max(0, min(left[1] + left[3], right[1] + right[3]) - y)
    area = width * height
    return area / (left[2] * left[3] + right[2] * right[3] - area)


def matches(predictions, expected):
    """Match each labeled target once; extra same-class candidates count as false positives."""
    possible = []
    for p, prediction in enumerate(predictions):
        for e, target in enumerate(expected):
            score = overlap(prediction, target)
            if score >= .5:
                possible.append((score, p, e))
    used_predictions, used_expected = set(), set()
    paired = []
    for _, p, e in sorted(possible, reverse=True):
        if p not in used_predictions and e not in used_expected:
            used_predictions.add(p)
            used_expected.add(e)
            paired.append((p, e))
    return paired


# ── exact binomial intervals ────────────────────────────────────────────────────────────────

def _log_pmf(k, n, p):
    if p <= 0:
        return 0. if k == 0 else -math.inf
    if p >= 1:
        return 0. if k == n else -math.inf
    return (math.lgamma(n + 1) - math.lgamma(k + 1) - math.lgamma(n - k + 1)
            + k * math.log(p) + (n - k) * math.log1p(-p))


def binomial_cdf(k, n, p):
    """P(X <= k) for X ~ Binomial(n, p)."""
    if k < 0:
        return 0.
    if k >= n:
        return 1.
    return min(1., sum(math.exp(_log_pmf(i, n, p)) for i in range(k + 1)))


def _bisect(function, target, increasing):
    low, high = 0., 1.
    for _ in range(100):
        middle = (low + high) / 2
        if (function(middle) < target) == increasing:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def lower_bound(successes, total, alpha=1 - CONFIDENCE):
    """One-sided exact (Clopper-Pearson) lower confidence bound of a proportion."""
    if total <= 0:
        return None
    if successes <= 0:
        return 0.
    # P(X >= successes; p) = 1 - cdf(successes - 1) rises with p; solve it equal to alpha.
    return _bisect(lambda p: 1 - binomial_cdf(successes - 1, total, p), alpha, True)


def upper_bound(successes, total, alpha=1 - CONFIDENCE):
    """One-sided exact (Clopper-Pearson) upper confidence bound of a proportion."""
    if total <= 0:
        return None
    if successes >= total:
        return 1.
    # P(X <= successes; p) falls with p; solve it equal to alpha.
    return _bisect(lambda p: binomial_cdf(successes, total, p), alpha, False)


def interval(successes, total, confidence=CONFIDENCE):
    """Two-sided exact interval; each tail holds (1 - confidence) / 2."""
    if total <= 0:
        return None
    alpha = (1 - confidence) / 2
    return [lower_bound(successes, total, alpha), upper_bound(successes, total, alpha)]


def groups_needed(target, failures=0, limit=100000):
    """Smallest number of scene groups whose lower bound reaches `target` with `failures` failed groups."""
    for total in range(failures + 1, limit):
        if lower_bound(total - failures, total) >= target:
            return total
    return None


# ── trials ─────────────────────────────────────────────────────────────────────────────────

def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_report(path, camera=None):
    """One evaluator report (schema 3) and its provenance; a failed evaluation is not a corpus."""
    path = Path(path).expanduser()
    if path.stat().st_size > 64 * 1024 ** 2:
        raise ValueError(f'{path} exceeds 64 MiB')
    report = json.loads(path.read_text())
    if not isinstance(report, dict) or report.get('schema') != 3 or 'error' in report:
        raise ValueError(f'{path} is not a completed schema-3 evaluator report')
    if report.get('split') not in ('development', 'heldout') or not isinstance(report.get('scenes'), list):
        raise ValueError(f'{path} lacks a split or scenes')
    return {'path': str(path), 'sha256': _sha256(path), 'camera': camera, 'report': report}


def trials(loaded):
    """One trial per labelled object: whether a same-label prediction matched it, with its scene's features."""
    report = loaded['report']
    result = []
    for scene in report['scenes']:
        camera = loaded['camera'] or scene.get('camera') or 'pooled'
        expected, predicted = scene.get('expected', []), scene.get('predictions', [])
        quality = scene.get('input_quality')
        for label in sorted({item['label'] for item in expected}):
            wanted = [item['bbox'] for item in expected if item['label'] == label]
            found = [item['bbox'] for item in predicted if item.get('label') == label]
            hit = {e for _, e in matches(found, wanted)}
            for index, box in enumerate(wanted):
                result.append({
                    'group': f"{camera}/{scene['scene_group']}", 'scene': scene['scene_id'], 'camera': camera,
                    'split': report['split'], 'label': label, 'detected': index in hit,
                    'apparent_px': float(min(box[2], box[3])),
                    'mean_luma': None if quality is None else float(quality['mean_luma']),
                    'sharpness': None if quality is None else float(quality['sharpness']),
                    'conditions': list(scene.get('conditions', [])),
                })
    return result


def tally(selected):
    """Trial- and group-level recall; a group succeeds only when every one of its trials does."""
    groups = defaultdict(lambda: True)
    for trial in selected:
        groups[trial['group']] &= trial['detected']
    hits = sum(trial['detected'] for trial in selected)
    group_hits = sum(groups.values())
    return {'trials': len(selected), 'detected': hits,
            'trial_recall': hits / len(selected) if selected else None,
            'trial_recall_95_ci': interval(hits, len(selected)),
            'groups': len(groups), 'groups_all_detected': group_hits,
            'group_recall': group_hits / len(groups) if groups else None,
            'group_recall_95_ci': interval(group_hits, len(groups)),
            'group_recall_95_lower_bound': lower_bound(group_hits, len(groups))}


def _trusted(trial, feature, cut, direction):
    value = trial[feature]
    return value >= cut if direction == 'above' else value <= cut


def fit_cut(development, heldout, feature, direction, target=DEFAULT_TARGET_RECALL):
    """The most permissive cut whose trusted side keeps group recall's lower bound at `target`.

    `direction` 'above' trusts values at or above the cut (fit the smallest), 'below' trusts values at
    or below it (fit the largest). The cut is scanned on development trials and confirmed on held-out
    trials; a cut that only passes on the data that chose it is not calibrated.
    """
    needed = groups_needed(target)
    development = [trial for trial in development if trial[feature] is not None]
    heldout = [trial for trial in heldout if trial[feature] is not None]
    result = {'feature': feature, 'trusted': direction, 'target_recall': target,
              'groups_needed_without_failures': needed,
              'development': tally(development), 'heldout': tally(heldout)}
    short = {name: needed - result[name]['groups'] for name in ('development', 'heldout')
             if result[name]['groups'] < needed}
    if short:
        reason = 'heldout_report_required' if not heldout else 'too_few_scene_groups'
        return {**result, 'status': 'insufficient_corpus', 'reason': reason, 'additional_groups_needed': short}
    cuts = sorted({trial[feature] for trial in development}, reverse=direction == 'below')
    chosen = None
    for cut in cuts:
        trusted = [trial for trial in development if _trusted(trial, feature, cut, direction)]
        bound = tally(trusted)['group_recall_95_lower_bound']
        if bound is not None and bound >= target:
            chosen = cut
            break
    if chosen is None:
        return {**result, 'status': 'no_feasible_cut', 'reason': 'no_cut_reaches_target_on_development'}
    trusted = [trial for trial in heldout if _trusted(trial, feature, chosen, direction)]
    flagged = [trial for trial in heldout if not _trusted(trial, feature, chosen, direction)]
    confirmed = tally(trusted)
    result.update({'cut': chosen, 'heldout_trusted': confirmed, 'heldout_flagged': tally(flagged)})
    bound = confirmed['group_recall_95_lower_bound']
    if bound is None or bound < target:
        return {**result, 'status': 'no_feasible_cut', 'reason': 'heldout_does_not_confirm_cut'}
    return {**result, 'status': 'calibrated', 'reason': 'heldout_confirms_cut'}


def _provenance(loaded):
    models = {item['report'].get('model_sha256') for item in loaded}
    if len(models) != 1 or None in models:
        raise ValueError('every report must evaluate the same model_sha256')
    return models.pop(), [{'path': item['path'], 'sha256': item['sha256'], 'camera': item['camera'],
                           'split': item['report']['split'],
                           'manifest_sha256': item['report'].get('manifest_sha256')} for item in loaded]


def _split(items, split):
    return [trial for trial in items if trial['split'] == split]


def calibrate_search(loaded, current, target=DEFAULT_TARGET_RECALL):
    """Fit `search_min_object_px` per camera; the robot uses the largest, i.e. most conservative, cut.

    The minimum object size and the height band are domain declarations: an image manifest records
    neither physical size nor height, so they are reported as unmeasured, never fitted.
    """
    model, inputs = _provenance(loaded)
    by_camera = defaultdict(list)
    for item in loaded:
        for trial in trials(item):
            by_camera[trial['camera']].append(trial)
    cameras = {camera: fit_cut(_split(items, 'development'), _split(items, 'heldout'),
                               'apparent_px', 'above', target)
               for camera, items in sorted(by_camera.items())}
    statuses = {fit['status'] for fit in cameras.values()} or {'insufficient_corpus'}
    status = ('calibrated' if statuses == {'calibrated'} else
              'no_feasible_cut' if 'no_feasible_cut' in statuses else 'insufficient_corpus')
    proposed = max(fit['cut'] for fit in cameras.values()) if status == 'calibrated' else None
    reason = ('no_labelled_objects' if not cameras else
              ','.join(sorted({f"{camera}:{fit['reason']}" for camera, fit in cameras.items()})))
    return {'schema': 1, 'procedure': SEARCH_PROCEDURE, 'status': status, 'reason': reason, 'confidence': CONFIDENCE,
            'uncertainty_unit': 'scene_group', 'model_sha256': model, 'inputs': inputs,
            'parameter': 'search_min_object_px', 'current_value': current.get('search_min_object_px'),
            'proposed_value': proposed, 'cameras': cameras,
            'apparent_size': 'min(bbox width, bbox height) of each labelled object, in pixels',
            'domain_declarations': {
                key: {'value': current.get(key), 'measured': False,
                      'reason': 'image manifests record no physical size or height; the recording protocol '
                                'must place objects of this size across this band'}
                for key in ('search_min_object_m', 'search_min_height_m', 'search_max_height_m')}}


def calibrate_input_quality(loaded, target=DEFAULT_TARGET_RECALL, current=None):
    """Fit the dark, overexposed and blurred limits; each on trials passing the other two current limits."""
    current = dict(current or INPUT_QUALITY_DEFAULTS)
    model, inputs = _provenance(loaded)
    every = [trial for item in loaded for trial in trials(item)]
    base = {'schema': 1, 'procedure': INPUT_QUALITY_PROCEDURE, 'confidence': CONFIDENCE,
            'uncertainty_unit': 'scene_group', 'model_sha256': model, 'inputs': inputs,
            'current_value': current, 'proposed_value': None}
    if not every or any(trial['mean_luma'] is None for trial in every):
        return {**base, 'status': 'insufficient_corpus', 'reason': 'report_lacks_input_quality'}
    conditions = {}
    for tag in sorted(set(HARD_CONDITIONS) | {tag for trial in every for tag in trial['conditions']}):
        tagged = [trial for trial in every if tag in trial['conditions']]
        conditions[tag] = tally(tagged)
    missing = {tag: MIN_CONDITION_GROUPS - conditions[tag]['groups'] for tag in HARD_CONDITIONS
               if conditions[tag]['groups'] < MIN_CONDITION_GROUPS}
    limits = {}
    specs = (('dark_luma', 'mean_luma', 'above'), ('bright_luma', 'mean_luma', 'below'),
             ('blur_variance', 'sharpness', 'above'))
    for name, feature, direction in specs:
        others = [(key, other, way) for key, other, way in specs if key != name]
        kept = [trial for trial in every if all(_trusted(trial, other, current[key], way)
                                                for key, other, way in others)]
        limits[name] = fit_cut(_split(kept, 'development'), _split(kept, 'heldout'), feature, direction, target)
    result = {**base, 'conditions': conditions, 'condition_groups_missing': missing, 'limits': limits}
    if missing:
        return {**result, 'status': 'insufficient_corpus', 'reason': 'hard_conditions_below_minimum'}
    statuses = {fit['status'] for fit in limits.values()}
    if statuses != {'calibrated'}:
        status = 'no_feasible_cut' if 'no_feasible_cut' in statuses else 'insufficient_corpus'
        return {**result, 'status': status, 'reason': 'a_limit_is_not_calibrated'}
    return {**result, 'status': 'calibrated', 'reason': 'heldout_confirms_every_limit',
            'proposed_value': {name: fit['cut'] for name, fit in limits.items()}}


# ── reviewed write ─────────────────────────────────────────────────────────────────────────

def record_path(root, calibration):
    heldout = sorted(item['manifest_sha256'] or item['sha256'] for item in calibration['inputs']
                     if item['split'] == 'heldout')
    digest = hashlib.sha256('\n'.join(heldout).encode()).hexdigest()[:12]
    return Path(root) / 'config/calibration' / f"{calibration['procedure'].replace('/', '-')}-{digest}.json"


def write_calibration(root, calibration):
    """Record a calibrated value in tracked files; the change then goes through a reviewed PR.

    Anything short of `calibrated` writes nothing, so a thin corpus can never replace a default.
    """
    if calibration['status'] != 'calibrated':
        raise ValueError(f"refusing to write a {calibration['status']} calibration")
    root = Path(root)
    record = record_path(root, calibration)
    relative = record.relative_to(root)
    if calibration['procedure'] == SEARCH_PROCEDURE:
        target = root / 'config/exploration.yaml'
        text = target.read_text()
        if len(SEARCH_PX_LINE.findall(text)) != 1:
            raise ValueError('config/exploration.yaml must hold exactly one search_min_object_px line')
        updated = SEARCH_PX_LINE.sub(
            lambda found: f"{found['indent']}search_min_object_px: {calibration['proposed_value']:g}"
                          f'  # calibrated: {relative}', text)
    elif calibration['procedure'] == INPUT_QUALITY_PROCEDURE:
        target = root / 'lekiwi_rmf/object_observations.py'
        text = target.read_text()
        if len(INPUT_QUALITY_LINE.findall(text)) != 1:
            raise ValueError('lekiwi_rmf/object_observations.py has no input-quality limits line; '
                             'the runtime limits land with cleanroom#100')
        value = calibration['proposed_value']
        updated = INPUT_QUALITY_LINE.sub(
            f"DARK_LUMA, BRIGHT_LUMA, BLUR_VARIANCE = {value['dark_luma']!r}, {value['bright_luma']!r}, "
            f"{value['blur_variance']!r}  # calibrated: {relative}", text)
    else:
        raise ValueError(f"unknown procedure {calibration['procedure']!r}")
    record.parent.mkdir(parents=True, exist_ok=True)
    record.write_text(json.dumps(calibration, indent=2, allow_nan=False) + '\n')
    target.write_text(updated)
    return [str(relative), str(target.relative_to(root))]
