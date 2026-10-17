#!/usr/bin/env python3
"""Evaluate pinned recognition on local held-out images; never command the robot."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_detector import detect
from lekiwi_rmf.object_recognition import Recognizer, use_model_runtime
from lekiwi_rmf.object_observations import observation


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


def counts(predictions, expected):
    paired = matches(predictions, expected)
    return len(paired), len(predictions) - len(paired), len(expected) - len(paired)


def interval(successes, total):
    if not total:
        return None
    z, rate = 1.96, successes / total
    denominator = 1 + z * z / total
    centre = (rate + z * z / (2 * total)) / denominator
    margin = z * math.sqrt(rate * (1 - rate) / total + z * z / (4 * total * total)) / denominator
    return [max(0, centre - margin), min(1, centre + margin)]


def load_scenes(manifest, split='heldout'):
    path = Path(manifest)
    if path.stat().st_size > 1024 * 1024:
        raise ValueError('manifest exceeds 1 MiB')
    manifest_bytes = path.read_bytes()
    scenes = json.loads(manifest_bytes)
    if not isinstance(scenes, list) or not scenes or len(scenes) > 10000:
        raise ValueError('manifest requires 1..10000 scenes')
    seen, image_hashes, pixel_hashes, selected, groups = set(), {}, {}, [], {}
    for scene in scenes:
        if (not isinstance(scene, dict) or not {'scene_id', 'split', 'image', 'cups'} <= set(scene)
                or set(scene) - {'scene_id', 'scene_group', 'split', 'image', 'cups'}):
            raise ValueError('each scene requires scene_id, split, image and cups')
        identifier = scene['scene_id']
        if not isinstance(identifier, str) or not 1 <= len(identifier) <= 128 or identifier in seen:
            raise ValueError('scene IDs must be unique; split by scene, not adjacent video frames')
        seen.add(identifier)
        group = scene.get('scene_group', identifier)
        if not isinstance(group, str) or not 1 <= len(group) <= 128:
            raise ValueError('scene_group requires a bounded arrangement/session identity')
        if group in groups and groups[group] != scene['split']:
            raise ValueError('scene group crosses development/heldout splits')
        groups[group] = scene['split']
        if scene['split'] not in ('development', 'heldout') or not isinstance(scene['image'], str):
            raise ValueError('invalid scene split/image')
        if not isinstance(scene['cups'], list) or len(scene['cups']) > 20:
            raise ValueError('cups requires at most 20 labeled boxes')
        for cup in scene['cups']:
            if not isinstance(cup, dict) or set(cup) != {'bbox', 'blue'} or type(cup['blue']) is not bool:
                raise ValueError('cup labels require bbox and boolean blue')
            box = cup['bbox']
            if (not isinstance(box, list) or len(box) != 4 or
                    any(type(value) not in (int, float) or not math.isfinite(value) for value in box)
                    or min(box[:2]) < 0 or min(box[2:]) <= 0):
                raise ValueError('invalid labeled cup box')
        image_path = Path(scene['image'])
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        if image_path.stat().st_size > 8 * 1024**2:
            raise ValueError('evaluation image exceeds 8 MiB')
        content = image_path.read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        pixels = cv2.imdecode(np.frombuffer(content, np.uint8), cv2.IMREAD_COLOR)
        if pixels is None or pixels.shape[0] > 1080 or pixels.shape[1] > 1920:
            raise ValueError('evaluation image unreadable or exceeds 1920x1080')
        pixel_digest = hashlib.sha256(str(pixels.shape).encode() + pixels.tobytes()).hexdigest()
        prior = image_hashes.get(digest) or pixel_hashes.get(pixel_digest)
        if prior:
            raise ValueError(f'duplicate image content in scenes {prior!r} and {identifier!r}; '
                             'scene IDs cannot make repeated captures independent')
        image_hashes[digest] = pixel_hashes[pixel_digest] = identifier
        for cup in scene['cups']:
            x, y, w, h = cup['bbox']
            if x + w > pixels.shape[1] or y + h > pixels.shape[0]:
                raise ValueError('labeled box exceeds its image')
        if scene['split'] == split:
            selected.append((scene, image_path, digest))
    if not selected:
        raise ValueError(f'manifest requires {split} scenes')
    return hashlib.sha256(manifest_bytes).hexdigest(), selected


def metrics(total):
    tp, fp, fn = total
    return {'tp': tp, 'fp': fp, 'fn': fn,
            'precision': tp / (tp + fp) if tp + fp else 0,
            'recall': tp / (tp + fn) if tp + fn else 0,
            'precision_95_ci': interval(tp, tp + fp), 'recall_95_ci': interval(tp, tp + fn)}


def scene_metrics(detections, cups, blue_threshold):
    predicted = [item['bbox'] for item in detections]
    expected = [item['bbox'] for item in cups]
    blue_predictions = [item['bbox'] for item in detections if item['blue_fraction'] >= blue_threshold]
    blue_expected = [item['bbox'] for item in cups if item['blue']]
    attribute = [0, 0, 0, 0]  # TP, FP, FN, TN on geometrically matched cups only.
    for p, e in matches(predicted, expected):
        actual, wanted = detections[p]['blue_fraction'] >= blue_threshold, cups[e]['blue']
        attribute[0 if actual and wanted else 1 if actual else 2 if wanted else 3] += 1
    def outcome(predictions, expected):
        correct = len(predictions) == len(expected) == 1 and overlap(predictions[0], expected[0]) >= .5
        return {'expected_unique': len(expected) == 1, 'correct': bool(correct),
                'false_verified': len(predictions) == 1 and not correct}
    return {'cup': counts(predicted, expected), 'blue_goal': counts(blue_predictions, blue_expected),
            'blue_attribute': attribute, 'predictions': detections, 'expected': cups,
            'goal_outcomes': {'cup': outcome(predicted, expected),
                              'blue': outcome(blue_predictions, blue_expected)}}


def evaluate(manifest, model_path=None, *, name='yolox', split='heldout', confidence=.5, blue_threshold=.35):
    if (split not in ('development', 'heldout') or not 0 < confidence <= 1
            or not 0 <= blue_threshold <= 1):
        raise ValueError('invalid frozen evaluation operating point')
    manifest_hash, heldout = load_scenes(manifest, split)
    cv2.setNumThreads(2)
    model, bridge = Recognizer(name, model_path, confidence), CvBridge()
    totals = {'cup': [0, 0, 0], 'blue_goal': [0, 0, 0], 'blue_attribute': [0, 0, 0, 0]}
    durations, hashes, results = [], [], []
    groups = {'all': set(), 'positive': set(), 'negative': set(), 'ambiguous': set()}
    for sequence, (scene, image_path, digest) in enumerate(heldout, 1):
        content = image_path.read_bytes()
        if hashlib.sha256(content).hexdigest() != digest:
            raise ValueError('evaluation image changed after dataset validation')
        pixels = cv2.imdecode(np.frombuffer(content, np.uint8), cv2.IMREAD_COLOR)
        header = Header(stamp=Time(sec=sequence), frame_id='offline_camera')
        start = time.monotonic()
        found = detect(model, pixels, header)
        image = bridge.cv2_to_imgmsg(pixels, 'bgr8')
        image.header = header
        record, _ = observation(image, found, model.sha256, 'offline')
        durations.append(time.monotonic() - start)
        result = scene_metrics(record['detections'], scene['cups'], blue_threshold)
        for key in totals:
            totals[key] = [a + b for a, b in zip(totals[key], result[key])]
        group = scene.get('scene_group', scene['scene_id'])
        groups['all'].add(group)
        groups['positive' if scene['cups'] else 'negative'].add(group)
        if len(scene['cups']) > 1:
            groups['ambiguous'].add(group)
        results.append({'scene_id': scene['scene_id'], 'scene_group': group, **result,
                        'processing_sec': durations[-1]})
        hashes.append(digest)
    measured = {key: metrics(value[:3]) for key, value in totals.items()}
    measured['blue_attribute']['tn'] = totals['blue_attribute'][3]
    p95 = sorted(durations)[max(0, math.ceil(.95 * len(durations)) - 1)] if durations else None
    gates = {'heldout_split': split == 'heldout', 'heldout_scenes': len(groups['all']) >= 80,
             'positive_scenes': len(groups['positive']) >= 20, 'negative_scenes': len(groups['negative']) >= 20,
             'ambiguous_scenes': len(groups['ambiguous']) >= 10, 'latency': p95 is not None and p95 <= 1.5,
             'no_false_verified': not any(outcome['false_verified'] for item in results
                                         for outcome in item['goal_outcomes'].values())}
    # Qualify with scene-group trials, not correlated objects or adjacent frames.
    for key in measured:
        grouped = {}
        for item in results:
            values = item[key][:3]
            prior = grouped.setdefault(item['scene_group'], [0, 0, 0])
            grouped[item['scene_group']] = [a + b for a, b in zip(prior, values)]
        for metric, threshold, denominator, error in [('precision', .98, 1, 1), ('recall', .9, 2, 2)]:
            trials = [value for value in grouped.values() if value[0] + value[denominator] > 0]
            bounds = interval(sum(value[error] == 0 for value in trials), len(trials))
            measured[key]['scene_' + metric + '_95_ci'] = bounds
            gates[key + '_' + metric] = measured[key][metric] >= threshold
            gates[key + '_' + metric + '_confidence'] = bounds is not None and bounds[0] >= threshold
    gates['blue_positive_matches'] = sum(totals['blue_attribute'][i] for i in (0, 2)) >= 20
    gates['nonblue_matches'] = sum(totals['blue_attribute'][i] for i in (1, 3)) >= 20
    screening = all(v for k, v in gates.items() if not k.endswith('_confidence'))
    return {'schema': 2, 'passed': all(gates.values()), 'screening_passed': screening,
            'gates': gates, 'metrics': measured,
            'goal_outcomes': {kind: {key: sum(item['goal_outcomes'][kind][key] for item in results)
                                    for key in ['expected_unique', 'correct', 'false_verified']}
                              for kind in ['cup', 'blue']}, 'model_sha256': model.sha256, 'operating_point': model.identity,
            'manifest_sha256': manifest_hash, 'image_hashes': hashes,
            'scenes': results, 'opencv_version': cv2.__version__, 'evaluated_images': len(heldout), 'split': split,
            'scene_groups': len(groups['all']), 'processing_p95_sec': p95,
            'confidence_threshold': confidence, 'blue_fraction_threshold': blue_threshold,
            'uncertainty_unit': 'scene_group',
            'scope': 'offline image recognition only; not metric or physical task acceptance'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--model-name', default='yolox', choices=['yolox', 'rtdetr_v2_r18', 'grounding_dino_tiny'])
    parser.add_argument('--model-directory')
    parser.add_argument('--output', required=True)
    parser.add_argument('--split', choices=['development', 'heldout'], default='heldout')
    parser.add_argument('--confidence', type=float, default=.5)
    parser.add_argument('--blue-fraction', type=float, default=.35)
    args = parser.parse_args()
    try:
        use_model_runtime(args.model_name)
        report = evaluate(args.manifest, args.model_directory, name=args.model_name, split=args.split,
                          confidence=args.confidence, blue_threshold=args.blue_fraction)
    except (ValueError, OSError, RuntimeError, RecursionError, cv2.error) as error:
        report = {'schema': 2, 'passed': False, 'error': str(error)}
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': report['passed'], 'report': str(output)}))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
