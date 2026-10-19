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
from lekiwi_rmf.object_observations import artifact_pixels, capture_artifact, observation
from lekiwi_rmf.perception_profiles import make_profile, tracked_profiles

# Hard conditions the corpus must cover before its results describe them (spec T-15); the report lists
# which are missing instead of implying coverage.
HARD_CONDITIONS = ('dark', 'overhead', 'occlusion', 'transparent', 'reflective', 'blur', 'similar_instances')


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


def scene_objects(scene):
    """Schema-1 manifests labeled only cups; they remain valid as the label `cup`."""
    if ('objects' in scene) == ('cups' in scene):
        raise ValueError('each scene requires exactly one of objects or legacy cups')
    if 'cups' in scene:
        if not isinstance(scene['cups'], list):
            raise ValueError('cups must be a list')
        return [{'label': 'cup', **cup} if isinstance(cup, dict) else cup for cup in scene['cups']]
    return scene['objects']


def load_scenes(manifest, split='heldout'):
    path = Path(manifest)
    if path.stat().st_size > 1024 * 1024:
        raise ValueError('manifest exceeds 1 MiB')
    manifest_bytes = path.read_bytes()
    scenes = json.loads(manifest_bytes)
    if not isinstance(scenes, list) or not scenes or len(scenes) > 10000:
        raise ValueError('manifest requires 1..10000 scenes')
    seen, image_hashes, pixel_hashes, selected, groups = set(), {}, {}, [], {}
    labels, blue_labels = set(), set()
    for scene in scenes:
        if (not isinstance(scene, dict) or not {'scene_id', 'split', 'image'} <= set(scene)
                or set(scene) - {'scene_id', 'scene_group', 'split', 'image', 'objects', 'cups', 'conditions'}):
            raise ValueError('each scene requires scene_id, split, image and objects')
        conditions = scene.get('conditions', [])
        if (not isinstance(conditions, list) or len(conditions) > 8 or len(set(conditions)) != len(conditions)
                or any(not isinstance(tag, str) or not 1 <= len(tag) <= 32
                       or any(c not in 'abcdefghijklmnopqrstuvwxyz_' for c in tag) for tag in conditions)):
            raise ValueError('conditions are up to 8 distinct lower-case tags')
        objects = scene_objects(scene)
        if 'cups' in scene:
            labels.add('cup')  # A legacy cup list declares cups labeled even when this scene has none.
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
        if not isinstance(objects, list) or len(objects) > 20:
            raise ValueError('objects requires at most 20 labeled boxes')
        for item in objects:
            if (not isinstance(item, dict) or not {'label', 'bbox'} <= set(item)
                    or set(item) - {'label', 'bbox', 'blue'} or 'blue' in item and type(item['blue']) is not bool):
                raise ValueError('object labels require label, bbox and optional boolean blue')
            if not isinstance(item['label'], str) or not 1 <= len(item['label']) <= 128:
                raise ValueError('object label must be a native label of 1..128 characters')
            box = item['bbox']
            if (not isinstance(box, list) or len(box) != 4 or
                    any(type(value) not in (int, float) or not math.isfinite(value) for value in box)
                    or min(box[:2]) < 0 or min(box[2:]) <= 0):
                raise ValueError('invalid labeled object box')
            labels.add(item['label'])
            if 'blue' in item:
                blue_labels.add(item['label'])
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
        for item in objects:
            x, y, w, h = item['bbox']
            if x + w > pixels.shape[1] or y + h > pixels.shape[0]:
                raise ValueError('labeled box exceeds its image')
        if scene['split'] == split:
            selected.append(({**scene, 'objects': objects}, image_path, digest))
    if not selected:
        raise ValueError(f'manifest requires {split} scenes')
    if not labels:
        raise ValueError('manifest labels no object; its vocabulary would score nothing')
    # Every scene must label all instances of every manifest label, so absence is a true negative.
    return hashlib.sha256(manifest_bytes).hexdigest(), selected, sorted(labels), sorted(blue_labels)


def metrics(total):
    tp, fp, fn = total
    return {'tp': tp, 'fp': fp, 'fn': fn,
            'precision': tp / (tp + fp) if tp + fp else 0,
            'recall': tp / (tp + fn) if tp + fn else 0,
            'precision_95_ci': interval(tp, tp + fp), 'recall_95_ci': interval(tp, tp + fn)}


def goal_keys(labels, blue_labels):
    return [*labels, *('blue ' + label for label in blue_labels)]


def scene_metrics(detections, objects, blue_threshold, labels, blue_labels):
    """Score each labeled class and blue-attribute goal; predictions outside the vocabulary are unscored."""
    def outcome(predictions, expected):
        correct = len(predictions) == len(expected) == 1 and overlap(predictions[0], expected[0]) >= .5
        return {'expected_unique': len(expected) == 1, 'correct': bool(correct),
                'false_verified': len(predictions) == 1 and not correct}
    goals, outcomes, attribute = {}, {}, [0, 0, 0, 0]  # attribute: TP, FP, FN, TN on matched labeled objects.
    for label in labels:
        found = [item for item in detections if item['label'] == label]
        wanted = [item for item in objects if item['label'] == label]
        predicted, expected = [item['bbox'] for item in found], [item['bbox'] for item in wanted]
        goals[label], outcomes[label] = counts(predicted, expected), outcome(predicted, expected)
        if label not in blue_labels:
            continue
        for p, e in matches(predicted, expected):
            if 'blue' in wanted[e]:
                actual = found[p]['blue_fraction'] >= blue_threshold
                attribute[0 if actual and wanted[e]['blue'] else 1 if actual else 2 if wanted[e]['blue'] else 3] += 1
        blue_predicted = [item['bbox'] for item in found if item['blue_fraction'] >= blue_threshold]
        blue_expected = [item['bbox'] for item in wanted if item.get('blue')]
        goals['blue ' + label] = counts(blue_predicted, blue_expected)
        outcomes['blue ' + label] = outcome(blue_predicted, blue_expected)
    return {'goals': goals, 'blue_attribute': attribute, 'goal_outcomes': outcomes,
            'unscored_predictions': [item for item in detections if item['label'] not in labels],
            'predictions': detections, 'expected': objects}


def evaluate(manifest, model_path=None, *, name='yolox', split='heldout', profile=None, blue_threshold=.35,
             recognizer=None, query=None):
    """Score one recognizer under one proposal profile; defaults to the tracked default profile."""
    if split not in ('development', 'heldout') or not 0 <= blue_threshold <= 1:
        raise ValueError('invalid frozen evaluation operating point')
    manifest_hash, heldout, labels, blue_labels = load_scenes(manifest, split)
    keys = goal_keys(labels, blue_labels)
    cv2.setNumThreads(2)
    model, bridge = recognizer or Recognizer(name, model_path), CvBridge()
    profile = profile or model.default_profile
    # An open recognizer is evaluated under its tracked default query unless a study fixes another.
    query = query or model.default_query
    totals, attribute = {key: [0, 0, 0] for key in keys}, [0, 0, 0, 0]
    durations, hashes, results = [], [], []
    groups = {key: {'positive': set(), 'negative': set(), 'ambiguous': set()} for key in keys}
    all_groups = set()
    for sequence, (scene, image_path, digest) in enumerate(heldout, 1):
        content = image_path.read_bytes()
        if hashlib.sha256(content).hexdigest() != digest:
            raise ValueError('evaluation image changed after dataset validation')
        pixels = cv2.imdecode(np.frombuffer(content, np.uint8), cv2.IMREAD_COLOR)
        header = Header(stamp=Time(sec=sequence), frame_id='offline_camera')
        start = time.monotonic()
        # The same declared normalization as the robot: the recognizer consumes the evidence artifact.
        artifact = capture_artifact(pixels)
        found, extras, _ = detect(model, artifact_pixels(artifact), header, query, profile, artifact)
        image = bridge.cv2_to_imgmsg(pixels, 'bgr8')
        image.header = header
        record, _ = observation(image, found, model.sha256, 'offline', query, profile=profile, extras=extras,
                                input_sha256=hashlib.sha256(artifact).hexdigest())
        durations.append(time.monotonic() - start)
        result = scene_metrics(record['detections'], scene['objects'], blue_threshold, labels, blue_labels)
        for key in keys:
            totals[key] = [a + b for a, b in zip(totals[key], result['goals'][key])]
        attribute = [a + b for a, b in zip(attribute, result['blue_attribute'])]
        group = scene.get('scene_group', scene['scene_id'])
        all_groups.add(group)
        for key in keys:
            present = result['goals'][key][0] + result['goals'][key][2]
            groups[key]['positive' if present else 'negative'].add(group)
            if present > 1:
                groups[key]['ambiguous'].add(group)
        results.append({'scene_id': scene['scene_id'], 'scene_group': group, 'conditions': scene.get('conditions', []),
                        **result, 'abstentions': record['abstentions'], 'input_quality': record['input_quality'],
                        'processing_sec': durations[-1]})
        hashes.append(digest)
    measured = {key: metrics(value) for key, value in totals.items()}
    p95 = sorted(durations)[max(0, math.ceil(.95 * len(durations)) - 1)] if durations else None
    gates = {'heldout_split': split == 'heldout', 'heldout_scenes': len(all_groups) >= 80,
             'latency': p95 is not None and p95 <= 1.5,
             'no_false_verified': not any(outcome['false_verified'] for item in results
                                         for outcome in item['goal_outcomes'].values())}
    for key in keys:
        gates[key + '_positive_scenes'] = len(groups[key]['positive']) >= 20
        gates[key + '_negative_scenes'] = len(groups[key]['negative']) >= 20
        gates[key + '_ambiguous_scenes'] = len(groups[key]['ambiguous']) >= 10
    # Qualify with scene-group trials, not correlated objects or adjacent frames.
    for key in keys:
        grouped = {}
        for item in results:
            prior = grouped.setdefault(item['scene_group'], [0, 0, 0])
            grouped[item['scene_group']] = [a + b for a, b in zip(prior, item['goals'][key])]
        for metric, threshold, denominator, error in [('precision', .98, 1, 1), ('recall', .9, 2, 2)]:
            trials = [value for value in grouped.values() if value[0] + value[denominator] > 0]
            bounds = interval(sum(value[error] == 0 for value in trials), len(trials))
            measured[key]['scene_' + metric + '_95_ci'] = bounds
            gates[key + '_' + metric] = measured[key][metric] >= threshold
            gates[key + '_' + metric + '_confidence'] = bounds is not None and bounds[0] >= threshold
    if blue_labels:
        measured['blue_attribute'] = {**metrics(attribute[:3]), 'tn': attribute[3]}
        gates['blue_positive_matches'] = attribute[0] + attribute[2] >= 20
        gates['nonblue_matches'] = attribute[1] + attribute[3] >= 20
    screening = all(v for k, v in gates.items() if not k.endswith('_confidence'))
    return {'schema': 3, 'passed': all(gates.values()), 'screening_passed': screening,
            'gates': gates, 'metrics': measured, 'labels': labels, 'blue_labels': blue_labels,
            'goal_outcomes': {key: {name: sum(item['goal_outcomes'][key][name] for item in results)
                                    for name in ['expected_unique', 'correct', 'false_verified']}
                              for key in keys},
            'unscored_predictions': sum(len(item['unscored_predictions']) for item in results),
            'model_sha256': model.sha256, 'operating_point': model.identity, 'profile': profile.document(),
            'query': query.document() if query is not None else None,
            'conditions': {tag: sum(tag in item['conditions'] for item in results)
                           for tag in sorted({tag for item in results for tag in item['conditions']} | set(HARD_CONDITIONS))},
            'missing_hard_conditions': [tag for tag in HARD_CONDITIONS
                                        if not any(tag in item['conditions'] for item in results)],
            'manifest_sha256': manifest_hash, 'image_hashes': hashes,
            'scenes': results, 'opencv_version': cv2.__version__, 'evaluated_images': len(heldout), 'split': split,
            'scene_groups': len(all_groups), 'processing_p95_sec': p95,
            'blue_fraction_threshold': blue_threshold,
            'uncertainty_unit': 'scene_group',
            'scope': 'offline image recognition only; not metric or physical task acceptance'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--model-name', default='yolox',
                        choices=['yolox', 'rtdetr_v2_r18', 'grounding_dino_tiny', 'grounding_dino_tiny_gpu'])
    parser.add_argument('--model-directory')
    parser.add_argument('--output', required=True)
    parser.add_argument('--split', choices=['development', 'heldout'], default='heldout')
    parser.add_argument('--profile', default='', help='tracked profile name (config/perception_profiles.yaml)')
    parser.add_argument('--profile-json', default='', help='complete candidate parameters as JSON')
    parser.add_argument('--blue-fraction', type=float, default=.35)
    args = parser.parse_args()
    try:
        use_model_runtime(args.model_name)
        model = Recognizer(args.model_name, args.model_directory)
        profile = None
        if args.profile and args.profile_json:
            raise ValueError('choose --profile or --profile-json')
        if args.profile:
            tracked = tracked_profiles()[1][model.backend]
            if args.profile not in tracked:
                raise ValueError(f'{args.profile} is not a tracked {model.backend} profile')
            profile = tracked[args.profile]
        elif args.profile_json:
            profile = make_profile(model.backend, json.loads(args.profile_json))
        report = evaluate(args.manifest, name=args.model_name, split=args.split, profile=profile,
                          blue_threshold=args.blue_fraction, recognizer=model)
    except (ValueError, OSError, RuntimeError, RecursionError, cv2.error) as error:
        report = {'schema': 3, 'passed': False, 'error': str(error)}
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': report['passed'], 'report': str(output)}))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
