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
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_detector import detect
from lekiwi_rmf.object_model import MODEL_NAME, MODEL_SHA256
from lekiwi_rmf.object_observations import observation
from lekiwi_rmf.opencv_yolox import YoloX


def overlap(left, right):
    x, y = max(left[0], right[0]), max(left[1], right[1])
    width = max(0, min(left[0] + left[2], right[0] + right[2]) - x)
    height = max(0, min(left[1] + left[3], right[1] + right[3]) - y)
    area = width * height
    return area / (left[2] * left[3] + right[2] * right[3] - area)


def counts(predictions, expected):
    """Match each labeled target once; extra same-class candidates count as false positives."""
    matches = []
    for p, prediction in enumerate(predictions):
        for e, target in enumerate(expected):
            score = overlap(prediction, target)
            if score >= .5:
                matches.append((score, p, e))
    used_predictions, used_expected = set(), set()
    for _, p, e in sorted(matches, reverse=True):
        if p not in used_predictions and e not in used_expected:
            used_predictions.add(p)
            used_expected.add(e)
    return len(used_predictions), len(predictions) - len(used_predictions), len(expected) - len(used_expected)


def interval(successes, total):
    if not total:
        return None
    z, rate = 1.96, successes / total
    denominator = 1 + z * z / total
    centre = (rate + z * z / (2 * total)) / denominator
    margin = z * math.sqrt(rate * (1 - rate) / total + z * z / (4 * total * total)) / denominator
    return [max(0, centre - margin), min(1, centre + margin)]


def evaluate(manifest, model_path):
    path = Path(manifest)
    if path.stat().st_size > 1024 * 1024:
        raise ValueError('manifest exceeds 1 MiB')
    scenes = json.loads(path.read_text())
    if not isinstance(scenes, list) or not scenes or len(scenes) > 10000:
        raise ValueError('manifest requires 1..10000 scenes')
    seen, heldout = set(), []
    for scene in scenes:
        if not isinstance(scene, dict) or set(scene) != {'scene_id', 'split', 'image', 'cups'}:
            raise ValueError('each scene requires scene_id, split, image and cups')
        identifier = scene['scene_id']
        if not isinstance(identifier, str) or not identifier or identifier in seen:
            raise ValueError('scene IDs must be unique; split by scene, not adjacent video frames')
        seen.add(identifier)
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
        if scene['split'] == 'heldout':
            heldout.append(scene)
    model_path = Path(model_path).expanduser()
    with model_path.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != MODEL_SHA256:
            raise ValueError('evaluation model hash differs from the pinned artifact')
    cv2.setNumThreads(2)
    model, bridge = YoloX(str(model_path), confThreshold=.5), CvBridge()
    totals, durations, hashes = [0, 0, 0], [], []
    positive = negative = ambiguous = 0
    for sequence, scene in enumerate(heldout, 1):
        image_path = Path(scene['image'])
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        if image_path.stat().st_size > 8 * 1024**2:
            raise ValueError('evaluation image exceeds 8 MiB')
        pixels = cv2.imread(str(image_path))
        if pixels is None or pixels.shape[0] > 1080 or pixels.shape[1] > 1920:
            raise ValueError('evaluation image unreadable or exceeds 1920x1080')
        for cup in scene['cups']:
            x, y, w, h = cup['bbox']
            if x + w > pixels.shape[1] or y + h > pixels.shape[0]:
                raise ValueError('labeled box exceeds its image')
        header = Header(stamp=Time(sec=sequence), frame_id='offline_camera')
        start = time.monotonic()
        found = detect(model, pixels, header)
        image = bridge.cv2_to_imgmsg(pixels, 'bgr8')
        image.header = header
        record, _ = observation(image, found, MODEL_SHA256, 'offline')
        durations.append(time.monotonic() - start)
        predictions = [item['bbox'] for item in record['detections'] if item['blue_fraction'] >= .35]
        expected = [item['bbox'] for item in scene['cups'] if item['blue']]
        positive += bool(expected)
        negative += not expected
        ambiguous += len(expected) > 1
        totals = [a + b for a, b in zip(totals, counts(predictions, expected))]
        hashes.append(hashlib.sha256(image_path.read_bytes()).hexdigest())
    tp, fp, fn = totals
    precision, recall = tp / (tp + fp) if tp + fp else 0, tp / (tp + fn) if tp + fn else 0
    p95 = sorted(durations)[max(0, math.ceil(.95 * len(durations)) - 1)] if durations else None
    gates = {'heldout_scenes': len(heldout) >= 80, 'positive_scenes': positive >= 20,
             'negative_scenes': negative >= 20, 'ambiguous_scenes': ambiguous >= 10,
             'precision': precision >= .98, 'recall': recall >= .9, 'latency': p95 is not None and p95 <= 1.5}
    return {'schema': 1, 'passed': all(gates.values()), 'gates': gates, 'model_sha256': MODEL_SHA256,
            'manifest_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'image_hashes': hashes,
            'opencv_version': cv2.__version__, 'heldout_scenes': len(heldout), 'tp': tp, 'fp': fp, 'fn': fn,
            'precision': precision, 'recall': recall, 'precision_95_ci': interval(tp, tp + fp),
            'recall_95_ci': interval(tp, tp + fn), 'processing_p95_sec': p95,
            'confidence_threshold': .5, 'blue_fraction_threshold': .35,
            'scope': 'offline image recognition only; not metric or physical task acceptance'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--model', default=str(Path.home() / '.ros/lekiwi/models' / MODEL_NAME))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        report = evaluate(args.manifest, args.model)
    except (ValueError, OSError, RuntimeError, RecursionError, cv2.error) as error:
        report = {'schema': 1, 'passed': False, 'error': str(error)}
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': report['passed'], 'report': str(output)}))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
