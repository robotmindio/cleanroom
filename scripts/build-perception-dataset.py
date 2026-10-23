#!/usr/bin/env python3
"""Freeze taught and corrected observations into a reviewable perception dataset; never commands the robot.

Input: an annotation file listing views and their labeled boxes, each annotation with its
author, method, confidence and revision:

  {"dataset_id": "kitchen-cans", "views": [
    {"view_id": "v1", "scene_group": "counter-2026-10-19", "split": "development", "conditions": ["dark"],
     "source": {"observation_id": "<sha256>"} | {"image": "photo.jpg"}, "provenance": "knowledge:<IRI>",
     "labeled_by": {"author": "user:ana", "method": "human_review"},
     "objects": [{"label": "soda can", "bbox": [x, y, w, h],
                  "annotation": {"author": "user:ana", "method": "human_correction", "confidence": 1.0,
                                 "revision": "r1"}}]}]}

`labeled_by` asserts that every instance of every dataset label in the view is boxed, so
absence is a true negative. An observation source is read byte-exact from the observation
store (the evidence artifact the recognizer consumed); an image source is normalized the way
the robot normalizes a frame (lekiwi_rmf.object_observations.capture_artifact), so every
stored image is a model input.

Rules: held-out views carry only human annotations and a human `labeled_by`, so automatic
annotations and pseudo-labels never become qualification evidence. `--unreviewed-heldout`
admits them anyway for a measurement-only dataset: its manifest says `heldout_authority:
unreviewed`, and scripts/register-recognizer.py never promotes on it. A scene group never spans
both splits; exact and near-duplicate images (8x8 average hash within 4 bits) never span
them either. The output directory holds images/, scenes.json (the input of
scripts/evaluate-object-recognition.py) and dataset.json (the manifest, with every file's
SHA-256), which scripts/train-recognizer.py verifies before training.
"""

import argparse
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import sys

import cv2
import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))
from lekiwi_rmf.object_observations import artifact_pixels, capture_artifact  # noqa: E402

HUMAN_METHODS = ('human_teaching', 'human_correction', 'human_review')
METHODS = (*HUMAN_METHODS, 'automatic')
NEAR_DUPLICATE_BITS = 4
STORE = Path.home() / '.ros/lekiwi/observations_0.sqlite3'


class DatasetError(ValueError):
    pass


def evaluator():
    spec = importlib.util.spec_from_file_location('evaluate_object_recognition',
                                                  REPOSITORY / 'scripts/evaluate-object-recognition.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def average_hash(pixels):
    gray = cv2.resize(cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY), (8, 8), interpolation=cv2.INTER_AREA)
    return int(''.join('1' if value else '0' for value in (gray > gray.mean()).flatten()), 2)


def provenance(value, name):
    keys = {'author', 'method', 'confidence', 'revision'} if name == 'annotation' else {'author', 'method'}
    if not isinstance(value, dict) or set(value) != keys:
        raise DatasetError(f'{name} needs exactly {sorted(keys)}')
    if not isinstance(value['author'], str) or not 1 <= len(value['author']) <= 128 or value['method'] not in METHODS:
        raise DatasetError(f'{name} needs an author and a method in {METHODS}')
    if name == 'annotation' and (type(value['confidence']) not in (int, float) or not 0 <= value['confidence'] <= 1
                                 or not isinstance(value['revision'], str) or not 1 <= len(value['revision']) <= 128):
        raise DatasetError('annotation confidence must be in [0, 1] and revision a short string')
    return dict(value)


def observation_artifact(connection, identifier):
    """The stored evidence bytes of one observation, proved by the digest its record declares."""
    if not isinstance(identifier, str) or not re.fullmatch(r'[0-9a-f]{64}', identifier):
        raise DatasetError('observation_id must be a SHA-256')
    row = connection.execute('SELECT document, evidence FROM observations WHERE id = ?', (identifier,)).fetchone()
    if row is None:
        raise DatasetError(f'observation {identifier[:12]} is not in the store')
    document, evidence = json.loads(row[0]), bytes(row[1])
    if hashlib.sha256(evidence).hexdigest() != document['evidence']['sha256']:
        raise DatasetError(f'observation {identifier[:12]} evidence does not match its record')
    return evidence, {'observation_id': identifier, 'camera_id': document['camera_id'], 'stamp': document['stamp'],
                      'model_sha256': document.get('model_sha256'), 'evidence_sha256': document['evidence']['sha256']}


def image_artifact(path):
    content = path.read_bytes()
    pixels = cv2.imdecode(np.frombuffer(content, np.uint8), cv2.IMREAD_COLOR)
    if pixels is None:
        raise DatasetError(f'{path}: unreadable image')
    return capture_artifact(pixels), {'image_sha256': hashlib.sha256(content).hexdigest(), 'file': path.name}


def build(annotations_path, output, store=None, unreviewed_heldout=False):
    annotations_path = Path(annotations_path)
    document = json.loads(annotations_path.read_text())
    if not isinstance(document, dict) or set(document) - {'dataset_id', 'description', 'views'}:
        raise DatasetError('annotation file needs dataset_id, views and an optional description')
    dataset_id, views = document.get('dataset_id'), document.get('views')
    if not isinstance(dataset_id, str) or not re.fullmatch(r'[a-z0-9][a-z0-9_.-]{0,63}', dataset_id):
        raise DatasetError('dataset_id must be a lowercase slug')
    if not isinstance(views, list) or not 1 <= len(views) <= 10000:
        raise DatasetError('a dataset needs 1..10000 views')
    final = Path(output)
    if final.exists():
        raise DatasetError(f'{final} exists; a frozen dataset is never rewritten')
    # Built beside its destination and renamed once complete, so a failure leaves no partial dataset.
    output = final.with_name(f'.{final.name}.partial')
    shutil.rmtree(output, ignore_errors=True)
    (output / 'images').mkdir(parents=True)
    try:
        manifest = _build(annotations_path, document, dataset_id, views, output, store, unreviewed_heldout)
    except BaseException:
        shutil.rmtree(output, ignore_errors=True)
        raise
    os.replace(output, final)
    return manifest


def _build(annotations_path, document, dataset_id, views, output, store, unreviewed_heldout):
    connection = None
    scenes, records, hashes = [], [], {}
    try:
        for view in views:
            keys = {'view_id', 'scene_group', 'split', 'source', 'objects', 'labeled_by'}
            if not isinstance(view, dict) or not keys <= set(view) or set(view) - keys - {'conditions', 'provenance'}:
                raise DatasetError(f'each view needs {sorted(keys)} and optional conditions, provenance')
            labeled_by = provenance(view['labeled_by'], 'labeled_by')
            source = view['source']
            if isinstance(source, dict) and set(source) == {'observation_id'}:
                if connection is None:
                    connection = sqlite3.connect(f'file:{Path(store or STORE)}?mode=ro', uri=True)
                artifact, origin = observation_artifact(connection, source['observation_id'])
            elif isinstance(source, dict) and set(source) == {'image'} and isinstance(source['image'], str):
                path = Path(source['image'])
                artifact, origin = image_artifact(path if path.is_absolute() else annotations_path.parent / path)
            else:
                raise DatasetError('source is {"observation_id": ...} or {"image": ...}')
            objects = view['objects'] if isinstance(view['objects'], list) else None
            if objects is None or any(not isinstance(item, dict) or set(item) != {'label', 'bbox', 'annotation'}
                                      for item in objects):
                raise DatasetError('objects are {label, bbox, annotation}')
            methods = [provenance(item['annotation'], 'annotation') for item in objects]
            if view['split'] == 'heldout' and not unreviewed_heldout and (
                    labeled_by['method'] not in HUMAN_METHODS or any(item['method'] not in HUMAN_METHODS
                                                                     for item in methods)):
                raise DatasetError(f'{view["view_id"]}: held-out views carry only human annotations; automatic '
                                   'labels and pseudo-labels are never evaluation evidence')
            digest = hashlib.sha256(artifact).hexdigest()
            image = f'images/{digest}.jpg'
            (output / image).write_bytes(artifact)
            hashes.setdefault(view['split'], []).append((average_hash(artifact_pixels(artifact)), view['view_id'],
                                                         view['scene_group']))
            scene = {'scene_id': view['view_id'], 'scene_group': view['scene_group'], 'split': view['split'],
                     'image': image, 'objects': [{'label': item['label'], 'bbox': item['bbox']} for item in objects]}
            if view.get('conditions'):
                scene['conditions'] = view['conditions']
            scenes.append(scene)
            records.append({'view_id': view['view_id'], 'image_sha256': digest, 'source': origin,
                            'provenance': str(view.get('provenance', ''))[:256], 'labeled_by': labeled_by,
                            'annotations': [{'label': item['label'], 'bbox': item['bbox'], **method}
                                            for item, method in zip(objects, methods)]})
    finally:
        if connection is not None:
            connection.close()
    # Near-identical views of one trial must not let held-out measure memorization of development.
    for held, view_id, group in hashes.get('heldout', []):
        for development, other, other_group in hashes.get('development', []):
            if bin(held ^ development).count('1') <= NEAR_DUPLICATE_BITS:
                raise DatasetError(f'held-out view {view_id} nearly duplicates development view {other} '
                                   f'({group} / {other_group}); split by session, not by frame')
    (output / 'scenes.json').write_text(json.dumps(scenes, indent=1, sort_keys=True) + '\n')
    module = evaluator()
    splits = {}
    for split in ('development', 'heldout'):
        if any(scene['split'] == split for scene in scenes):
            # The evaluator's own validation: unique IDs, groups within one split, boxes inside images,
            # no repeated content, every instance of every label boxed.
            _, selected, labels, _ = module.load_scenes(output / 'scenes.json', split)
            splits[split] = {'scenes': len(selected), 'scene_groups': len({scene['scene_group'] for scene, _, _ in selected})}
    if 'development' not in splits or 'heldout' not in splits:
        raise DatasetError('a training dataset needs development and held-out views')
    files = {path.relative_to(output).as_posix(): {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                                  'bytes': path.stat().st_size}
             for path in sorted(output.rglob('*')) if path.is_file()}
    counts = {}
    for record in records:
        for item in record['annotations']:
            counts[item['method']] = counts.get(item['method'], 0) + 1
    manifest = {'schema': 1, 'kind': 'lekiwi_perception_dataset', 'dataset_id': dataset_id,
                'description': str(document.get('description', ''))[:1000],
                'created_utc': datetime.datetime.now(datetime.UTC).isoformat(timespec='seconds'),
                'input_normalization': 'evidence_jpeg_q80', 'labels': labels,
                'heldout_authority': 'unreviewed' if unreviewed_heldout else 'human',
                'scenes_sha256': files['scenes.json']['sha256'], 'splits': splits, 'annotation_methods': counts,
                'annotation_file_sha256': hashlib.sha256(annotations_path.read_bytes()).hexdigest(),
                'views': records, 'files': files}
    temporary = output / '.dataset.json.tmp'
    temporary.write_text(json.dumps(manifest, indent=1, sort_keys=True) + '\n')
    os.replace(temporary, output / 'dataset.json')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('annotations', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='new dataset directory')
    parser.add_argument('--store', type=Path, default=STORE, help='observation store for observation_id sources')
    parser.add_argument('--unreviewed-heldout', action='store_true',
                        help='admit non-human held-out labels; the dataset can never qualify a recognizer')
    args = parser.parse_args()
    try:
        manifest = build(args.annotations, args.output, args.store, args.unreviewed_heldout)
    except (DatasetError, ValueError, OSError, sqlite3.Error) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    print(json.dumps({'output': str(args.output), 'labels': manifest['labels'], 'splits': manifest['splits'],
                      'annotation_methods': manifest['annotation_methods']}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
