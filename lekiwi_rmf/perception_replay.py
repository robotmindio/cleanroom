"""Replay a stored capture under a chosen model, profile and query.

A capture artifact is the exact bytes a recognizer consumed (the observation's evidence JPEG, or the same
bytes stored byte-exact in Knowledge), so replaying it under the original operating point reproduces the
original inference, and replaying it under another operating point is a new inference with its own ID.
Nothing here commands the robot or writes the observation store.
"""

import hashlib
import math

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from lekiwi_rmf.object_detector import detect
from lekiwi_rmf.object_observations import artifact_pixels, inference_id
from lekiwi_rmf.perception_profiles import recorded_profile
from lekiwi_rmf.perception_query import recorded_query

SCORE_TOLERANCE, BOX_TOLERANCE = 1e-4, 1e-2


def detections_of(found, extras):
    result = []
    for detection, extra in zip(found.detections, extras):
        box, hypothesis = detection.bbox, detection.results[0].hypothesis
        result.append({'label': hypothesis.class_id, 'score': hypothesis.score,
                       'bbox': [box.center.position.x - box.size_x / 2, box.center.position.y - box.size_y / 2,
                                box.size_x, box.size_y], **extra})
    return result


def same_detections(left, right):
    if len(left) != len(right):
        return False
    key = lambda item: (item['label'], -item['score'])  # noqa: E731
    return all(a['label'] == b['label'] and abs(a['score'] - b['score']) <= SCORE_TOLERANCE
               and all(math.isclose(x, y, abs_tol=BOX_TOLERANCE) for x, y in zip(a['bbox'], b['bbox']))
               for a, b in zip(sorted(left, key=key), sorted(right, key=key)))


def replay(record, artifact, recognizer, *, profile=None, query=None):
    """Re-run `record`'s capture from `artifact`; defaults to the record's own operating point."""
    evidence = record.get('evidence') or {}
    if hashlib.sha256(artifact).hexdigest() != evidence.get('sha256'):
        raise ValueError('artifact is not the byte-exact evidence of that observation')
    original_profile = recorded_profile(record['profile'], recognizer.backend) if record.get('model_sha256') == recognizer.sha256 else None
    profile = profile or original_profile or recognizer.default_profile
    original_query = recorded_query(record['query']) if record.get('vocabulary') == 'open' else None
    if recognizer.open_vocabulary:
        query = query or original_query or recognizer.default_query
    elif query is not None:
        raise ValueError('a closed-vocabulary recognizer cannot answer a query')
    header = Header(stamp=Time(sec=record['stamp']['sec'], nanosec=record['stamp']['nanosec']), frame_id=record['frame_id'])
    found, extras, run = detect(recognizer, artifact_pixels(artifact), header, query, profile, artifact)
    identifier = inference_id(record['capture_id'], recognizer.sha256, profile, query)
    detections = detections_of(found, extras)
    same = identifier == record['id']
    return {'schema': 1, 'kind': 'perception_replay', 'id': identifier, 'source_observation_id': record['id'],
            'capture_id': record['capture_id'], 'artifact_sha256': evidence['sha256'],
            'model_sha256': recognizer.sha256, 'profile': profile.document(),
            'query': query.document() if query is not None else None, 'detections': detections,
            'provider_run': run, 'same_operating_point': same,
            # The original consumed these bytes only if it recorded them as its model input.
            'reproduced': (same_detections(detections, [{key: item[key] for key in ('label', 'score', 'bbox')}
                                                         for item in record['detections']])
                           if same and evidence.get('model_input') else None)}
