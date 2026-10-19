"""Proposal profiles: the post-model operating point of a recognizer, separate from its model identity.

A profile is a complete, validated set of backend parameters (thresholds, suppression, resolution,
budget) with a content revision. The model identity (`model_sha256`) pins weights, runtime and
preprocessing; the profile revision pins how its raw output becomes detections. Every frame records
both, so evidence produced under one profile is never credited to another.
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import yaml

from lekiwi_rmf.object_cameras import config_directory

FAMILY = 'proposal'
# The observation and Fiber contracts cap one capture at 20 detections.
MAX_DETECTIONS = 20


def _number(minimum, maximum, unit, cost, semantics, *, integer=False, exclusive_minimum=False):
    return {'type': 'integer' if integer else 'number', 'minimum': minimum, 'maximum': maximum,
            'exclusive_minimum': exclusive_minimum, 'unit': unit, 'cost': cost, 'semantics': semantics}


SCORE = _number(0, 1, 'score', 'postprocess', 'candidates scoring at or below this are discarded',
                exclusive_minimum=True)
NMS_IOU = _number(.05, 1, 'intersection over union', 'postprocess',
                  'a candidate overlapping a stronger one of its suppression group above this is suppressed; 1 disables')
MAX = _number(1, MAX_DETECTIONS, 'detections', 'postprocess', 'highest-scoring detections kept per frame', integer=True)
NMS_SCOPE = {'type': 'enum', 'values': ['label', 'prompt'], 'unit': None, 'cost': 'postprocess',
             'semantics': "'label' suppresses only within one label; 'prompt' treats every query phrase as one group "
                          "(synonyms), keeping the suppressed phrases as alternatives of the survivor"}
TEXT = _number(0, 1, 'token probability', 'inference',
               'prompt tokens above this name a detection; changing it needs a new inference', exclusive_minimum=True)
EDGE = _number(240, 1333, 'pixels', 'inference', 'shortest image edge after resizing; longest edge stays capped at 1333',
               integer=True)
FLOOR = _number(0, 1, 'score', 'retention', 'the provider retains proposals above this for replay; at most box_threshold',
                exclusive_minimum=True)

OPEN = {'box_threshold': SCORE, 'text_threshold': TEXT, 'nms_iou': NMS_IOU, 'nms_scope': NMS_SCOPE,
        'max_detections': MAX, 'shortest_edge': EDGE}
CLOSED = {'score_threshold': SCORE, 'nms_iou': NMS_IOU, 'max_detections': MAX}
PARAMETERS = {'yolox': CLOSED, 'rtdetr_v2': CLOSED, 'grounding_dino': OPEN,
              'remote_grounding_dino': {**OPEN, 'proposal_floor': FLOOR}}
# Parameters a backend cannot honour, with the reason; asking for one is refused, never silently ignored.
CLOSED_REASON = 'a closed catalogue has no phrase grounding; use score_threshold'
UNSUPPORTED = {
    'yolox': {'shortest_edge': 'the pinned ONNX graph has a fixed 640x640 letterboxed input',
              'box_threshold': CLOSED_REASON, 'text_threshold': CLOSED_REASON, 'nms_scope': 'classes are suppressed per label',
              'proposal_floor': 'local inference retains no proposals for replay'},
    'rtdetr_v2': {'shortest_edge': 'input_size is part of the pinned model spec; another size is another qualification',
                  'box_threshold': CLOSED_REASON, 'text_threshold': CLOSED_REASON, 'nms_scope': 'classes are suppressed per label',
                  'proposal_floor': 'local inference retains no proposals for replay'},
    'grounding_dino': {'score_threshold': 'phrase grounding uses box_threshold and text_threshold',
                       'proposal_floor': 'local inference retains no proposals for replay'},
    'remote_grounding_dino': {'score_threshold': 'phrase grounding uses box_threshold and text_threshold'},
}


def specification(backend):
    if backend not in PARAMETERS:
        raise ValueError(f'no proposal profile parameters for backend {backend}')
    return {'family': FAMILY, 'backend': backend, 'parameters': PARAMETERS[backend],
            'unsupported': UNSUPPORTED[backend]}


def _validate(backend, parameters):
    spec = specification(backend)['parameters']
    if not isinstance(parameters, dict):
        raise ValueError('profile parameters must be an object')
    for name in parameters:
        if name not in spec:
            reason = UNSUPPORTED[backend].get(name)
            raise ValueError(f'{name} is not supported by {backend}: {reason}' if reason else
                             f'{name} is not a {backend} profile parameter')
    missing = sorted(set(spec) - set(parameters))
    if missing:
        raise ValueError(f'profile is incomplete; missing {", ".join(missing)}')
    values = {}
    for name, rule in spec.items():
        value = parameters[name]
        if rule['type'] == 'enum':
            if value not in rule['values']:
                raise ValueError(f'{name} must be one of {", ".join(rule["values"])}')
        else:
            kinds = (int,) if rule['type'] == 'integer' else (int, float)
            if type(value) not in kinds or not math.isfinite(value):
                raise ValueError(f'{name} must be a finite {rule["type"]}')
            low_ok = value > rule['minimum'] if rule['exclusive_minimum'] else value >= rule['minimum']
            if not low_ok or value > rule['maximum']:
                bracket = '(' if rule['exclusive_minimum'] else '['
                raise ValueError(f'{name} must be within {bracket}{rule["minimum"]}, {rule["maximum"]}]')
            value = int(value) if rule['type'] == 'integer' else float(value)
        values[name] = value
    if 'proposal_floor' in values and values['proposal_floor'] > values['box_threshold']:
        raise ValueError('proposal_floor must not exceed box_threshold')
    return values


@dataclass(frozen=True)
class Profile:
    name: str
    backend: str
    items: tuple

    @property
    def parameters(self):
        return dict(self.items)

    def get(self, key, default=None):
        return self.parameters.get(key, default)

    @property
    def revision(self):
        canonical = json.dumps({'schema': 1, 'family': FAMILY, 'backend': self.backend, 'parameters': self.parameters},
                               sort_keys=True, separators=(',', ':'))
        return hashlib.sha256(canonical.encode()).hexdigest()

    def document(self):
        return {'name': self.name, 'family': FAMILY, 'backend': self.backend, 'revision': self.revision,
                'parameters': self.parameters}


def make_profile(backend, parameters, name='candidate'):
    if not isinstance(name, str) or not 1 <= len(name) <= 64 or any(
            not (char.isalnum() or char in '-_.') for char in name):
        raise ValueError('profile names are 1..64 letters, digits, hyphens, underscores or dots')
    return Profile(name, backend, tuple(sorted(_validate(backend, parameters).items())))


def recorded_profile(document, backend):
    """A profile as a frame carried it; its revision must match its own parameters and backend."""
    if (not isinstance(document, dict) or set(document) != {'name', 'family', 'backend', 'revision', 'parameters'}
            or document['family'] != FAMILY or document['backend'] != backend):
        raise ValueError('frame profile is not a proposal profile of this recognizer')
    profile = make_profile(backend, document['parameters'], document['name'])
    if profile.revision != document['revision']:
        raise ValueError('frame profile revision does not match its parameters')
    return profile


def tracked_profiles(path=None):
    """The tracked profiles, by backend and name; startup uses each backend's `default`."""
    path = Path(path) if path else config_directory() / 'perception_profiles.yaml'
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or document.get('schema') != 1 or set(document) != {'schema', 'default', 'profiles'}:
        raise ValueError('perception profiles require schema 1, default and profiles')
    profiles = document['profiles']
    if not isinstance(profiles, dict) or set(profiles) != set(PARAMETERS):
        raise ValueError(f'perception profiles must cover exactly {", ".join(sorted(PARAMETERS))}')
    result = {}
    for backend, named in profiles.items():
        if not isinstance(named, dict) or document['default'] not in named:
            raise ValueError(f'{backend} needs the default profile {document["default"]!r}')
        result[backend] = {name: make_profile(backend, parameters, name) for name, parameters in named.items()}
    return document['default'], result


def default_profile(backend, path=None):
    default, profiles = tracked_profiles(path)
    return profiles[backend][default]
