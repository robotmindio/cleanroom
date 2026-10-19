"""Registered skill implementations: a tracked contract, independent of ROS and of any one task.

A skill is data: an identity, a version, an implementation artifact pinned by
digest, the observations and action space it was trained on, and limits. Adding a
skill edits config/robot_skills.yaml; no runtime code names a skill.
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import yaml

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS
from lekiwi_rmf.object_cameras import config_directory

SKILL_ID = re.compile(r'^[a-z][a-z0-9_]{0,63}$')
KINDS = ('lerobot_policy',)
STATUSES = ('experimental', 'qualified')
BASE_COMMANDS = ('base_x_vel', 'base_y_vel', 'base_yaw_rate')
CAMERAS = ('front', 'wrist', 'astra')
PARAMETER_TYPES = {'number': (int, float), 'boolean': (bool,), 'string': (str,)}
ARTIFACT_SUFFIXES = ('.safetensors', '.json')
MAX_JSON_BYTES = 1024**2
# Keys through which Hugging Face, LeRobot and Hydra-style configurations name code to import.
IMPORT_KEYS = frozenset({'class', 'auto_map', '_target_', 'trust_remote_code', 'custom_pipeline'})
# Provenance records registration adds beside the checkpoint. LeRobot never opens them, and the
# evaluation report is written after the digest it names, so they are outside the artifact digest.
PROVENANCE_RECORDS = frozenset({'training-manifest.json', 'evaluation-report.json'})


@dataclass(frozen=True)
class Skill:
    skill_id: str
    version: int
    description: str
    status: str
    kind: str
    checkpoint: Path
    artifact_sha256: str
    action_names: tuple
    cameras: tuple
    fps: int
    max_duration_sec: float
    max_joint_step_rad: float
    parameters: dict

    def catalog(self):
        return {'skill_id': self.skill_id, 'version': self.version, 'status': self.status,
                'description': self.description, 'parameters': self.parameters,
                'max_duration_sec': self.max_duration_sec}


def _skill(skill_id, value, artifacts):
    if not SKILL_ID.match(skill_id) or not isinstance(value, dict):
        raise ValueError(f'invalid skill entry {skill_id!r}')
    known = {'version', 'description', 'status', 'kind', 'checkpoint', 'artifact_sha256', 'action_names',
             'cameras', 'fps', 'max_duration_sec', 'max_joint_step_rad', 'parameters'}
    if 'weights_sha256' in value:
        # The old field pinned model.safetensors alone; such an entry never vouches for a whole checkpoint.
        raise ValueError(f'{skill_id}: weights_sha256 pins only the weights; re-register for artifact_sha256')
    if set(value) != known:
        raise ValueError(f'{skill_id}: fields must be exactly {sorted(known)}')
    if type(value['version']) is not int or value['version'] < 1:
        raise ValueError(f'{skill_id}: version must be a positive integer')
    if value['status'] not in STATUSES or value['kind'] not in KINDS:
        raise ValueError(f'{skill_id}: status must be {STATUSES} and kind {KINDS}')
    if not isinstance(value['description'], str) or not 1 <= len(value['description']) <= 500:
        raise ValueError(f'{skill_id}: description must be 1..500 characters')
    checkpoint = Path(str(value['checkpoint']))
    if checkpoint.is_absolute() or '..' in checkpoint.parts:
        raise ValueError(f'{skill_id}: checkpoint must be relative to the skill artifact directory')
    digest = value['artifact_sha256']
    if not isinstance(digest, str) or not re.fullmatch(r'[0-9a-f]{64}', digest):
        raise ValueError(f'{skill_id}: artifact_sha256 must be lowercase SHA-256')
    names = value['action_names']
    if (not isinstance(names, list) or len(set(names)) != len(names)
            or set(names) - set(ARM_JOINTS) - set(BASE_COMMANDS) or not set(names) & set(ARM_JOINTS)):
        raise ValueError(f'{skill_id}: action_names must be distinct arm joints, optionally with base commands')
    cameras = value['cameras']
    if not isinstance(cameras, list) or len(set(cameras)) != len(cameras) or set(cameras) - set(CAMERAS):
        raise ValueError(f'{skill_id}: cameras must be distinct names from {CAMERAS}')
    if type(value['fps']) is not int or not 1 <= value['fps'] <= 30:
        raise ValueError(f'{skill_id}: fps must be an integer in 1..30')
    for name, upper in (('max_duration_sec', 120.0), ('max_joint_step_rad', .3)):
        number = value[name]
        if type(number) not in (int, float) or not math.isfinite(number) or not 0 < number <= upper:
            raise ValueError(f'{skill_id}: {name} must be in (0, {upper}]')
    parameters = value['parameters']
    if not isinstance(parameters, dict) or any(
            not re.fullmatch(r'[a-z][a-z0-9_]{0,31}', name) or not isinstance(spec, dict)
            or spec.get('type') not in PARAMETER_TYPES or set(spec) - {'type', 'min', 'max', 'max_length'}
            for name, spec in parameters.items()):
        raise ValueError(f'{skill_id}: parameters map names to {{type, min, max, max_length}}')
    return Skill(skill_id, value['version'], value['description'], value['status'], value['kind'],
                 artifacts / checkpoint, digest, tuple(names), tuple(cameras), value['fps'],
                 float(value['max_duration_sec']), float(value['max_joint_step_rad']), parameters)


def load_registry(path=None, artifacts=None):
    """Validate the whole registry; one malformed entry rejects it, never a partial load."""
    path = Path(path) if path else config_directory() / 'robot_skills.yaml'
    artifacts = Path(artifacts).expanduser() if artifacts else Path.home() / '.ros/lekiwi/skills'
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or set(document) != {'skills'} or not isinstance(document['skills'], dict):
        raise ValueError('skill registry must be a mapping with only `skills`')
    return {skill_id: _skill(skill_id, value, artifacts) for skill_id, value in document['skills'].items()}


def catalog(registry, allow_experimental=False):
    """What may be requested: qualified skills, plus experimental ones only for an acquisition protocol."""
    return [skill.catalog() for skill in registry.values() if skill.status == 'qualified' or allow_experimental]


def _import_keys(value):
    if isinstance(value, dict):
        return (set(value) & IMPORT_KEYS) | {key for item in value.values() for key in _import_keys(item)}
    if isinstance(value, list):
        return {key for item in value for key in _import_keys(item)}
    return set()


def artifact_files(directory):
    """Every file the runner may load from a checkpoint directory, by name, after the data-only checks."""
    directory = Path(directory)
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError(f'{directory}: checkpoint must be a real directory')
    if directory.stat().st_mode & 0o002:
        raise ValueError(f'{directory}: the checkpoint must not be world-writable')
    files = []
    for path in sorted(directory.iterdir()):
        if path.is_symlink() or not path.is_file() or path.suffix not in ARTIFACT_SUFFIXES:
            raise ValueError(f'{path.name} is not a safetensors or JSON file')
        if path.stat().st_mode & 0o002:
            raise ValueError(f'{path.name}: the checkpoint must not be world-writable')
        if path.suffix == '.json':
            if path.stat().st_size > MAX_JSON_BYTES:
                raise ValueError(f'{path.name} exceeds {MAX_JSON_BYTES} bytes')
            try:
                keys = _import_keys(json.loads(path.read_text()))
            except (RecursionError, UnicodeDecodeError, ValueError) as error:
                raise ValueError(f'{path.name} is not valid JSON') from error
            if keys:
                raise ValueError(f'{path.name} names code to import ({", ".join(sorted(keys))})')
        if path.name not in PROVENANCE_RECORDS:
            files.append(path)
    if 'model.safetensors' not in {path.name for path in files}:
        raise ValueError('the checkpoint has no model.safetensors')
    return files


def artifact_digest(directory):
    """SHA-256 of the checkpoint manifest: one `<sha256>  <name>` line per loadable file, sorted by name.

    It covers the weights, the policy configuration, the processor configurations and their
    normalization tensors, so changing, adding or removing any of them changes the digest.
    """
    lines = []
    for path in artifact_files(directory):
        with path.open('rb') as stream:
            lines.append(f'{hashlib.file_digest(stream, "sha256").hexdigest()}  {path.name}\n')
    return hashlib.sha256(''.join(lines).encode()).hexdigest()


def verify_artifact(skill):
    """The implementation that runs is exactly the one that was evaluated, and loading it runs no code.

    A checkpoint is data: tensors in safetensors and configuration in JSON. Pickle-based weights
    (.bin, .pt, .pth, .ckpt, .pkl), Python files, links and subdirectories are refused, and no JSON
    file may name a class to import: LeRobot imports a processor step's `class` path and instantiates
    it with the step's config, which would run arbitrary installed code. Registered `registry_name`
    steps resolve only to LeRobot's own processors.
    """
    try:
        digest = artifact_digest(skill.checkpoint)
    except ValueError as error:
        raise ValueError(f'{skill.skill_id}: {error}') from error
    # Checked immediately before loading. The checkpoint is not world-writable, so changing it between
    # this check and LeRobot's reads needs the robot account or its private group.
    if digest != skill.artifact_sha256:
        raise ValueError(f'{skill.skill_id}: the checkpoint does not match the registered artifact digest')
    return skill.checkpoint


def skill_parameters(skill, text):
    if len(text.encode()) > 4096:
        raise ValueError('skill parameters exceed 4096 bytes')
    try:
        values = json.loads(text or '{}')
    except (RecursionError, ValueError) as error:
        raise ValueError('skill parameters must be a JSON object') from error
    if not isinstance(values, dict) or set(values) - set(skill.parameters):
        raise ValueError(f'{skill.skill_id} accepts only {sorted(skill.parameters)}')
    for name, value in values.items():
        spec = skill.parameters[name]
        allowed = PARAMETER_TYPES[spec['type']]
        if type(value) not in allowed or (spec['type'] == 'number' and not math.isfinite(value)):
            raise ValueError(f'{name} must be a {spec["type"]}')
        if spec['type'] == 'number' and not spec.get('min', -math.inf) <= value <= spec.get('max', math.inf):
            raise ValueError(f'{name} is outside its declared range')
        if spec['type'] == 'string' and len(value) > spec.get('max_length', 256):
            raise ValueError(f'{name} is too long')
    return values


def arm_targets(skill, measured, chunk):
    """Joint targets for one action chunk, rate-limited per step and kept inside joint limits.

    Base commands in the chunk are ignored: learned skills drive only the arm until base
    actuation has its own guarded path. The rows returned are what the trajectory carries.
    """
    previous = {name: measured[name] for name in ARM_JOINTS}
    rows = []
    for action in chunk:
        if len(action) != len(skill.action_names) or not all(math.isfinite(float(v)) for v in action):
            raise ValueError('policy produced a malformed or non-finite action')
        target = dict(previous)
        for name, value in zip(skill.action_names, action):
            if name in ARM_JOINTS:
                lower, upper = JOINT_LIMITS[name]
                step = max(-skill.max_joint_step_rad, min(skill.max_joint_step_rad, float(value) - previous[name]))
                target[name] = max(lower, min(upper, previous[name] + step))
        rows.append(target)
        previous = target
    return rows
