#!/usr/bin/env python3
"""Register a trained skill as experimental, or promote it to qualified; never commands the robot.

Registering copies the checkpoint of a scripts/train-skill.py directory to
~/.ros/lekiwi/skills/<id>/<version> and writes its config/robot_skills.yaml entry
with the weights digest and `status: experimental`. The observation, camera and
action contract comes from the training manifest, not from the operator.

`--promote REPORT` sets `status: qualified` only when the scripts/evaluate-skill.py
report passed and names this skill, version and weights digest, and the installed
weights still hash to it. The whole registry is validated before it is replaced.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.skills import SKILL_ID, load_registry

REGISTRY = Path(__file__).resolve().parents[1] / 'config/robot_skills.yaml'
ARTIFACTS = Path.home() / '.ros/lekiwi/skills'


class RegistrationError(RuntimeError):
    pass


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read_registry(path):
    lines = path.read_text().splitlines(keepends=True)
    header = []
    for line in lines:
        if not line.startswith('#'):
            break
        header.append(line)
    return ''.join(header), yaml.safe_load(''.join(lines))


def staged_registry(path, header, document, artifacts):
    """The complete registry the runner will load, validated beside the tracked file."""
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(header + yaml.safe_dump(document, sort_keys=True, default_flow_style=False))
    try:
        load_registry(temporary, artifacts)
    except (OSError, ValueError) as error:
        temporary.unlink()
        raise RegistrationError(f'the resulting registry would not load: {error}') from error
    return temporary


def contract(manifest):
    dataset = manifest['dataset']
    names = dataset.get('action_names')
    if not isinstance(names, list):
        raise RegistrationError('the training manifest does not name the action features')
    if type(dataset.get('fps')) is not int:
        raise RegistrationError('the training manifest does not record the dataset rate')
    return {'action_names': names, 'cameras': dataset['cameras'], 'fps': dataset['fps']}


def install(training, artifacts, skill_id, version, weights):
    """Copy the checkpoint once; an existing version must be byte-identical, never replaced."""
    target = artifacts / skill_id / str(version)
    if target.exists():
        installed = target / 'model.safetensors'
        if not installed.is_file() or digest(installed) != weights:
            raise RegistrationError(f'{target} already holds different weights; register a new version')
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.with_name(f'.{version}.tmp')
    shutil.rmtree(staging, ignore_errors=True)
    shutil.copytree(training / 'checkpoint', staging)
    shutil.copy(training / 'manifest.json', staging / 'training-manifest.json')
    if digest(staging / 'model.safetensors') != weights:
        shutil.rmtree(staging)
        raise RegistrationError('copied weights do not match the training manifest')
    os.replace(staging, target)
    return target


def register(args):
    manifest = json.loads((args.training / 'manifest.json').read_text())
    if manifest.get('kind') != 'lekiwi_skill_training':
        raise RegistrationError(f'{args.training}: not a scripts/train-skill.py directory')
    weights = manifest['checkpoint']['weights_sha256']
    if digest(args.training / 'checkpoint/model.safetensors') != weights:
        raise RegistrationError('the checkpoint changed after training')
    header, document = read_registry(args.registry)
    current = document['skills'].get(args.skill_id)
    if current is not None and current['version'] != args.version and not args.replace:
        raise RegistrationError(f'{args.skill_id} is registered as version {current["version"]} ({current["status"]}); '
                                'pass --replace to supersede it')
    if current is not None and current['version'] == args.version and current['weights_sha256'] != weights:
        raise RegistrationError(f'{args.skill_id} version {args.version} is registered with different weights')
    entry = {'version': args.version, 'description': args.description, 'status': 'experimental',
             'kind': 'lerobot_policy', 'checkpoint': f'{args.skill_id}/{args.version}', 'weights_sha256': weights,
             **contract(manifest), 'max_duration_sec': args.max_duration_sec,
             'max_joint_step_rad': args.max_joint_step_rad, 'parameters': args.parameters}
    document['skills'][args.skill_id] = entry
    staged = staged_registry(args.registry, header, document, args.artifacts)
    try:
        target = install(args.training, args.artifacts, args.skill_id, args.version, weights)
    except BaseException:
        staged.unlink()
        raise
    os.replace(staged, args.registry)
    return {'skill_id': args.skill_id, 'version': args.version, 'status': 'experimental',
            'artifact': str(target), 'weights_sha256': weights}


def promote(args):
    report = json.loads(args.promote.read_text())
    header, document = read_registry(args.registry)
    entry = document['skills'].get(args.skill_id)
    if entry is None:
        raise RegistrationError(f'{args.skill_id} is not registered')
    skill = load_registry(args.registry, args.artifacts)[args.skill_id]
    if report.get('kind') != 'lekiwi_skill_evaluation' or report.get('passed') is not True:
        raise RegistrationError('the evaluation report did not pass')
    if (report.get('skill_id'), report.get('version')) != (args.skill_id, entry['version']):
        raise RegistrationError(f'the report evaluated {report.get("skill_id")} version {report.get("version")}, '
                                f'not the registered version {entry["version"]}')
    if report.get('weights_sha256') != entry['weights_sha256']:
        raise RegistrationError('the report evaluated different weights than the registered entry')
    installed = skill.checkpoint / 'model.safetensors'
    if not installed.is_file() or digest(installed) != entry['weights_sha256']:
        raise RegistrationError(f'{installed} does not hold the registered weights')
    entry['status'] = 'qualified'
    staged = staged_registry(args.registry, header, document, args.artifacts)
    shutil.copy(args.promote, skill.checkpoint / 'evaluation-report.json')
    os.replace(staged, args.registry)
    return {'skill_id': args.skill_id, 'version': entry['version'], 'status': 'qualified',
            'success_rate': report.get('success_rate'), 'wilson_interval': report.get('wilson_interval')}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('skill_id')
    parser.add_argument('--training', type=Path, help='scripts/train-skill.py output directory to register')
    parser.add_argument('--promote', type=Path, metavar='REPORT', help='passing scripts/evaluate-skill.py report')
    parser.add_argument('--version', type=int)
    parser.add_argument('--description', default='')
    parser.add_argument('--max-duration-sec', type=float, default=30.0)
    parser.add_argument('--max-joint-step-rad', type=float, default=.05)
    parser.add_argument('--parameters', default='{}', help='JSON parameter declarations (see config/robot_skills.yaml)')
    parser.add_argument('--replace', action='store_true', help='supersede a registered entry with a new version')
    parser.add_argument('--registry', type=Path, default=REGISTRY)
    parser.add_argument('--artifacts', type=Path, default=ARTIFACTS)
    args = parser.parse_args()
    if not SKILL_ID.match(args.skill_id):
        parser.error('skill_id must match [a-z][a-z0-9_]{0,63}')
    if (args.training is None) == (args.promote is None):
        parser.error('pass exactly one of --training or --promote')
    if args.training is not None and (args.version is None or not args.description):
        parser.error('registering requires --version and --description')
    args.artifacts = args.artifacts.expanduser()
    try:
        args.parameters = json.loads(args.parameters)
        result = register(args) if args.training is not None else promote(args)
    except (RegistrationError, OSError, ValueError, KeyError) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
