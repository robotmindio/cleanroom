#!/usr/bin/env python3
"""Register a trained skill as experimental, or promote it to qualified; never commands the robot.

Registering copies the checkpoint of a scripts/train-skill.py directory to
~/.ros/lekiwi/skills/<id>/<version> and writes its config/robot_skills.yaml entry
with the checkpoint's artifact digest (every file the runner loads, lekiwi_rmf.skills.artifact_digest)
and `status: experimental`. The observation, camera and
action contract comes from the training manifest, not from the operator.

`--promote REPORT [REPORT ...]` sets `status: qualified` only when every
scripts/evaluate-skill.py report is a passing physical evaluation of this skill, version
and artifact digest, the installed checkpoint still hashes to it, and the reports cover every
stored qualification suite of the skill (config/qualification_suites.yaml): a version
that replaces a qualified one must pass what qualified its predecessor. A simulation
report never promotes. The passing reports become the suites' new baselines (a first
qualification stores its protocols as the suites). The whole registry is validated
before it is replaced.
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
from lekiwi_rmf.qualification import load_suites, save_suites, store_report  # noqa: E402
from lekiwi_rmf.skills import SKILL_ID, artifact_digest, load_registry  # noqa: E402

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


def install(training, artifacts, skill_id, version, artifact):
    """Copy the checkpoint once; an existing version must be byte-identical, never replaced."""
    target = artifacts / skill_id / str(version)
    if target.exists():
        if installed_digest(target) != artifact:
            raise RegistrationError(f'{target} already holds a different checkpoint; register a new version')
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.with_name(f'.{version}.tmp')
    shutil.rmtree(staging, ignore_errors=True)
    shutil.copytree(training / 'checkpoint', staging)
    shutil.copy(training / 'manifest.json', staging / 'training-manifest.json')
    if installed_digest(staging) != artifact:
        shutil.rmtree(staging)
        raise RegistrationError('the copied checkpoint does not match the training manifest')
    os.replace(staging, target)
    return target


def installed_digest(directory):
    try:
        return artifact_digest(directory)
    except ValueError as error:
        raise RegistrationError(f'{directory}: {error}') from error


def register(args):
    manifest = json.loads((args.training / 'manifest.json').read_text())
    if manifest.get('kind') != 'lekiwi_skill_training':
        raise RegistrationError(f'{args.training}: not a scripts/train-skill.py directory')
    artifact = manifest['checkpoint'].get('artifact_sha256')
    if artifact is None:
        raise RegistrationError('the training manifest has no artifact_sha256; train again with scripts/train-skill.py')
    if installed_digest(args.training / 'checkpoint') != artifact:
        raise RegistrationError('the checkpoint changed after training')
    header, document = read_registry(args.registry)
    current = document['skills'].get(args.skill_id)
    if current is not None and current['version'] != args.version and not args.replace:
        raise RegistrationError(f'{args.skill_id} is registered as version {current["version"]} ({current["status"]}); '
                                'pass --replace to supersede it')
    if current is not None and current['version'] == args.version and current.get('artifact_sha256') != artifact:
        raise RegistrationError(f'{args.skill_id} version {args.version} is registered with a different checkpoint')
    entry = {'version': args.version, 'description': args.description, 'status': 'experimental',
             'kind': 'lerobot_policy', 'checkpoint': f'{args.skill_id}/{args.version}', 'artifact_sha256': artifact,
             **contract(manifest), 'max_duration_sec': args.max_duration_sec,
             'max_joint_step_rad': args.max_joint_step_rad, 'parameters': args.parameters}
    document['skills'][args.skill_id] = entry
    staged = staged_registry(args.registry, header, document, args.artifacts)
    try:
        target = install(args.training, args.artifacts, args.skill_id, args.version, artifact)
    except BaseException:
        staged.unlink()
        raise
    os.replace(staged, args.registry)
    return {'skill_id': args.skill_id, 'version': args.version, 'status': 'experimental',
            'artifact': str(target), 'artifact_sha256': artifact}


def promote(args):
    reports = [(path, json.loads(path.read_text())) for path in args.promote]
    header, document = read_registry(args.registry)
    entry = document['skills'].get(args.skill_id)
    if entry is None:
        raise RegistrationError(f'{args.skill_id} is not registered')
    skill = load_registry(args.registry, args.artifacts)[args.skill_id]
    for _, report in reports:
        if report.get('kind') != 'lekiwi_skill_evaluation' or report.get('schema') != 2 or report.get('passed') is not True:
            raise RegistrationError('every evaluation report must be a passing schema-2 report')
        if report.get('domain') != 'physical' or report.get('promotable') is not True:
            raise RegistrationError('a simulation result never qualifies a skill; evaluate it on the robot')
        if (report.get('skill_id'), report.get('version')) != (args.skill_id, entry['version']):
            raise RegistrationError(f'the report evaluated {report.get("skill_id")} version {report.get("version")}, '
                                    f'not the registered version {entry["version"]}')
        if report.get('artifact_sha256') != entry['artifact_sha256']:
            raise RegistrationError('the report evaluated a different checkpoint than the registered entry')
    if installed_digest(skill.checkpoint) != entry['artifact_sha256']:
        raise RegistrationError(f'{skill.checkpoint} does not hold the registered checkpoint')
    suites = load_suites(args.suites)
    stored = suites['skills'].get(args.skill_id, [])
    by_protocol = {report['protocol_sha256']: path for path, report in reports}
    missing = [suite['name'] for suite in stored if suite['protocol_sha256'] not in by_protocol]
    if missing:
        raise RegistrationError(f'version {entry["version"]} has not passed the stored suites {missing}; '
                                'run scripts/evaluate-skill.py --suite NAME for each')
    known = {suite['protocol_sha256'] for suite in stored}
    updated = [{**suite, 'qualified_version': entry['version'],
                'report_sha256': store_report(by_protocol[suite['protocol_sha256']], args.store)} for suite in stored]
    names = {suite['name'] for suite in updated}
    for path, report in reports:
        if report['protocol_sha256'] in known:
            continue
        name = report.get('suite') or f'qualification-{report["protocol_sha256"][:8]}'
        if name in names:
            name = f'{name}-{report["protocol_sha256"][:8]}'
        names.add(name)
        known.add(report['protocol_sha256'])
        updated.append({'name': name, 'protocol': report['protocol'], 'protocol_sha256': report['protocol_sha256'],
                        'qualified_version': entry['version'], 'report_sha256': store_report(path, args.store)})
    entry['status'] = 'qualified'
    staged = staged_registry(args.registry, header, document, args.artifacts)
    suites['skills'][args.skill_id] = updated
    try:
        save_suites(args.suites, suites)
    except BaseException:
        staged.unlink()
        raise
    # The reports themselves are kept content-addressed with the suites; the first stays beside the artifact
    # under the provenance name the artifact digest excludes.
    shutil.copy(reports[0][0], skill.checkpoint / 'evaluation-report.json')
    os.replace(staged, args.registry)
    first = reports[0][1]
    return {'skill_id': args.skill_id, 'version': entry['version'], 'status': 'qualified',
            'suites': [suite['name'] for suite in updated], 'success_rate': first.get('success_rate'),
            'wilson_interval': first.get('wilson_interval')}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('skill_id')
    parser.add_argument('--training', type=Path, help='scripts/train-skill.py output directory to register')
    parser.add_argument('--promote', type=Path, nargs='+', metavar='REPORT',
                        help='passing physical scripts/evaluate-skill.py reports, one per stored suite')
    parser.add_argument('--version', type=int)
    parser.add_argument('--description', default='')
    parser.add_argument('--max-duration-sec', type=float, default=30.0)
    parser.add_argument('--max-joint-step-rad', type=float, default=.05)
    parser.add_argument('--parameters', default='{}', help='JSON parameter declarations (see config/robot_skills.yaml)')
    parser.add_argument('--replace', action='store_true', help='supersede a registered entry with a new version')
    parser.add_argument('--registry', type=Path, default=REGISTRY)
    parser.add_argument('--artifacts', type=Path, default=ARTIFACTS)
    parser.add_argument('--suites', type=Path, default=REGISTRY.with_name('qualification_suites.yaml'))
    parser.add_argument('--store', type=Path, default=None, help='qualification report store')
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
