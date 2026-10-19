#!/usr/bin/env python3
"""Register a trained recognizer as experimental, or promote it to qualified; never commands the robot.

Registering installs the model files of a scripts/train-recognizer.py run into
~/.ros/lekiwi/models/<name> and writes its config/recognition_models.yaml entry with every
file's SHA-256 and size, `status: experimental`, the recognizer it `replaces` (by default
the base it was trained from) and the training identity. The catalogue must keep every
base label at its base index. An experimental recognizer is refused by the detector; the
offline evaluator (scripts/evaluate-object-recognition.py) runs it.

`--promote REPORT [--regression REPORT ...]` sets `status: qualified` only when:
- REPORT is a passing schema-3 evaluation of exactly this recognizer (its model_sha256) on
  the held-out split of its own training dataset, whose held-out labels are human;
- for every stored qualification suite of the recognizer it replaces
  (config/qualification_suites.yaml), a --regression report of this recognizer on that
  suite's manifest shows no label or condition regression against the suite's baseline
  (lekiwi_rmf.perception_study.regressions), so previously qualified labels are measured,
  not assumed.
Promotion then records those reports as this recognizer's suites, plus its own held-out suite.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

import yaml

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))
from lekiwi_rmf.object_cameras import config_directory  # noqa: E402
from lekiwi_rmf.qualification import load_suites, save_suites, store_report, stored_report  # noqa: E402

MODELS = Path.home() / '.ros/lekiwi/models'
NAME_FIELDS = 'abcdefghijklmnopqrstuvwxyz0123456789_'


class RegistrationError(RuntimeError):
    pass


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read_registry(path):
    document = yaml.safe_load(Path(path).read_text())
    if not isinstance(document, dict):
        raise RegistrationError(f'{path} is not a recognizer registry')
    return document


def write_registry(path, text):
    temporary = Path(path).with_name(f'.{Path(path).name}.tmp')
    temporary.write_text(text)
    os.replace(temporary, path)


def append_entry(path, name, entry):
    """Append one entry, leaving every other entry and comment byte-identical."""
    block = yaml.safe_dump({name: entry}, sort_keys=True, default_flow_style=False)
    text = Path(path).read_text()
    write_registry(path, text + ('' if text.endswith('\n') else '\n')
                   + '# Trained by scripts/train-recognizer.py; registered by scripts/register-recognizer.py.\n' + block)


def set_status(path, name, status):
    """Change one entry's status line in place; the rest of the registry stays byte-identical."""
    lines = Path(path).read_text().splitlines(keepends=True)
    start = next((index for index, line in enumerate(lines) if line.rstrip('\n') == f'{name}:'), None)
    if start is None:
        raise RegistrationError(f'{name} is not a top-level registry entry')
    for index in range(start + 1, len(lines)):
        if lines[index] and not lines[index][0].isspace() and not lines[index].startswith('#'):
            break
        if lines[index].strip().startswith('status:'):
            indent = lines[index][:len(lines[index]) - len(lines[index].lstrip())]
            lines[index] = f'{indent}status: {status}\n'
            write_registry(path, ''.join(lines))
            return
    raise RegistrationError(f'{name} has no status field')


def install(model, target, files):
    """Copy the model files once; an existing installation must be byte-identical, never replaced."""
    if target.exists():
        for name, expected in files.items():
            if not (target / name).is_file() or digest(target / name) != expected['sha256']:
                raise RegistrationError(f'{target} already holds a different model; register another name')
        return target
    staging = target.with_name(f'.{target.name}.tmp')
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir(parents=True)
    for name, expected in files.items():
        shutil.copy(model / name, staging / name)
        if digest(staging / name) != expected['sha256'] or (staging / name).stat().st_size != expected['size']:
            shutil.rmtree(staging)
            raise RegistrationError(f'copied {name} does not match the training manifest')
    os.replace(staging, target)
    return target


def register(args):
    manifest = json.loads((args.training / 'manifest.json').read_text())
    if manifest.get('kind') != 'lekiwi_recognizer_training':
        raise RegistrationError(f'{args.training}: not a scripts/train-recognizer.py directory')
    spec = manifest['artifact']['spec']
    model = args.training / manifest['artifact']['path']
    for name, expected in spec['files'].items():
        if digest(model / name) != expected['sha256']:
            raise RegistrationError(f'{name} changed after training')
    document = read_registry(args.registry)
    base = manifest['base']['name']
    if document.get(base, {}).get('files') != manifest['base']['files']:
        raise RegistrationError(f'the registry no longer pins the base {base} this model was trained from')
    # The new catalogue keeps the base catalogue and decoder: same labels at the same indices.
    labels = json.loads((model / 'config.json').read_text())['id2label']
    base_labels = manifest['vocabulary']['labels'][:manifest['vocabulary']['base_labels']]
    if [labels[str(index)] for index in range(len(base_labels))] != base_labels:
        raise RegistrationError('the trained catalogue moved or dropped a base label')
    replaces = args.replaces or base
    if replaces not in document:
        raise RegistrationError(f'{replaces} is not registered')
    entry = {**spec, 'status': 'experimental', 'replaces': replaces,
             'training': {'manifest_sha256': digest(args.training / 'manifest.json'),
                          'dataset_id': manifest['dataset']['dataset_id'], 'dataset_sha256': manifest['dataset']['sha256'],
                          'heldout_manifest': str(Path(manifest['dataset']['root']) / 'scenes.json'),
                          'heldout_manifest_sha256': manifest['dataset']['scenes_sha256'],
                          'heldout_authority': manifest['dataset']['heldout_authority'],
                          'new_labels': manifest['vocabulary']['new_labels']}}
    current = document.get(args.name)
    if current is not None:
        if current.get('files') != entry['files']:
            raise RegistrationError(f'{args.name} is registered with other files; register a new name')
        return {'name': args.name, 'status': current.get('status'), 'artifact': str(args.models / args.name)}
    target = install(model, args.models / args.name, spec['files'])
    append_entry(args.registry, args.name, entry)
    check = read_registry(args.registry)
    if check.get(args.name) != entry:
        raise RegistrationError('the registry did not record the entry as written')
    return {'name': args.name, 'status': 'experimental', 'replaces': replaces, 'artifact': str(target),
            'new_labels': entry['training']['new_labels']}


def regression_failures(suite, report, identity, store):
    """Why `report` does not show this recognizer holding `suite`, or [] when it does."""
    from lekiwi_rmf.perception_study import regressions
    if report.get('schema') != 3 or report.get('model_sha256') != identity:
        return [f'{suite["name"]}: the regression report did not evaluate this recognizer']
    if report.get('manifest_sha256') != suite['manifest_sha256'] or report.get('split') != suite['split']:
        return [f'{suite["name"]}: the regression report evaluated another manifest or split']
    baseline = stored_report(suite['report_sha256'], store)
    found = regressions(baseline['scenes'], report['scenes'], suite['labels'])
    return [f'{suite["name"]}: {item["scope"]} {item["name"]} {item["metric"]} '
            f'{item["current"]} -> {item["candidate"]}' for item in found]


def promote(args):
    from lekiwi_rmf.object_recognition import model_spec, recognition_identity, verify_model
    document = read_registry(args.registry)
    entry = document.get(args.name)
    if entry is None or entry.get('status') != 'experimental':
        raise RegistrationError(f'{args.name} is not an experimental registry entry')
    if model_spec(args.name) != entry:
        raise RegistrationError('the runtime reads another registry than --registry; promote against the installed one')
    verify_model(args.models / args.name, entry)
    identity = recognition_identity(args.name)[0]
    report = json.loads(args.promote.read_text())
    if report.get('schema') != 3 or report.get('passed') is not True:
        raise RegistrationError('the evaluation report did not pass')
    if report.get('model_sha256') != identity:
        raise RegistrationError('the report evaluated another recognizer, runtime or artifact')
    if entry['training']['heldout_authority'] != 'human':
        raise RegistrationError('its held-out labels were never reviewed by a person; they measure, never qualify')
    if report.get('split') != 'heldout' or report.get('manifest_sha256') != entry['training']['heldout_manifest_sha256']:
        raise RegistrationError('the report is not the held-out split of this recognizer\'s training dataset')
    suites = load_suites(args.suites)
    inherited = suites['recognizers'].get(entry['replaces'], [])
    regressions = [json.loads(path.read_text()) for path in args.regression]
    reasons, passed = [], []
    for suite in inherited:
        candidates = [(path, item) for path, item in zip(args.regression, regressions)
                      if item.get('manifest_sha256') == suite['manifest_sha256']]
        if not candidates:
            reasons.append(f'{suite["name"]}: no --regression report on its manifest {suite["manifest_sha256"][:12]}')
            continue
        failures = regression_failures(suite, candidates[0][1], identity, args.store)
        reasons.extend(failures)
        if not failures:
            passed.append((suite, candidates[0][0]))
    if reasons:
        raise RegistrationError('not promoted; previously qualified labels are not shown to hold:\n  ' + '\n  '.join(reasons))
    own = {'name': f'{entry["training"]["dataset_id"]}-heldout', 'manifest': entry['training']['heldout_manifest'],
           'manifest_sha256': entry['training']['heldout_manifest_sha256'], 'split': 'heldout',
           'labels': report['labels'], 'qualified_model': args.name, 'report_sha256': store_report(args.promote, args.store)}
    recorded = [{**suite, 'qualified_model': args.name, 'report_sha256': store_report(path, args.store)}
                for suite, path in passed]
    names = {suite['name'] for suite in recorded}
    if own['name'] in names:
        own['name'] = f'{own["name"]}-{own["manifest_sha256"][:8]}'
    suites['recognizers'][args.name] = [*recorded, own]
    save_suites(args.suites, suites)
    set_status(args.registry, args.name, 'qualified')
    return {'name': args.name, 'status': 'qualified', 'model_sha256': identity,
            'suites': [suite['name'] for suite in suites['recognizers'][args.name]]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name', help='new registry name, for example soda_can_v1')
    parser.add_argument('--training', type=Path, help='scripts/train-recognizer.py directory to register')
    parser.add_argument('--replaces', help='recognizer this one supersedes (default: its base)')
    parser.add_argument('--promote', type=Path, metavar='REPORT', help='passing held-out evaluation report')
    parser.add_argument('--regression', type=Path, action='append', default=[],
                        help='evaluation of this recognizer on a suite of the recognizer it replaces')
    parser.add_argument('--registry', type=Path, default=config_directory() / 'recognition_models.yaml')
    parser.add_argument('--suites', type=Path, default=config_directory() / 'qualification_suites.yaml')
    parser.add_argument('--models', type=Path, default=MODELS)
    parser.add_argument('--store', type=Path, default=None, help='qualification report store')
    args = parser.parse_args()
    if not args.name or args.name[0].isdigit() or any(c not in NAME_FIELDS for c in args.name) or len(args.name) > 64:
        parser.error('name must be lower-case letters, digits and underscores')
    if (args.training is None) == (args.promote is None):
        parser.error('pass exactly one of --training or --promote')
    args.models = args.models.expanduser()
    try:
        if args.promote is not None:
            # The identity hashes the recognizer's runtime packages, so promotion runs where the evaluator runs.
            from lekiwi_rmf.object_recognition import use_model_runtime
            use_model_runtime(args.name)
        result = register(args) if args.training is not None else promote(args)
    except (RegistrationError, OSError, ValueError, KeyError) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
