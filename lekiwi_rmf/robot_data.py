"""Robot-local data classes, backup, restore and post-restore verification (config/data_retention.yaml).

A backup holds the robot's journals and records (task journal, observation store with its evidence,
spatial memory, arm calibration) and manifests of the artifacts it runs. Restoring checks every file
against the backup manifest, keeps what it replaces, rebuilds SQLite indexes and reconciles each
observation record with its evidence bytes: a store whose records restored without matching images
recovers its history but is reported as visual evidence unverified, never as intact memory.
"""

import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3

import yaml

from lekiwi_rmf.object_cameras import config_directory

SCHEMA = 'lekiwi.robot_data_backup/1'
CLASSES = ('transient_buffer', 'pinned_evidence', 'training_samples', 'model_artifacts')
KINDS = ('sqlite', 'file')
# Schema versions the owners open (TaskJournal, ObservationStore); test_robot_data ties these to them.
SUPPORTED_VERSIONS = {'task_journal': (0, 1, 2, 3, 4), 'observations': (0, 1)}


def _relative(text):
    path = Path(str(text))
    if path.is_absolute() or '..' in path.parts:
        raise ValueError(f'{text} must be relative to the robot home')
    return path


def load_policy(path=None):
    """Validate the tracked policy and resolve every retention reference to its enforcing parameter."""
    path = Path(path) if path else config_directory() / 'data_retention.yaml'
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or set(document) != {'classes', 'stores', 'artifact_manifests'}:
        raise ValueError('data retention must define classes, stores and artifact_manifests')
    classes, stores = document['classes'], document['stores']
    if not isinstance(classes, dict) or set(classes) != set(CLASSES):
        raise ValueError(f'data retention must define exactly the classes {CLASSES}')
    if not isinstance(stores, dict) or not stores:
        raise ValueError('data retention needs at least one store')
    for name, store in stores.items():
        if (not isinstance(store, dict) or set(store) != {'path', 'kind', 'lock'} or store['kind'] not in KINDS
                or type(store['lock']) is not bool):
            raise ValueError(f'store {name} needs path, kind {KINDS} and a boolean lock')
        _relative(store['path'])
    resolved = {}
    for name, value in classes.items():
        if (not isinstance(value, dict) or len(set(value) & {'store', 'locations'}) != 1
                or set(value) - {'store', 'locations'} != {'holds', 'retention', 'expiry', 'mode'}):
            raise ValueError(f'class {name} needs holds, retention, expiry, mode and one store or locations')
        if 'store' in value and value['store'] not in stores:
            raise ValueError(f'class {name} names an unknown store')
        if 'locations' in value:
            if not isinstance(value['locations'], list) or not value['locations']:
                raise ValueError(f'class {name} locations must be a non-empty list')
            for location in value['locations']:
                _relative(location)
        # Recorded images and journals are owner-only; no class may be writable by other accounts.
        if value['mode'] not in ('0600', '0700', '0775'):
            raise ValueError(f'class {name} mode must be 0600, 0700 or 0775')
        retention = value['retention']
        if retention is not None:
            if not isinstance(retention, dict) or set(retention) != {'file', 'node', 'parameter'}:
                raise ValueError(f'class {name} retention must name file, node and parameter')
            owner = yaml.safe_load((config_directory() / retention['file']).read_text())
            seconds = owner.get(retention['node'], {}).get('ros__parameters', {}).get(retention['parameter'])
            if type(seconds) not in (int, float) or seconds <= 0:
                raise ValueError(f'class {name} retention does not resolve to a positive tracked parameter')
            retention = float(seconds)
        resolved[name] = {**value, 'retention_sec': retention}
    manifests = document['artifact_manifests']
    if not isinstance(manifests, dict):
        raise ValueError('artifact_manifests must map names to directories')
    for directory in manifests.values():
        _relative(directory)
    return {'classes': resolved, 'stores': stores, 'artifact_manifests': manifests}


def store_paths(policy, home, domain=0, sim=False):
    """Where each store lives for this ROS domain, with the same suffix rule as the owning nodes."""
    suffix = (f'_{domain}' if domain else '') + ('_sim' if sim else '')
    return {name: Path(home) / store['path'].format(suffix=suffix) for name, store in policy['stores'].items()}


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _snapshot(source, target):
    """SQLite's online backup: consistent even while the owning node writes."""
    reader = sqlite3.connect(f'file:{source}?mode=ro', uri=True)
    writer = sqlite3.connect(target)
    try:
        reader.backup(writer)
    finally:
        writer.close()
        reader.close()


def artifact_manifest(directory):
    """Every file under an artifact directory by relative path, size and digest; weights are not copied."""
    directory = Path(directory)
    if not directory.is_dir():
        return {}
    return {str(path.relative_to(directory)): {'size': path.stat().st_size, 'sha256': digest(path)}
            for path in sorted(directory.rglob('*')) if path.is_file() and not path.is_symlink()}


def backup(home, destination, *, domain=0, sim=False, policy=None, now=None):
    policy = policy or load_policy()
    home, destination = Path(home), Path(destination)
    destination.mkdir(mode=0o700, parents=True, exist_ok=False)
    files, absent = {}, []
    for name, source in store_paths(policy, home, domain, sim).items():
        if not source.is_file():
            absent.append(name)
            continue
        target = destination / f'{name}{source.suffix}'
        if policy['stores'][name]['kind'] == 'sqlite':
            _snapshot(source, target)
        else:
            shutil.copyfile(source, target)
        target.chmod(0o600)
        files[name] = {'file': target.name, 'source': str(source.relative_to(home)),
                       'size': target.stat().st_size, 'sha256': digest(target)}
    revision = home / '.ros/lekiwi/deployed-revision'
    manifest = {
        'schema': SCHEMA, 'created_at': (now or datetime.datetime.now(datetime.timezone.utc)).isoformat(),
        'domain': domain, 'sim': sim, 'deployed_revision': revision.read_text().strip() if revision.is_file() else None,
        'stores': files, 'absent': absent,
        'artifacts': {name: artifact_manifest(home / directory) for name, directory in policy['artifact_manifests'].items()}}
    target = destination / 'manifest.json'
    target.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    target.chmod(0o600)
    return manifest


def read_backup(directory):
    """The manifest, after every listed file proved byte-identical to what was backed up."""
    directory = Path(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    if manifest.get('schema') != SCHEMA or not isinstance(manifest.get('stores'), dict):
        raise ValueError('not a robot data backup')
    for name, entry in manifest['stores'].items():
        path = directory / Path(entry['file']).name
        if not path.is_file() or path.stat().st_size != entry['size'] or digest(path) != entry['sha256']:
            raise ValueError(f'{name} in the backup is missing or corrupt')
    return manifest


def _held(path):
    """Whether the owning node holds the store's lock (it is running)."""
    lock = Path(f'{path}.lock')
    if not lock.exists():
        return False
    with lock.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(stream, fcntl.LOCK_UN)
    return False


def restore(directory, home, *, policy=None, stamp=None):
    """Replace the stores from a verified backup; each replaced file is kept beside it, never deleted."""
    policy = policy or load_policy()
    directory, home = Path(directory), Path(home)
    manifest = read_backup(directory)
    paths = store_paths(policy, home, manifest['domain'], manifest['sim'])
    if set(manifest['stores']) - set(paths):
        raise ValueError('the backup holds stores this policy does not know')
    for name in manifest['stores']:
        if policy['stores'][name]['lock'] and _held(paths[name]):
            raise RuntimeError(f'{name} is open by its node; stop the robot services before restoring')
    stamp = stamp or datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    kept = {}
    for name, entry in manifest['stores'].items():
        target = paths[name]
        target.parent.mkdir(parents=True, exist_ok=True)
        staging = target.with_name(f'.{target.name}.restoring')
        shutil.copyfile(directory / Path(entry['file']).name, staging)
        staging.chmod(0o600)
        if target.exists():
            previous = target.with_name(f'{target.name}.pre-restore-{stamp}')
            os.replace(target, previous)
            kept[name] = str(previous)
        os.replace(staging, target)
    return {'restored': sorted(manifest['stores']), 'kept': kept, 'absent': manifest['absent'],
            'verification': verify(home, policy=policy, domain=manifest['domain'], sim=manifest['sim'])}


def _evidence(database):
    counts = {'records': 0, 'matched': 0, 'mismatched': 0}
    if database.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='observations'").fetchone() is None:
        return counts  # created by the owner on first open
    for document, evidence in database.execute('SELECT document, evidence FROM observations'):
        counts['records'] += 1
        try:
            expected = json.loads(document)['evidence']['sha256']
        except (KeyError, TypeError, ValueError):
            expected = None
        counts['matched' if hashlib.sha256(evidence).hexdigest() == expected else 'mismatched'] += 1
    return counts


def verify(home, *, policy=None, domain=0, sim=False):
    """Integrity, schema version and rebuilt indexes per store, and records reconciled with evidence."""
    policy = policy or load_policy()
    report, healthy = {}, True
    for name, path in store_paths(policy, home, domain, sim).items():
        if not path.is_file():
            report[name] = {'present': False}
            continue
        entry = {'present': True}
        if policy['stores'][name]['kind'] == 'sqlite':
            database = sqlite3.connect(path)
            try:
                entry['integrity'] = database.execute('PRAGMA integrity_check').fetchone()[0]
                entry['schema_version'] = database.execute('PRAGMA user_version').fetchone()[0]
                database.execute('REINDEX')
                entry['indexes_rebuilt'] = True
                if name == 'observations':
                    entry['evidence'] = _evidence(database)
            except sqlite3.DatabaseError as error:
                entry['integrity'] = f'unreadable: {error}'
            finally:
                database.close()
            supported = SUPPORTED_VERSIONS.get(name)
            entry['schema_supported'] = supported is None or entry.get('schema_version') in supported
            healthy &= entry['integrity'] == 'ok' and entry['schema_supported']
        report[name] = entry
    observations = report.get('observations', {})
    counts = observations.get('evidence')
    report['visual_evidence'] = ('verified' if counts and counts['records'] and not counts['mismatched']
                                 else 'empty' if counts and not counts['records'] else 'unverified')
    report['healthy'] = bool(healthy)
    return report


def audit(home, *, policy=None, domain=0, sim=False):
    """Every path of each class whose permission bits exceed the class mode; nothing is changed."""
    policy = policy or load_policy()
    paths = store_paths(policy, home, domain, sim)
    findings = []
    for name, value in policy['classes'].items():
        targets = [paths[value['store']]] if value.get('store') else [Path(home) / item for item in value['locations']]
        allowed = int(value['mode'], 8)
        for target in targets:
            if not target.exists() or target.is_symlink():
                continue
            for path in [target] + (sorted(target.rglob('*')) if target.is_dir() else []):
                mode = path.lstat().st_mode & 0o777
                if mode & ~allowed:
                    findings.append({'class': name, 'path': str(path), 'mode': f'{mode:04o}', 'allowed': value['mode']})
    return findings
