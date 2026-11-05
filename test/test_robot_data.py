"""Robot-local data classes, and backup, restore and index rebuild reconciling records with evidence (T-32)."""

import json
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest
import yaml

from lekiwi_rmf.object_cameras import config_directory
from lekiwi_rmf.object_observations import ObservationStore, observation
from lekiwi_rmf.robot_data import CLASSES, SUPPORTED_VERSIONS, audit, backup, load_policy, read_backup, restore, store_paths, verify
from lekiwi_rmf.spatial_anchors import SpatialStore
from lekiwi_rmf.task_journal import TaskJournal
from test_object_observations import MODEL_SHA256, PROFILE, capture

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot-data.py'


def populated(home, domain=0, sim=False):
    """The real owners write their own stores, as on the robot."""
    simulation = '_sim' if sim else ''
    paths = {
        'task_journal': home / f'.ros/lekiwi_tasks{f"_{domain}" if domain else ""}{simulation}.sqlite3',
        'observations': home / f'.ros/lekiwi/observations_{domain}{simulation}.sqlite3',
        'spatial_memory': home / f'.ros/lekiwi/spatial_{domain}{simulation}.sqlite3',
        'arm_calibration': home / '.ros/lekiwi_arm_calibration.json',
    }
    journal = TaskJournal(paths['task_journal'])
    journal.admit('task-1', 'explore', '{}', b'\x01' * 16)
    journal.close()
    store = ObservationStore(paths['observations'])
    documents = []
    for stamp in (1, 2):
        document, jpeg = observation(*capture(stamp=stamp), MODEL_SHA256, 'front', profile=PROFILE)
        store.put(document, jpeg)
        documents.append(document)
    store.close()
    SpatialStore(paths['spatial_memory'], site_ref='home', observer_ref='lekiwi').close()
    paths['arm_calibration'].write_text('{"arm": {}}')
    models = home / '.ros/lekiwi/models/yolox'
    models.mkdir(parents=True)
    (models / 'weights.onnx').write_bytes(b'weights')
    return paths, documents


def test_tracked_policy_defines_every_class_from_its_enforcing_parameters():
    policy = load_policy()
    assert tuple(policy['classes']) == CLASSES
    observations = yaml.safe_load((config_directory() / 'object_perception.yaml').read_text())
    retention = observations['object_observations']['ros__parameters']['retention_sec']
    assert policy['classes']['transient_buffer']['retention_sec'] == retention
    assert policy['classes']['pinned_evidence']['retention_sec'] == retention
    # Nothing expires training samples or a referenced model behind its operator's back.
    assert policy['classes']['training_samples']['retention_sec'] is None
    assert policy['classes']['model_artifacts']['retention_sec'] is None
    assert all(not int(value['mode'], 8) & 0o002 for value in policy['classes'].values())
    assert all(policy['classes'][name]['mode'] in ('0600', '0700') for name in CLASSES[:3])


@pytest.mark.parametrize('change', [
    lambda d: d['classes'].pop('training_samples'),
    lambda d: d['classes']['transient_buffer'].update(mode='0644'),
    lambda d: d['classes']['transient_buffer'].update(store='nowhere'),
    lambda d: d['classes']['transient_buffer'].update(retention={'file': 'object_perception.yaml',
                                                                  'node': 'object_observations', 'parameter': 'none'}),
    lambda d: d['classes']['model_artifacts'].update(locations=['/etc']),
    lambda d: d['stores']['task_journal'].update(path='../outside.sqlite3'),
    lambda d: d['stores']['task_journal'].update(kind='tar')])
def test_malformed_policies_are_rejected(tmp_path, change):
    document = yaml.safe_load((config_directory() / 'data_retention.yaml').read_text())
    change(document)
    path = tmp_path / 'policy.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_policy(path)


@pytest.mark.parametrize('domain, sim', [(0, False), (0, True), (5, False), (5, True)])
def test_backup_restore_round_trip_rebuilds_indexes_and_reconciles_evidence(tmp_path, domain, sim):
    home, target = tmp_path / 'robot', tmp_path / 'replacement'
    paths, documents = populated(home, domain, sim)
    manifest = backup(home, tmp_path / 'backup', domain=domain, sim=sim)
    assert set(manifest['stores']) == {'task_journal', 'observations', 'spatial_memory', 'arm_calibration'}
    assert manifest['absent'] == [] and manifest['artifacts']['models']['yolox/weights.onnx']['size'] == 7
    assert not any(path.suffix == '.onnx' for path in (tmp_path / 'backup').iterdir())  # weights by digest only
    assert all((tmp_path / 'backup' / entry['file']).stat().st_mode & 0o777 == 0o600
               for entry in manifest['stores'].values())

    result = restore(tmp_path / 'backup', target)
    assert result['restored'] == ['arm_calibration', 'observations', 'spatial_memory', 'task_journal'] and not result['kept']
    report = result['verification']
    assert report['healthy'] and report['visual_evidence'] == 'verified'
    assert report['observations']['evidence'] == {'records': 2, 'matched': 2, 'mismatched': 0}
    assert all(report[name]['integrity'] == 'ok' and report[name]['indexes_rebuilt']
               for name in ('task_journal', 'observations', 'spatial_memory'))
    # The owners reopen what was restored, with the same records and identities.
    restored = store_paths(load_policy(), target, domain, sim)
    journal = TaskJournal(restored['task_journal'])
    assert journal.row('task-1')['capability'] == 'explore'
    journal.close()
    store = ObservationStore(restored['observations'])
    try:
        ids = {item['id'] for item in store.query('{"limit": 10}')['observations']}
        assert ids == {document['id'] for document in documents}
        assert store.evidence(documents[0]['id'])
    finally:
        store.close()


@pytest.mark.parametrize('evidence', [b'\x00', 'not a binary image', 42])
def test_records_without_matching_media_restore_history_but_not_visual_memory(tmp_path, evidence):
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    database = sqlite3.connect(paths['observations'])
    with database:
        database.execute('UPDATE observations SET evidence=? WHERE sequence=1', (evidence,))
    database.close()
    report = verify(home)
    assert report['healthy'] and report['observations']['evidence']['mismatched'] == 1
    assert report['visual_evidence'] == 'unverified'


def test_restore_keeps_replaced_files_and_refuses_corrupt_backups_or_running_nodes(tmp_path):
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    backup(home, tmp_path / 'backup')
    result = restore(tmp_path / 'backup', home, stamp='20261019T000000Z')
    assert set(result['kept']) == set(result['restored'])
    assert (paths['task_journal'].parent / 'lekiwi_tasks.sqlite3.pre-restore-20261019T000000Z').is_file()
    journal = TaskJournal(paths['task_journal'])  # a running gateway holds the journal lock
    try:
        with pytest.raises(RuntimeError, match='open by its node'):
            restore(tmp_path / 'backup', home)
    finally:
        journal.close()
    manifest = json.loads((tmp_path / 'backup/manifest.json').read_text())
    (tmp_path / 'backup' / manifest['stores']['observations']['file']).write_bytes(b'corrupt')
    with pytest.raises(ValueError, match='missing or corrupt'):
        read_backup(tmp_path / 'backup')


def test_audit_reports_class_data_beyond_its_mode(tmp_path):
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    assert audit(home) == []
    paths['observations'].chmod(0o644)
    episodes = home / '.ros/lekiwi/episodes/one'
    episodes.mkdir(parents=True)
    episodes.chmod(0o755)
    findings = {(item['class'], item['path']) for item in audit(home)}
    assert ('transient_buffer', str(paths['observations'])) in findings
    assert ('pinned_evidence', str(paths['observations'])) in findings
    assert ('training_samples', str(episodes)) in findings


def test_restore_preserves_the_original_copy_when_stamp_is_reused(tmp_path):
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    backup(home, tmp_path / 'backup')
    restore(tmp_path / 'backup', home, stamp='same')
    preserved = paths['arm_calibration'].with_name(paths['arm_calibration'].name + '.pre-restore-same')
    before = preserved.read_bytes()
    paths['arm_calibration'].write_text('new calibration')
    with pytest.raises(ValueError, match='already exists'):
        restore(tmp_path / 'backup', home, stamp='same')
    assert preserved.read_bytes() == before
    assert paths['arm_calibration'].read_text() == 'new calibration'


def test_restore_holds_owner_locks_through_verification(tmp_path, monkeypatch):
    from lekiwi_rmf import robot_data
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    backup(home, tmp_path / 'backup')
    original = robot_data.verify
    def check(*args, **kwargs):
        with pytest.raises(BlockingIOError):
            TaskJournal(paths['task_journal'])
        return original(*args, **kwargs)
    monkeypatch.setattr(robot_data, 'verify', check)
    assert restore(tmp_path / 'backup', home)['verification']['healthy']
    TaskJournal(paths['task_journal']).close()


def test_restore_keeps_crash_wal_with_old_database_not_restored_database(tmp_path):
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    backup(home, tmp_path / 'backup')
    path = paths['task_journal']
    subprocess.run([sys.executable, '-c', '''
import os, sqlite3, sys
db = sqlite3.connect(sys.argv[1])
db.execute('PRAGMA journal_mode=WAL')
db.execute('CREATE TABLE crash_canary(value)')
db.execute('INSERT INTO crash_canary VALUES (42)')
db.commit()
os._exit(0)
''', str(path)], check=True)
    assert Path(f'{path}-wal').is_file()
    result = restore(tmp_path / 'backup', home)
    with sqlite3.connect(path) as db:
        assert not db.execute("SELECT 1 FROM sqlite_master WHERE name='crash_canary'").fetchall()
    with sqlite3.connect(result['kept']['task_journal']) as db:
        assert db.execute('SELECT value FROM crash_canary').fetchone() == (42,)


@pytest.mark.parametrize('change', [
    lambda d: d.update(domain='../escape'),
    lambda d: d.update(sim='false'),
    lambda d: d['stores']['observations'].update(file='../observations.sqlite3'),
    lambda d: d['stores']['observations'].update(size=True),
])
def test_restore_rejects_malformed_manifests_before_replacing_data(tmp_path, change):
    home = tmp_path / 'robot'
    paths, _ = populated(home)
    directory = tmp_path / 'backup'
    manifest = backup(home, directory)
    change(manifest)
    (directory / 'manifest.json').write_text(json.dumps(manifest))
    before = paths['task_journal'].read_bytes()
    with pytest.raises(ValueError):
        restore(directory, home)
    assert paths['task_journal'].read_bytes() == before


def test_cli_backs_up_and_verifies_a_home(tmp_path):
    home = tmp_path / 'robot'
    populated(home, domain=5)

    def run(*args):
        return subprocess.run([sys.executable, str(SCRIPT), *args, '--home', str(home), '--domain', '5'],
                              capture_output=True, text=True, check=False)
    assert run('backup', str(tmp_path / 'backup')).returncode == 0
    verified = run('verify')
    assert verified.returncode == 0 and json.loads(verified.stdout)['visual_evidence'] == 'verified'
    assert run('restore').returncode == 2  # restore needs the backup directory


@pytest.mark.parametrize('name, owner', [('task_journal', TaskJournal), ('observations', ObservationStore)])
def test_restored_schema_versions_are_judged_as_their_owner_would(tmp_path, name, owner):
    """T-61: an older restored schema is opened and migrated by its owner; a newer one is refused, not guessed."""
    home = tmp_path / 'robot'
    path = store_paths(load_policy(), home)[name]
    path.parent.mkdir(parents=True)
    newest = max(SUPPORTED_VERSIONS[name])
    for version, supported in ((newest, True), (newest + 1, False)):
        path.unlink(missing_ok=True)
        database = sqlite3.connect(path)
        database.execute(f'PRAGMA user_version={version}')
        database.close()
        assert verify(home)[name]['schema_supported'] is supported and verify(home)['healthy'] is supported
        if supported:
            owner(path).close()
        else:
            with pytest.raises((RuntimeError, ValueError), match='unsupported'):
                owner(path)
