"""Recovery distinguishes proven-unsent work from uncertain action delivery."""

import json
import sqlite3

import pytest

from lekiwi_rmf.task_journal import Refused, TaskJournal

SCHEMA_5_COLUMNS = ('owner', 'generation', 'reconciled_generation', 'resources', 'api_schema', 'digest',
                    'cancel_receipt', 'checkpoint', 'compensation', 'compensates')


def test_dispatch_proof_survives_restart(tmp_path):
    path = tmp_path / 'tasks.db'
    journal = TaskJournal(path)
    journal.admit('unsent', 'explore', '{}', bytes(16))
    journal.admit('attempted', 'explore', '{}', bytes(range(16)))
    journal.mark_dispatched('attempted')
    journal.close()
    recovered = TaskJournal(path)
    try:
        assert not recovered.row('unsent')['dispatched']
        assert recovered.row('attempted')['dispatched']
    finally:
        recovered.close()


def test_migration_never_replays_old_rows(tmp_path):
    path = tmp_path / 'tasks.db'
    with sqlite3.connect(path) as database:
        database.execute('''CREATE TABLE tasks (
            id TEXT PRIMARY KEY, capability TEXT, parameters TEXT, goal_id BLOB,
            state TEXT, detail TEXT, result TEXT DEFAULT '{}', cancel_requested INTEGER DEFAULT 0)''')
        database.execute("INSERT INTO tasks (id,state) VALUES ('old','starting')")
        database.execute('PRAGMA user_version=1')
    journal = TaskJournal(path)
    assert journal.row('old')['dispatched']
    journal.close()
    reopened = TaskJournal(path)
    assert reopened.row('old')['dispatched']
    reopened.close()


def test_terminal_result_is_immutable_and_deadline_reason_survives_cancel(tmp_path):
    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'navigate', '{}', bytes(16))
        journal.request_cancel('task', reason=2)
        journal.request_cancel('task')
        assert journal.row('task')['cancel_requested'] == 2
        assert journal.update('task', 'failed', 'deadline reached', {'error': 1})
        assert not journal.update('task', 'unknown', 'late transport error')
        assert journal.row('task')['state'] == 'failed'
        assert journal.row('task')['result'] == '{"error": 1}'
    finally:
        journal.close()


def test_rejected_journal_version_releases_lock(tmp_path):
    path = tmp_path / 'tasks.db'
    with sqlite3.connect(path) as database:
        database.execute('PRAGMA user_version=99')
    with pytest.raises(RuntimeError, match='unsupported'):
        TaskJournal(path)
    with sqlite3.connect(path) as database:
        database.execute('PRAGMA user_version=0')
    journal = TaskJournal(path)
    journal.close()


def test_navigation_resume_requires_terminal_proof_and_preserves_budget(tmp_path):
    path = tmp_path / 'tasks.db'
    journal = TaskJournal(path)
    journal.admit('task', 'navigate', '{"x":1,"y":2}', bytes(16))
    journal.mark_dispatched('task', 'original-server')
    journal.navigation_budget('task', 123.4)
    with pytest.raises(RuntimeError, match='confirmed termination'):
        journal.resume_navigation('task', bytes(range(16)))
    journal.navigation_stopped('task')
    journal.close()
    journal = TaskJournal(path)
    try:
        assert journal.row('task')['server_id'] == 'original-server'
        journal.resume_navigation('task', bytes(range(16)))
        row = journal.row('task')
        assert row['remaining_sec'] == 123.4
        assert row['goal_id'] == bytes(range(16))
        assert not row['dispatched'] and not row['navigation_stopped'] and not row['pause_requested']
        journal.navigation_stopped('task')
        journal.request_cancel('task')
        with pytest.raises(RuntimeError, match='confirmed termination'):
            journal.resume_navigation('task', bytes(16))
    finally:
        journal.close()


def test_version_two_migration_preserves_unsent_rows(tmp_path):
    path = tmp_path / 'tasks.db'
    with sqlite3.connect(path) as database:
        database.execute('''CREATE TABLE tasks (
            id TEXT PRIMARY KEY, capability TEXT, parameters TEXT, goal_id BLOB,
            state TEXT, detail TEXT, result TEXT DEFAULT '{}', cancel_requested INTEGER DEFAULT 0,
            dispatched INTEGER NOT NULL DEFAULT 1)''')
        database.execute("INSERT INTO tasks (id,state,dispatched) VALUES ('old','starting',0)")
        database.execute('PRAGMA user_version=2')
    journal = TaskJournal(path)
    try:
        row = journal.row('old')
        assert not row['dispatched'] and not row['navigation_stopped'] and not row['pause_requested']
        assert row['server_id'] == '' and row['remaining_sec'] is None
    finally:
        journal.close()


def test_pause_intent_survives_restart_and_clears_only_when_resuming(tmp_path):
    path = tmp_path / 'tasks.db'
    journal = TaskJournal(path)
    journal.admit('task', 'navigate', '{}', bytes(16))
    journal.mark_dispatched('task', 'server')
    journal.request_navigation_pause('task')
    journal.close()
    journal = TaskJournal(path)
    try:
        assert journal.row('task')['pause_requested']
        assert not journal.row('task')['navigation_stopped']
        with pytest.raises(RuntimeError, match='confirmed termination'):
            journal.resume_navigation('task', bytes(range(16)))
        journal.navigation_stopped('task')
        journal.resume_navigation('task', bytes(range(16)))
        assert not journal.row('task')['pause_requested']
    finally:
        journal.close()


def test_version_three_migration_recovers_explicit_pending_pause(tmp_path):
    path = tmp_path / 'tasks.db'
    journal = TaskJournal(path)
    journal.admit('task', 'navigate', '{}', bytes(16))
    journal.mark_dispatched('task', 'server')
    journal.navigation_budget('task', 123.4)
    journal.update('task', 'paused', 'paused: StopZone')
    journal.close()
    with sqlite3.connect(path) as database:
        for column in ('pause_requested', *SCHEMA_5_COLUMNS):
            database.execute(f'ALTER TABLE tasks DROP COLUMN {column}')
        database.execute('DROP TABLE events')
        database.execute('DROP TABLE meta')
        database.execute('PRAGMA user_version=3')
    journal = TaskJournal(path)
    try:
        row = journal.row('task')
        assert row['pause_requested'] and not row['navigation_stopped']
        assert row['remaining_sec'] == 123.4 and row['server_id'] == 'server'
    finally:
        journal.close()


def schema_4(path, rows):
    """A journal exactly as a schema-4 gateway left it."""
    TaskJournal(path).close()
    with sqlite3.connect(path) as database:
        for column in SCHEMA_5_COLUMNS:
            database.execute(f'ALTER TABLE tasks DROP COLUMN {column}')
        database.execute('DROP TABLE events')
        database.execute('DROP TABLE meta')
        for identifier, capability, state in rows:
            database.execute("""INSERT INTO tasks (id,capability,parameters,goal_id,state,detail,dispatched)
                VALUES (?,?,'{}',?,?,'',1)""", (identifier, capability, identifier.encode().ljust(16, b'.'), state))
        database.execute('PRAGMA user_version=4')


def test_schema_4_active_task_keeps_its_resources_after_migration(tmp_path):
    path = tmp_path / 'tasks.db'
    schema_4(path, [('live', 'search', 'running'), ('done', 'explore', 'succeeded')])
    journal = TaskJournal(path)
    try:
        assert journal.resource_state() == {
            'base': {'task_id': 'live', 'owner': '', 'generation': 0, 'state': 'running'},
            'arm': None, 'perception_query': {'task_id': 'live', 'owner': '', 'generation': 0, 'state': 'running'}}
        with pytest.raises(Refused) as refused:
            journal.admit('next', 'skill', '{}', bytes(16), resources=('arm', 'base'))
        assert refused.value.code == 'resource_busy'
        assert journal.row('next') is None
        assert journal.events_since(0)['events'] == []
    finally:
        journal.close()


def test_uncertain_holder_is_reported_as_uncertain_not_busy(tmp_path):
    path = tmp_path / 'tasks.db'
    schema_4(path, [('lost', 'navigate', 'unknown')])
    journal = TaskJournal(path)
    try:
        with pytest.raises(Refused) as refused:
            journal.require_free(('base',))
        assert refused.value.code == 'execution_uncertain'
        assert refused.value.data['holders'][0]['task_id'] == 'lost'
    finally:
        journal.close()


def test_resources_are_released_only_by_a_terminal_row(tmp_path):
    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        first = journal.admit('first', 'skill', '{}', bytes(16), resources=('arm', 'base'), owner='agent-a')
        journal.update('first', 'unknown', 'result transport failed; termination unconfirmed')
        with pytest.raises(Refused):
            journal.admit('second', 'navigate', '{}', bytes(range(16)), resources=('base',), owner='agent-b')
        journal.update('first', 'canceled', 'stopped')
        second = journal.admit('second', 'navigate', '{}', bytes(range(16)), resources=('base',), owner='agent-b')
        assert second['generation'] > first['generation']
        released = [event for event in journal.events_since(0, 'first')['events'] if event['data'].get('terminal')]
        assert released[0]['data']['released'] == ['arm', 'base']
    finally:
        journal.close()


def test_cancel_receipt_is_durable_and_idempotent(tmp_path):
    path = tmp_path / 'tasks.db'
    journal = TaskJournal(path)
    journal.admit('task', 'explore', '{}', bytes(16), resources=('base',))
    receipt = journal.request_cancel('task', requester='agent-a')
    assert journal.request_cancel('task', reason=2, requester='deadline') == receipt
    journal.close()
    journal = TaskJournal(path)
    try:
        assert journal.request_cancel('task') == receipt
        assert journal.row('task')['cancel_requested'] == 2
        kinds = [event['kind'] for event in journal.events_since(0, 'task')['events']]
        assert kinds.count('cancel_requested') == 1
    finally:
        journal.close()


def test_handoff_fences_the_previous_owner_and_requires_reconciliation(tmp_path):
    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        row = journal.admit('task', 'explore', '{}', bytes(16), resources=('base',), owner='agent-a')
        handed = journal.transfer('task', row['generation'], 'agent-a', 'agent-b', {'summary': 'continue'})
        assert handed['owner'] == 'agent-b' and handed['generation'] > row['generation']
        for generation, owner, code in ((row['generation'], 'agent-a', 'stale_generation'),
                                        (handed['generation'], 'agent-a', 'authority_denied')):
            with pytest.raises(Refused) as refused:
                journal.fence('task', generation, owner)
            assert refused.value.code == code
        with pytest.raises(Refused) as refused:
            journal.fence('task', handed['generation'], 'agent-b', reconciled=True)
        assert refused.value.code == 'execution_uncertain'
        journal.reconcile('task', row['generation'])
        assert journal.row('task')['reconciled_generation'] == row['generation']
        journal.reconcile('task', handed['generation'])
        assert journal.fence('task', handed['generation'], 'agent-b', reconciled=True)['owner'] == 'agent-b'
        # The superseded owner cannot hand it back.
        with pytest.raises(Refused):
            journal.transfer('task', row['generation'], 'agent-a', 'agent-a', {'summary': 'steal'})
    finally:
        journal.close()


def test_events_resume_from_a_cursor_and_report_truncation(tmp_path, monkeypatch):
    import lekiwi_rmf.task_journal as module
    monkeypatch.setattr(module, 'EVENT_RETENTION', 3)
    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'explore', '{}', bytes(16))
        first = journal.events_since(0)
        assert [event['kind'] for event in first['events']] == ['admitted'] and not first['truncated']
        for index in range(5):
            journal.update('task', 'running', f'stage {index}')
        later = journal.events_since(first['cursor'])
        assert later['truncated'] and later['cursor'] == later['head']
        assert journal.events_since(later['cursor'])['events'] == []
        assert journal.events_since(later['head'] + 10)['truncated']
    finally:
        journal.close()


def test_compensation_never_changes_the_original_outcome(tmp_path):
    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        row = journal.admit('weld', 'skill', '{}', bytes(16), resources=('arm', 'base'), owner='agent-a')
        journal.update('weld', 'failed', 'implementation aborted')
        plan = {'plan': {'intent': 'isolate', 'steps': [{'capability': 'navigate', 'parameters': {'x': 0, 'y': 0}}],
                         'note': 'move away from the part'}, 'status': 'planned', 'tasks': {}}
        journal.record_compensation('weld', row['generation'], 'agent-a', plan)
        with pytest.raises(Refused):
            journal.record_compensation('weld', row['generation'], 'agent-a', plan)
        journal.admit('away', 'navigate', '{"x":0,"y":0}', bytes(range(16)), resources=('base',),
                      owner='agent-a', compensates='weld')
        journal.update('away', 'succeeded', 'reached')
        original = journal.row('weld')
        assert original['state'] == 'failed'
        assert json.loads(original['compensation'])['status'] == 'succeeded'
    finally:
        journal.close()
