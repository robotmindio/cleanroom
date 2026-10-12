"""Recovery distinguishes proven-unsent work from uncertain action delivery."""

import sqlite3

import pytest

from lekiwi_rmf.task_journal import TaskJournal


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
        assert not row['dispatched'] and not row['navigation_stopped']
        journal.navigation_stopped('task')
        journal.request_cancel('task')
        with pytest.raises(RuntimeError, match='confirmed termination'):
            journal.resume_navigation('task', bytes(16))
    finally:
        journal.close()
