"""Durable task state, independent of ROS transport and motion control."""

import fcntl
import json
from pathlib import Path
import sqlite3

TERMINAL = {'succeeded', 'canceled', 'failed'}


class TaskJournal:
    def __init__(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        # Two gateways must never race to submit the same journaled action.
        self._lock = Path(f'{path}.lock').open('a')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._db = sqlite3.connect(path, check_same_thread=False)
            path.chmod(0o600)
            self._db.row_factory = sqlite3.Row
            version = self._db.execute('PRAGMA user_version').fetchone()[0]
            if version not in (0, 1, 2):
                raise RuntimeError('unsupported task journal version')
            with self._db:
                self._db.execute('BEGIN')
                self._db.execute('''CREATE TABLE IF NOT EXISTS tasks (
                    id TEXT PRIMARY KEY, capability TEXT NOT NULL, parameters TEXT NOT NULL,
                    goal_id BLOB NOT NULL, state TEXT NOT NULL, detail TEXT NOT NULL,
                    result TEXT NOT NULL DEFAULT '{}', cancel_requested INTEGER NOT NULL DEFAULT 0)''')
                if version < 2:
                    # Old rows may already have been sent; never replay them after migration.
                    self._db.execute('ALTER TABLE tasks ADD COLUMN dispatched INTEGER NOT NULL DEFAULT 1')
                self._db.execute('CREATE INDEX IF NOT EXISTS tasks_goal_id ON tasks(goal_id)')
                self._db.execute("CREATE INDEX IF NOT EXISTS tasks_active ON tasks(id) WHERE state NOT IN ('succeeded','canceled','failed')")
                self._db.execute('PRAGMA user_version=2')
        except BaseException:
            if hasattr(self, '_db'):
                self._db.close()
            self._lock.close()
            raise

    def active(self):
        return self._db.execute("SELECT * FROM tasks WHERE state NOT IN ('succeeded','canceled','failed')").fetchall()

    def row(self, identifier):
        return self._db.execute('SELECT * FROM tasks WHERE id=?', (identifier,)).fetchone()

    def goal(self, goal_id):
        return self._db.execute('SELECT * FROM tasks WHERE goal_id=?', (goal_id,)).fetchone()

    def admit(self, identifier, capability, parameters, goal_id):
        with self._db:
            self._db.execute('''INSERT INTO tasks
                (id,capability,parameters,goal_id,state,detail,dispatched) VALUES (?,?,?,?,?,?,0)''',
                (identifier, capability, parameters, goal_id, 'starting',
                 'durably admitted; awaiting action acceptance'))

    def update(self, identifier, state, detail, result=None):
        row = self.row(identifier)
        if row['state'] in TERMINAL or (row['state'] == state and row['detail'] == detail and result is None):
            return False
        with self._db:
            self._db.execute('UPDATE tasks SET state=?,detail=?,result=? WHERE id=?',
                             (state, detail, json.dumps(result, allow_nan=False) if result is not None else row['result'], identifier))
        return True

    def mark_dispatched(self, identifier):
        # Commit before the transport call: an uncertain delivery must never be replayed.
        with self._db:
            self._db.execute('UPDATE tasks SET dispatched=1 WHERE id=?', (identifier,))

    def request_cancel(self, identifier, reason=1):
        with self._db:
            self._db.execute('UPDATE tasks SET cancel_requested=MAX(cancel_requested,?) WHERE id=?', (reason, identifier))

    def close(self):
        self._db.close()
        self._lock.close()
