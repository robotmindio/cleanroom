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
            if version not in (0, 1, 2, 3, 4):
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
                if version < 3:
                    self._db.execute("ALTER TABLE tasks ADD COLUMN server_id TEXT NOT NULL DEFAULT ''")
                    self._db.execute('ALTER TABLE tasks ADD COLUMN remaining_sec REAL')
                    self._db.execute('ALTER TABLE tasks ADD COLUMN navigation_stopped INTEGER NOT NULL DEFAULT 0')
                if version < 4:
                    self._db.execute('ALTER TABLE tasks ADD COLUMN pause_requested INTEGER NOT NULL DEFAULT 0')
                    if version == 3:
                        self._db.execute("UPDATE tasks SET pause_requested=1 WHERE capability='navigate' AND dispatched=1 AND state='paused'")
                self._db.execute('CREATE INDEX IF NOT EXISTS tasks_goal_id ON tasks(goal_id)')
                self._db.execute("CREATE INDEX IF NOT EXISTS tasks_active ON tasks(id) WHERE state NOT IN ('succeeded','canceled','failed')")
                self._db.execute('PRAGMA user_version=4')
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

    def admit(self, identifier, capability, parameters, goal_id, result=None):
        with self._db:
            self._db.execute('''INSERT INTO tasks
                (id,capability,parameters,goal_id,state,detail,result,dispatched) VALUES (?,?,?,?,?,?,?,0)''',
                (identifier, capability, parameters, goal_id, 'starting',
                 'durably admitted; awaiting action acceptance', json.dumps(result or {}, allow_nan=False)))

    def update(self, identifier, state, detail, result=None):
        row = self.row(identifier)
        if row['state'] in TERMINAL or (row['state'] == state and row['detail'] == detail and result is None):
            return False
        with self._db:
            self._db.execute('UPDATE tasks SET state=?,detail=?,result=? WHERE id=?',
                             (state, detail, json.dumps(result, allow_nan=False) if result is not None else row['result'], identifier))
        return True

    def mark_dispatched(self, identifier, server_id=''):
        # Commit before the transport call: an uncertain delivery must never be replayed.
        with self._db:
            self._db.execute('UPDATE tasks SET dispatched=1,server_id=? WHERE id=?', (server_id, identifier))

    def navigation_budget(self, identifier, remaining):
        with self._db:
            self._db.execute('UPDATE tasks SET remaining_sec=? WHERE id=?', (max(0.0, remaining), identifier))

    def request_navigation_pause(self, identifier):
        with self._db:
            self._db.execute('UPDATE tasks SET pause_requested=1 WHERE id=?', (identifier,))

    def navigation_stopped(self, identifier):
        with self._db:
            self._db.execute('UPDATE tasks SET navigation_stopped=1 WHERE id=?', (identifier,))

    def resume_navigation(self, identifier, goal_id):
        with self._db:
            cursor = self._db.execute("""UPDATE tasks SET goal_id=?,dispatched=0,navigation_stopped=0,pause_requested=0,server_id='',
                state='starting',detail='resuming navigation after confirmed stop'
                WHERE id=? AND capability='navigate' AND navigation_stopped=1 AND cancel_requested=0
                AND state NOT IN ('succeeded','canceled','failed')""", (goal_id, identifier))
            if cursor.rowcount != 1:
                raise RuntimeError('navigation cannot resume without confirmed termination')

    def request_cancel(self, identifier, reason=1):
        with self._db:
            self._db.execute('UPDATE tasks SET cancel_requested=MAX(cancel_requested,?) WHERE id=?', (reason, identifier))

    def close(self):
        self._db.close()
        self._lock.close()
