"""Durable task state, independent of ROS transport and motion control.

The journal also owns physical resource exclusion and fencing. A task holds its
resources from admission until its row is terminal, and a terminal row always
means the executor was proven to have finished (or never started), so release
never follows from a timeout or an empty process memory. Every grant of
responsibility (admission or handoff) takes a fresh, monotonic generation; a
command that names an older generation is refused.
"""

import fcntl
import json
from pathlib import Path
import sqlite3
import time
import uuid

TERMINAL = {'succeeded', 'canceled', 'failed'}
SCHEMA = 5
# Exclusive physical resources. Fixed cameras are read concurrently and are not
# reserved; the wrist camera is aimed by the arm and so follows the arm.
RESOURCES = ('base', 'arm', 'perception_query')
# A learned skill keeps the base still while it moves the arm; search changes
# what perception reports, so it reserves the active open-vocabulary query.
CAPABILITY_RESOURCES = {'navigate': ('base',), 'explore': ('base',),
                        'search': ('base', 'perception_query'), 'skill': ('arm', 'base')}
EVENT_RETENTION = 10000


class Refused(ValueError):
    """A request refused before any state changed, with a stable machine-readable code."""

    def __init__(self, code, detail, **data):
        super().__init__(detail)
        self.code, self.data = code, data


def _json(value):
    return json.dumps(value, sort_keys=True, allow_nan=False)


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
            if version not in range(SCHEMA + 1):
                raise RuntimeError('unsupported task journal version')
            if 0 < version < SCHEMA:
                # Earlier releases refuse a newer journal; keep a consistent copy so a rollback can restore it.
                backup = sqlite3.connect(f'{path}.v{version}.bak')
                try:
                    self._db.backup(backup)
                finally:
                    backup.close()
                Path(f'{path}.v{version}.bak').chmod(0o600)
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
                if version < 5:
                    for column in ("owner TEXT NOT NULL DEFAULT ''", 'generation INTEGER NOT NULL DEFAULT 0',
                                   'reconciled_generation INTEGER NOT NULL DEFAULT 0',
                                   "resources TEXT NOT NULL DEFAULT '[]'", 'api_schema INTEGER NOT NULL DEFAULT 1',
                                   "digest TEXT NOT NULL DEFAULT ''", "cancel_receipt TEXT NOT NULL DEFAULT ''",
                                   "checkpoint TEXT NOT NULL DEFAULT ''", "compensation TEXT NOT NULL DEFAULT ''",
                                   "compensates TEXT NOT NULL DEFAULT ''"):
                        self._db.execute(f'ALTER TABLE tasks ADD COLUMN {column}')
                    # Older gateways admitted one task at a time; it keeps holding what it physically uses.
                    for capability, names in CAPABILITY_RESOURCES.items():
                        self._db.execute("""UPDATE tasks SET resources=? WHERE capability=?
                            AND state NOT IN ('succeeded','canceled','failed')""", (_json(list(names)), capability))
                self._db.execute('''CREATE TABLE IF NOT EXISTS events (
                    seq INTEGER PRIMARY KEY AUTOINCREMENT, task_id TEXT NOT NULL, kind TEXT NOT NULL,
                    generation INTEGER NOT NULL, at REAL NOT NULL, data TEXT NOT NULL)''')
                self._db.execute('CREATE INDEX IF NOT EXISTS events_task ON events(task_id, seq)')
                self._db.execute('CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
                self._db.execute("INSERT OR IGNORE INTO meta VALUES ('journal_id', ?)", (uuid.uuid4().hex,))
                self._db.execute('CREATE INDEX IF NOT EXISTS tasks_goal_id ON tasks(goal_id)')
                self._db.execute("CREATE INDEX IF NOT EXISTS tasks_active ON tasks(id) WHERE state NOT IN ('succeeded','canceled','failed')")
                self._db.execute(f'PRAGMA user_version={SCHEMA}')
            # Cursors are only comparable within one journal; a replaced file is a new identity.
            self.journal_id = self._db.execute("SELECT value FROM meta WHERE key='journal_id'").fetchone()[0]
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

    def _event(self, row, kind, data=None):
        """Append inside the caller's transaction, so an event exists exactly when its change does."""
        self._db.execute('INSERT INTO events (task_id,kind,generation,at,data) VALUES (?,?,?,?,?)',
                         (row['id'], kind, row['generation'], time.time(), _json(data or {})))
        self._db.execute('DELETE FROM events WHERE seq <= (SELECT MAX(seq) FROM events) - ?', (EVENT_RETENTION,))

    def event(self, identifier, kind, data=None):
        """Record an observation that changes no task state, such as a late reply."""
        row = self.row(identifier)
        if row is not None:
            with self._db:
                self._event(row, kind, data)

    def holders(self, resources, exclude=''):
        """Active tasks holding any of these resources; an `unknown` holder still holds them."""
        wanted = set(resources)
        held = []
        for row in self.active():
            for name in sorted(wanted & set(json.loads(row['resources']))):
                if row['id'] != exclude:
                    held.append({'resource': name, 'task_id': row['id'], 'owner': row['owner'],
                                 'generation': row['generation'], 'state': row['state']})
        return held

    def resource_state(self):
        state = {name: None for name in RESOURCES}
        for holder in self.holders(RESOURCES):
            state[holder['resource']] = {key: holder[key] for key in ('task_id', 'owner', 'generation', 'state')}
        return state

    def require_free(self, resources):
        holders = self.holders(resources)
        if holders:
            held = ', '.join(f"{item['resource']} by task {item['task_id']} (generation {item['generation']}, "
                             f"{item['state']})" for item in holders)
            # An unconfirmed holder is not a busy one: reconcile it before planning around it.
            uncertain = any(item['state'] == 'unknown' for item in holders)
            raise Refused('execution_uncertain' if uncertain else 'resource_busy',
                          f'resources held: {held}; another managed task is active or its result is unconfirmed',
                          holders=holders)

    def admit(self, identifier, capability, parameters, goal_id, result=None, *, resources=(),
              owner='', api_schema=1, digest='', compensates=''):
        with self._db:
            self._db.execute('BEGIN IMMEDIATE')
            self.require_free(resources)
            generation = self._db.execute('SELECT COALESCE(MAX(generation), 0) + 1 FROM tasks').fetchone()[0]
            self._db.execute('''INSERT INTO tasks
                (id,capability,parameters,goal_id,state,detail,result,dispatched,owner,generation,
                 reconciled_generation,resources,api_schema,digest,compensates) VALUES (?,?,?,?,?,?,?,0,?,?,?,?,?,?,?)''',
                (identifier, capability, parameters, goal_id, 'starting', 'durably admitted; awaiting action acceptance',
                 json.dumps(result or {}, allow_nan=False), owner, generation, generation, _json(list(resources)),
                 api_schema, digest, compensates))
            row = self.row(identifier)
            self._event(row, 'admitted', {'capability': capability, 'owner': owner, 'resources': list(resources),
                                          'state': row['state'], 'detail': row['detail']})
            if compensates:
                original = self.row(compensates)
                plan = json.loads(original['compensation'])
                plan['tasks'] = {**plan.get('tasks', {}), identifier: 'starting'}
                plan['status'] = 'in_progress'
                self._db.execute('UPDATE tasks SET compensation=? WHERE id=?', (_json(plan), compensates))
                self._event(original, 'compensation_started', {'task_id': identifier})
        return row

    def update(self, identifier, state, detail, result=None):
        row = self.row(identifier)
        if row['state'] in TERMINAL or (row['state'] == state and row['detail'] == detail and result is None):
            return False
        with self._db:
            self._db.execute('UPDATE tasks SET state=?,detail=?,result=? WHERE id=?',
                             (state, detail, json.dumps(result, allow_nan=False) if result is not None else row['result'], identifier))
            terminal = state in TERMINAL
            self._event(row, 'state', {'state': state, 'detail': detail, 'terminal': terminal,
                                       **({'released': json.loads(row['resources'])} if terminal else {})})
            if terminal and row['compensates']:
                self._compensation_finished(row, state)
        return True

    def _compensation_finished(self, row, state):
        original = self.row(row['compensates'])
        plan = json.loads(original['compensation'])
        plan['tasks'] = {**plan.get('tasks', {}), row['id']: state}
        done = sum(value == 'succeeded' for value in plan['tasks'].values())
        plan['status'] = ('failed' if state != 'succeeded' else
                          'succeeded' if done >= len(plan['plan']['steps']) else 'in_progress')
        self._db.execute('UPDATE tasks SET compensation=? WHERE id=?', (_json(plan), original['id']))
        # The original task's own state is never changed by its compensation.
        self._event(original, 'compensation_' + ('finished' if plan['status'] != 'in_progress' else 'step'),
                    {'task_id': row['id'], 'state': state, 'status': plan['status']})

    def mark_dispatched(self, identifier, server_id=''):
        # Commit before the transport call: an uncertain delivery must never be replayed.
        with self._db:
            self._db.execute('UPDATE tasks SET dispatched=1,server_id=? WHERE id=?', (server_id, identifier))
            self._event(self.row(identifier), 'dispatched', {'server_id': server_id})

    def navigation_budget(self, identifier, remaining):
        with self._db:
            self._db.execute('UPDATE tasks SET remaining_sec=? WHERE id=?', (max(0.0, remaining), identifier))

    def request_navigation_pause(self, identifier):
        with self._db:
            self._db.execute('UPDATE tasks SET pause_requested=1 WHERE id=?', (identifier,))

    def navigation_stopped(self, identifier):
        with self._db:
            self._db.execute('UPDATE tasks SET navigation_stopped=1 WHERE id=?', (identifier,))
            self._event(self.row(identifier), 'navigation_stopped')

    def resume_navigation(self, identifier, goal_id):
        with self._db:
            cursor = self._db.execute("""UPDATE tasks SET goal_id=?,dispatched=0,navigation_stopped=0,pause_requested=0,server_id='',
                state='starting',detail='resuming navigation after confirmed stop'
                WHERE id=? AND capability='navigate' AND navigation_stopped=1 AND cancel_requested=0
                AND state NOT IN ('succeeded','canceled','failed')""", (goal_id, identifier))
            if cursor.rowcount != 1:
                raise RuntimeError('navigation cannot resume without confirmed termination')
            self._event(self.row(identifier), 'resumed')

    def request_cancel(self, identifier, reason=1, requester=''):
        """Durable cancellation intent; repeated requests return the first receipt."""
        row = self.row(identifier)
        receipt = json.loads(row['cancel_receipt'] or 'null')
        with self._db:
            if receipt is None:
                receipt = {'receipt_id': uuid.uuid4().hex, 'task_id': identifier, 'requested_at': time.time(),
                           'reason': 'deadline' if reason == 2 else 'requested', 'requester': requester,
                           'generation': row['generation']}
                self._db.execute('UPDATE tasks SET cancel_receipt=? WHERE id=?', (_json(receipt), identifier))
                self._event(row, 'cancel_requested', receipt)
            self._db.execute('UPDATE tasks SET cancel_requested=MAX(cancel_requested,?) WHERE id=?', (reason, identifier))
        return receipt

    def fence(self, identifier, generation, owner=None, reconciled=False):
        """Refuse a command from a superseded owner, or one that has not reconciled since a handoff."""
        row = self.row(identifier)
        if row is None:
            raise Refused('not_found', 'task not found')
        if generation != row['generation']:
            raise Refused('stale_generation', f'generation {generation} was superseded by {row["generation"]}',
                          generation=row['generation'], owner=row['owner'])
        if owner is not None and owner != row['owner']:
            raise Refused('authority_denied', 'owner does not hold this generation')
        if reconciled and row['reconciled_generation'] != row['generation']:
            raise Refused('execution_uncertain', 'read status or events with this generation before acting')
        return row

    def reconcile(self, identifier, generation):
        """The holder of `generation` has read the journal; it may now act on the task."""
        row = self.row(identifier)
        if row is not None and generation == row['generation'] != row['reconciled_generation']:
            with self._db:
                self._db.execute('UPDATE tasks SET reconciled_generation=? WHERE id=?', (generation, identifier))
                self._event(row, 'reconciled', {'owner': row['owner']})

    def transfer(self, identifier, generation, owner, new_owner, checkpoint):
        with self._db:
            self._db.execute('BEGIN IMMEDIATE')
            # An unreconciled successor can still pass responsibility on; it cannot act.
            self.fence(identifier, generation, owner)
            successor = self._db.execute('SELECT MAX(generation) + 1 FROM tasks').fetchone()[0]
            # The successor must reconcile before acting, so its generation starts unreconciled.
            self._db.execute('UPDATE tasks SET owner=?,generation=?,checkpoint=? WHERE id=?',
                             (new_owner, successor, _json(checkpoint), identifier))
            self._event(self.row(identifier), 'handoff', {'from_owner': owner, 'to_owner': new_owner,
                                                          'previous_generation': generation,
                                                          'summary': checkpoint.get('summary', '')})
        return self.row(identifier)

    def record_compensation(self, identifier, generation, owner, compensation):
        with self._db:
            self._db.execute('BEGIN IMMEDIATE')
            row = self.fence(identifier, generation, owner, reconciled=True)
            if row['compensation']:
                raise Refused('identity_conflict', 'a compensation plan is already recorded for this task')
            self._db.execute('UPDATE tasks SET compensation=? WHERE id=?', (_json(compensation), identifier))
            self._event(row, 'compensation_planned', {'intent': compensation['plan']['intent'],
                                                      'steps': len(compensation['plan']['steps'])})
        return self.row(identifier)

    def events_since(self, cursor, identifier='', limit=100):
        query, values = 'SELECT * FROM events WHERE seq>?', [cursor]
        if identifier:
            query, values = query + ' AND task_id=?', [*values, identifier]
        rows = self._db.execute(query + ' ORDER BY seq LIMIT ?', (*values, limit)).fetchall()
        oldest, head = self._db.execute('SELECT MIN(seq), MAX(seq) FROM events').fetchone()
        return {'journal_id': self.journal_id, 'events': [
                    {'seq': row['seq'], 'task_id': row['task_id'], 'kind': row['kind'],
                     'generation': row['generation'], 'at': row['at'], 'data': json.loads(row['data'])}
                    for row in rows],
                'cursor': rows[-1]['seq'] if rows else min(cursor, head or 0), 'head': head or 0,
                # Older events were pruned or the cursor came from another journal: reconcile by status.
                'truncated': (oldest is not None and cursor < oldest - 1) or cursor > (head or 0),
                'more': len(rows) == limit}

    def times(self, identifier):
        admitted, dispatched, finished = self._db.execute('''SELECT
            MIN(CASE WHEN kind='admitted' THEN at END), MIN(CASE WHEN kind='dispatched' THEN at END),
            MAX(CASE WHEN kind='state' AND json_extract(data, '$.terminal') THEN at END)
            FROM events WHERE task_id=?''', (identifier,)).fetchone()
        return {'admitted_at': admitted, 'dispatched_at': dispatched, 'finished_at': finished}

    def last_event(self, identifier):
        return self._db.execute('SELECT COALESCE(MAX(seq), 0) FROM events WHERE task_id=?', (identifier,)).fetchone()[0]

    def close(self):
        self._db.close()
        self._lock.close()
