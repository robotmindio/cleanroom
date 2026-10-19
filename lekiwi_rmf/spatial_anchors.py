"""ROS-free spatial memory: map identities, revisions, anchors and reprojection.

Conventions (shared with Knowledge, schema ``robotmind.spacetime/1``):

* ``T_A_B`` maps coordinates from frame B into frame A: column vectors, metres,
  radians, right-handed rotation. A pose serializes as ``{"t": [x, y, z],
  "q": [x, y, z, w]}`` with a unit quaternion.
* A covariance is nine floats, row-major, in the frame named by its container.
* ``frame_id='map'`` alone never identifies a map. A place is
  ``(site_ref, map_ref, revision)``: ``map_ref`` is a persistent cartography,
  a revision is one optimized solution of its graph, and an anchor is one graph
  node whose map pose each revision states.

An estimate keeps its capture-frame measurement and its anchor:
``p_map(r) = T_map_anchor(r) * T_anchor_camera * p_camera``. A SLAM optimization
creates a revision with new anchor poses; projecting through it moves the
estimate without touching the stored observation, so a corrected map never
reads as an object that moved.
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import time
import uuid

import numpy as np

SCHEMA = 'robotmind.spacetime/1'
REF = re.compile(r'^[A-Za-z0-9._:/-]{1,128}$')
CAUSES = ('initial', 'optimization', 'loop_closure', 'merge', 'relocalization', 'static')
# An anchor that moved less than this between graph solutions keeps its pose row.
TRANSLATION_TOLERANCE_M = 1e-3
ROTATION_TOLERANCE_RAD = 1e-3
# Relinks of compacted anchors form chains; a longer one is reported, never followed.
MAX_RELINKS = 8


class SpatialError(ValueError):
    """A projection that cannot be made; ``code`` is the shared error vocabulary."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def valid_ref(value):
    return isinstance(value, str) and REF.match(value) is not None


def _finite(values, count, name):
    if not isinstance(values, (list, tuple)) or len(values) != count:
        raise ValueError(f'{name} requires {count} numbers')
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} requires finite numbers')
        result.append(float(value))
    return tuple(result)


@dataclass(frozen=True)
class Transform:
    """``T_A_B``: rotation quaternion (x, y, z, w) and translation of B's origin in A."""

    t: tuple
    q: tuple

    @classmethod
    def identity(cls):
        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    @classmethod
    def create(cls, t, q):
        t, q = _finite(t, 3, 'translation'), _finite(q, 4, 'quaternion')
        norm = math.sqrt(sum(v * v for v in q))
        if abs(norm - 1) > 1e-3:
            raise ValueError('quaternion is not normalized')
        q = tuple(v / norm for v in q)
        # One sign per rotation keeps digests and comparisons deterministic.
        if q[3] < 0:
            q = tuple(-v for v in q)
        return cls(t, q)

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {'t', 'q'}:
            raise ValueError('a pose is {"t": [x, y, z], "q": [x, y, z, w]}')
        return cls.create(value['t'], value['q'])

    @classmethod
    def from_matrix(cls, rotation, translation):
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError('rotation must be a finite 3x3 matrix')
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation) - 1) > 1e-6:
            raise ValueError('rotation must be orthonormal with determinant +1')
        m = rotation
        w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
        x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
        y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
        z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
        return cls.create(list(translation), [x, y, z, w])

    def to_dict(self):
        return {'t': [round(v, 6) for v in self.t], 'q': [round(v, 9) for v in self.q]}

    def rotation(self):
        x, y, z, w = self.q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    def compose(self, other):
        """``T_A_B.compose(T_B_C)`` is ``T_A_C``."""
        x1, y1, z1, w1 = self.q
        x2, y2, z2, w2 = other.q
        q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
             w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
        return Transform.create(self.apply(other.t), q)

    def inverse(self):
        x, y, z, w = self.q
        rotation = self.rotation().T
        return Transform.create(list(-(rotation @ np.asarray(self.t))), [-x, -y, -z, w])

    def apply(self, point):
        return tuple(float(v) for v in self.rotation() @ np.asarray(point, dtype=float) + np.asarray(self.t))

    def rotate_covariance(self, covariance):
        """Express a 3x3 position covariance of frame B in frame A."""
        matrix = np.asarray(_finite(covariance, 9, 'covariance')).reshape(3, 3)
        if not np.allclose(matrix, matrix.T, atol=1e-9) or np.linalg.eigvalsh(matrix).min() < -1e-12:
            raise ValueError('covariance must be symmetric positive semidefinite')
        rotation = self.rotation()
        return [float(v) for v in (rotation @ matrix @ rotation.T).reshape(-1)]

    def differs(self, other):
        translation = math.dist(self.t, other.t)
        dot = abs(sum(a * b for a, b in zip(self.q, other.q)))
        angle = 2 * math.acos(min(1.0, dot))
        return translation > TRANSLATION_TOLERANCE_M or angle > ROTATION_TOLERANCE_RAD


def table_digest(map_ref, parent, anchors):
    """Identity of one revision: its map, its parent and every anchor pose, canonically rounded.

    The parent is part of it, so a graph that returns to an earlier solution is a
    new revision rather than a jump back in history.
    """
    rows = [[ref, *pose.to_dict()['t'], *pose.to_dict()['q']] for ref, pose in sorted(anchors.items())]
    return hashlib.sha256(json.dumps([map_ref, parent, rows], separators=(',', ':')).encode()).hexdigest()


def rtabmap_database_tag(path):
    """Identity of one RTAB-Map database: its first node and when it entered.

    Node ids restart at 1 in a recreated database; without this tag a new map's
    nodes would masquerade as the anchors of the old one.
    """
    path = Path(path).expanduser()
    if not path.exists():
        raise SpatialError('anchor_identity_unavailable', f'RTAB-Map database {path} does not exist')
    try:
        connection = sqlite3.connect(f'file:{path}?mode=ro', uri=True, timeout=1.0)
        try:
            row = connection.execute('SELECT id, time_enter FROM Node ORDER BY id LIMIT 1').fetchone()
        finally:
            connection.close()
    except sqlite3.Error as error:
        raise SpatialError('anchor_identity_unavailable', f'RTAB-Map database is unreadable: {error}') from error
    if row is None:
        raise SpatialError('anchor_identity_unavailable', 'RTAB-Map database has no nodes yet')
    return hashlib.sha256(f'{row[0]}\x1f{row[1]}'.encode()).hexdigest()[:16]


@dataclass(frozen=True)
class Revision:
    sequence: int
    revision: str
    map_ref: str
    site_ref: str
    parent: str | None
    created_at: float
    cause: str
    session: str
    observer_ref: str
    anchors: int

    def to_dict(self):
        return {'schema': SCHEMA, 'site_ref': self.site_ref, 'map_ref': self.map_ref, 'revision': self.revision,
                'parent_revision': self.parent, 'cause': self.cause, 'created_at_unix_sec': self.created_at,
                'localization_session': self.session, 'observer_ref': self.observer_ref,
                'anchor_count': self.anchors, 'sequence': self.sequence}


class SpatialStore:
    """Durable maps, revisions, anchor poses, sessions and alignments for one site.

    Anchor poses are stored only when they change, so a revision is the newest
    pose row of each anchor at or before its sequence within its map. A NULL row
    marks an anchor that left the graph; it is relinked to a surviving anchor so
    estimates on it remain projectable.
    """

    def __init__(self, path, *, site_ref, observer_ref, retention_sec=7 * 86400.0, max_revisions=2000):
        if not valid_ref(site_ref) or not valid_ref(observer_ref):
            raise ValueError('site_ref and observer_ref must be opaque references')
        if not math.isfinite(retention_sec) or retention_sec <= 0 or type(max_revisions) is not int or max_revisions < 2:
            raise ValueError('invalid spatial retention limits')
        self.site_ref, self.observer_ref = site_ref, observer_ref
        self.retention_sec, self.max_revisions = retention_sec, max_revisions
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch(mode=0o600, exist_ok=True)
        self._db = sqlite3.connect(path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._db:
            self._db.executescript('''
                CREATE TABLE IF NOT EXISTS site (site_ref TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS maps (map_ref TEXT PRIMARY KEY, created_at REAL NOT NULL,
                    merged_into TEXT);
                CREATE TABLE IF NOT EXISTS revisions (sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    revision TEXT NOT NULL UNIQUE, map_ref TEXT NOT NULL, parent TEXT, created_at REAL NOT NULL,
                    cause TEXT NOT NULL, session TEXT NOT NULL, observer_ref TEXT NOT NULL, anchors INTEGER NOT NULL);
                CREATE TABLE IF NOT EXISTS poses (anchor_ref TEXT NOT NULL, sequence INTEGER NOT NULL,
                    map_ref TEXT NOT NULL, pose TEXT, PRIMARY KEY (anchor_ref, map_ref, sequence));
                CREATE TABLE IF NOT EXISTS relinks (anchor_ref TEXT NOT NULL, map_ref TEXT NOT NULL,
                    sequence INTEGER NOT NULL, via_anchor TEXT NOT NULL, pose TEXT NOT NULL,
                    PRIMARY KEY (anchor_ref, map_ref, sequence));
                CREATE TABLE IF NOT EXISTS sessions (session TEXT PRIMARY KEY, observer_ref TEXT NOT NULL,
                    started_at REAL NOT NULL, map_ref TEXT, relocalized_at REAL);
                CREATE TABLE IF NOT EXISTS alignments (from_map TEXT NOT NULL, from_revision TEXT NOT NULL,
                    to_map TEXT NOT NULL, to_revision TEXT NOT NULL, pose TEXT NOT NULL, method TEXT NOT NULL,
                    created_at REAL NOT NULL, PRIMARY KEY (from_map, to_map, to_revision));
                PRAGMA user_version=1;''')
            stored = self._db.execute('SELECT site_ref FROM site').fetchone()
            if stored is None:
                self._db.execute('INSERT INTO site VALUES (?)', (site_ref,))
            elif stored[0] != site_ref:
                # One store is one site's cartography; reusing it elsewhere would merge two places.
                raise ValueError(f'spatial store belongs to site {stored[0]!r}, not {site_ref!r}')
        self._current = {}
        for row in self._db.execute('''SELECT p.anchor_ref, p.map_ref, p.pose FROM poses p
                JOIN (SELECT anchor_ref, MAX(sequence) AS sequence FROM poses GROUP BY anchor_ref) latest
                ON latest.anchor_ref = p.anchor_ref AND latest.sequence = p.sequence'''):
            if row['pose'] is not None:
                self._current[row['anchor_ref']] = (row['map_ref'], Transform.from_dict(json.loads(row['pose'])))
        self.session = None
        self._tables = {}

    def close(self):
        self._db.close()

    # ── sessions ─────────────────────────────────────────────────────────────

    def start_session(self, now=None):
        """A new localization process: same maps, new session; history is not reusable until relocalized."""
        self.session = uuid.uuid4().hex
        with self._db:
            self._db.execute('INSERT INTO sessions VALUES (?,?,?,NULL,NULL)',
                             (self.session, self.observer_ref, time.time() if now is None else now))
        return self.session

    def session_state(self, session=None):
        row = self._db.execute('SELECT * FROM sessions WHERE session=?', (session or self.session,)).fetchone()
        return None if row is None else {'session': row['session'], 'observer_ref': row['observer_ref'],
                                         'started_at_unix_sec': row['started_at'], 'map_ref': row['map_ref'],
                                         'relocalized_at_unix_sec': row['relocalized_at']}

    # ── revisions ────────────────────────────────────────────────────────────

    def _revision(self, row):
        return None if row is None else Revision(row['sequence'], row['revision'], row['map_ref'], self.site_ref,
                                                 row['parent'], row['created_at'], row['cause'], row['session'],
                                                 row['observer_ref'], row['anchors'])

    def revision(self, revision):
        return self._revision(self._db.execute('SELECT * FROM revisions WHERE revision=?', (revision,)).fetchone())

    def latest(self, map_ref=None):
        if map_ref is None:
            row = self._db.execute('SELECT * FROM revisions ORDER BY sequence DESC LIMIT 1').fetchone()
        else:
            row = self._db.execute('SELECT * FROM revisions WHERE map_ref=? ORDER BY sequence DESC LIMIT 1',
                                   (map_ref,)).fetchone()
        return self._revision(row)

    def surviving_map(self, map_ref):
        """Follow merges: a map absorbed by a loop closure lives on in the map that absorbed it."""
        for _ in range(64):
            row = self._db.execute('SELECT merged_into FROM maps WHERE map_ref=?', (map_ref,)).fetchone()
            if row is None or row[0] is None:
                return map_ref
            map_ref = row[0]
        raise SpatialError('map_unaligned', 'map merge chain is too long')

    def ingest(self, poses, *, created_at, cause='optimization'):
        """Record one graph solution; return the new revision, or the current one when nothing moved.

        The graph decides the map: sharing anchors with one known map continues
        it, sharing none starts a new map (a disconnected mapping session), and
        sharing several merges them into the oldest with a recorded alignment.
        """
        if self.session is None:
            raise ValueError('start a localization session before ingesting a graph')
        if cause not in CAUSES:
            raise ValueError(f'unknown revision cause {cause!r}')
        if not poses or any(not valid_ref(ref) or not isinstance(pose, Transform) for ref, pose in poses.items()):
            raise ValueError('a graph requires anchor references and transforms')
        shared = {self.surviving_map(self._current[ref][0]) for ref in poses if ref in self._current}
        with self._db:
            if not shared:
                map_ref = uuid.uuid4().hex
                self._db.execute('INSERT INTO maps VALUES (?,?,NULL)', (map_ref, created_at))
                cause = 'initial'
                absorbed = []
            else:
                order = {row[0]: row[1] for row in self._db.execute(
                    'SELECT map_ref, created_at FROM maps WHERE map_ref IN (%s)' % ','.join('?' * len(shared)),
                    sorted(shared))}
                map_ref = min(shared, key=lambda ref: (order[ref], ref))
                absorbed = sorted(shared - {map_ref})
                if absorbed:
                    cause = 'merge'
            session = self.session_state()
            if session['map_ref'] is None:
                self._db.execute('UPDATE sessions SET map_ref=?, relocalized_at=? WHERE session=?',
                                 (map_ref, created_at if shared else None, self.session))
                if shared and cause == 'optimization':
                    cause = 'relocalization'
            previous = self.latest(map_ref)
            members = {ref for ref, (owner, _) in self._current.items() if self.surviving_map(owner) == map_ref}
            changed = {ref: pose for ref, pose in poses.items()
                       if ref not in self._current or self._current[ref][0] != map_ref
                       or self._current[ref][1].differs(pose)}
            removed = members - set(poses)
            if previous is not None and not changed and not removed and not absorbed:
                return previous
            table = {ref: (pose if ref in changed else self._current[ref][1]) for ref, pose in poses.items()}
            digest = table_digest(map_ref, previous.revision if previous else None, table)
            cursor = self._db.execute('INSERT INTO revisions (revision,map_ref,parent,created_at,cause,session,'
                                      'observer_ref,anchors) VALUES (?,?,?,?,?,?,?,?)',
                                      (digest, map_ref, previous.revision if previous else None, created_at, cause,
                                       self.session, self.observer_ref, len(table)))
            sequence = cursor.lastrowid
            for ref in absorbed:
                old = self.latest(ref)
                common = sorted(r for r in poses if r in self._current and self.surviving_map(self._current[r][0]) == ref)
                # The absorbed map's frame is now one rigid transform away from the survivor's.
                alignment = table[common[-1]].compose(self._current[common[-1]][1].inverse())
                self._db.execute('UPDATE maps SET merged_into=? WHERE map_ref=?', (map_ref, ref))
                self._db.execute('INSERT OR REPLACE INTO alignments VALUES (?,?,?,?,?,?,?)',
                                 (ref, old.revision, map_ref, digest, json.dumps(alignment.to_dict()),
                                  'shared_anchor_merge', created_at))
            for ref, pose in changed.items():
                self._db.execute('INSERT INTO poses VALUES (?,?,?,?)', (ref, sequence, map_ref, json.dumps(pose.to_dict())))
            # Relate a compacted anchor to a survivor through poses of one solution, the previous one.
            survivors = sorted(ref for ref in table if ref in members and ref not in changed) or \
                sorted(ref for ref in table if ref in members) or sorted(table)
            positions = np.array([self._current[ref][1].t if ref in members else table[ref].t for ref in survivors])
            for ref in sorted(removed):
                before = self._current[ref][1]
                nearest = survivors[int(np.argmin(np.linalg.norm(positions - np.asarray(before.t), axis=1)))]
                via = self._current[nearest][1] if nearest in members else table[nearest]
                # The compacted anchor keeps its last relation to a survivor (spec 12.4).
                relation = via.inverse().compose(before)
                self._db.execute('INSERT INTO poses VALUES (?,?,?,NULL)', (ref, sequence, map_ref))
                self._db.execute('INSERT OR REPLACE INTO relinks VALUES (?,?,?,?,?)',
                                 (ref, map_ref, sequence, nearest, json.dumps(relation.to_dict())))
            for ref in removed:
                self._current.pop(ref, None)
            for ref, pose in changed.items():
                self._current[ref] = (map_ref, pose)
            self._prune(created_at)
        return self.revision(digest)

    def _prune(self, now):
        """Bound history: drop old revisions, keeping each map's newest and the rows it still needs."""
        cutoff = now - self.retention_sec
        keep = {row[0] for row in self._db.execute('SELECT MAX(sequence) FROM revisions GROUP BY map_ref')}
        stale = [row[0] for row in self._db.execute(
            'SELECT sequence FROM revisions WHERE created_at < ? OR sequence <= '
            '(SELECT COALESCE(MAX(sequence), 0) - ? FROM revisions)', (cutoff, self.max_revisions))
            if row[0] not in keep]
        if not stale:
            return
        self._db.execute('DELETE FROM revisions WHERE sequence IN (%s)' % ','.join('?' * len(stale)), stale)
        oldest = self._db.execute('SELECT MIN(sequence) FROM revisions').fetchone()[0]
        # A pose row superseded at or before the oldest retained revision is unreachable.
        self._db.execute('''DELETE FROM poses WHERE EXISTS (SELECT 1 FROM poses newer
            WHERE newer.anchor_ref = poses.anchor_ref AND newer.map_ref = poses.map_ref
            AND newer.sequence > poses.sequence AND newer.sequence <= ?)''', (oldest,))
        self._db.execute('DELETE FROM alignments WHERE to_revision NOT IN (SELECT revision FROM revisions)')

    # ── anchor tables ────────────────────────────────────────────────────────

    def anchor_pose(self, revision, anchor_ref):
        """``T_map_anchor`` in ``revision``, following relinks of compacted anchors."""
        relinks = []
        for _ in range(MAX_RELINKS + 1):
            row = self._db.execute('SELECT pose FROM poses WHERE anchor_ref=? AND map_ref=? AND sequence<=? '
                                   'ORDER BY sequence DESC LIMIT 1',
                                   (anchor_ref, revision.map_ref, revision.sequence)).fetchone()
            if row is None:
                raise SpatialError('anchor_missing', f'anchor {anchor_ref} is not in revision {revision.revision}')
            if row[0] is not None:
                pose = Transform.from_dict(json.loads(row[0]))
                for relation in reversed(relinks):
                    pose = pose.compose(relation)
                return pose, len(relinks)
            link = self._db.execute('SELECT via_anchor, pose FROM relinks WHERE anchor_ref=? AND map_ref=? '
                                    'AND sequence<=? ORDER BY sequence DESC LIMIT 1',
                                    (anchor_ref, revision.map_ref, revision.sequence)).fetchone()
            if link is None:
                raise SpatialError('anchor_missing', f'anchor {anchor_ref} left the graph without a relink')
            relinks.append(Transform.from_dict(json.loads(link[1])))
            anchor_ref = link[0]
        raise SpatialError('anchor_missing', 'anchor relink chain is too long')

    def anchors(self, revision, *, after='', limit=5000):
        """One page of a revision's anchor table, ordered by reference."""
        rows = self._db.execute('''SELECT p.anchor_ref, p.pose FROM poses p
            JOIN (SELECT anchor_ref, MAX(sequence) AS sequence FROM poses
                  WHERE map_ref=? AND sequence<=? AND anchor_ref>? GROUP BY anchor_ref) latest
            ON latest.anchor_ref = p.anchor_ref AND latest.sequence = p.sequence AND p.map_ref=?
            WHERE p.pose IS NOT NULL ORDER BY p.anchor_ref LIMIT ?''',
                                (revision.map_ref, revision.sequence, after, revision.map_ref, limit + 1)).fetchall()
        page = [{'anchor_ref': row[0], 'pose': json.loads(row[1])} for row in rows[:limit]]
        return page, len(rows) > limit

    def nearest_anchor(self, revision, point):
        table = self._tables.get(revision.sequence)
        if table is None:
            rows = self._db.execute('''SELECT p.anchor_ref, p.pose FROM poses p
                JOIN (SELECT anchor_ref, MAX(sequence) AS sequence FROM poses
                      WHERE map_ref=? AND sequence<=? GROUP BY anchor_ref) latest
                ON latest.anchor_ref = p.anchor_ref AND latest.sequence = p.sequence AND p.map_ref=?
                WHERE p.pose IS NOT NULL''', (revision.map_ref, revision.sequence, revision.map_ref)).fetchall()
            if not rows:
                raise SpatialError('anchor_missing', f'revision {revision.revision} has no anchors')
            poses = [Transform.from_dict(json.loads(row[1])) for row in rows]
            table = ([row[0] for row in rows], poses, np.array([pose.t for pose in poses]))
            # Revisions are immutable, so a small cache of their tables is always current.
            if len(self._tables) >= 4:
                self._tables.pop(next(iter(self._tables)))
            self._tables[revision.sequence] = table
        refs, poses, positions = table
        index = int(np.argmin(np.linalg.norm(positions - np.asarray(point), axis=1)))
        return refs[index], poses[index]

    def alignment(self, from_map, to_map):
        row = self._db.execute('SELECT * FROM alignments WHERE from_map=? AND to_map=? ORDER BY created_at DESC LIMIT 1',
                               (from_map, to_map)).fetchone()
        return None if row is None else (Transform.from_dict(json.loads(row['pose'])), row['from_revision'],
                                          row['to_revision'], row['method'])

    def maps(self):
        return [{'map_ref': row['map_ref'], 'created_at_unix_sec': row['created_at'], 'merged_into': row['merged_into'],
                 'latest_revision': (self.latest(row['map_ref']).revision if self.latest(row['map_ref']) else None)}
                for row in self._db.execute('SELECT * FROM maps ORDER BY created_at')]

    def sessions(self, limit=20):
        return [self.session_state(row[0]) for row in
                self._db.execute('SELECT session FROM sessions ORDER BY started_at DESC LIMIT ?', (limit,))]

    def alignments(self, limit=20):
        return [{'from': {'map_ref': row['from_map'], 'revision': row['from_revision']},
                 'to': {'map_ref': row['to_map'], 'revision': row['to_revision']},
                 'transform': json.loads(row['pose']), 'cov_xyz': None, 'method': row['method'],
                 'aligned_at_unix_sec': row['created_at']}
                for row in self._db.execute('SELECT * FROM alignments ORDER BY created_at DESC LIMIT ?', (limit,))]

    # ── anchoring and reprojection ───────────────────────────────────────────

    def anchor(self, map_pose, points=(), uncertainties=()):
        """Anchor a capture: the nearest graph node of its capture revision carries the camera.

        ``map_pose`` is the observation's capture-time ``T_map_camera`` with its
        site, map and revision. Points are capture-frame measurements; each
        returns its anchor-frame position and covariance, the Knowledge
        ``estimate`` fields.
        """
        revision, camera = self._capture(map_pose)
        anchor_ref, anchor = self.nearest_anchor(revision, camera.t)
        anchor_camera = anchor.inverse().compose(camera)
        estimates = []
        for point, uncertainty in zip(points, uncertainties):
            variance = float(uncertainty) ** 2
            estimates.append({'anchor_xyz': list(anchor_camera.apply(_finite(point, 3, 'point'))),
                              'cov_xyz': anchor_camera.rotate_covariance([variance, 0, 0, 0, variance, 0, 0, 0, variance])})
        return {'site_ref': self.site_ref, 'map_ref': revision.map_ref, 'revision': revision.revision,
                'anchor_ref': anchor_ref, 'localization_session': revision.session,
                'anchor_camera': anchor_camera.to_dict(), 'estimates': estimates}

    def _capture(self, map_pose):
        if not isinstance(map_pose, dict) or map_pose.get('site_ref') != self.site_ref:
            raise SpatialError('site_mismatch', 'the capture belongs to another site; places never mix')
        revision = self.revision(map_pose.get('revision'))
        if revision is None or revision.map_ref != map_pose.get('map_ref'):
            raise SpatialError('revision_unknown', 'the capture revision is unknown or was pruned')
        return revision, Transform.from_dict(map_pose.get('pose'))

    def project(self, map_pose, points=(), target=None):
        """Camera pose and points of a capture in ``target`` (default: newest revision of its map)."""
        revision, camera = self._capture(map_pose)
        anchor_ref, anchor = self.nearest_anchor(revision, camera.t)
        anchor_camera = anchor.inverse().compose(camera)
        surviving = self.surviving_map(revision.map_ref)
        if target is None:
            target_revision = self.latest(surviving)
        else:
            target_revision = self.revision(target.get('revision'))
            if target_revision is None or target_revision.map_ref != target.get('map_ref'):
                raise SpatialError('revision_unknown', 'the target revision is unknown or was pruned')
        alignment = None
        if target_revision.map_ref != surviving:
            raise SpatialError('map_unaligned', 'the capture map and the target map have no alignment')
        if revision.map_ref != surviving:
            # Absorbed by a merge: anchors of the old map are rows of the surviving map from the merge on.
            alignment = self.alignment(revision.map_ref, surviving)
        try:
            target_anchor, relinks = self.anchor_pose(target_revision, anchor_ref)
            method = 'anchor' if relinks == 0 else 'relinked_anchor'
        except SpatialError:
            if alignment is None:
                raise
            # The anchor never joined the survivor's graph; the merge transform still places it.
            target_anchor, method = alignment[0].compose(anchor), 'merge_alignment'
        moved = target_anchor.compose(anchor_camera)
        capture_points = [camera.apply(_finite(p, 3, 'point')) for p in points]
        return {'site_ref': self.site_ref, 'map_ref': target_revision.map_ref, 'revision': target_revision.revision,
                'capture_revision': revision.revision, 'anchor_ref': anchor_ref, 'method': method,
                'camera_pose': moved.to_dict(),
                'points': [list(moved.apply(p)) for p in points],
                'shift_m': [round(math.dist(a, b), 6) for a, b in zip(capture_points, (moved.apply(p) for p in points))]
                or [round(math.dist(camera.t, moved.t), 6)]}
