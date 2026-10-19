"""Bounded image observations; no goal state, object identity or metric claims."""

import fcntl
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time
import uuid

import cv2
import numpy as np


def query_parameters(text):
    if len(text.encode()) > 4096:
        raise ValueError('query exceeds 4096 bytes')
    try:
        values = json.loads(text)
    except RecursionError as error:
        raise ValueError('query is nested too deeply') from error
    if not isinstance(values, dict) or set(values) - {'after_sequence', 'observation_ids', 'limit', 'epoch', 'pin_request_id'}:
        raise ValueError('unknown observation query fields')
    limit = values.get('limit', 20)
    if type(limit) is not int or not 1 <= limit <= 100:
        raise ValueError('limit must be an integer in 1..100')
    ids = values.get('observation_ids')
    if ids is not None:
        if ('after_sequence' in values or not isinstance(ids, list) or not 1 <= len(ids) <= 100
                or any(not valid_id(identifier) for identifier in ids)):
            raise ValueError('observation_ids requires 1..100 SHA-256 IDs without a cursor')
    request = values.get('pin_request_id')
    if request is not None and (ids is None or not isinstance(request, str) or not 1 <= len(request) <= 128
            or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in request)):
        raise ValueError('pin receipt requires IDs and a valid request identity')
    cursor = values.get('after_sequence', 0)
    if type(cursor) is not int or not 0 <= cursor < 2**63:
        raise ValueError('after_sequence must be a nonnegative SQLite integer')
    epoch = values.get('epoch')
    if epoch is not None and (not isinstance(epoch, str) or len(epoch) != 32 or any(c not in '0123456789abcdef' for c in epoch)):
        raise ValueError('invalid observation store epoch')
    if cursor and epoch is None:
        raise ValueError('a nonzero cursor requires its store epoch')
    return cursor, ids, limit, epoch


def valid_id(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def blue_mask(pixels):
    return cv2.inRange(cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV), (90, 60, 40), (130, 255, 255))


def observation(image, detections, model_sha256, camera_id, query=None):
    """Use an exact synchronized ROS image/detection pair; preserve all candidates.

    `query` is the open-vocabulary Query the frame ran under, or None for a closed native catalogue.
    """
    if (image.header != detections.header or not image.header.frame_id
            or image.header.stamp.sec < 0 or not 0 <= image.header.stamp.nanosec < 10**9
            or image.header.stamp.sec == image.header.stamp.nanosec == 0):
        raise ValueError('image and detections require the same nonzero capture header')
    if not valid_id(model_sha256) or not camera_id or len(camera_id) > 128:
        raise ValueError('camera identity and model SHA-256 are required')
    if image.encoding not in ('bgr8', 'rgb8') or image.is_bigendian:
        raise ValueError('expected little-endian bgr8/rgb8 image')
    if not 1 <= image.width <= 1920 or not 1 <= image.height <= 1080:
        raise ValueError('image dimensions exceed 1920x1080')
    if not image.width * 3 <= image.step <= image.width * 3 + 4096 or len(image.data) != image.step * image.height:
        raise ValueError('image stride or payload does not match its dimensions')
    if len(detections.detections) > 20:
        raise ValueError('more than 20 detections in one capture')
    pixels = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.step)
    pixels = pixels[:, :image.width * 3].reshape(image.height, image.width, 3)
    if image.encoding == 'rgb8':
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    results = []
    for detection in detections.detections:
        if detection.header != image.header or len(detection.results) != 1:
            raise ValueError('each detection requires the source header and one class hypothesis')
        hypothesis = detection.results[0].hypothesis
        label, score = hypothesis.class_id, hypothesis.score
        if not label or len(label) > 128 or not math.isfinite(score) or not 0 <= score <= 1:
            raise ValueError('invalid class label or confidence')
        box = detection.bbox
        cx, cy = box.center.position.x, box.center.position.y
        values = (cx, cy, box.size_x, box.size_y, box.center.theta)
        if not all(math.isfinite(value) for value in values) or box.center.theta != 0:
            raise ValueError('expected a finite axis-aligned detection')
        x, y, w, h = cx - box.size_x / 2, cy - box.size_y / 2, box.size_x, box.size_y
        if w <= 0 or h <= 0 or x < -1e-6 or y < -1e-6 or x + w > image.width + 1e-6 or y + h > image.height + 1e-6:
            raise ValueError('detection is outside the source image')
        # DEBT: blue_fraction is a color attribute Fiber's cup/blue FindRequest still requires; drop it once that
        # consumer matches attributes by query. An interior sample reduces background; it is not a probability.
        left, top = int(x + w * .2), int(y + h * .2)
        right, bottom = max(left + 1, int(x + w * .8)), max(top + 1, int(y + h * .8))
        blue = blue_mask(pixels[top:bottom, left:right])
        results.append({'label': label, 'score': score, 'bbox': [x, y, w, h],
                        'blue_fraction': float(np.count_nonzero(blue) / blue.size)})
    ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError('observation JPEG encoding failed')
    evidence = encoded.tobytes()
    if len(evidence) > 384 * 1024:
        raise ValueError('encoded evidence exceeds 384 KiB')
    document = {'schema': 1, 'camera_id': camera_id, 'frame_id': image.header.frame_id,
                'stamp': {'sec': image.header.stamp.sec, 'nanosec': image.header.stamp.nanosec},
                'width': image.width, 'height': image.height, 'model_sha256': model_sha256,
                'grounding': 'image_only', 'position': None, 'detections': results,
                'evidence': {'sha256': hashlib.sha256(evidence).hexdigest(), 'size': len(evidence), 'format': 'image/jpeg'}}
    # Closed records answer only their native catalogue, never an open query they cannot express.
    document['vocabulary'] = 'closed' if query is None else 'open'
    if query is not None:
        document['query'] = query.document()
    capture = json.dumps([camera_id, document['frame_id'], document['stamp']], sort_keys=True).encode()
    document['capture_id'] = hashlib.sha256(capture).hexdigest()
    # One capture may be recognized under several operating points; each inference is its own observation.
    # The query is part of an open recognizer's operating point, so the same capture under another query
    # is another inference with another ID.
    inference = model_sha256 if query is None else f'{model_sha256}:{query.revision}'
    document['id'] = hashlib.sha256(f"{document['capture_id']}:{inference}".encode()).hexdigest()
    return document, evidence


class ObservationStore:
    """One SQLite transaction owns metadata and JPEG, including retention."""

    def __init__(self, path, *, max_bytes=2 * 1024**3, max_records=100000, retention_sec=7 * 86400):
        if type(max_records) is not int or max_records < 1 or type(max_bytes) is not int or max_bytes < 65536:
            raise ValueError('invalid observation storage limits')
        if not math.isfinite(retention_sec) or retention_sec <= 0:
            raise ValueError('retention must be positive and finite')
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = Path(f'{path}.lock').open('a')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            path.touch(mode=0o600, exist_ok=True)
            path.chmod(0o600)
            self._db = sqlite3.connect(path, check_same_thread=False)
            self._db.row_factory = sqlite3.Row
            version = self._db.execute('PRAGMA user_version').fetchone()[0]
            if version not in (0, 1):
                raise ValueError('unsupported observation database version')
            with self._db:
                self._db.execute('PRAGMA auto_vacuum=FULL')
                self._db.execute('''CREATE TABLE IF NOT EXISTS observations (
                    sequence INTEGER PRIMARY KEY AUTOINCREMENT, id TEXT NOT NULL UNIQUE,
                    stored_at REAL NOT NULL, document TEXT NOT NULL, evidence BLOB NOT NULL,
                    bytes INTEGER NOT NULL)''')
                self._db.execute('CREATE INDEX IF NOT EXISTS observations_time ON observations(stored_at)')
                self._db.execute('CREATE INDEX IF NOT EXISTS observations_size ON observations(bytes)')
                self._db.execute('''CREATE TABLE IF NOT EXISTS pins (
                    request_id TEXT NOT NULL, observation_id TEXT NOT NULL, expires_at REAL NOT NULL,
                    PRIMARY KEY(request_id,observation_id))''')
                self._db.execute('CREATE TABLE IF NOT EXISTS metadata (epoch TEXT NOT NULL)')
                if self._db.execute('SELECT epoch FROM metadata').fetchone() is None:
                    self._db.execute('INSERT INTO metadata VALUES (?)', (uuid.uuid4().hex,))
                # Highest sequence ever deleted; pinned survivors make MIN(sequence) unable to reveal gaps.
                self._db.execute('CREATE TABLE IF NOT EXISTS evicted (sequence INTEGER NOT NULL)')
                if self._db.execute('SELECT sequence FROM evicted').fetchone() is None:
                    self._db.execute('INSERT INTO evicted VALUES (0)')
                self._db.execute('PRAGMA user_version=1')
            self.epoch = self._db.execute('SELECT epoch FROM metadata').fetchone()[0]
            page_size = self._db.execute('PRAGMA page_size').fetchone()[0]
            # Eviction counts payload bytes; page slack, indexes and pins need room above that quota.
            self._db.execute(f'PRAGMA max_page_count={max_bytes * 5 // 4 // page_size}')
            self.max_bytes, self.max_records, self.retention_sec = max_bytes, max_records, retention_sec
        except BaseException:
            if hasattr(self, '_db'):
                self._db.close()
            self._lock.close()
            raise

    def _delete(self, where, parameters):
        self._db.execute(f'UPDATE evicted SET sequence=MAX(sequence, COALESCE((SELECT MAX(sequence) FROM observations WHERE {where}), 0))',
                         parameters)
        self._db.execute(f'DELETE FROM observations WHERE {where}', parameters)

    def _evict_oldest(self):
        oldest = self._db.execute('SELECT MIN(sequence) FROM observations WHERE id NOT IN (SELECT observation_id FROM pins)').fetchone()[0]
        if oldest is None:
            raise ValueError('storage quota reserved by pinned evidence')
        self._delete('sequence=?', (oldest,))

    def _prune(self, now):
        self._db.execute('DELETE FROM pins WHERE expires_at<?', (now,))
        self._delete('stored_at<? AND id NOT IN (SELECT observation_id FROM pins)', (now - self.retention_sec,))

    def pin(self, text, now=None):
        if len(text.encode()) > 4096:
            raise ValueError('pin request exceeds 4096 bytes')
        try:
            values = json.loads(text)
        except RecursionError as error:
            raise ValueError('pin request is nested too deeply') from error
        if not isinstance(values, dict) or set(values) != {'request_id', 'observation_ids'}:
            raise ValueError('pin requires request_id and observation_ids')
        request, ids = values['request_id'], values['observation_ids']
        if (not isinstance(request, str) or not 1 <= len(request) <= 128
                or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in request)
                or not isinstance(ids, list) or not 1 <= len(ids) <= 20 or any(not valid_id(i) for i in ids)
                or len(set(ids)) != len(ids)):
            raise ValueError('invalid evidence pin identity')
        now = time.time() if now is None else now
        with self._db:
            self._prune(now)
            previous = self._db.execute('SELECT observation_id FROM pins WHERE request_id=?', (request,)).fetchall()
            if previous:
                if {row[0] for row in previous} != set(ids):
                    raise ValueError('pin request ID already has different evidence')
                return
            if self._db.execute('SELECT COUNT(DISTINCT request_id) FROM pins').fetchone()[0] >= 1000:
                raise ValueError('evidence pin request quota reached')
            pinned = self._db.execute(
                'SELECT COALESCE(SUM(bytes),0) FROM observations WHERE id IN (SELECT observation_id FROM pins) OR id IN (%s)'
                % ','.join('?' for _ in ids), ids).fetchone()[0]
            if pinned > self.max_bytes // 2:
                raise ValueError('pinned evidence would exceed half of the storage quota')
            for identifier in ids:
                if self._db.execute('SELECT 1 FROM observations WHERE id=?', (identifier,)).fetchone() is None:
                    raise ValueError('cannot pin missing or expired evidence')
                self._db.execute('INSERT INTO pins VALUES (?,?,?)', (request, identifier, now + self.retention_sec))

    def put(self, document, evidence, now=None):
        now = time.time() if now is None else now
        text = json.dumps(document, allow_nan=False, sort_keys=True)
        size = len(text.encode()) + len(evidence)
        if size > self.max_bytes or size > 512 * 1024:
            raise ValueError('observation exceeds storage quota')
        with self._db:
            self._prune(now)
            row = self._db.execute('SELECT * FROM observations WHERE id=?', (document['id'],)).fetchone()
            if row is not None:
                if row['document'] != text or bytes(row['evidence']) != evidence:
                    raise ValueError('capture ID was reused with different content')
                return row['sequence']
            while True:
                count, used = self._db.execute('SELECT COUNT(*),COALESCE(SUM(bytes),0) FROM observations INDEXED BY observations_size').fetchone()
                if count < self.max_records and used + size <= self.max_bytes:
                    break
                self._evict_oldest()
            while True:
                try:
                    cursor = self._db.execute('INSERT INTO observations (id,stored_at,document,evidence,bytes) VALUES (?,?,?,?,?)',
                                              (document['id'], now, text, evidence, size))
                    return cursor.lastrowid
                except sqlite3.OperationalError as error:
                    # The file cap can still be reached through page overhead; free space instead of failing forever.
                    if 'full' not in str(error):
                        raise
                    self._evict_oldest()

    def query(self, text, now=None):
        cursor, ids, limit, epoch = query_parameters(text)
        if epoch is not None and epoch != self.epoch:
            raise ValueError('observation store epoch changed; restart from cursor zero')
        with self._db:
            self._prune(time.time() if now is None else now)
        if ids is None:
            rows = self._db.execute('SELECT sequence,stored_at,document FROM observations WHERE sequence>? ORDER BY sequence LIMIT ?',
                                    (cursor, limit)).fetchall()
        else:
            placeholders = ','.join('?' for _ in ids)
            rows = self._db.execute(f'SELECT sequence,stored_at,document FROM observations WHERE id IN ({placeholders}) ORDER BY sequence LIMIT ?',
                                    (*ids, limit)).fetchall()
        first = self._db.execute('SELECT MIN(sequence) FROM observations').fetchone()[0]
        evicted = self._db.execute('SELECT sequence FROM evicted').fetchone()[0]
        last = self._db.execute("SELECT seq FROM sqlite_sequence WHERE name='observations'").fetchone()
        records, size = [], 0
        for row in rows:
            if size + len(row['document'].encode()) > 60 * 1024:
                break
            records.append({**json.loads(row['document']), 'sequence': row['sequence'], 'stored_at_unix_sec': row['stored_at']})
            size += len(row['document'].encode())
        request = json.loads(text).get('pin_request_id')
        pins = {} if request is None else {row[0]: row[1] for row in self._db.execute(
            'SELECT observation_id,expires_at FROM pins WHERE request_id=?', (request,))}
        return {'schema': 1, 'epoch': self.epoch, 'observations': records, 'pins': pins,
                'next_sequence': records[-1]['sequence'] if records else cursor,
                'latest_sequence': last[0] if last is not None else 0,
                'cursor_gap': bool(ids is None and cursor and (cursor < evicted or
                    (first is not None and cursor < first - 1) or (first is None and last is not None and cursor < last[0])))}

    def evidence(self, identifier, now=None):
        if not valid_id(identifier):
            raise ValueError('evidence requires a SHA-256 observation ID')
        with self._db:
            self._prune(time.time() if now is None else now)
        row = self._db.execute('SELECT document,evidence FROM observations WHERE id=?', (identifier,)).fetchone()
        if row is None:
            raise ValueError('observation not found or evidence expired')
        evidence = bytes(row['evidence'])
        digest = hashlib.sha256(evidence).hexdigest()
        if digest != json.loads(row['document'])['evidence']['sha256']:
            raise RuntimeError('stored evidence hash mismatch')
        return evidence, digest

    def close(self):
        self._db.close()
        self._lock.close()
