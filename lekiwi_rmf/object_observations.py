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
    if not isinstance(values, dict) or set(values) - {'after_sequence', 'observation_ids', 'limit', 'epoch'}:
        raise ValueError('unknown observation query fields')
    limit = values.get('limit', 20)
    if type(limit) is not int or not 1 <= limit <= 100:
        raise ValueError('limit must be an integer in 1..100')
    ids = values.get('observation_ids')
    if ids is not None:
        if ('after_sequence' in values or not isinstance(ids, list) or not 1 <= len(ids) <= 100
                or any(not valid_id(identifier) for identifier in ids)):
            raise ValueError('observation_ids requires 1..100 SHA-256 IDs without a cursor')
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


def observation(image, detections, model_sha256, camera_id):
    """Use an exact synchronized ROS image/detection pair; preserve all candidates."""
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
        # An interior sample reduces background; this attribute is not calibrated probability.
        left, top = int(x + w * .2), int(y + h * .2)
        right, bottom = max(left + 1, int(x + w * .8)), max(top + 1, int(y + h * .8))
        hsv = cv2.cvtColor(pixels[top:bottom, left:right], cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, (90, 60, 40), (130, 255, 255))
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
    capture = json.dumps([camera_id, document['frame_id'], document['stamp']], sort_keys=True).encode()
    document['id'] = hashlib.sha256(capture).hexdigest()
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
                self._db.execute('CREATE TABLE IF NOT EXISTS metadata (epoch TEXT NOT NULL)')
                if self._db.execute('SELECT epoch FROM metadata').fetchone() is None:
                    self._db.execute('INSERT INTO metadata VALUES (?)', (uuid.uuid4().hex,))
                self._db.execute('PRAGMA user_version=1')
            self.epoch = self._db.execute('SELECT epoch FROM metadata').fetchone()[0]
            page_size = self._db.execute('PRAGMA page_size').fetchone()[0]
            self._db.execute(f'PRAGMA max_page_count={max_bytes // page_size}')
            self.max_bytes, self.max_records, self.retention_sec = max_bytes, max_records, retention_sec
        except BaseException:
            if hasattr(self, '_db'):
                self._db.close()
            self._lock.close()
            raise

    def _prune(self, now):
        self._db.execute('DELETE FROM observations WHERE stored_at<?', (now - self.retention_sec,))

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
                self._db.execute('DELETE FROM observations WHERE sequence=(SELECT MIN(sequence) FROM observations)')
            cursor = self._db.execute('INSERT INTO observations (id,stored_at,document,evidence,bytes) VALUES (?,?,?,?,?)',
                                      (document['id'], now, text, evidence, size))
            return cursor.lastrowid

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
        last = self._db.execute("SELECT seq FROM sqlite_sequence WHERE name='observations'").fetchone()
        records, size = [], 0
        for row in rows:
            if size + len(row['document'].encode()) > 60 * 1024:
                break
            records.append({**json.loads(row['document']), 'sequence': row['sequence'], 'stored_at_unix_sec': row['stored_at']})
            size += len(row['document'].encode())
        return {'schema': 1, 'epoch': self.epoch, 'observations': records,
                'next_sequence': records[-1]['sequence'] if records else cursor,
                'latest_sequence': last[0] if last is not None else 0,
                'cursor_gap': bool(ids is None and cursor and
                    ((first is not None and cursor < first - 1) or (first is None and last is not None and cursor < last[0])))}

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
