"""Bounded image observations; no goal state, object identity or metric claims."""

import base64
import fcntl
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import threading
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


EVIDENCE_QUALITY = 80
MAX_EVIDENCE_BYTES = 384 * 1024


def capture_artifact(pixels):
    """The frame's stored capture: a JPEG that is both the evidence and the recognizer's input.

    libjpeg is deterministic for the same pixels and parameters, so the detector and the collector
    encode the same bytes; every recognizer consumes the decoded artifact, which makes each stored
    observation replayable byte-exact (lekiwi_rmf/perception_replay.py).
    """
    ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, EVIDENCE_QUALITY])
    if not ok:
        raise RuntimeError('capture JPEG encoding failed')
    data = encoded.tobytes()
    if len(data) > MAX_EVIDENCE_BYTES:
        raise ValueError('encoded evidence exceeds 384 KiB')
    return data


def artifact_pixels(data):
    pixels = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if pixels is None:
        raise ValueError('capture artifact cannot be decoded')
    return pixels


# DEBT: input-quality limits are engineering defaults, not measured on the robot's cameras; calibrate them
# with the hard-condition corpus (cleanroom#98) before an abstention reason gates a decision.
DARK_LUMA, BRIGHT_LUMA, BLUR_VARIANCE = 40., 225., 30.
NEXT_REQUIREMENT = {
    'no_candidate': 'another_viewpoint_or_memory_query',
    'dark': 'more_light_or_another_camera',
    'overexposed': 'less_glare_or_another_viewpoint',
    'blurred': 'hold_still_and_recapture',
    'geometry_unavailable': 'depth_observation',
    'provider_unavailable': 'retry_primary_provider',
}


def input_quality(pixels):
    gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
    luma, sharpness = float(gray.mean()), float(cv2.Laplacian(gray, cv2.CV_64F).var())
    flags = [flag for flag, bad in [('dark', luma < DARK_LUMA), ('overexposed', luma > BRIGHT_LUMA),
                                    ('blurred', sharpness < BLUR_VARIANCE)] if bad]
    return {'mean_luma': round(luma, 2), 'sharpness': round(sharpness, 2), 'flags': flags}


def assign_parts(results, query):
    """A declared part is tied to the most specific detection of its whole that contains it, never guessed."""
    wholes = dict(query.parts) if query is not None else {}
    for item in results:
        whole = wholes.get(item['label'])
        if whole is None:
            continue
        x, y, w, h = item['bbox']
        best = None
        for index, other in enumerate(results):
            if other['label'] != whole:
                continue
            ox, oy, ow, oh = other['bbox']
            inside = max(0., min(x + w, ox + ow) - max(x, ox)) * max(0., min(y + h, oy + oh) - max(y, oy)) / (w * h)
            if inside >= .8 and (best is None or ow * oh < best[2]):
                best = (index, inside, ow * oh)
        item['part_of'] = None if best is None else {'index': best[0], 'label': whole, 'containment': round(best[1], 4)}
        if best is None:
            item['part_status'] = 'no_referent'


def abstentions(record, query):
    """Typed reasons this frame cannot answer part of its query, each with the information that would.

    A closed record answers only its native catalogue and abstains on nothing; a frame the fallback served
    because the primary was unavailable says so instead of answering the query.
    """
    result = []
    degraded = record.get('degraded')
    if degraded:
        result.append({'label': None, 'reason': 'provider_unavailable', 'detail': degraded['detail'],
                       'next_requirement': NEXT_REQUIREMENT['provider_unavailable']})
    if query is not None:
        found = {item['label'] for item in record['detections']}
        flags = record['input_quality']['flags']
        for label in query.reportable:
            if label not in found:
                reason = 'low_quality_input' if flags else 'no_candidate'
                result.append({'label': label, 'reason': reason, 'detail': ','.join(flags),
                               'next_requirement': NEXT_REQUIREMENT[flags[0] if flags else 'no_candidate']})
    for index, item in enumerate(record['detections']):
        if item.get('geometry_unavailable'):
            result.append({'label': item['label'], 'detection': index, 'reason': 'geometry_unavailable',
                           'detail': item['geometry_unavailable'], 'next_requirement': NEXT_REQUIREMENT['geometry_unavailable']})
    record['abstentions'] = result
    return result


def detection_extras(extras, count, query):
    """Per-detection evidence the detector attached: suppressed alternatives and concept matches."""
    extras = extras if extras is not None else [{}] * count
    if not isinstance(extras, list) or len(extras) != count:
        raise ValueError('detection evidence does not align with the detections')
    revisions = {concept.revision: concept.name for concept in (query.concepts if query is not None else ())}
    for item in extras:
        if not isinstance(item, dict) or set(item) - {'alternatives', 'concept'}:
            raise ValueError('invalid detection evidence')
        alternatives = item.get('alternatives', [])
        if (not isinstance(alternatives, list) or len(alternatives) > 4 or any(
                not isinstance(other, dict) or set(other) != {'label', 'score'} or not isinstance(other['label'], str)
                or not 1 <= len(other['label']) <= 128 or type(other['score']) not in (int, float)
                or not 0 <= other['score'] <= 1 for other in alternatives)):
            raise ValueError('invalid suppressed alternatives')
        concept = item.get('concept')
        if concept is not None and (not isinstance(concept, dict) or revisions.get(concept.get('revision')) != concept.get('name')):
            raise ValueError('concept match outside the frame query')
    return extras


def inference_id(capture_id, model_sha256, profile, query=None):
    """One capture may be recognized under several operating points; each inference is its own observation.

    The profile and an open recognizer's query are part of the operating point, so the same capture under
    another profile or query is another inference with another ID, and a replay under the same operating
    point has the original's ID.
    """
    inference = f'{model_sha256}:{profile.revision}' + ('' if query is None else f':{query.revision}')
    return hashlib.sha256(f'{capture_id}:{inference}'.encode()).hexdigest()


def observation(image, detections, model_sha256, camera_id, query=None, *, profile, extras=None, run=None,
                degraded=None, input_sha256=None):
    """Use an exact synchronized ROS image/detection pair; preserve all candidates.

    `query` is the open-vocabulary Query the frame ran under, or None for a closed native catalogue;
    `profile` is the proposal profile that turned the model output into these detections.
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
    extras = detection_extras(extras, len(detections.detections), query)
    pixels = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.step)
    pixels = pixels[:, :image.width * 3].reshape(image.height, image.width, 3)
    if image.encoding == 'rgb8':
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    results = []
    for detection, extra in zip(detections.detections, extras):
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
                        'blue_fraction': float(np.count_nonzero(blue) / blue.size), **extra})
    assign_parts(results, query)
    evidence = capture_artifact(pixels)
    digest = hashlib.sha256(evidence).hexdigest()
    document = {'schema': 1, 'camera_id': camera_id, 'frame_id': image.header.frame_id,
                'stamp': {'sec': image.header.stamp.sec, 'nanosec': image.header.stamp.nanosec},
                'width': image.width, 'height': image.height, 'model_sha256': model_sha256,
                'grounding': 'image_only', 'position': None, 'detections': results,
                # model_input: the recognizer consumed exactly these bytes, so replaying them reproduces it.
                'evidence': {'sha256': digest, 'size': len(evidence), 'format': 'image/jpeg',
                             'model_input': input_sha256 == digest},
                'profile': profile.document(), 'input_quality': input_quality(pixels)}
    if run is not None:
        document['provider_run'] = run
    if degraded is not None:
        document['degraded'] = degraded
    # Closed records answer only their native catalogue, never an open query they cannot express.
    document['vocabulary'] = 'closed' if query is None else 'open'
    if query is not None:
        document['query'] = query.document()
    capture = json.dumps([camera_id, document['frame_id'], document['stamp']], sort_keys=True).encode()
    document['capture_id'] = hashlib.sha256(capture).hexdigest()
    document['id'] = inference_id(document['capture_id'], model_sha256, profile, query)
    abstentions(document, query)
    return document, evidence


def _serialized(method):
    # The connection is shared across threads (check_same_thread=False); sqlite3 objects are not.
    def call(self, *args, **kwargs):
        with self._access:
            return method(self, *args, **kwargs)
    return call


class ObservationStore:
    """One SQLite transaction owns metadata and JPEG, including retention."""

    def __init__(self, path, *, max_bytes=2 * 1024**3, max_records=100000, retention_sec=7 * 86400):
        self._access = threading.RLock()
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

    @_serialized
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

    @_serialized
    def prepare(self, text, now=None):
        """Reserve the newest two distinct captures and return their immutable proof in one call.

        Selection narrows transport only; the caller still verifies the goal and every detection.
        The reentrant store lock keeps selection, JPEG validation and pinning together.
        """
        if len(text.encode()) > 4096:
            raise ValueError('preparation request exceeds 4096 bytes')
        try:
            values = json.loads(text)
        except RecursionError as error:
            raise ValueError('preparation request is nested too deeply') from error
        fields = {'request_id', 'camera_id', 'recognizers', 'after', 'max_age_sec'}
        if not isinstance(values, dict) or set(values) != fields:
            raise ValueError('invalid preparation request fields')
        prefix, camera, recognizers = values['request_id'], values['camera_id'], values['recognizers']
        if (not isinstance(prefix, str) or not 1 <= len(prefix) <= 63
                or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in prefix)
                or not isinstance(camera, str) or not 1 <= len(camera) <= 128
                or not isinstance(recognizers, list) or not 1 <= len(recognizers) <= 17
                or any(not isinstance(r, list) or len(r) != 2 or not valid_id(r[0])
                       or (r[1] != '' and not valid_id(r[1])) for r in recognizers)):
            raise ValueError('invalid preparation identity')
        after, age = values['after'], values['max_age_sec']
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (after, age))
                or after < 0 or not 0 < age <= 2):
            raise ValueError('invalid preparation capture window')
        now = time.time() if now is None else now
        admitted = {tuple(r) for r in recognizers}
        selected, captures = [], set()
        for row in self._db.execute('SELECT document FROM observations ORDER BY sequence DESC LIMIT 100'):
            record = json.loads(row[0])
            stamp = record['stamp']['sec'] + record['stamp']['nanosec'] / 1e9
            identity = (record['model_sha256'], record.get('profile', {}).get('revision', ''))
            if (record['camera_id'] != camera or identity not in admitted
                    or record.get('clock_domain') != 'ros_system_time'
                    or not after <= stamp <= now or now - stamp > age
                    or record['capture_id'] in captures):
                continue
            captures.add(record['capture_id'])
            selected.append(record)
        selected.sort(key=lambda r: (r['stamp']['sec'], r['stamp']['nanosec']))
        ids = [r['id'] for r in selected[-2:]]
        if len(ids) != 2:
            return {'schema': 1, 'epoch': self.epoch, 'observations': [], 'pins': {}, 'evidence': {}}
        images = {}
        for identifier in ids:
            data, digest = self.evidence(identifier, now=now)
            images[identifier] = {'mime_type': 'image/jpeg', 'sha256': digest,
                                  'data_base64': base64.b64encode(data).decode('ascii')}
        digest = hashlib.sha256(json.dumps(sorted(ids), separators=(',', ':')).encode()).hexdigest()
        request = f'{prefix}-{digest}'
        self.pin(json.dumps({'request_id': request, 'observation_ids': ids}), now=now)
        result = self.query(json.dumps({'observation_ids': ids, 'pin_request_id': request}), now=now)
        result.update(request_id=request, evidence=images)
        return result

    @_serialized
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

    @_serialized
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

    @_serialized
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

    @_serialized
    def close(self):
        self._db.close()
        self._lock.close()
