"""ROS-free object map: one anchored, updated entry per thing the robot keeps seeing.

This is the robot's local landmark layer, not identity: an entry says "a thing
with this label was estimated here, by this method, last seen then". Whether
two entries are one object, or which object it is, belongs to Knowledge.

An entry keeps its position relative to a graph anchor of the spatial store, so
it follows every map revision and outlives the revision it was made in. Same
label and place is the only association rule: one candidate is updated, none is
a new entry, several are left untouched and recorded as ambiguous. Entries are
never deleted for not being seen; views proven empty step their state instead.
"""

from dataclasses import dataclass
import json
import math
import uuid

import numpy as np

from lekiwi_rmf.coverage_search import normalize_label, pose_from_transform
from lekiwi_rmf.object_estimates import METHODS, Wedge, estimates, triangulated
from lekiwi_rmf.spatial_anchors import SpatialError, Transform

SCHEMA = 'lekiwi.object_map/1'
# `moved` asserts that two entries are one object, an identity decision of Knowledge; nothing here assigns it.
STATES = ('present', 'unconfirmed', 'moved', 'absent_at_last_place')
QUERY_FIELDS = {'object_id', 'label', 'region', 'near_object_id', 'radius_m', 'since', 'state', 'limit'}


@dataclass(frozen=True)
class Limits:
    """Association, estimation, forgetting and retention limits (config/spatial_memory.yaml)."""
    gate_sigmas: float = 3.0
    min_sigma_m: float = 0.05
    max_range_m: float = 6.0
    max_tilt_rad: float = 0.6
    min_baseline_m: float = 0.3
    min_parallax_rad: float = 0.1
    max_objects: int = 2000
    max_history: int = 20000
    max_support: int = 8
    max_wedges: int = 8
    history_interval_sec: float = 60.0
    absent_retention_sec: float = 30 * 86400.0
    misses_unconfirmed: int = 2
    misses_absent: int = 4
    miss_interval_sec: float = 5.0
    absence_cell_m: float = 0.1
    absence_max_radius_m: float = 0.5
    absence_max_checks: int = 1

    def __post_init__(self):
        numbers = [getattr(self, name) for name in self.__dataclass_fields__]
        if (any(isinstance(value, bool) or not math.isfinite(value) or value <= 0 for value in numbers)
                or any(type(getattr(self, name)) is not int for name in (
                    'max_objects', 'max_history', 'max_support', 'max_wedges', 'misses_unconfirmed', 'misses_absent',
                    'absence_max_checks'))
                or self.misses_absent < self.misses_unconfirmed):
            raise ValueError('object map limits must be positive, with misses_absent >= misses_unconfirmed')


@dataclass(frozen=True)
class Absence:
    """What makes an empty view evidence of absence.

    ``profile`` is the search's coverage_search.Profile; ``models`` maps each
    admitted model_sha256 to ``(profile revision, normalized labels it can report)``.
    """
    profile: object
    models: dict


def query_labels(record):
    """The normalized labels an open-vocabulary frame was asked for; None for a closed catalogue frame."""
    if record.get('vocabulary') != 'open':
        return None
    query = record.get('query') if isinstance(record.get('query'), dict) else {}
    names = list(query.get('labels') or []) + [item.get('name') for item in query.get('concepts') or []
                                               if isinstance(item, dict)]
    return {normalize_label(name) for name in names if isinstance(name, str)}


def parse_query(text):
    """Validate an object query; every field is optional."""
    if len(text.encode()) > 4096:
        raise ValueError('object query exceeds 4096 bytes')
    try:
        value = json.loads(text)
    except RecursionError as error:
        raise ValueError('object query is nested too deeply') from error
    if not isinstance(value, dict) or set(value) - QUERY_FIELDS:
        raise ValueError(f'object query fields are {sorted(QUERY_FIELDS)}')

    def number(item, low, high):
        return type(item) in (int, float) and math.isfinite(item) and low <= item <= high

    result = {'limit': value.get('limit', 20)}
    if type(result['limit']) is not int or not 1 <= result['limit'] <= 100:
        raise ValueError('limit must be an integer in 1..100')
    if 'object_id' in value:
        identifier = value['object_id']
        if not isinstance(identifier, str) or len(identifier) != 32 or set(identifier) - set('0123456789abcdef'):
            raise ValueError('object_id is 32 lowercase hexadecimal characters')
        result['object_id'] = identifier
    if 'label' in value:
        if not isinstance(value['label'], str) or not 1 <= len(value['label']) <= 128:
            raise ValueError('label must be 1..128 characters')
        result['label'] = normalize_label(value['label'])
    if 'since' in value:
        if not number(value['since'], 0, 1e12):
            raise ValueError('since must be a Unix time in seconds')
        result['since'] = float(value['since'])
    if 'state' in value:
        states = value['state'] if isinstance(value['state'], list) else [value['state']]
        if not states or any(state not in STATES for state in states):
            raise ValueError(f'state must be one or a list of {list(STATES)}')
        result['states'] = set(states)
    region, near = value.get('region'), value.get('near_object_id')
    if region is not None and near is not None:
        raise ValueError('region and near_object_id are alternatives')
    if region is not None:
        if (not isinstance(region, dict) or set(region) != {'x', 'y', 'radius_m'} or 'radius_m' in value
                or not number(region['x'], -1e6, 1e6) or not number(region['y'], -1e6, 1e6)
                or not number(region['radius_m'], 1e-3, 100)):
            raise ValueError('region is {x, y, radius_m} with a radius in (0, 100] metres')
        result['region'] = (float(region['x']), float(region['y']), float(region['radius_m']))
    elif near is not None:
        if (not isinstance(near, str) or len(near) != 32 or any(c not in '0123456789abcdef' for c in near)
                or not number(value.get('radius_m'), 1e-3, 100)):
            raise ValueError('near_object_id needs an object id and radius_m in (0, 100] metres')
        result['near'] = (near, float(value['radius_m']))
    elif 'radius_m' in value:
        raise ValueError('radius_m belongs to near_object_id')
    return result


def _points(wedge):
    """A wedge as three points, so any rigid transform can carry it."""
    return [(wedge.x, wedge.y, 0.0), (wedge.x + math.cos(wedge.low), wedge.y + math.sin(wedge.low), 0.0),
            (wedge.x + math.cos(wedge.high), wedge.y + math.sin(wedge.high), 0.0)]


def _wedge(points):
    origin, low, high = points
    start = math.atan2(low[1] - origin[1], low[0] - origin[0])
    width = math.remainder(math.atan2(high[1] - origin[1], high[0] - origin[0]) - start, 2 * math.pi)
    return Wedge(origin[0], origin[1], start, start + abs(width))


def _same_line(first, second, baseline):
    """Two bearings that cannot be triangulated yet agree: overlapping, from viewpoints on one line of sight."""
    across = abs(-math.sin(first.bearing) * (second.x - first.x) + math.cos(first.bearing) * (second.y - first.y))
    apart = abs(math.remainder(first.bearing - second.bearing, 2 * math.pi))
    return across < baseline and apart <= first.half_width + second.half_width


class ObjectMap:
    """Object entries, their supporting observations and history, in the spatial store's database."""

    def __init__(self, store, limits=Limits()):
        self._store, self._db, self.limits = store, store.connection, limits
        self._anchors = (None, {})
        # When each entry's place was last put to the visibility proof, so a place that cannot be proven
        # empty is not tried again by every capture.
        self._proved = {}

    # ── placement ────────────────────────────────────────────────────────────

    def _anchor(self, map_ref, anchor_ref):
        """``SpatialStore.current_anchor``, remembered until the next revision of any map."""
        head = self._db.execute('SELECT MAX(sequence) FROM revisions').fetchone()[0]
        if self._anchors[0] != head:
            self._anchors = (head, {})
        known = self._anchors[1]
        if (map_ref, anchor_ref) not in known:
            try:
                known[map_ref, anchor_ref] = self._store.current_anchor(map_ref, anchor_ref)
            except SpatialError as error:
                known[map_ref, anchor_ref] = error
        if isinstance(known[map_ref, anchor_ref], SpatialError):
            raise known[map_ref, anchor_ref]
        return known[map_ref, anchor_ref]

    def _place(self, row):
        """An entry in the newest revision of the map it lives on in."""
        target, anchor, how = self._anchor(row['map_ref'], row['anchor_ref'])
        entry = dict(row)
        entry.update(target=target, projection=how, wedges=[], stored_wedges=json.loads(row['wedges']),
                     point=None if row['anchor_xyz'] is None else anchor.apply(json.loads(row['anchor_xyz'])))
        for item in entry['stored_wedges']:
            carrier = self._anchor(row['map_ref'], item['anchor_ref'])[1]
            entry['wedges'].append(_wedge([carrier.apply(point) for point in item['points']]))
        return entry

    def _entries(self, map_ref, where='1', parameters=()):
        """Placeable entries of one surviving map."""
        entries = []
        for row in self._db.execute(f'SELECT * FROM objects WHERE {where} ORDER BY last_seen DESC', parameters):
            try:
                if self._store.surviving_map(row['map_ref']) == map_ref:
                    entries.append(self._place(row))
            except SpatialError:
                continue  # It keeps its record; a query reports why it cannot be placed.
        return entries

    # ── ingestion ────────────────────────────────────────────────────────────

    def ingest(self, record, camera, octomap=None, *, epoch, evicted=0, absence=None):
        """Place one stored observation's detections, then count it against entries it proves absent.

        ``camera`` is the coverage_search.Camera of the record's camera and
        ``octomap`` the 3D map of the capture's own revision, or None; ``epoch``
        and ``evicted`` are the observation store's, as its query reports them.
        Returns counters; a capture that cannot be placed is ``skipped`` with
        its reason. The cursor moves in the same transaction as the entries, so
        a record read twice changes nothing the second time.
        """
        cursor = self.cursor()
        if cursor is not None and cursor[0] == epoch and record['sequence'] <= cursor[1]:
            return {'skipped': 'already_ingested'}
        stamp = record['stamp']['sec'] + record['stamp']['nanosec'] / 1e9
        map_pose = record.get('map_pose')
        if not map_pose:
            return {'skipped': record.get('map_pose_unavailable_reason') or 'no_map_pose'}
        if camera is None:
            return {'skipped': 'no_intrinsics'}
        try:
            anchored = self._store.anchor(map_pose)
            seen_from = self._store.anchor_pose(self._store.revision(anchored['revision']), anchored['anchor_ref'])[0]
            target, anchor, _ = self._anchor(anchored['map_ref'], anchored['anchor_ref'])
            camera_in_map = Transform.from_dict(map_pose['pose'])
            pose = pose_from_transform(camera_in_map.t, camera_in_map.q)
        except SpatialError as error:
            return {'skipped': error.code}
        except (ValueError, KeyError, TypeError):
            return {'skipped': 'invalid_map_pose'}
        # Estimates are made in the capture revision, where its octomap lives, and compared in the newest.
        capture = {'id': record['id'], 'stamp': stamp, 'epoch': epoch, 'sequence': record['sequence'],
                   'map_ref': anchored['map_ref'], 'anchor_ref': anchored['anchor_ref'], 'anchor': anchor,
                   'to_anchor': seen_from.inverse(), 'to_now': anchor.compose(seen_from.inverse())}
        counts = dict.fromkeys(('created', 'updated', 'ambiguous', 'unplaced', 'capacity', 'missed'), 0)
        sightings = []
        for detection in record.get('detections', []):
            found = estimates(detection, record['frame_id'], camera, record['height'], pose, camera_in_map, octomap,
                              max_range_m=self.limits.max_range_m, max_tilt_rad=self.limits.max_tilt_rad)
            placed = next((item for item in found if item['map_xyz'] is not None), None)
            wedge = next((item['wedge'] for item in found if item['wedge'] is not None), None)
            sightings.append({
                'label': normalize_label(detection['label']), 'score': float(detection['score']),
                'estimate': placed or (found[-1] if found else None), 'crossings': {},
                'point': None if placed is None else capture['to_now'].apply(placed['map_xyz']),
                'wedge': None if wedge is None else _wedge([capture['to_now'].apply(p) for p in _points(wedge)]),
                'stored_wedge': None if wedge is None else {
                    'anchor_ref': capture['anchor_ref'],
                    'points': [list(capture['to_anchor'].apply(p)) for p in _points(wedge)]}})
        labels = sorted({sighting['label'] for sighting in sightings})
        entries = self._entries(target.map_ref, 'label IN (%s)' % ','.join('?' * len(labels)), labels) if labels else []
        claims = {}
        for sighting in sightings:
            if sighting['estimate'] is None:
                counts['unplaced'] += 1
                continue
            # Candidates come from the entries that existed before this capture: its own detections never merge.
            sighting['candidates'] = [entry for entry in entries if entry['label'] == sighting['label']
                                      and self._matches(sighting, entry)]
            for entry in sighting['candidates']:
                claims[entry['object_id']] = claims.get(entry['object_id'], 0) + 1
        with self._db:
            touched = set()
            for sighting in sightings:
                candidates = sighting.get('candidates')
                if candidates is None:
                    continue
                if not candidates:
                    counts['created' if self._create(capture, sighting) else 'capacity'] += 1
                elif len(candidates) == 1 and claims[candidates[0]['object_id']] == 1:
                    self._update(capture, sighting, candidates[0])
                    touched.add(candidates[0]['object_id'])
                    counts['updated'] += 1
                else:
                    # Several entries, or several detections for one entry: choosing would be a guess.
                    detail = {'candidates': [entry['object_id'] for entry in candidates]}
                    for entry in candidates:
                        self._note(entry['object_id'], entry['label'], stamp, 'ambiguous', capture['id'], detail)
                    touched.update(detail['candidates'])
                    counts['ambiguous'] += 1
            if absence is not None and octomap is not None:
                counts['missed'] = self._absences(record, capture, pose, camera, octomap, absence, sightings,
                                                  target, touched)
            self._prune(stamp)
            self._set_cursor(epoch, record['sequence'], evicted)
        return counts

    def _matches(self, sighting, entry):
        gate, estimate, point, wedge = self.limits.gate_sigmas, sighting['estimate'], sighting['point'], sighting['wedge']
        if point is not None and entry['point'] is not None:
            axes = 3 if entry['height_known'] else 2
            return math.dist(point[:axes], entry['point'][:axes]) <= gate * math.hypot(entry['sigma_m'], estimate['sigma_m'])
        if point is not None:
            return all(known.contains(point, gate * estimate['sigma_m']) for known in entry['wedges'])
        if entry['point'] is not None:
            return (math.dist(entry['point'][:2], (wedge.x, wedge.y)) <= self.limits.max_range_m
                    and wedge.contains(entry['point'], gate * entry['sigma_m']))
        if any(_same_line(known, wedge, self.limits.min_baseline_m) for known in entry['wedges']):
            return True
        crossing = self._crossing(entry['wedges'] + [wedge])
        if crossing is not None:
            sighting['crossings'][entry['object_id']] = crossing
        return crossing is not None

    def _crossing(self, wedges):
        return triangulated(wedges, min_baseline_m=self.limits.min_baseline_m,
                            min_parallax_rad=self.limits.min_parallax_rad, max_range_m=self.limits.max_range_m)

    def _position(self, capture, point, height_known):
        """Columns of a position in the newest revision, kept relative to the capture's anchor."""
        return {'map_ref': capture['map_ref'], 'anchor_ref': capture['anchor_ref'], 'height_known': int(height_known),
                'anchor_xyz': json.dumps(list(capture['anchor'].inverse().apply(point)))}

    def _support(self, object_id, capture):
        self._db.execute('INSERT OR REPLACE INTO object_support VALUES (?,?,?,?,?)',
                         (object_id, capture['id'], capture['epoch'], capture['sequence'], capture['stamp']))
        self._db.execute('''DELETE FROM object_support WHERE object_id=? AND observation_id NOT IN (
            SELECT observation_id FROM object_support WHERE object_id=? ORDER BY stamp DESC, sequence DESC LIMIT ?)''',
                         (object_id, object_id, self.limits.max_support))

    def _note(self, object_id, label, stamp, event, observation_id, detail):
        """Bounded history: a repeated event is one row per ``history_interval_sec``."""
        last = self._db.execute('SELECT event, stamp FROM object_history WHERE object_id=? ORDER BY id DESC LIMIT 1',
                                (object_id,)).fetchone()
        if last is not None and last['event'] == event and stamp - last['stamp'] < self.limits.history_interval_sec:
            return
        self._db.execute('INSERT INTO object_history (object_id,label,stamp,event,observation_id,detail) '
                         'VALUES (?,?,?,?,?,?)', (object_id, label, stamp, event, observation_id, json.dumps(detail)))

    def _create(self, capture, sighting):
        if self._db.execute('SELECT COUNT(*) FROM objects').fetchone()[0] >= self.limits.max_objects:
            # Room is made only by an entry already proven absent, never by one that was merely not seen.
            oldest = self._db.execute("SELECT object_id FROM objects WHERE state='absent_at_last_place' "
                                      'ORDER BY last_verified LIMIT 1').fetchone()
            if oldest is None:
                return False
            self._forget(oldest[0])
        estimate, point, object_id = sighting['estimate'], sighting['point'], uuid.uuid4().hex
        row = {'object_id': object_id, 'label': sighting['label'], 'map_ref': capture['map_ref'],
               'anchor_ref': capture['anchor_ref'], 'anchor_xyz': None, 'height_known': 0,
               'sigma_m': None if point is None else max(self.limits.min_sigma_m, estimate['sigma_m']),
               'wedges': json.dumps([] if point is not None else [sighting['stored_wedge']]),
               'method': estimate['method'], 'qualified': int(estimate['qualified']),
               'calibration_sha256': estimate['calibration_sha256'], 'assumptions': json.dumps(estimate['assumptions']),
               'state': 'present', 'score': sighting['score'], 'first_seen': capture['stamp'],
               'last_seen': capture['stamp'], 'last_verified': capture['stamp'], 'miss_count': 0, 'last_miss': None,
               'sightings': 1, 'first_xyz': None if point is None else json.dumps(list(point))}
        if point is not None:
            row.update(self._position(capture, point, True))
        self._db.execute('INSERT INTO objects (%s) VALUES (%s)' % (','.join(row), ','.join('?' * len(row))),
                         list(row.values()))
        self._support(object_id, capture)
        self._note(object_id, sighting['label'], capture['stamp'], 'created', capture['id'],
                   {'method': estimate['method']})
        return True

    def _update(self, capture, sighting, entry):
        estimate, point, limits = sighting['estimate'], sighting['point'], self.limits
        changes, event = {}, 'confirmed'

        def adopt(source, position, height_known):
            changes.update(self._position(capture, position, height_known), method=source['method'],
                           sigma_m=max(limits.min_sigma_m, source['sigma_m']), qualified=int(source['qualified']),
                           calibration_sha256=source['calibration_sha256'], assumptions=json.dumps(source['assumptions']))
            if entry['first_xyz'] is None:
                changes['first_xyz'] = json.dumps(list(position))

        if point is not None and (entry['point'] is None or estimate['qualified'] and not entry['qualified']):
            adopt(estimate, point, True)
            event = 'placed'
        elif point is not None and entry['qualified'] and not estimate['qualified']:
            pass  # An inference never dilutes a measured position; it only confirms presence.
        elif point is not None:
            # Inverse-variance fusion; the floor keeps repeated views of one scene from claiming false precision.
            old, new = 1 / entry['sigma_m'] ** 2, 1 / estimate['sigma_m'] ** 2
            fused = [(old * a + new * b) / (old + new) for a, b in zip(entry['point'], point)]
            if not entry['height_known']:
                fused[2] = point[2]
            changes.update(self._position(capture, fused, True),
                           sigma_m=max(limits.min_sigma_m, math.sqrt(1 / (old + new))))
            if METHODS.index(estimate['method']) < METHODS.index(entry['method']):
                changes.update(method=estimate['method'], assumptions=json.dumps(estimate['assumptions']),
                               calibration_sha256=estimate['calibration_sha256'])
            event = 'updated'
        elif entry['method'] in ('bearing_only', 'triangulated'):
            stored = (entry['stored_wedges'] + [sighting['stored_wedge']])[-limits.max_wedges:]
            changes['wedges'] = json.dumps(stored)
            crossing = sighting['crossings'].get(entry['object_id'])
            if crossing is None and entry['method'] == 'triangulated':
                crossing = self._crossing((entry['wedges'] + [sighting['wedge']])[-limits.max_wedges:])
            if crossing is not None:
                # Height is unknown: the point sits at the anchor's height and is reported without z.
                adopt(crossing, [*crossing['map_xyz'][:2], capture['anchor'].t[2]], False)
                event = 'placed' if entry['method'] == 'bearing_only' else 'updated'
        if entry['state'] != 'present':
            event = 'reappeared'
        changes.update(state='present', score=sighting['score'], miss_count=0, last_miss=None,
                       last_seen=max(entry['last_seen'], capture['stamp']), sightings=entry['sightings'] + 1,
                       last_verified=max(entry['last_verified'], capture['stamp']))
        self._db.execute('UPDATE objects SET %s WHERE object_id=?' % ','.join(f'{name}=?' for name in changes),
                         [*changes.values(), entry['object_id']])
        self._support(entry['object_id'], capture)
        self._note(entry['object_id'], entry['label'], capture['stamp'], event, capture['id'],
                   {'method': changes.get('method', entry['method']), 'was': entry['state']})

    # ── forgetting by evidence ───────────────────────────────────────────────

    def _absences(self, record, capture, pose, camera, octomap, absence, sightings, target, touched):
        """Count this capture against entries whose whole uncertainty square it saw empty.

        The gates are QueryCoverage.add's: an admitted model under its admitted
        profile, asked for the label, from a stationary camera. The proof is
        the search's: every column of the square, over the profile's height
        band, known free from the camera in the capture-time octomap.
        """
        admitted = absence.models.get(record.get('model_sha256'))
        profile, limits, stamp = absence.profile, self.limits, capture['stamp']
        if (admitted is None or (record.get('profile') or {}).get('revision') != admitted[0]
                or (profile.require_stationary and record.get('stationary') is not True)):
            return 0
        asked, reach = query_labels(record), profile.range_limit(camera)
        back = capture['to_now'].inverse()
        entries = self._entries(
            target.map_ref, "state IN ('present','unconfirmed') AND height_known=1 AND anchor_xyz IS NOT NULL "
            'AND last_seen<? AND (last_miss IS NULL OR last_miss<=?)', (stamp, stamp - limits.miss_interval_sec))
        missed = proofs = 0
        for entry in sorted(entries, key=lambda item: math.dist(item['point'][:2], capture['to_now'].apply(
                (pose.x, pose.y, pose.z))[:2])):
            # Back in the capture revision, where the octomap and the camera pose are.
            centre = back.apply(entry['point'])
            radius = limits.gate_sigmas * entry['sigma_m']
            if (entry['object_id'] in touched or entry['label'] not in admitted[1]
                    or (asked is not None and entry['label'] not in asked) or radius > limits.absence_max_radius_m
                    or not profile.min_height_m <= centre[2] <= profile.max_height_m
                    or math.dist(centre[:2], (pose.x, pose.y)) > reach):
                continue
            if any(sighting['label'] == entry['label'] and (
                    sighting['estimate'] is None or (sighting['point'] is None and sighting['wedge'].contains(
                        entry['point'], radius))) for sighting in sightings):
                continue  # A same-label detection that may be this entry is not a view of an empty place.
            if stamp - self._proved.get(entry['object_id'], -math.inf) < limits.miss_interval_sec:
                continue
            if proofs == limits.absence_max_checks:
                break  # The proof is the costly step; the places left are tried by later captures.
            proofs += 1
            if len(self._proved) >= 2 * limits.max_objects:
                self._proved.clear()
            self._proved[entry['object_id']] = stamp
            side = max(1, math.ceil(2 * radius / limits.absence_cell_m))
            origin = (centre[0] - side * limits.absence_cell_m / 2, centre[1] - side * limits.absence_cell_m / 2, 0.0)
            # Row by row, so an occluded place is given up at its first hidden column.
            if not all(len(octomap.visible_columns(np.arange(row * side, (row + 1) * side), (side, side),
                                                   limits.absence_cell_m, origin, pose, camera, profile)) == side
                       for row in range(side)):
                continue
            count = entry['miss_count'] + 1
            state = ('absent_at_last_place' if count >= limits.misses_absent else
                     'unconfirmed' if count >= limits.misses_unconfirmed else entry['state'])
            self._db.execute('UPDATE objects SET miss_count=?, last_miss=?, last_verified=?, state=? WHERE object_id=?',
                             (count, stamp, stamp, state, entry['object_id']))
            if state != entry['state']:
                self._note(entry['object_id'], entry['label'], stamp, state, capture['id'], {'miss_count': count})
            missed += 1
        return missed

    # ── retention ────────────────────────────────────────────────────────────

    def _forget(self, object_id):
        self._db.execute('DELETE FROM objects WHERE object_id=?', (object_id,))
        self._db.execute('DELETE FROM object_support WHERE object_id=?', (object_id,))

    def _prune(self, now):
        for row in self._db.execute("SELECT object_id FROM objects WHERE state='absent_at_last_place' "
                                    'AND last_verified<?', (now - self.limits.absent_retention_sec,)).fetchall():
            self._forget(row[0])
        self._db.execute('DELETE FROM object_history WHERE id<=(SELECT MAX(id) FROM object_history)-?',
                         (self.limits.max_history,))

    # ── observation cursor ───────────────────────────────────────────────────

    def cursor(self):
        """``(epoch, sequence, evicted)`` of the observation store as last ingested, or None."""
        row = self._db.execute('SELECT epoch, sequence, evicted FROM object_cursor').fetchone()
        return None if row is None else tuple(row)

    def _set_cursor(self, epoch, sequence, evicted):
        self._db.execute('DELETE FROM object_cursor')
        self._db.execute('INSERT INTO object_cursor VALUES (?,?,?)', (epoch, sequence, evicted))

    def set_cursor(self, epoch, sequence, evicted):
        """Record how far the observation store has been read, past records that placed nothing."""
        with self._db:
            self._set_cursor(epoch, sequence, evicted)

    # ── queries ──────────────────────────────────────────────────────────────

    def history(self, object_id):
        return [{'stamp_unix_sec': row['stamp'], 'event': row['event'], 'observation_id': row['observation_id'],
                 **json.loads(row['detail'])}
                for row in self._db.execute('SELECT * FROM object_history WHERE object_id=? ORDER BY id', (object_id,))]

    def query(self, request):
        """Objects matching a ``parse_query`` request, each placed in the newest revision of its map."""
        session = self._store.session_state() if self._store.session else None
        current = None
        if session is not None and session['map_ref'] is not None:
            current = self._store.latest(self._store.surviving_map(session['map_ref']))
        centre = None
        if 'region' in request:
            if current is None:
                raise ValueError('localization unavailable: a region needs the map the robot is localized in')
            centre = (current.map_ref, request['region'][:2], request['region'][2])
        elif 'near' in request:
            row = self._db.execute('SELECT * FROM objects WHERE object_id=?', (request['near'][0],)).fetchone()
            try:
                origin = None if row is None else self._place(row)
            except SpatialError as error:
                raise ValueError(f'near_object_id cannot be placed: {error.code}') from error
            if origin is None or origin['point'] is None:
                raise ValueError('near_object_id is unknown or has no position estimate')
            centre = (origin['target'].map_ref, origin['point'][:2], request['near'][1])
        cursor = self.cursor()
        where, parameters = ['1'], []
        if 'object_id' in request:
            where.append('object_id=?')
            parameters.append(request['object_id'])
        if 'label' in request:
            where.append('label=?')
            parameters.append(request['label'])
        if 'since' in request:
            where.append('last_seen>=?')
            parameters.append(request['since'])
        if 'states' in request:
            where.append('state IN (%s)' % ','.join('?' * len(request['states'])))
            parameters.extend(sorted(request['states']))
        objects = []
        for row in self._db.execute(f'SELECT * FROM objects WHERE {" AND ".join(where)} ORDER BY last_seen DESC',
                                    parameters).fetchall():
            try:
                entry, reason = self._place(row), None
            except SpatialError as error:
                entry, reason = {**dict(row), 'target': None, 'point': None, 'wedges': [], 'projection': None}, error.code
            if centre is not None and (
                    entry['point'] is None or entry['target'].map_ref != centre[0]
                    or math.dist(entry['point'][:2], centre[1]) > centre[2]
                    or ('near' in request and entry['object_id'] == request['near'][0])):
                continue
            if len(objects) == request['limit']:
                return self._answer(current, objects, True)
            objects.append(self._document(entry, reason, cursor))
        return self._answer(current, objects, False)

    def _answer(self, current, objects, truncated):
        return {'schema': SCHEMA, 'site_ref': self._store.site_ref, 'localized': current is not None,
                'map': None if current is None else {'map_ref': current.map_ref, 'revision': current.revision},
                # A map entry is what was last observed; nothing here re-verifies it at query time.
                'verification': 'last_observed', 'objects': objects, 'truncated': truncated}

    def _document(self, entry, reason, cursor):
        point, first = entry['point'], entry['first_xyz'] and json.loads(entry['first_xyz'])
        axes = 3 if entry['height_known'] else 2
        support = self._db.execute('SELECT * FROM object_support WHERE object_id=? ORDER BY stamp DESC, sequence DESC',
                                   (entry['object_id'],)).fetchall()
        document = {
            'object_id': entry['object_id'], 'label': entry['label'], 'state': entry['state'],
            'site_ref': self._store.site_ref,
            'map_ref': entry['target'].map_ref if entry['target'] else entry['map_ref'],
            'revision': entry['target'].revision if entry['target'] else None,
            'position': None if point is None else {
                'x': round(point[0], 4), 'y': round(point[1], 4),
                'z': round(point[2], 4) if entry['height_known'] else None},
            'bearings': [wedge.to_dict() for wedge in entry['wedges']] if point is None else [],
            'method': entry['method'], 'sigma_m': entry['sigma_m'], 'qualified': bool(entry['qualified']),
            'assumptions': json.loads(entry['assumptions']), 'calibration_sha256': entry['calibration_sha256'],
            'score': {'kind': 'detector_score', 'value': entry['score']},
            'first_seen_unix_sec': entry['first_seen'], 'last_seen_unix_sec': entry['last_seen'],
            'last_verified_unix_sec': entry['last_verified'], 'miss_count': entry['miss_count'],
            'sightings': entry['sightings'],
            'shift_m': None if point is None or not first else round(math.dist(point[:axes], first[:axes]), 4),
            'anchor_ref': entry['anchor_ref'], 'projection': entry['projection'],
            # Expired: the observation store has evicted up to this record (a pinned record may still exist).
            'observations': [{'id': row['observation_id'], 'stamp_unix_sec': row['stamp'],
                              'evidence_expired': cursor is None or row['epoch'] != cursor[0] or row['sequence'] <= cursor[2]}
                             for row in support]}
        if reason is not None:
            document['position_unavailable_reason'] = reason
        return document
