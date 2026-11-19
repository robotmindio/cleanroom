"""The object map: anchored entries, association, forgetting by evidence and queries, without ROS."""

import json
import math
import sqlite3

import pytest

from lekiwi_rmf.coverage_search import Profile, pose_from_transform
from lekiwi_rmf.object_map import Absence, Limits, ObjectMap, parse_query
from lekiwi_rmf.spatial_anchors import SCHEMA_VERSION, SpatialStore, Transform
from test_object_estimates import CAMERA, HEIGHT, WIDTH, box, octree, voxel
from test_spatial_anchors import NODE_CAMERA, corridor, yaw

MODEL, REVISION, EPOCH = 'a' * 64, 'b' * 64, 'e' * 32
# The active search's criteria (config/exploration.yaml): 0.02..0.30 m band, stationary camera.
ABSENCE = Absence(Profile(0.08, 24.0, 4.0, 0.02, 0.30, 900.0, True), {MODEL: (REVISION, frozenset({'cup', 'bottle'}))})
# Voxel centres in view of a camera at node 3, which sits at (3.1, 0, 0.3) looking along x.
A, NEAR_A, B = (4.325, 0.025, 0.125), (4.325, 0.225, 0.125), (4.325, -0.575, 0.125)


@pytest.fixture
def store(tmp_path):
    value = SpatialStore(tmp_path / 'spatial.db', site_ref='home', observer_ref='robot-a')
    value.start_session(now=0.0)
    yield value
    value.close()


def record(store, revision, node, targets, *, sequence, stamp, offset=Transform.identity(), label='cup', **fields):
    """A stored observation of ``targets`` (map points) from a camera at ``node`` of ``revision``."""
    camera = store.anchor_pose(revision, f'rtabmap:test:{node}')[0].compose(offset).compose(NODE_CAMERA)
    pose = pose_from_transform(camera.t, camera.q)
    return {'id': f'{sequence:064x}', 'sequence': sequence, 'stamp': {'sec': stamp, 'nanosec': 0},
            'camera_id': 'front', 'frame_id': 'front_optical', 'width': WIDTH, 'height': HEIGHT,
            'model_sha256': MODEL, 'profile': {'revision': REVISION}, 'stationary': True, 'vocabulary': 'closed',
            'detections': [{'label': label, 'score': 0.9, 'bbox': box(pose, target)} for target in targets],
            'map_pose': {'site_ref': store.site_ref, 'map_ref': revision.map_ref, 'revision': revision.revision,
                         'pose': camera.to_dict()}, **fields}


def ask(objects, **request):
    return objects.query(parse_query(json.dumps(request)))


def xyz(document):
    return [document['position'][axis] for axis in 'xyz']


def test_loop_closure_keeps_one_entry_that_follows_the_map(store):
    objects = ObjectMap(store)
    first = store.ingest(corridor(range(1, 11)), created_at=1.0)
    counts = objects.ingest(record(store, first, 3, [A], sequence=1, stamp=10), CAMERA, octree([voxel(A)]), epoch=EPOCH)
    assert counts['created'] == 1
    seen = ask(objects, label='Cup')['objects'][0]
    assert math.dist(xyz(seen), A) < 0.05 and seen['revision'] == first.revision and seen['shift_m'] == 0
    assert seen['method'] == 'ray_octomap_hit' and seen['qualified'] is False and seen['assumptions']
    assert seen['state'] == 'present' and seen['anchor_ref'] == 'rtabmap:test:3' and seen['sigma_m'] > 0.05

    # The loop closure moves node 3, and everything seen from it, 0.6 m: far beyond the association gate.
    closed = store.ingest(corridor(range(1, 11), lambda i: (0.0, 0.2 * i, 0.0)), created_at=20.0, cause='loop_closure')
    corrected = (A[0], A[1] + 0.6, A[2])
    followed = ask(objects, label='cup')['objects'][0]
    assert followed['object_id'] == seen['object_id'] and followed['revision'] == closed.revision
    assert math.dist(xyz(followed), corrected) < 0.05 and followed['shift_m'] == pytest.approx(0.6, abs=1e-3)
    assert followed['projection'] == 'anchor'

    # Seen again in the corrected map, the unmoved object is the same entry, not a second one.
    counts = objects.ingest(record(store, closed, 3, [corrected], sequence=2, stamp=30), CAMERA,
                            octree([voxel(corrected)]), epoch=EPOCH)
    assert counts['updated'] == 1 and counts['created'] == 0
    result = ask(objects)
    assert len(result['objects']) == 1 and not result['truncated'] and result['verification'] == 'last_observed'
    again = result['objects'][0]
    assert again['object_id'] == seen['object_id'] and again['sightings'] == 2
    assert again['sigma_m'] < seen['sigma_m'] and again['last_seen_unix_sec'] == 30
    assert [item['id'] for item in again['observations']] == [f'{2:064x}', f'{1:064x}']
    assert [event['event'] for event in objects.history(seen['object_id'])] == ['created', 'updated']


def test_two_cups_stay_two_and_an_ambiguous_sighting_changes_neither(store):
    objects = ObjectMap(store)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    pair = octree([voxel(A), voxel(NEAR_A)])
    # 0.2 m apart is inside the association gate, yet two detections of one capture are two things.
    assert objects.ingest(record(store, revision, 3, [A, NEAR_A], sequence=1, stamp=10), CAMERA, pair,
                          epoch=EPOCH)['created'] == 2
    before = ask(objects)['objects']
    assert len(before) == 2 and math.dist(xyz(before[0]), xyz(before[1])) == pytest.approx(0.2, abs=0.05)
    # Each later detection could be either entry: nothing is updated and nothing is invented.
    counts = objects.ingest(record(store, revision, 3, [A, NEAR_A], sequence=2, stamp=20), CAMERA, pair, epoch=EPOCH)
    assert counts['ambiguous'] == 2 and counts['created'] == counts['updated'] == 0
    between = (A[0], 0.125, A[2])
    counts = objects.ingest(record(store, revision, 3, [between], sequence=3, stamp=30), CAMERA,
                            octree([voxel(between)]), epoch=EPOCH)
    assert counts['ambiguous'] == 1 and counts['created'] == counts['updated'] == 0
    after = ask(objects)['objects']
    assert [(item['object_id'], item['position'], item['sightings'], item['last_seen_unix_sec']) for item in after] \
        == [(item['object_id'], item['position'], 1, 10) for item in before]
    ids = sorted(item['object_id'] for item in after)
    for identifier in ids:
        events = objects.history(identifier)
        assert [event['event'] for event in events] == ['created', 'ambiguous']
        assert sorted(events[1]['candidates']) == ids


def test_a_moved_object_is_a_new_entry_and_the_old_place_is_downgraded_only_by_evidence(store):
    objects = ObjectMap(store)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    at_a, at_b = octree([voxel(A)]), octree([voxel(B)])

    def see(target, tree, sequence, stamp, **fields):
        return objects.ingest(record(store, revision, 3, [target], sequence=sequence, stamp=stamp, **fields), CAMERA,
                              tree, epoch=EPOCH, absence=ABSENCE)

    assert see(A, at_a, 1, 10)['created'] == 1
    old = ask(objects)['objects'][0]
    # The cup now stands 0.6 m away. The old entry is not dragged there: a second entry appears.
    counts = see(B, at_b, 2, 20)
    assert counts['created'] == 1 and counts['missed'] == 1
    new = next(item for item in ask(objects)['objects'] if item['object_id'] != old['object_id'])
    assert math.dist(xyz(new), B) < 0.05

    def old_entry():
        return next(item for item in ask(objects)['objects'] if item['object_id'] == old['object_id'])
    assert old_entry()['position'] == old['position'] and old_entry()['state'] == 'present'
    assert old_entry()['miss_count'] == 1 and old_entry()['last_verified_unix_sec'] == 20
    # Views that prove nothing: too soon after the last one, from a moving camera, from an unadmitted
    # model or profile, or of an open query that did not ask for the label.
    assert see(B, at_b, 3, 22)['missed'] == 0
    assert see(B, at_b, 4, 30, stationary=False)['missed'] == 0
    assert see(B, at_b, 5, 31, model_sha256='f' * 64)['missed'] == 0
    assert see(B, at_b, 6, 32, profile={'revision': 'f' * 64})['missed'] == 0
    assert see(B, at_b, 7, 33, vocabulary='open', query={'labels': ['bottle']}, label='bottle')['missed'] == 0
    assert old_entry()['miss_count'] == 1
    assert see(B, at_b, 8, 40)['missed'] == 1
    assert old_entry()['state'] == 'unconfirmed'
    see(B, at_b, 9, 50)
    assert old_entry()['state'] == 'unconfirmed'
    see(B, at_b, 10, 60)
    gone = old_entry()
    assert gone['state'] == 'absent_at_last_place' and gone['miss_count'] == 4
    assert gone['last_seen_unix_sec'] == 10 and gone['last_verified_unix_sec'] == 60 and gone['position'] == old['position']
    assert [item['object_id'] for item in ask(objects, state='absent_at_last_place')['objects']] == [old['object_id']]
    assert [item['object_id'] for item in ask(objects, state=['present'], label='cup')['objects']] == [new['object_id']]
    assert [event['event'] for event in objects.history(old['object_id'])] == [
        'created', 'unconfirmed', 'absent_at_last_place']
    # No longer counted once absent; and the entry is still there to be found again.
    assert see(B, at_b, 11, 70)['missed'] == 0
    counts = see(A, at_a, 12, 80)
    assert counts['updated'] == 1 and counts['created'] == 0
    back = old_entry()
    assert back['state'] == 'present' and back['miss_count'] == 0 and back['last_seen_unix_sec'] == 80
    assert objects.history(old['object_id'])[-1]['event'] == 'reappeared'


class Occluded:
    """A 3D map double: surfaces as in ``tree``, but one column of any place is never proven visible."""

    def __init__(self, tree):
        self.resolution, self.first_hit = tree.resolution, tree.first_hit

    def visible_columns(self, cells, *_):
        return cells[:-1]


def test_absence_needs_the_whole_place_proven_visible_and_inside_the_height_band(store):
    objects = ObjectMap(store)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    shelf = (4.325, 0.025, 0.525)
    objects.ingest(record(store, revision, 3, [A], sequence=1, stamp=10), CAMERA, octree([voxel(A)]), epoch=EPOCH)
    objects.ingest(record(store, revision, 3, [shelf], sequence=2, stamp=10, label='bottle'), CAMERA,
                   octree([voxel(shelf)]), epoch=EPOCH)
    empty = octree()

    def look(sequence, stamp, tree, targets=(), **fields):
        return objects.ingest(record(store, revision, 3, list(targets), sequence=sequence, stamp=stamp, **fields),
                              CAMERA, tree, epoch=EPOCH, absence=ABSENCE)
    # Without a 3D map, with part of the place occluded, or with unknown space, nothing is proven.
    assert look(3, 20, None)['missed'] == 0
    assert look(4, 30, Occluded(empty))['missed'] == 0
    assert look(5, 40, octree(known=((0, -64, 0), (84, 64, 16))))['missed'] == 0
    # A same-label detection that cannot be placed, or a bearing over the place, is not an empty view.
    assert look(6, 50, Occluded(octree(known=((0, 0, 0), (1, 1, 1)))), [A])['missed'] == 0
    # Seen empty, the cup on the floor is missed; the bottle above the 0.30 m band never can be.
    assert look(7, 60, empty)['missed'] == 1
    # The same place is not put to the proof again within the interval, whatever the last result was.
    assert look(8, 62, empty)['missed'] == 0
    states = {item['label']: (item['state'], item['miss_count']) for item in ask(objects)['objects']}
    assert states == {'cup': ('present', 1), 'bottle': ('present', 0)}
    # A place too uncertain to be proven empty is left alone.
    wide = ObjectMap(store, Limits(absence_max_radius_m=0.1))
    assert wide.ingest(record(store, revision, 3, [], sequence=9, stamp=70), CAMERA, empty, epoch=EPOCH,
                       absence=ABSENCE)['missed'] == 0


def test_bearings_from_two_places_become_a_triangulated_point_without_any_depth(store):
    objects = ObjectMap(store)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    assert objects.ingest(record(store, revision, 3, [A], sequence=1, stamp=10), CAMERA, None, epoch=EPOCH)['created'] == 1
    bearing = ask(objects)['objects'][0]
    assert bearing['method'] == 'bearing_only' and bearing['position'] is None and bearing['sigma_m'] is None
    assert len(bearing['bearings']) == 1 and bearing['bearings'][0]['origin'] == pytest.approx([3.1, 0.0])
    assert bearing['qualified'] is False and bearing['shift_m'] is None
    # A region names a place; an entry without a range is in none.
    assert ask(objects, region={'x': A[0], 'y': A[1], 'radius_m': 5.0})['objects'] == []
    # The same view again confirms the entry and adds nothing a crossing could use.
    assert objects.ingest(record(store, revision, 3, [A], sequence=2, stamp=11), CAMERA, None, epoch=EPOCH)['updated'] == 1
    assert ask(objects)['objects'][0]['position'] is None
    # One metre to the side, the two bearings cross.
    side = yaw(-0.5, (0.0, 1.0, 0.0))
    counts = objects.ingest(record(store, revision, 3, [A], sequence=3, stamp=20, offset=side), CAMERA, None, epoch=EPOCH)
    assert counts['updated'] == 1 and counts['created'] == 0
    crossed = ask(objects)['objects'][0]
    assert crossed['object_id'] == bearing['object_id'] and crossed['method'] == 'triangulated'
    assert crossed['qualified'] is False and 'height unknown' in crossed['assumptions']
    assert crossed['position']['z'] is None and crossed['bearings'] == []
    assert math.dist((crossed['position']['x'], crossed['position']['y']), A[:2]) < crossed['sigma_m'] < 0.3
    assert [event['event'] for event in objects.history(bearing['object_id'])] == ['created', 'confirmed', 'placed']
    # A bearing that passes over the point confirms it; a surface hit then gives it a height.
    assert objects.ingest(record(store, revision, 3, [A], sequence=4, stamp=30), CAMERA, None, epoch=EPOCH)['updated'] == 1
    objects.ingest(record(store, revision, 3, [A], sequence=5, stamp=40), CAMERA, octree([voxel(A)]), epoch=EPOCH)
    fused = ask(objects)['objects']
    assert len(fused) == 1 and fused[0]['method'] == 'ray_octomap_hit' and fused[0]['position']['z'] is not None
    assert math.dist(xyz(fused[0]), A) < 0.1 and fused[0]['sightings'] == 5


def test_a_measured_position_replaces_an_inference_and_is_not_diluted_by_one(store):
    objects = ObjectMap(store)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    tree = octree([voxel(A)])
    objects.ingest(record(store, revision, 3, [A], sequence=1, stamp=10), CAMERA, tree, epoch=EPOCH)
    measured = record(store, revision, 3, [A], sequence=2, stamp=20)
    surface = (A[0] - 0.03, A[1], A[2])
    in_camera = Transform.from_dict(measured['map_pose']['pose']).inverse().apply(surface)
    measured['detections'][0]['position'] = {
        'mode': 'camera', 'xyz': list(in_camera), 'frame_id': 'front_optical', 'uncertainty_m': 0.03,
        'calibration_sha256': 'c' * 64, 'map': {'uncertainty_m': 0.12}}
    assert objects.ingest(measured, CAMERA, tree, epoch=EPOCH)['updated'] == 1
    entry = ask(objects)['objects'][0]
    assert entry['method'] == 'rgbd_surface' and entry['qualified'] is True and entry['sigma_m'] == 0.12
    assert entry['calibration_sha256'] == 'c' * 64 and math.dist(xyz(entry), surface) < 1e-3
    assert objects.ingest(record(store, revision, 3, [A], sequence=3, stamp=30), CAMERA, tree, epoch=EPOCH)['updated'] == 1
    later = ask(objects)['objects'][0]
    assert later['position'] == entry['position'] and later['qualified'] is True and later['sightings'] == 3
    assert objects.history(entry['object_id'])[-1]['event'] == 'confirmed'


def test_entries_survive_a_restart_and_a_record_read_twice_counts_once(tmp_path):
    path = tmp_path / 'spatial.db'
    store = SpatialStore(path, site_ref='home', observer_ref='robot-a')
    store.start_session(now=0.0)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    objects, tree = ObjectMap(store), octree([voxel(A)])
    first = record(store, revision, 3, [A], sequence=1, stamp=10)
    assert objects.cursor() is None
    assert objects.ingest(first, CAMERA, tree, epoch=EPOCH, evicted=0)['created'] == 1
    assert objects.ingest(first, CAMERA, tree, epoch=EPOCH) == {'skipped': 'already_ingested'}
    assert objects.cursor() == (EPOCH, 1, 0)
    before = ask(objects)['objects']
    store.close()

    store = SpatialStore(path, site_ref='home', observer_ref='robot-a')
    store.start_session(now=100.0)
    objects = ObjectMap(store)
    assert objects.cursor() == (EPOCH, 1, 0)
    assert objects.ingest(first, CAMERA, tree, epoch=EPOCH) == {'skipped': 'already_ingested'}
    result = ask(objects)
    # The new session has not localized: entries are still answered, in their own map, without a current one.
    assert result['objects'] == before and result['localized'] is False and result['map'] is None
    with pytest.raises(ValueError, match='localization unavailable'):
        ask(objects, region={'x': 0, 'y': 0, 'radius_m': 1})
    # A capture that cannot be placed says why and leaves the map alone.
    for change, reason in (({'map_pose': None, 'map_pose_unavailable_reason': 'not_localized'}, 'not_localized'),
                           ({'map_pose': None}, 'no_map_pose'),
                           ({'map_pose': {**first['map_pose'], 'revision': 'pruned'}}, 'revision_unknown'),
                           ({'map_pose': {**first['map_pose'], 'site_ref': 'shop'}}, 'site_mismatch'),
                           ({'map_pose': {**first['map_pose'], 'pose': {}}}, 'invalid_map_pose')):
        assert objects.ingest({**first, 'sequence': 2, **change}, CAMERA, tree, epoch=EPOCH) == {'skipped': reason}
    assert objects.ingest({**first, 'sequence': 2}, None, tree, epoch=EPOCH) == {'skipped': 'no_intrinsics'}
    # A recreated observation store restarts its sequences under a new epoch; its records are new.
    assert objects.ingest({**first, 'id': 'd' * 64}, CAMERA, tree, epoch='f' * 32)['updated'] == 1
    assert ask(objects)['objects'][0]['sightings'] == 2
    store.close()


def test_entries_outlive_the_revision_they_were_made_in(tmp_path):
    store = SpatialStore(tmp_path / 'spatial.db', site_ref='home', observer_ref='robot-a',
                         retention_sec=10.0, max_revisions=3)
    store.start_session(now=0.0)
    old = store.ingest(corridor(range(1, 11)), created_at=1.0)
    objects = ObjectMap(store)
    late = record(store, old, 3, [A], sequence=2, stamp=3)
    objects.ingest(record(store, old, 3, [A], sequence=1, stamp=2), CAMERA, octree([voxel(A)]), epoch=EPOCH)
    seen = ask(objects)['objects'][0]
    for step in range(1, 6):
        latest = store.ingest(corridor(range(1, 11), lambda i, s=step: (0.0, 0.01 * s * i, 0.0)), created_at=20.0 + step)
    assert store.revision(old.revision) is None
    kept = ask(objects)['objects'][0]
    assert kept['revision'] == latest.revision and kept['position']['y'] == pytest.approx(seen['position']['y'] + 0.15, abs=1e-3)
    assert kept['shift_m'] == pytest.approx(0.15, abs=1e-3)
    # A capture of the pruned revision can no longer be placed, and is not guessed.
    assert objects.ingest(late, CAMERA, None, epoch=EPOCH) == {'skipped': 'revision_unknown'}
    store.close()


def test_queries_filter_by_label_place_time_and_state_and_report_expired_evidence(store):
    objects = ObjectMap(store)
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    objects.ingest(record(store, revision, 3, [A], sequence=4, stamp=10), CAMERA, octree([voxel(A)]), epoch=EPOCH)
    objects.ingest(record(store, revision, 3, [B], sequence=5, stamp=20, label='Water  Bottle'), CAMERA,
                   octree([voxel(B)]), epoch=EPOCH)
    objects.ingest(record(store, revision, 6, [(7.325, 0.025, 0.125)], sequence=6, stamp=30), CAMERA,
                   octree([voxel((7.325, 0.025, 0.125))]), epoch=EPOCH)
    everything = ask(objects)
    assert [item['label'] for item in everything['objects']] == ['cup', 'water bottle', 'cup']
    assert everything['site_ref'] == 'home' and everything['localized']
    assert everything['map'] == {'map_ref': revision.map_ref, 'revision': revision.revision}
    cup = everything['objects'][2]
    assert [item['last_seen_unix_sec'] for item in ask(objects, label='cup')['objects']] == [30, 10]
    assert [item['label'] for item in ask(objects, label='WATER bottle')['objects']] == ['water bottle']
    assert [item['object_id'] for item in ask(objects, object_id=cup['object_id'])['objects']] == [cup['object_id']]
    assert ask(objects, object_id='0' * 32)['objects'] == []
    assert [item['label'] for item in ask(objects, since=15)['objects']] == ['cup', 'water bottle']
    assert ask(objects, state='unconfirmed')['objects'] == []
    limited = ask(objects, limit=2)
    assert len(limited['objects']) == 2 and limited['truncated']
    here = ask(objects, region={'x': A[0], 'y': A[1], 'radius_m': 0.3})['objects']
    assert [item['object_id'] for item in here] == [cup['object_id']]
    nearby = ask(objects, near_object_id=cup['object_id'], radius_m=1.0)['objects']
    assert [item['label'] for item in nearby] == ['water bottle']
    assert ask(objects, near_object_id=cup['object_id'], radius_m=5.0, label='cup')['objects'][0]['last_seen_unix_sec'] == 30
    with pytest.raises(ValueError, match='unknown or has no position'):
        ask(objects, near_object_id='0' * 32, radius_m=1.0)
    # Evidence: retained until the observation store reports having evicted past it, or was recreated.
    assert [item['evidence_expired'] for item in cup['observations']] == [False]
    objects.set_cursor(EPOCH, 6, 4)
    states = {item['label']: item['observations'][0]['evidence_expired'] for item in ask(objects, since=15)['objects']}
    assert ask(objects, label='cup')['objects'][1]['observations'][0]['evidence_expired'] and not any(states.values())
    objects.set_cursor('f' * 32, 0, 0)
    assert all(item['observations'][0]['evidence_expired'] for item in ask(objects)['objects'])


@pytest.mark.parametrize('text', [
    '[]', '{"x": 1}', '{"limit": 0}', '{"limit": 101}', '{"label": ""}', '{"since": -1}', '{"state": "lost"}',
    '{"state": []}', '{"region": {"x": 0, "y": 0}}', '{"region": {"x": 0, "y": 0, "radius_m": 0}}',
    '{"region": {"x": 0, "y": 0, "radius_m": 1}, "near_object_id": "' + '0' * 32 + '", "radius_m": 1}',
    '{"region": {"x": 0, "y": 0, "radius_m": 1}, "radius_m": 1}', '{"near_object_id": "abc", "radius_m": 1}',
    '{"near_object_id": "' + '0' * 32 + '"}', '{"radius_m": 1}', '{"since": NaN}', ' ' * 4097])
def test_query_rejects_untrusted_input(text):
    with pytest.raises(ValueError):
        parse_query(text)


def test_bounds_never_drop_an_entry_for_not_being_seen(store):
    with pytest.raises(ValueError):
        Limits(misses_unconfirmed=3, misses_absent=2)
    with pytest.raises(ValueError):
        Limits(max_objects=0)
    # A tighter gate, so that a second place fits inside the range at which a view proves absence.
    objects = ObjectMap(store, Limits(max_objects=1, max_history=2, misses_unconfirmed=1, misses_absent=1,
                                      absent_retention_sec=100.0, gate_sigmas=2.0))
    revision = store.ingest(corridor(range(1, 11)), created_at=1.0)
    B = (4.325, -0.275, 0.125)
    at_a, at_b = octree([voxel(A)]), octree([voxel(B)])
    objects.ingest(record(store, revision, 3, [A], sequence=1, stamp=10), CAMERA, at_a, epoch=EPOCH)
    # Full: a new thing is refused rather than an unseen entry dropped for it.
    counts = objects.ingest(record(store, revision, 3, [B], sequence=2, stamp=20), CAMERA, at_b, epoch=EPOCH)
    assert counts['capacity'] == 1 and counts['created'] == 0
    assert [math.dist(xyz(item), A) < 0.05 for item in ask(objects)['objects']] == [True]
    # Proven absent, the old entry may give its room to the new one.
    counts = objects.ingest(record(store, revision, 3, [B], sequence=3, stamp=30), CAMERA, at_b, epoch=EPOCH,
                            absence=ABSENCE)
    assert counts['capacity'] == 1 and counts['missed'] == 1
    assert ask(objects)['objects'][0]['state'] == 'absent_at_last_place'
    assert objects.ingest(record(store, revision, 3, [B], sequence=4, stamp=40), CAMERA, at_b, epoch=EPOCH)['created'] == 1
    assert [math.dist(xyz(item), B) < 0.05 for item in ask(objects)['objects']] == [True]
    # History is bounded; an absent entry leaves after its retention.
    assert store.connection.execute('SELECT COUNT(*) FROM object_history').fetchone()[0] <= 2
    objects.ingest(record(store, revision, 3, [], sequence=5, stamp=50), CAMERA, octree(), epoch=EPOCH, absence=ABSENCE)
    assert ask(objects)['objects'][0]['state'] == 'absent_at_last_place'
    objects.ingest(record(store, revision, 3, [], sequence=6, stamp=200), CAMERA, None, epoch=EPOCH)
    assert ask(objects)['objects'] == []


VERSION_1 = '''
    CREATE TABLE site (site_ref TEXT NOT NULL);
    CREATE TABLE maps (map_ref TEXT PRIMARY KEY, created_at REAL NOT NULL, merged_into TEXT);
    CREATE TABLE revisions (sequence INTEGER PRIMARY KEY AUTOINCREMENT, revision TEXT NOT NULL UNIQUE,
        map_ref TEXT NOT NULL, parent TEXT, created_at REAL NOT NULL, cause TEXT NOT NULL, session TEXT NOT NULL,
        observer_ref TEXT NOT NULL, anchors INTEGER NOT NULL);
    CREATE TABLE poses (anchor_ref TEXT NOT NULL, sequence INTEGER NOT NULL, map_ref TEXT NOT NULL, pose TEXT,
        PRIMARY KEY (anchor_ref, map_ref, sequence));
    CREATE TABLE relinks (anchor_ref TEXT NOT NULL, map_ref TEXT NOT NULL, sequence INTEGER NOT NULL,
        via_anchor TEXT NOT NULL, pose TEXT NOT NULL, PRIMARY KEY (anchor_ref, map_ref, sequence));
    CREATE TABLE sessions (session TEXT PRIMARY KEY, observer_ref TEXT NOT NULL, started_at REAL NOT NULL,
        map_ref TEXT, relocalized_at REAL);
    CREATE TABLE alignments (from_map TEXT NOT NULL, from_revision TEXT NOT NULL, to_map TEXT NOT NULL,
        to_revision TEXT NOT NULL, pose TEXT NOT NULL, method TEXT NOT NULL, created_at REAL NOT NULL,
        PRIMARY KEY (from_map, to_map, to_revision));
    PRAGMA user_version=1;'''


def test_a_version_1_store_gains_the_object_tables_and_keeps_its_cartography(tmp_path):
    path = tmp_path / 'spatial.db'
    database = sqlite3.connect(path)
    database.executescript(VERSION_1)
    pose = json.dumps(yaw(0.0, (3.0, 0.0, 0.0)).to_dict())
    database.execute("INSERT INTO site VALUES ('home')")
    database.execute("INSERT INTO maps VALUES ('m1', 1.0, NULL)")
    database.execute("INSERT INTO revisions (revision,map_ref,parent,created_at,cause,session,observer_ref,anchors) "
                     "VALUES ('r1','m1',NULL,1.0,'initial','s1','robot-a',1)")
    database.execute("INSERT INTO poses VALUES ('rtabmap:test:3', 1, 'm1', ?)", (pose,))
    database.commit()
    database.close()

    store = SpatialStore(path, site_ref='home', observer_ref='robot-a')
    assert store.connection.execute('PRAGMA user_version').fetchone()[0] == SCHEMA_VERSION == 2
    revision = store.latest()
    assert revision.revision == 'r1' and store.anchor_pose(revision, 'rtabmap:test:3')[0].t == (3.0, 0.0, 0.0)
    objects = ObjectMap(store)
    assert objects.ingest(record(store, revision, 3, [A], sequence=1, stamp=10), CAMERA, octree([voxel(A)]),
                          epoch=EPOCH)['created'] == 1
    assert math.dist(xyz(ask(objects)['objects'][0]), A) < 0.05
    store.close()
    # Reopening is idempotent; a store written by newer software is refused, not misread.
    SpatialStore(path, site_ref='home', observer_ref='robot-a').close()
    database = sqlite3.connect(path)
    database.execute('PRAGMA user_version=3')
    database.close()
    with pytest.raises(ValueError, match='newer than this software'):
        SpatialStore(path, site_ref='home', observer_ref='robot-a')
