"""Map identities, revisions, anchoring and reprojection without ROS (spec 12, T-26..T-29)."""

import math
import sqlite3

import numpy as np
import pytest

from lekiwi_rmf.spatial_anchors import (
    SpatialError, SpatialStore, Transform, rtabmap_database_tag, table_digest,
)


def yaw(angle, t=(0.0, 0.0, 0.0)):
    return Transform.create(list(t), [0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])


def corridor(ids, correction=lambda i: (0.0, 0.0, 0.0)):
    """Graph nodes one metre apart along x; ``correction`` is a per-node (dx, dy, dyaw) of one solution."""
    poses = {}
    for i in ids:
        dx, dy, dtheta = correction(i)
        poses[f'rtabmap:test:{i}'] = yaw(dtheta, (float(i) + dx, dy, 0.0))
    return poses


# Camera 0.1 m ahead of a node and 0.3 m up, optical z looking along the node's x axis.
NODE_CAMERA = Transform.create([0.1, 0.0, 0.3], [-0.5, 0.5, -0.5, 0.5])
OBJECT_IN_CAMERA = (0.0, 0.0, 1.2)


def capture(store, revision, node, offset=Transform.identity()):
    """The capture-time map pose a camera at ``node`` reports through TF in ``revision``."""
    node_pose = store.anchor_pose(revision, f'rtabmap:test:{node}')[0]
    camera = node_pose.compose(offset).compose(NODE_CAMERA)
    return {'site_ref': store.site_ref, 'map_ref': revision.map_ref, 'revision': revision.revision,
            'localization_session': revision.session, 'pose': camera.to_dict()}


def map_point(map_pose, point=OBJECT_IN_CAMERA):
    return Transform.from_dict(map_pose['pose']).apply(point)


@pytest.fixture
def store(tmp_path):
    value = SpatialStore(tmp_path / 'spatial.db', site_ref='home', observer_ref='robot-a')
    value.start_session(now=0.0)
    yield value
    value.close()


def test_transform_conventions():
    a_b = yaw(math.pi / 2, (1.0, 0.0, 0.0))
    # T_A_B maps B coordinates into A: B's x axis is A's y axis, shifted by 1 m along A's x.
    assert np.allclose(a_b.apply((1.0, 0.0, 0.0)), (1.0, 1.0, 0.0))
    assert np.allclose(a_b.compose(a_b.inverse()).t, (0, 0, 0), atol=1e-9)
    assert np.allclose(Transform.from_matrix(a_b.rotation(), a_b.t).q, a_b.q)
    with pytest.raises(ValueError):
        Transform.create([0, 0, 0], [0, 0, 0, 2])
    with pytest.raises(ValueError):
        Transform.from_matrix(np.diag([1.0, 1.0, -1.0]), [0, 0, 0])
    with pytest.raises(ValueError):
        Transform.create([math.nan, 0, 0], [0, 0, 0, 1])
    covariance = a_b.rotate_covariance([4, 0, 0, 0, 1, 0, 0, 0, 1])
    assert np.allclose(np.reshape(covariance, (3, 3)), np.diag([1.0, 4.0, 1.0]))


def test_identical_or_jittered_graph_keeps_its_revision(store):
    first = store.ingest(corridor(range(1, 8)), created_at=1.0)
    assert first.cause == 'initial' and first.parent is None and first.anchors == 7
    assert store.ingest(corridor(range(1, 8)), created_at=2.0) == first
    jitter = store.ingest(corridor(range(1, 8), lambda i: (2e-4, 0.0, 0.0)), created_at=3.0)
    assert jitter == first
    grown = store.ingest(corridor(range(1, 9)), created_at=4.0)
    assert grown.parent == first.revision and grown.map_ref == first.map_ref and grown.anchors == 8


def test_t26_loop_closure_reprojects_without_fictitious_motion(store):
    drifted = store.ingest(corridor(range(1, 11)), created_at=1.0)
    before = capture(store, drifted, 3)
    # Loop closure: a non-rigid correction, larger along the corridor.
    closed = store.ingest(corridor(range(1, 11), lambda i: (0.01 * i, 0.04 * i, 0.01 * i)),
                          created_at=2.0, cause='loop_closure')
    assert closed.cause == 'loop_closure' and closed.parent == drifted.revision
    after = capture(store, closed, 3)
    # Raw capture-revision coordinates of one unmoved object disagree by the correction itself.
    assert math.dist(map_point(before), map_point(after)) > 0.1
    projected = store.project(before, [OBJECT_IN_CAMERA])
    assert projected['revision'] == closed.revision and projected['capture_revision'] == drifted.revision
    assert projected['method'] == 'anchor' and projected['shift_m'][0] > 0.1
    assert math.dist(projected['points'][0], map_point(after)) < 1e-6
    # The stored measurement is untouched: projecting into the capture revision returns it.
    original = store.project(before, [OBJECT_IN_CAMERA], target={'map_ref': drifted.map_ref,
                                                                 'revision': drifted.revision})
    assert math.dist(original['points'][0], map_point(before)) < 1e-6


def test_t27_real_motion_survives_reprojection(store):
    drifted = store.ingest(corridor(range(1, 11)), created_at=1.0)
    before = capture(store, drifted, 3)
    closed = store.ingest(corridor(range(1, 11), lambda i: (0.0, 0.03 * i, 0.0)), created_at=2.0)
    moved = (0.5, 0.0, 1.2)  # The object was carried 0.5 m across the camera's view.
    after = capture(store, closed, 3)
    a = store.project(before, [OBJECT_IN_CAMERA])['points'][0]
    b = store.project(after, [moved])['points'][0]
    assert math.dist(a, b) == pytest.approx(0.5, abs=1e-6)


def test_anchor_reports_knowledge_estimate_fields(store):
    revision = store.ingest(corridor(range(1, 6)), created_at=1.0)
    anchored = store.anchor(capture(store, revision, 4), [OBJECT_IN_CAMERA], [0.03])
    assert anchored['anchor_ref'] == 'rtabmap:test:4' and anchored['revision'] == revision.revision
    estimate = anchored['estimates'][0]
    node = store.anchor_pose(revision, 'rtabmap:test:4')[0]
    assert np.allclose(node.apply(estimate['anchor_xyz']), map_point(capture(store, revision, 4)))
    assert np.allclose(np.trace(np.reshape(estimate['cov_xyz'], (3, 3))), 3 * 0.03 ** 2)


def test_t28_restart_keeps_the_map_and_separates_the_session(tmp_path):
    path = tmp_path / 'spatial.db'
    store = SpatialStore(path, site_ref='home', observer_ref='robot-a')
    first_session = store.start_session(now=0.0)
    old = store.ingest(corridor(range(1, 11)), created_at=1.0)
    before = capture(store, old, 2)
    store.close()

    store = SpatialStore(path, site_ref='home', observer_ref='robot-a')
    session = store.start_session(now=10.0)
    assert session != first_session and store.session_state()['map_ref'] is None
    relocalized = store.ingest(corridor(range(1, 11), lambda i: (0.0, 0.002 * i, 0.0)), created_at=11.0)
    assert relocalized.map_ref == old.map_ref and relocalized.cause == 'relocalization'
    assert relocalized.session == session != old.session
    assert store.session_state()['relocalized_at_unix_sec'] == 11.0
    assert store.project(before, [OBJECT_IN_CAMERA])['revision'] == relocalized.revision

    # A mapping restart that has not recognised the place yet is a different map.
    store.close()
    store = SpatialStore(path, site_ref='home', observer_ref='robot-a')
    store.start_session(now=20.0)
    fresh = store.ingest(corridor(range(100, 106), lambda i: (-95.0, 3.0, 0.0)), created_at=21.0)
    assert fresh.map_ref != old.map_ref and fresh.cause == 'initial'
    assert store.session_state()['relocalized_at_unix_sec'] is None
    with pytest.raises(SpatialError) as error:
        store.project(before, [OBJECT_IN_CAMERA], target={'map_ref': fresh.map_ref, 'revision': fresh.revision})
    assert error.value.code == 'map_unaligned'
    in_fresh = capture(store, fresh, 102)

    # A verified loop closure joins both graphs: the newer map is absorbed with an alignment.
    joined = {**corridor(range(1, 11), lambda i: (0.0, 0.002 * i, 0.0)),
              **corridor(range(100, 106), lambda i: (-95.0, 0.0, 0.0))}
    merged = store.ingest(joined, created_at=22.0, cause='loop_closure')
    assert merged.map_ref == old.map_ref and merged.cause == 'merge'
    assert store.surviving_map(fresh.map_ref) == old.map_ref
    alignment = store.alignment(fresh.map_ref, old.map_ref)
    assert alignment is not None and np.allclose(alignment[0].t, (0.0, -3.0, 0.0), atol=1e-6)
    projected = store.project(in_fresh, [OBJECT_IN_CAMERA])
    assert projected['revision'] == merged.revision
    assert math.dist(projected['points'][0], alignment[0].apply(map_point(in_fresh))) < 1e-6
    assert store.project(before, [OBJECT_IN_CAMERA])['revision'] == merged.revision
    store.close()


def test_t29_two_maps_named_map_at_different_sites_never_mix(tmp_path):
    home = SpatialStore(tmp_path / 'home.db', site_ref='home', observer_ref='robot-a')
    shop = SpatialStore(tmp_path / 'shop.db', site_ref='workshop', observer_ref='robot-b')
    home.start_session(now=0.0)
    shop.start_session(now=0.0)
    # Identical node ids and coordinates, both in a frame called "map".
    home_revision = home.ingest(corridor(range(1, 6)), created_at=1.0)
    shop_revision = shop.ingest(corridor(range(1, 6)), created_at=1.0)
    assert home_revision.revision != shop_revision.revision
    for source, target in ((home, shop), (shop, home)):
        foreign = capture(source, source.latest(), 2)
        with pytest.raises(SpatialError) as error:
            target.project(foreign, [OBJECT_IN_CAMERA])
        assert error.value.code == 'site_mismatch'
    forged = {**capture(home, home_revision, 2), 'site_ref': 'workshop'}
    with pytest.raises(SpatialError) as error:
        shop.project(forged, [OBJECT_IN_CAMERA])
    assert error.value.code == 'revision_unknown'
    home.close()
    shop.close()
    with pytest.raises(ValueError, match='belongs to site'):
        SpatialStore(tmp_path / 'home.db', site_ref='workshop', observer_ref='robot-a')


def test_compacted_anchor_is_relinked_and_follows_its_neighbour(store):
    first = store.ingest(corridor(range(1, 8)), created_at=1.0)
    before = capture(store, first, 5)
    # Node 5 leaves the graph (memory management); the rest is corrected by +0.2 m in y.
    remaining = corridor([1, 2, 3, 4, 6, 7], lambda i: (0.0, 0.2, 0.0))
    second = store.ingest(remaining, created_at=2.0)
    projected = store.project(before, [OBJECT_IN_CAMERA])
    assert projected['revision'] == second.revision and projected['method'] == 'relinked_anchor'
    expected = np.asarray(map_point(before)) + (0.0, 0.2, 0.0)
    assert np.allclose(projected['points'][0], expected, atol=1e-6)
    pages, truncated = store.anchors(second)
    assert [row['anchor_ref'] for row in pages] == sorted(remaining) and not truncated
    # Returning to the graph restores a measured pose instead of the relink.
    third = store.ingest(corridor(range(1, 8), lambda i: (0.0, 0.2, 0.0)), created_at=3.0)
    assert store.project(before, [OBJECT_IN_CAMERA], target={'map_ref': third.map_ref,
                                                             'revision': third.revision})['method'] == 'anchor'


def test_retention_prunes_revisions_and_reports_the_gap(tmp_path):
    store = SpatialStore(tmp_path / 'spatial.db', site_ref='home', observer_ref='robot-a',
                         retention_sec=10.0, max_revisions=3)
    store.start_session(now=0.0)
    old = store.ingest(corridor(range(1, 5)), created_at=1.0)
    before = capture(store, old, 2)
    for step in range(1, 6):
        latest = store.ingest(corridor(range(1, 5), lambda i, s=step: (0.0, 0.01 * s, 0.0)), created_at=20.0 + step)
    assert store.revision(old.revision) is None and store.latest() == latest
    with pytest.raises(SpatialError) as error:
        store.project(before, [OBJECT_IN_CAMERA])
    assert error.value.code == 'revision_unknown'
    assert store.anchors(latest)[0]  # The newest table survives pruning intact.
    store.close()


def test_revision_identity_includes_its_parent():
    table = corridor(range(1, 4))
    assert table_digest('m', None, table) != table_digest('m', 'parent', table)


def test_database_tag_changes_when_the_database_is_recreated(tmp_path):
    path = tmp_path / 'rtabmap.db'
    with pytest.raises(SpatialError):
        rtabmap_database_tag(path)

    def create(entered):
        if path.exists():
            path.unlink()
        connection = sqlite3.connect(path)
        connection.execute('CREATE TABLE Node (id INTEGER PRIMARY KEY, time_enter DATE)')
        connection.execute('INSERT INTO Node VALUES (1, ?)', (entered,))
        connection.commit()
        connection.close()

    create('2026-10-01 08:00:00')
    first = rtabmap_database_tag(path)
    assert first == rtabmap_database_tag(path)
    create('2026-10-19 09:00:00')
    assert rtabmap_database_tag(path) != first
