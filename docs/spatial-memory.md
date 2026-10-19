# Spatial memory: maps, revisions and anchors

The robot localizes in local SLAM frames without GPS. A coordinate is only
meaningful together with the place it belongs to, so every map position the
robot stores names `(site_ref, map_ref, revision)`. A frame called `map` alone
identifies nothing: two robots, two sites or two disconnected mapping sessions
all publish a `map` frame.

| Identity | Meaning on this robot |
| --- | --- |
| `site_ref` | The physical place, from `config/spatial_memory.yaml`. A store refuses to reopen under another site. |
| `map_ref` | One persistent cartography: a connected RTAB-Map graph. Opaque and stable across restarts. |
| `revision` | One optimized solution of that graph, with its parent. A loop closure or optimization that moves any node by more than 1 mm or 1 mrad creates one. |
| `anchor_ref` | One graph node, `rtabmap:<database tag>:<node id>`. The tag comes from the database's first node, so a recreated database never reuses old anchors. |
| `localization_session` | One run of the localizer. A restart creates a new session, not a new map. |

Conventions follow the shared `robotmind.spacetime/1` contract: `T_A_B` maps B
coordinates into A, metres and radians, poses as `{"t": [x, y, z], "q": [x, y,
z, w]}` with a unit quaternion, covariances as nine row-major floats in the
frame of their container.

## How revisions are made

`spatial_memory` (launched with RTAB-Map by `slam.launch.py`) turns each
`/mapGraph` into a revision of a map:

- A graph sharing nodes with one known map continues it. The first graph of a
  new session that shares nodes is a `relocalization`.
- A graph sharing no node starts a new map (`initial`), for example a mapping
  restart that has not recognised the place yet. Its coordinates are not
  comparable with the earlier map.
- A graph sharing nodes with several maps merges them into the oldest
  (`merge`): a verified loop closure joined the sessions. The absorbed map
  records `merged_into` and an alignment computed from a shared node.
- A node that leaves the graph (memory management) is relinked to the nearest
  surviving node with their last relative pose, so estimates on it stay
  projectable and say so (`relinked_anchor`).

Anchor poses are stored only when they change. Revisions older than
`retention_sec` (the observation store's seven days) or beyond `max_revisions`
are pruned, keeping each map's newest; a projection from a pruned revision fails
with `revision_unknown` instead of guessing.

## Captures and reprojection

Each stored observation records `map_pose`: the capture-time `T_map_camera`
from TF, the revision in force at the capture stamp, the site, map and session.
Without a revision at capture time, a session that has not localized, or
without historical TF, the observation records `map_pose: null` and
`map_pose_unavailable_reason`. The original record is never rewritten.

`/robot/spatial/query` (`lekiwi_rmf/srv/QuerySpatial`) answers:

| `op` | Result |
| --- | --- |
| `current` | Readiness, session, and the newest revision of the session's map. |
| `maps` | Maps with merges, recent sessions and alignments. |
| `revision` | One revision with a page of its anchor table (`after_anchor`, `limit` up to 5000). |
| `anchor` | For captures (`map_pose`, capture-frame `points`, `uncertainties_m`): the nearest node of the capture revision, `T_anchor_camera`, and each point's `anchor_xyz` and `cov_xyz`. These are the Knowledge `estimate` fields. |
| `project` | Camera pose and points in a target revision (default: the newest of the capture's map), the capture revision, the method (`anchor`, `relinked_anchor`, `merge_alignment`) and the reprojection shift. |

Projection computes `p_map(r) = T_map_anchor(r) · T_anchor_camera · p_camera`.
A loop closure therefore moves estimates with the part of the graph they were
seen from, and two observations of an unmoved object agree in one revision even
though their capture-revision coordinates differ. A real move remains a
displacement in that common revision. Captures from another site fail with
`site_mismatch`, and from an unaligned map with `map_unaligned`.

The latched `/robot/spatial/revision` topic announces each revision with its
session.

## Limits

- A capture takes the newest revision whose graph stamp is not after the capture
  stamp. RTAB-Map publishes TF and the graph in one update, so a capture made in
  the instant between them can take the previous revision. The size of that
  window on the robot is part of the qualification in
  [#91](https://github.com/robotmindio/cleanroom/issues/91).
- An RTAB-Map respawn inside one `spatial_memory` process keeps the session.
- Anchor-pose uncertainty is not propagated; `cov_xyz` is the measurement's.
- Knowledge receives revisions and anchored estimates through the robot's
  Knowledge bridge; this repository produces them but does not send them.
