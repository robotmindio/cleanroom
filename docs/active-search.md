# Active search and negative evidence

`/robot/search` (`lekiwi_rmf/action/Search`) looks for one label: any label an
open-vocabulary recognizer can be asked for, or a label of a closed recognizer's
catalogue (see [the label and the query](#the-label-and-the-perception-query)).
It is served by `robot_explorer`, so every viewpoint
is driven through exploration's guarded Nav2 path: the owner-guard lease, the
known-space planner, the whole-footprint route check, the collision monitor,
the base-motion permission, the arm-stow gate and the region stopping margin.
Search adds no motion channel and changes no limit. The tracked
`allow_search: true` in `config/exploration.yaml` enables it when native RGB-D
is available and exploration is allowed. It stays idle until an explicit goal
and needs object perception running (`start_object_perception`). This enables
attended operation; the physical qualification below remains outstanding.

## The label and the perception query

A search submitted through the [task API](robot-tasks.md) is self-sufficient: no
client sets the detector's query first or restores it afterwards.

- The task holds the `perception_query` resource from admission until its journal
  row is terminal, and the gateway states the search's label with the holder on
  `/robot/resources` (`resources.perception_query.query`).
- When the query in force does not already ask for that label, the detector asks
  for exactly it while the task holds the reservation. Exactly, not the earlier
  query plus the label: phrases of one prompt compete for the same proposals and
  lower each other's scores, and the prompt is bounded, so an extended query
  would make both the detection and the evidence of absence depend on whatever
  happened to be in force. A label the query already asks for changes nothing.
- When the holder is gone (succeeded, failed, canceled, or ended by restart
  reconciliation), the detector puts back the query it replaced: the exact query
  object, with its parts and learned concepts. `set_query` is refused for
  everyone while the reservation is held, so nothing can change in between. The
  reservation is in the durable task journal: a gateway or Explorer restart
  neither loses nor releases it. A detector restart loses its runtime query,
  like any detector restart: it starts from the tracked `default_query`, asks for
  the label again while the reservation is still held, and restores the default.
- The search stands still until the published profile confirms the label
  (`searching: awaiting the perception query`, at most `service_timeout_sec`), and
  then until a new frame that can speak for the label arrives (at most
  `search_dwell_sec`). Frames from before the change ran under another query and
  count as `ignored_other_query`; they are never evidence for this label.

A search that cannot get an instrument ends `incapable` before any motion, with
the cause in `report_json.cause` and as the task's `reason_code`:

| Cause | Meaning |
| --- | --- |
| `closed_vocabulary` | Only a closed-vocabulary recognizer is in force or serving (for example the local fallback while the open-vocabulary provider is down) and the label is outside its catalogue. Its own labels remain searchable. |
| `query_unconfirmed` | An open-vocabulary recognizer exists but was not asked for the label: the goal was sent straight to the action, which reserves nothing; the label cannot be a query phrase (letters, digits, spaces, hyphens, apostrophes; 48 characters); or the detector did not confirm in time. |

A goal sent straight to `/robot/search` searches only labels the profile in
force already reports.

## What counts as searched

The detector publishes its operating points on the latched
`/robot/perception/profile` topic: the camera bindings and, for the primary and
any fallback recognizer, the model SHA-256, proposal profile (revision and
parameters), label catalogue and whether it is open-vocabulary. A search session
credits only frames under the profile revision it admitted at its start; frames
after a profile change count as `ignored_other_profile`. An open-vocabulary primary's
catalogue is the active query (see `object-observations.md`), and the topic
republishes when it changes. Each stored frame keeps the SHA-256 of the
recognizer that served it, and an open frame also keeps the query it ran under:
it speaks only for labels that query asked for, and frames under another query
are counted as `ignored_other_query`. A frame speaks only for labels in
its own recognizer's catalogue: while the closed-set fallback serves, a label
only the open-vocabulary primary can report gains no evidence of absence, and
those frames are counted as ignored in the report. A cell has evidence of
absence for a label only when a stored observation from a recognizer that can
report it:

- has a capture-time `map` pose for its optical frame and the camera's intrinsics;
- was taken while the base (and, for the wrist camera, the arm) was stationary;
- saw the entire cell and `search_min_height_m`..`search_max_height_m` volume
  inside the camera's full optical frustum, within
  `f·search_min_object_m / search_min_object_px` and `search_max_range_m`,
  with the camera-to-cell volume known free in capture-time 3D geometry;
- had no detection of the label on that bearing; and
- is newer than `search_max_age_sec` and newer than any sighting of the label there.

A visit, a provider failure, a frame without pose or 3D geometry, or a capture
from another model proves nothing. An image box without depth is a bearing, so
every cell along it remains a possible location. With optical pitch or roll,
the horizontal box interval alone is ambiguous: all visible cells remain
possible locations for that label. Positive detections still count.
Coverage is a per-query grid bounded by
the map's cell limit, built from a bounded log of posed captures
(`search_capture_log`). A capture's pose is its recorded capture-time `map_pose`,
projected by [spatial memory](spatial-memory.md) into the current map revision.
A record without one falls back to TF while TF still holds its time.

The 2D occupancy fan ranks candidate viewpoints; negative evidence additionally
requires a fresh binary `OcTree` or `ColorOcTree` on `search_geometry_topic`
(default `/octomap_binary`) in frame `map`. Its timestamp must match the graph
publication in `/robot/spatial/revision`, precede the image, and be no older
than `search_geometry_max_age_sec` (default 1 second). Unknown voxels, occupied
voxels and grazing contacts block visibility. Map resolution must resolve
`search_min_object_m`; malformed, oversized, stale or replayed maps earn no
coverage. `missing_3d_geometry` reports captures without an eligible proof.

When the tracked `allow_search` switch is enabled, bringup also enables
RTAB-Map's 3D depth-image ray tracing. On real hardware it uses only the
native Astra RGB-D stream for mapping occupancy, excluding the front view's
depth reprojected from the merged navigation cloud. A real deployment without
the Astra keeps search unavailable and retains its ordinary mapping mode.
In simulation the depth projection uses only
the front depth camera's own point cloud, without filling holes. The ordinary
search-disabled mapping configuration is unchanged. Mapping also updates
while stationary, so new depth frames can produce fresh geometry during a
search dwell. Camera calibration and the resulting free-space coverage still
require physical acceptance. Previously flattened map cells do not become
3D evidence merely by enabling the switch; record a 3D map or independently
validate regeneration before accepting coverage.

When SLAM revises the map, every anchored capture in the log is projected into
the new revision and the query's coverage and prior sightings are rebuilt from
them, so evidence moves with the corrected map. Captures without a recorded map
pose cannot be placed in the new revision and leave the log
(`unanchored_after_revision` in the rejection counts).
Each geometric proof retains its original camera pose and references a shared
capture-time map, so the occluders move with that camera's corrected anchor.
The map cache is bounded to 32 MiB of binary data and four decoded trees;
each payload is at most 2 MiB. Evicted evidence becomes unobserved rather than
being reconstructed against a newer scene.
Capture-time geometry metadata is retained up to the bounded capture-log capacity,
so inference finishing after several newer map updates can still use its original
scene. The one-second capture-to-geometry bound and revision checks remain enforced.
With range/RGB-D synchronization, the mapper keeps 15 synchronization candidates
and pairs stamps at most 150 ms apart; the five-message transport queue stays short.

Search never drives without a map. A goal sent where no `/map` exists, or where
localization has no `map` pose yet, is accepted and ends at once with outcome
`unmapped` and no motion, so a caller can run `explore` and then search again.

Exploration does not require search coverage or a previously saved map. An explicit
Explore goal with no `/map` enables native RTAB-Map mapping immediately, even when
bringup selected localization and Nav2 is still waiting for that first map. It then
uses the first sensor-built grid and real map transform. Before any navigation it
requires the ordinary fresh permission, stow, SLAM, collision and ownership checks;
unknown space is never fabricated as free. Cancellation remains effective while
waiting for initial sensors. Missing first observations terminate at the requested
task deadline rather than leaving an uncancelable startup.

## Choosing viewpoints

Each step weights searchable cells (mapped open cells and obstacle surfaces in
the region) that are unobserved or stale; cells where the label was last seen
get `search_prior_weight` extra. Candidates are reachable, footprint-clear base
poses on a `search_viewpoint_spacing_m` lattice, including turning in place,
for each base-fixed camera at `search_headings` headings. Utility is expected
newly resolved cells per metre of straight-line travel plus
`search_viewpoint_overhead_m`. This is an uncalibrated heuristic, not a
probability, and the report says so. At a viewpoint the search waits up to
`search_dwell_sec` for a capture that resolves part of the planned view. A view
whose dwell resolves no cell is not retried, and the report counts it by cause in
`progress.viewpoints_unproductive`: `no_stationary_capture`, or `no_new_cells`
when the captures were eligible but the view was occluded or unknown in 3D.
When a whole dwell brings captures but none can count anywhere (only a recognizer
that cannot report the label served, another profile or open-vocabulary query was
in force, or no capture had eligible 3D geometry), no other viewpoint can do
better: the search ends `incapable` with that cause instead of spending its
budget. The same dwell runs once where the robot stands before its first plan, so
a recognizer that cannot report the label ends the search without motion. Safety faults, stale perception and missing odometry
pause the search with the robot held still, preserving the active-time budget,
as exploration does.

While driving, new captures are integrated as they are stored. When they leave
less than `search_replan_fraction` of the planned gain, or a SLAM loop closure
revises the map, the owned Nav2 goal is canceled and a new viewpoint is chosen.
Each plan revision records its reason and the observation IDs that caused it.

## Termination and the report

| Outcome | Meaning | Task state |
| --- | --- | --- |
| `found` | The label was detected after the search started; `evidence_ids` are its observation IDs. | succeeded |
| `coverage_reached` | `target_coverage` of searchable mapped cells in the region has fresh evidence of absence. | succeeded |
| `inaccessible` | No reachable viewpoint can observe the remaining unobserved cells. | failed |
| `budget_exhausted` | Active duration or odometry travel budget reached. | failed |
| `incapable` | No recognizer in force or serving can report the label, or no fresh posed perception exists (no motion; `cause` `closed_vocabulary` or `query_unconfirmed` when the vocabulary is why), or a whole dwell at a viewpoint had no admitted recognizer frame or no eligible 3D geometry. | failed |
| `unmapped` | No map or map pose exists here yet. No motion; explore first. | failed |

`report_json` holds the `cause` beside the outcome (null unless stated above),
the coverage fraction and areas (covered, unobserved, stale,
unmapped), the largest unobserved regions, the negative-evidence summary (model,
criteria, cameras, observation count, recent IDs and viewpoints, time span,
rejected captures), prior sightings, and plan revisions. Its statement claims
absence only for the covered cells: unobserved, unmapped and stale space was not
searched. A sighting from before the search start is a prior, never a result.

## Limits

- Unknown or incomplete 3D mapping reduces coverage. Conservative volume
  intersection can reject a clear view; it must not infer clearance from a
  few sampled rays. Physical scene and sensor accuracy still require qualification.
- Reprojection moves coverage with the graph node each capture was anchored to;
  anchor-pose uncertainty is not added to the evidence.
- A fresh map publication does not date each voxel. Global occupancy assumes
  static occluders; dynamic scenes require separate live qualification and
  per-frame depth evidence before relying on negative coverage.
- Travel cost is a straight line; Nav2 validates the real route.
- `search_min_object_px` is an uncalibrated default; it is fitted by
  [threshold calibration](threshold-calibration.md) once the recorded corpus exists.
- Physical qualification on the robot is outstanding; tracked in
  [#88](https://github.com/robotmindio/cleanroom/issues/88) and `DEFERRED.md`.
