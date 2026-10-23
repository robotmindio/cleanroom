# Active search and negative evidence

`/robot/search` (`lekiwi_rmf/action/Search`) looks for any native label of the
detector profile in force. It is served by `robot_explorer`, so every viewpoint
is driven through exploration's guarded Nav2 path: the owner-guard lease, the
known-space planner, the whole-footprint route check, the collision monitor,
the base-motion permission, the arm-stow gate and the region stopping margin.
Search adds no motion channel and changes no limit. It is off until
`allow_search: true` in `config/exploration.yaml`, and it needs object perception
running (`start_object_perception`).

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
- saw the cell unoccluded in the occupancy map, inside the horizontal field of
  view, within `f·search_min_object_m / search_min_object_px` and
  `search_max_range_m`, with the whole `search_min_height_m`..`search_max_height_m`
  band inside the vertical field of view;
- had no detection of the label on that bearing; and
- is newer than `search_max_age_sec` and newer than any sighting of the label there.

A visit, a provider failure, a frame without pose, or a capture from another
model proves nothing. An image box without depth is a bearing, so every cell
along it remains a possible location. Coverage is a per-query grid bounded by
the map's cell limit, built from a bounded log of posed captures
(`search_capture_log`). A capture's pose is its recorded capture-time `map_pose`,
projected by [spatial memory](spatial-memory.md) into the current map revision.
A record without one falls back to TF while TF still holds its time.

When SLAM revises the map, every anchored capture in the log is projected into
the new revision and the query's coverage and prior sightings are rebuilt from
them, so evidence moves with the corrected map. Captures without a recorded map
pose cannot be placed in the new revision and leave the log
(`unanchored_after_revision` in the rejection counts).

## Choosing viewpoints

Each step weights searchable cells (mapped open cells and obstacle surfaces in
the region) that are unobserved or stale; cells where the label was last seen
get `search_prior_weight` extra. Candidates are reachable, footprint-clear base
poses on a `search_viewpoint_spacing_m` lattice, including turning in place,
for each base-fixed camera at `search_headings` headings. Utility is expected
newly resolved cells per metre of straight-line travel plus
`search_viewpoint_overhead_m`. This is an uncalibrated heuristic, not a
probability, and the report says so. At a viewpoint the search waits up to
`search_dwell_sec` for a stationary capture from the chosen camera; a view that
yields none is not retried. Safety faults, stale perception and missing odometry
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
| `incapable` | The label is outside the catalogue, or no fresh posed perception exists. No motion. | failed |

`report_json` holds the coverage fraction and areas (covered, unobserved, stale,
unmapped), the largest unobserved regions, the negative-evidence summary (model,
criteria, cameras, observation count, recent IDs and viewpoints, time span,
rejected captures), prior sightings, and plan revisions. Its statement claims
absence only for the covered cells: unobserved, unmapped and stale space was not
searched. A sighting from before the search start is a prior, never a result.

## Limits

- Occlusion uses the 2D occupancy map at integration time. Objects above or
  below the lidar plane can be hidden by furniture the map does not contain.
- Reprojection moves coverage with the graph node each capture was anchored to;
  anchor-pose uncertainty is not added to the evidence.
- Optical roll is ignored and travel cost is a straight line; Nav2 validates
  the real route.
- `search_min_object_px` is an uncalibrated default; it is fitted by
  [threshold calibration](threshold-calibration.md) once the recorded corpus exists.
- Physical qualification on the robot is outstanding; tracked in
  [#88](https://github.com/robotmindio/cleanroom/issues/88) and `DEFERRED.md`.
