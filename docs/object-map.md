# Object map

The robot keeps a persistent, queryable map of the things it sees: one entry per
thing, with an estimated place in the map, updated by every later sighting,
carried along by every map correction, kept across restarts, and downgraded
when its place is later seen empty.

It is the robot's **local landmark layer**. An entry says "something with this
label was estimated here, by this method, last seen then". It does not say
which object that is: whether two entries are one object, and what a thing's
lasting identity is, belongs to Knowledge. Nothing here is a measured position
unless the validated RGB-D path produced it, and every answer says which it is.

`spatial_memory` hosts it: `lekiwi_rmf/object_estimates.py` makes the
estimates, `lekiwi_rmf/object_map.py` holds the entries in the
[spatial memory](spatial-memory.md) database, and the node reads the
[observation store](object-observations.md) behind a persisted cursor. Limits
are in `config/spatial_memory.yaml` (`objects_*`).

## From a detection to an estimate

A stored observation gives an image box, the camera and the capture-time
`map_pose`. Each detection is placed by the first method that applies; a
detection no method can place creates nothing.

| Method | What it uses | Uncertainty reported | Accuracy class |
| --- | --- | --- | --- |
| `rgbd_surface` | The existing metric path: registered depth under a validated calibration (`position.mode == "camera"`). | The path's own `uncertainty_m`; its map uncertainty when the capture also has a validated map position. | Measured. `qualified: true` only when the path validated the map position of that capture. |
| `ray_octomap_hit` | The ray through the box centre, cast into the 3D map of the capture's own revision, stopped at the first occupied voxel. Unknown space on the way refuses the estimate. | `sqrt(voxel² + (range · half box angle)²)` | Inferred, decimetres at best. Wrong by metres when the box centre misses the object and the ray continues to what is behind it. Unqualified. |
| `triangulated` | The crossing of bearings of one label from two or more viewpoints at least `objects_min_baseline_m` apart with at least `objects_min_parallax_rad` between them. x and y only. | `range · tan(half box angle) / sin(parallax)` | Inferred, x and y only, no height. Assumes every bearing is the same object and that it did not move. Unqualified. |
| `bearing_only` | The map bearings the box spans, from the rays through its outline. | None: a direction with `range_m: [0, null]`, no point. | A direction, not a place. Unqualified. |

No method assumes the object stands on the floor.

Bearings are computed from the box outline in 3D, so they hold for a camera
with roll or pitch, up to `objects_max_tilt_rad`; beyond it an image box gives
no bearing. Every estimate lists its `assumptions`, its `calibration_sha256`
(null for everything but `rgbd_surface`) and `qualified`.

**No estimate is written to an observation.** The observation's `position`
stays what the metric gate in `lekiwi_rmf/object_geometry.py` decided, so a
consumer that reads `position.mode == "camera"` as a measurement still reads
only measurements.

## Entries

| Field | Meaning |
| --- | --- |
| `object_id` | Opaque, stable for the life of the entry. Not an identity. |
| `label` | The detector label, lower-cased with single spaces. |
| `site_ref`, `map_ref` | The place and cartography the entry belongs to. |
| anchor and offset | The graph node the thing was seen from and the estimate relative to it. The map position is computed from the anchor's pose in the newest revision each time it is asked for. |
| `sigma_m`, `method`, `qualified`, `assumptions` | As estimated; after fusion, of the fused estimate. |
| `state` | `present`, `unconfirmed`, `absent_at_last_place`; `moved` is reserved (see below). |
| `score` | The latest detector score, as `{kind: "detector_score"}`. Not a probability. |
| first seen, last seen, last verified | Capture times: of the first and last sighting, and of the last view that confirmed or refuted the entry at its place. |
| `miss_count` | Views that proved the place empty since the last sighting. |
| supporting observations | The newest `objects_max_support` observation ids. |
| history | Bounded events: `created`, `placed`, `updated`, `confirmed`, `ambiguous`, `unconfirmed`, `absent_at_last_place`, `reappeared`. A repeated event is one row per `objects_history_interval_sec`. |

Because the position is stored relative to an anchor, an entry needs no
revision to stay placeable: it follows a loop closure without being seen again,
follows a relinked anchor and a map merge, and outlives the pruning of the
revision it was made in. The tables are version 2 of the spatial database;
opening a version 1 store adds them and keeps its cartography.

## Association

A sighting can only join an entry with the same label on the same surviving
map. Both are compared in the newest revision.

- A point joins a point within `objects_gate_sigmas · sqrt(σ_entry² + σ_sighting²)`.
- A bearing joins a point it passes over (within the same gate and
  `objects_max_range_m`), and a point joins a bearing entry it lies on.
- Two bearings join when they cross as a `triangulated` estimate, or when they
  overlap from viewpoints on one line of sight (the same view again).

Then:

| Candidates | Result |
| --- | --- |
| exactly one | The entry is updated. Two points of the same class are fused by inverse variance, never below `objects_min_sigma_m`. A qualified position replaces an unqualified one and is never changed by one. A bearing confirms a point without moving it. |
| none | A new entry. |
| two or more, or two detections claiming one entry | Nothing is updated or created; `ambiguous` is recorded on each candidate. |

Two detections of one capture never merge: they are compared only with the
entries that existed before the capture. A sighting far from an entry of the
same label does not move that entry; it creates another, and the old one
changes only through evidence of absence.

## Forgetting by evidence

An entry is never removed or downgraded for not having been seen. A capture
counts against an entry only when all of this holds:

- the frame is from a model admitted for the label under its admitted profile
  revision, an open-vocabulary frame was asked for the label, and the camera
  was stationary (the active search's gates);
- the capture has the 3D map of its own revision, no older than
  `search_geometry_max_age_sec`;
- the square around the entry of half-width `objects_gate_sigmas · σ` (at most
  `objects_absence_max_radius_m`) is, over the whole height band, inside the
  frustum, within resolving range and known free from the camera
  (`SearchOctomap.visible_columns`, the search's visibility proof);
- the entry's estimated height lies inside that band;
- no detection of the label in the capture could be the entry;
- the last counted view was at least `objects_miss_interval_sec` earlier.

`objects_misses_unconfirmed` such views make a `present` entry `unconfirmed`,
`objects_misses_absent` make it `absent_at_last_place`. A sighting resets the
count and returns the entry to `present`. An `absent_at_last_place` entry is
removed after `objects_absent_retention_sec`, or to make room for a new entry
when the map holds `objects_max_objects`; if no entry is proven absent, the new
one is refused and counted instead.

The criteria and the height band are the active search's, read from
`config/exploration.yaml`: 0.02–0.30 m. **An object on a table, a shelf or any
surface above 0.30 m can therefore not be downgraded**, and neither can a
`triangulated` entry, whose height is unknown. They stay `present` with an
ageing `last_seen`. The 3D map is published only when the search is enabled in
the launch configuration; without it there is no `ray_octomap_hit` and no
evidence of absence.

## Queries

`/robot/objects/query` uses the `lekiwi_rmf/srv/QuerySpatial` type
(`request_json` → `success`, `detail`, `response_json`). Every field is
optional; the request is capped at 4 KiB.

```json
{"label": "cup", "region": {"x": 1.2, "y": 0.4, "radius_m": 1.5}, "since": 1700000000.0,
 "state": ["present", "unconfirmed"], "limit": 20}
```

```json
{"near_object_id": "<32 hex>", "radius_m": 1.0}
```

| Field | Meaning |
| --- | --- |
| `label` | Normalized like stored labels. |
| `region` | A disk in the newest revision of the map the robot is localized in. Refused with `localization unavailable` when the session has not localized. |
| `near_object_id`, `radius_m` | Entries within `radius_m` of that entry's position, itself excluded. Alternative to `region`. |
| `since` | `last_seen` at or after this Unix time. |
| `state` | One state or a list. |
| `limit` | 1–100, default 20; newest sighting first. |

An entry without a position (a bearing, or one that cannot be placed) is in no
region and near nothing.

```json
{"schema": "lekiwi.object_map/1", "site_ref": "home", "localized": true,
 "map": {"map_ref": "...", "revision": "..."}, "verification": "last_observed", "truncated": false,
 "ingestion": {"enabled": true, "observation_sequence": 1234, "counts": {"created": 3, "updated": 40}},
 "objects": [{
   "object_id": "...", "label": "cup", "state": "present",
   "site_ref": "home", "map_ref": "...", "revision": "...",
   "position": {"x": 4.31, "y": 0.02, "z": 0.12}, "bearings": [],
   "method": "ray_octomap_hit", "sigma_m": 0.089, "qualified": false,
   "assumptions": ["the object is the first surface on the ray through the box centre", "..."],
   "calibration_sha256": null, "score": {"kind": "detector_score", "value": 0.9},
   "first_seen_unix_sec": 0.0, "last_seen_unix_sec": 0.0, "last_verified_unix_sec": 0.0,
   "miss_count": 0, "sightings": 12, "shift_m": 0.6,
   "anchor_ref": "rtabmap:<tag>:<node>", "projection": "anchor",
   "observations": [{"id": "<64 hex>", "stamp_unix_sec": 0.0, "evidence_expired": false}]}]}
```

- `position` is in `revision`, the newest revision of the entry's map, computed
  at query time. `z` is null for a `triangulated` entry. For a `bearing_only`
  entry `position` is null and `bearings` lists `{origin: [x, y], bearing_rad:
  [low, high]}`. An entry whose anchor cannot be placed has `position: null`,
  `revision: null` and `position_unavailable_reason`.
- `projection` is how the anchor was placed: `anchor`, `relinked_anchor` or
  `merge_alignment`.
- `shift_m` is the horizontal distance from the entry's first estimate: map
  corrections and later sightings together.
- `verification` is always `last_observed`. The map reports what was seen and
  when; it never re-verifies at query time. Look again before acting on an entry.
- `evidence_expired` is true once the observation store reports having evicted
  past that record, or was recreated. A pinned record may still exist.
- `ingestion` says how far the map has read the observation store, so a stale
  map is distinguishable from an empty one.

An entry with a point position is a navigation target: `navigate` with
`target: {object: {object_id, standoff_m}}` stands that far from the entry's
position in the newest revision and faces it, and reports the entry's `state`,
`method`, `sigma_m`, `qualified` and `last_seen_unix_sec` with the pose. An
`absent_at_last_place` entry, a bearing, an entry that cannot be placed and one
whose `sigma_m` is too large for the standoff are refused: see
[navigation targets](robot-tasks.md#navigation-targets). The query has no
lookup by `object_id`, so the task API finds an entry only among the 100 most
recently seen.

`/robot/spatial/query` with `{"op": "pose"}` returns where the robot is, for
remembering a spot: see [spatial memory](spatial-memory.md).

`/robot/observations/query` recalls the observations themselves by label, time,
camera and capture place: see [object observations](object-observations.md).

## What is not qualified

Everything below is reported `qualified: false`, or is a limit of what the map
can claim, until it is measured on the robot:

- **Every method but `rgbd_surface`.** `ray_octomap_hit`, `triangulated` and
  `bearing_only` are inferences from an image box. Their `sigma_m` is a model
  of the stated assumptions, not a measured error.
- **Astra intrinsics, registration and depth scale.** No camera has a validated
  calibration in `config/object_cameras.yaml`, so no detection has a metric
  position and `rgbd_surface` produces nothing.
- **Front camera intrinsics, height and pitch.** Rays and bearings use the
  `camera_info` pinhole model without distortion and the URDF mount through TF;
  neither has been checked against known object positions.
- **Detector recall for absence.** An empty view counts against an entry on the
  search's criteria (`search_min_object_px` is itself an uncalibrated default);
  the detector's miss rate at that size and range has not been measured, so
  `absent_at_last_place` means "not detected in views that should have shown
  it", not "proven gone".
- **Association and fusion limits.** `objects_gate_sigmas`,
  `objects_min_sigma_m` and the triangulation limits are engineering defaults.
- **Anchor-pose uncertainty** is not propagated: `sigma_m` is the estimate's
  alone, and a capture in the instant between RTAB-Map's TF and graph updates
  can be anchored in the previous revision.
- **`moved`** is never assigned here. Saying that an entry which disappeared is
  the one that appeared elsewhere is an identity decision; the vocabulary keeps
  the state for the resolver that may make it.
