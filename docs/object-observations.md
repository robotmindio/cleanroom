# Object perception and evidence

Perception publishes bounded candidates for every native class of the selected
checkpoint, plus immutable image evidence. Goal-specific class filtering (for
example Fiber's cup request) belongs to the consumer. Perception cannot
authorize motion or complete a goal. Fiber's existing verifier owns goal
completion. Image recognition, color classification, metric grounding and moving
search have separate qualification gates.

## Install and describe the operating point

```bash
scripts/install-object-model.py
scripts/object-perception.sh --describe
scripts/object-perception.sh
```

The installer follows the model selected in `config/object_perception.yaml`.
Artifacts are pinned by upstream revision, file size and SHA-256 in
`config/recognition_models.yaml`. Startup verifies every file and never downloads
weights. RT-DETR and Grounding DINO use a separate CPU environment under
`~/.ros/lekiwi/perception-venv`; the installer creates it with tracked package
requirements and a compatible CPU Torch/Torchvision pair. ROS and motor-driver
Python installations are not modified. Models remain outside Git.

The current **development** operating point is RT-DETRv2-R18 at 384×384 and a
0.50 score threshold. It detected the three retained patterned-mug frames at
0.693–0.706, where YOLOX missed them. Warm processing was approximately 0.89 s;
the first measured frame took 6.76 s. This one scene establishes a reproducible
improvement, not site qualification. Production start remains disabled in
`config/object_perception_deployment.yaml`.

`model_sha256` now identifies the complete recognition operating point: pinned
artifacts, backend, input size/prompt, threshold, preprocessing revision, label
output (`native`) and actual runtime package versions. It is not just the weight-file hash. Model,
threshold or runtime changes require a new admitted profile identity. The
`--describe` output supplies the exact model and camera identities to put in a
reviewed Fiber profile. Old weight-only profiles must not be reused.

The fixed comparison choices are `yolox`, `rtdetr_v2_r18` and
`grounding_dino_tiny`. Install an alternative with `--name NAME`. The latter is
an offline open-vocabulary diagnostic with a frozen `a cup. a mug.` prompt;
its labels are the matched prompt phrases and its scores are not
interchangeable with closed-vocabulary detector scores. YOLOX labels come from
its 80 COCO classes and RT-DETR labels from the checkpoint's `id2label`.
Closed-vocabulary detections are suppressed per class, so overlapping objects of
different classes both survive; Grounding DINO prompt phrases share one
suppression group because synonymous phrases propose the same object. A frame
keeps its 20 highest-scoring detections; lower-scored objects in a more
cluttered frame are absent from that observation.

Perception parameters are fixed at startup because they form the admitted
operating point. A runtime `ros2 param set` on them is rejected with a reason
instead of succeeding without effect; change the tracked configuration and
restart. No
model cascade or language-model completion authority is introduced. The
vendored YOLOX implementation lives directly in the Python package; its
upstream license/provenance remain in `thirdparty/opencv_yolox/`.

## Camera binding and coverage

`config/object_cameras.yaml` binds each camera ID to its source topic, optical
frame, mount, CameraInfo, registered depth and optional measured calibration.
`config/object_perception_deployment.yaml` selects the required camera IDs.
The default is `front`; front and wrist are image-only. Astra's registered depth
belongs to Astra. Changing an image topic cannot silently inherit Astra identity
or depth. Calibration paths must be relative tracked files. Aliased image
sources, duplicate optical frames and calibration identity mismatches fail at
startup; runtime captures with the wrong frame are rejected.

`camera_sha256` fingerprints the binding and calibration content. Changing a
binding requires a new matching Fiber profile even if its camera ID is unchanged.

One recognizer fairly schedules the newest pending capture of each selected
camera. One collector owns the database and global services. All selected
streams are required: a healthy camera cannot mask a missing required camera.
Select additional cameras only after their coverage and aggregate processing
budget are qualified; the two-Hz setting is a total inference limit, not two Hz
per camera. Do not increase front resolution without measuring optical quality,
transport, processing latency and the camera's corresponding calibration.

## Freshness and stationary evidence

Sensor-compatible KEEP_LAST depth one, separate ROS callback groups and a
two-thread executor let receipt continue while one inference runs. At most one
frame per camera and one inference are pending. Newer frames replace pending
ones; they do not create a processing backlog. Both start and completion check
capture freshness, with the existing two-second limit. Clock changes clear
pending captures and invalidate in-flight results. The collector rechecks age
after image encoding and geometry work.

`/diagnostics` exposes per-camera received/published/rejected/overwritten counts,
source/output age, inference duration and rolling p95, pending frames and the
operating-point identities. Query/status expose each camera's ingestion age and
readiness. ROS launch supervises process shutdown; under severe CPU/memory pressure an
in-flight Torch operation may require its existing termination escalation.
A stale or overloaded pipeline reports unavailability rather than
pretending that an empty historical result is current.

Each collected observation has a `stationary` flag derived from read-only
`/wheel/odometry` feedback at capture time. The preceding window must cover at
least 0.25 s without gaps over 0.15 s, ending within 0.10 s of capture. Speed must
remain within 0.01 m/s and 0.03 rad/s, displacement within 5 mm and yaw change
within 0.02 rad. Arm-mounted cameras additionally require complete joint feedback
stable within 0.01 rad. Missing, malformed or stale feedback makes the flag false.
This does not change torque, arming or any motion state.

Moving observations remain useful search candidates. They cannot complete a
goal: the verifier requires two stationary, fresh, distinct captures of the same
unambiguous candidate within the admitted confirmation window (at most two
seconds), with matching camera/frame/dimensions, consistent boxes and compatible
metric positions when requested. It re-reads authoritative records and pinned
JPEG hashes before its existing atomic completion transaction. Repeated frames
do not make systematic recognition errors independent.

## ROS contracts

| Interface | Contract |
| --- | --- |
| `/robot/perception/<camera>/detections` | `vision_msgs/Detection2DArray`, original capture header, source-pixel boxes, native label as `class_id`, at most 20 by score. Persistent track IDs stay empty. |
| `/robot/perception/<camera>/source_image` | Canonical BGR image, exact same capture header and no overlays. |
| `/robot/observations/events` | Schema, immutable observation ID and sequence; a wake-up hint, not authority. |
| `/robot/observations/query` | Bounded cursor/ID metadata query, `streams`, aggregate `stream_ready`, epoch and cursor-gap state. |
| `/robot/observations/evidence` | ID-only JPEG lookup with SHA-256; no filesystem path accepted. |
| `/robot/observations/pin` | Idempotent goal-owned evidence reservation within existing quotas. |
| `/robot/observations/status` | Requires all configured camera streams to be fresh and healthy. |

The collector uses ROS `message_filters.TimeSynchronizer` for exact image and
detection stamps. A standard ROS detector can replace the recognizer without
replacing storage, services or Fiber. Its source binding and model identity must
be configured truthfully.

Queries accept `{"after_sequence":0,"limit":20}`. Subsequent cursors include the
returned `epoch`; a new database requires restarting from its head. ID queries
use `{"observation_ids":["<64 lowercase hex characters>"],"limit":20}`. Inputs
are capped at 4 KiB, metadata at 64 KiB, and evidence at 384 KiB before base64.
`latest_sequence` is a live-consumer admission head; `next_sequence` paginates
history and does not prove freshness.

`capture_id` hashes camera/frame/stamp. The observation `id` hashes the capture
ID with `model_sha256`, so one capture recognized under two operating points
yields two coexisting observations. Same-content replay is idempotent; different
content for the same observation ID fails explicitly. Metadata and JPEG are
one private SQLite transaction. A process lock prevents competing writers.
Journals are separated by ROS domain and simulated-time mode. Retention is
bounded by seven days, 100,000 records and the configured storage quota. Pins
preserve admitted evidence until their bounded expiry; quota pressure cannot
silently evict pinned evidence. Expired or corrupt evidence cannot verify a goal.

## Independent evaluation

Use `ros2 bag record --topics` for permitted camera input and read-only odometry
and joint feedback. Replay only on an **isolated ROS domain**, using
`ros2 bag play --clock` and `scripts/object-perception.sh use_sim_time:=true`.
Do not record command topics or replay a bag into the live robot graph.
Simulated-time records cannot satisfy real-robot goals.

Prepare a private manifest; images stay outside Git:

```json
[
  {"scene_id":"view-001","scene_group":"arrangement-session-001",
   "split":"development","image":"frames/mug.jpg",
   "objects":[{"label":"cup","bbox":[42,89,185,144],"blue":false},
              {"label":"bowl","bbox":[300,120,90,60]}]},
  {"scene_id":"view-002","scene_group":"arrangement-session-002",
   "split":"heldout","image":"frames/negative.jpg","objects":[]}
]
```

Labels are the detector's native labels. The manifest's vocabulary is every
label it uses, so label every visible instance of every vocabulary class in
every scene: absence is scored as a true negative. Predictions of classes
outside the vocabulary are reported as `unscored_predictions`, not false
positives. `blue` is optional per object. Earlier manifests with
`"cups":[{"bbox":…,"blue":…}]` remain valid and declare the label `cup`. Keep adjacent frames, crops and alternate cameras from
the same arrangement/session in the same scene group and split. The loader
validates both splits, rejects group leakage, repeated bytes and identical
decoded pixels, and checks that images did not change after preflight. Group
names cannot themselves establish physical independence; review provenance.

```bash
source scripts/setup.bash
scripts/evaluate-object-recognition.py /path/to/scenes.json \
  --model-name rtdetr_v2_r18 --output /path/to/report.json
```

`--split development`, `--confidence` and `--blue-fraction` support operating-point
selection before held-out evaluation. Freeze the selected values before opening
the held-out results. Reusing a held-out set to tune a model invalidates it.
Non-OpenCV models automatically run in the installed perception environment.

Schema 3 reports, per vocabulary label, **class detection** and **false
unique-target successes**; for labels with `blue` annotations it adds
**blue-attribute errors on matched objects** and a **`blue <label>` goal**, with
per-image boxes, hashes, timings and scene-group confidence bounds. Missing the
cream mug counts as a cup false negative even though it is not a blue cup. A
single detected blue cup in a truly ambiguous scene counts as a false verified
outcome, not a successful unique-object request.

`screening_passed` requires at least 80 held-out groups and, per goal, 20
positive, 20 negative and 10 ambiguous groups, sufficient matched blue and
nonblue examples when blue is annotated, measured
precision ≥98%, recall ≥90%, zero observed false unique successes and p95
processing ≤1.5 s. `passed` additionally requires the lower 95% scene-group Wilson
bounds to meet the precision/recall targets for every class and blue-goal
metric. Correlated frames or objects do not add independent confidence trials.
Eighty scenes are an initial screen, not proof of 98% operational precision.
Development-only or insufficient datasets produce a report and exit nonzero.

## Color, geometry and completion scope

`blue_fraction` is a fixed HSV measurement of the interior 20–80% of a detected
box, kept because Fiber's current cup/blue request requires it. It is an
attribute of the observation only; geometry does not use it. It is not calibrated probability and deliberately excludes much of a rim.
A cream mug with a blue rim is a cup positive, not automatically a blue-cup
positive. The API supports `color: null` for an unrestricted cup request only
when the frozen profile explicitly enables `allow_any_color`. The UI exposes
this distinction; omitted color retains the existing blue-cup request.

Metric mode requires a reviewed schema-1 calibration marked `validated` and
`depth_registered`, matching camera/frame, measured intrinsics, distortion,
raster correspondence and dimensions. Uncertainty limits remain 0.10 m camera,
0.20 m map and 0.02 s clock; depth/CameraInfo must match capture within 50 ms.
The ROS depth/CameraInfo caches cover the configured inference-age window at
the tracked 30-Hz depth rate. Depth must be actual registered 16-bit surface
depth within the calibration's measured `depth_range_m: [min, max]` (Astra:
0.55–3.0 m). A floor projection cannot substitute for the object's surface.

Geometry samples the interior 20–80% of the box for any object, independent of
color, and keeps a robust depth cluster; dispersed samples or insufficient
consensus remain image-only. Calibrations with `sampling: blue` are rejected. It estimates a visible surface, not object center or grasp readiness.
Front/wrist have no registered-depth binding and cannot silently return metric
positions. Map results additionally require capture-time TF and compatible
approved map identity/epoch. Never copy synthetic test calibration onto hardware.

Every observation also records `map_pose`, the capture-time camera pose with
the site, map, revision and localization session it was solved in, or
`map_pose_unavailable_reason`. [Spatial memory](spatial-memory.md) anchors and
reprojects it after SLAM revises the map.

## Release order and remaining physical gates

Build and deploy the compatible ROS producer first, install the selected model
runtime, then deploy Fiber's matching verifier and reviewed profile. The profile
must freeze `camera_sha256`, the full `model_sha256`, color permissions,
confirmation window and policy revision. Old records without stationary/binding
proof remain history and cannot complete a new goal.

The software and retained-mug regression do not replace independent held-out
qualification, measured geometry, the stationary physical canary matrix or
moving-search acceptance. Keep promotion disabled until the applicable stage
passes. Outcomes distinguish verified, ambiguous, unverifiable and not found
within inspected views; a detector miss never proves global absence. Existing
Nav2 safeguards, evidence authority, scoped approvals and deployment re-arm
behavior remain in force.
