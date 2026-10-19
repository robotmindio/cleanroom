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

The installer follows the local model selected in `config/object_perception.yaml`,
which is the fallback when the primary is a remote provider.
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
artifacts, backend, input size, threshold, preprocessing revision, label
output (`native`, or `query` for open-vocabulary recognizers) and actual runtime
package versions. It is not just the weight-file hash. Model,
threshold or runtime changes require a new admitted profile identity. The
`--describe` output supplies the exact model and camera identities to put in a
reviewed Fiber profile. Old weight-only profiles must not be reused.

The fixed comparison choices are `yolox`, `rtdetr_v2_r18` and
`grounding_dino_tiny`. Install an alternative with `--name NAME`. The latter is
an offline open-vocabulary diagnostic that localizes the active query (below);
its labels are the matched query phrases and its scores are not
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

## Remote GPU provider and local fallback

`config/object_perception.yaml` selects `grounding_dino_tiny_gpu`: the same pinned
Grounding DINO weights as `grounding_dino_tiny`, served by
RobotMind's `perception-provider` on the cluster GTX 1070. The endpoint lives in
`config/perception_providers.yaml` (Tailscale address, 1.5 s timeout) and is not
part of any identity. The recognizer sends lossless PNG pixels plus the frame's
query phrases and thresholds, and accepts a response only when it echoes the pinned
`provider_model_sha256` (the provider's hash of its weight files, Torch/CUDA and
transformers build, and GPU), the exact effective prompt and thresholds, and the
SHA-256 and size of the image it was sent. Boxes then go through the same
suppression as the local Grounding DINO. Measured on the cluster, GPU and CPU
results for one image agreed to 1e-4 in score and 0.01 px.

`fallback_model_name` (`rtdetr_v2_r18`) serves frames while the provider is
unreachable, busy, not ready, too slow, or answering outside its pinned identity.
After a failure, frames skip the primary for 10 s, so an outage costs one timeout
per window. Fallback frames go to `/robot/perception/<camera>/fallback/...`, and
the collector records them under the fallback's own `model_sha256`, so every
observation names the recognizer that produced it. `--describe` prints both
identities. A Fiber profile that admits both accepts either source; one that
admits only the GPU identity ignores fallback observations. Diagnostics report
`served_by`, `fallback_published` and the primary failure reason, and warn while
the fallback is serving. A local CPU Grounding DINO is not a useful fallback:
at 15–19 s per frame it cannot meet the 2 s freshness limit.

A provider rebuild that changes Torch, CUDA, transformers or the GPU changes its
hash. Until `provider_model_sha256` and the Fiber profile are updated together,
the fallback serves every frame.

## Open-vocabulary query

An open-vocabulary recognizer localizes the **active query**, which is runtime
state owned by the detector, not a parameter. Clients set it with
`/robot/perception/set_query` (`lekiwi_rmf/srv/SetPerceptionQuery`):

- Labels are NFKC-normalized, lower-cased and whitespace-collapsed, then
  de-duplicated in order. A label may contain only letters, digits, spaces,
  hyphens and apostrophes; `.` is Grounding DINO's phrase separator.
- At most 16 labels of at most 48 characters, and a prompt (`a. b.`) of at most
  254 characters, so Grounding DINO's 256-token text limit never truncates it.
- Empty labels restore the tracked `default_query` from
  `config/recognition_models.yaml`. The default is not part of `model_sha256`.
- `expected_revision` applies the change only while that revision is in force,
  so a client restoring its predecessor never clobbers a newer query.
- The response returns the query in force and the one it replaced; a rejection
  leaves the query unchanged and says why. Closed-vocabulary primaries reject
  every query.

The query's revision is the SHA-256 of its canonical labels (plus parts and
concepts when present). Each inference snapshots the query and the proposal
profile before it runs, and every frame is published with a capture-stamped
`lekiwi_rmf/msg/PerceptionOperatingPoint` on
`/robot/perception/<camera>[/fallback]/operating_point`: model SHA-256, profile
document, query document, the SHA-256 of the input artifact, per-detection
evidence, the provider run reference and, for fallback frames, why the primary
was unavailable. The collector synchronizes it exactly with the image and
detections and accepts it only if the profile proves its revision and the query
is canonical. A query or profile change during inference therefore never
relabels the frame that was already running. The observation records
`vocabulary: open` and `query` (revision, labels, prompt), and its `id` is
`sha256(capture_id:model_sha256:profile_revision:query_revision)`. The same
capture under another query or profile is a distinct inference. Fallback frames
are recorded as `vocabulary: closed` without a query, with
`sha256(capture_id:model_sha256:profile_revision)` and a `degraded` reason: they
never claim to answer a query they cannot express. `/robot/perception/profile`
republishes on every change, with the primary's `labels` equal to the labels and
concept names in force, and `/robot/capabilities` advertises the services under
`observations.query` and `observations.proposal_profile`.

A label may declare the whole it is a part of (`part_of`, parallel to `labels`).
A part detection is tied to the smallest detection of that whole containing at
least 80% of it (`part_of: {index, label, containment}`); without one it stays
unassigned (`part_status: no_referent`). States such as on, hot or held are not
visual labels and are not produced here.

## Proposal profiles

Thresholds, suppression, resolution and the per-frame budget form a **proposal
profile**, separate from the model identity (`lekiwi_rmf/perception_profiles.py`).
`model_sha256` pins weights, runtime and input normalization only. Each backend
specifies its parameters with type, range, unit, cost (`postprocess`,
`inference` or `retention`) and meaning; a parameter a backend cannot honour is
refused with its reason rather than ignored, and a profile must be complete.
`config/perception_profiles.yaml` holds the tracked profiles; every recognizer
starts on `default`. The tracked `standard` profiles keep the previous operating
point: 0.50 score or box threshold, 0.25 text threshold, NMS IoU 0.50 (per label
for closed catalogues, across the prompt for open queries, keeping suppressed
phrases as `alternatives`), at most 20 detections, Grounding DINO shortest edge
800, and a provider proposal floor of 0.30 for replay.

`/robot/perception/apply_profile` (`lekiwi_rmf/srv/ApplyPerceptionProfile`)
switches one recognizer (`role` primary or fallback). It requires the
`expected_revision` in force and exactly one source: a tracked `profile_name`,
`revert_to_revision` from this detector's history (the exact earlier revision),
or `parameters_json` with the `evidence_id` of a calibration study that selected
exactly those parameters for this recognizer. Before the swap the detector runs
the candidate once on a synthetic capture on the same recognizer; a remote
provider must echo the candidate's parameters. A failed probe leaves the
previous profile active and publishes nothing. The response reads back the
profile in force, the profile topic republishes, and `profile_applied` is
logged only after the swap. Search coverage credits only frames whose profile
revision is the one its session admitted (`ignored_other_profile` otherwise).
Runtime ROS parameter changes remain rejected; a persistent default is a
tracked change to `config/perception_profiles.yaml`.

## Capture artifacts and replay

Every recognizer consumes the frame's stored evidence: the detector encodes the
capture once as JPEG quality 80 (`capture_artifact`), sends those bytes to a
remote provider and decodes them for local models; the collector encodes the
same pixels to the same bytes and records `evidence.model_input` when they match
the frame's input digest. A stored observation, or the same bytes stored
byte-exact in Knowledge, is therefore a replayable capture.
`scripts/replay-perception.py RECORD ARTIFACT --model-name NAME` re-runs it under
the record's own operating point (and reports `reproduced`) or under another
`--profile`, `--profile-json` or `--labels`, which produces a new inference ID on
the same `capture_id`. It writes a report and never touches the robot or the
store. The remote provider additionally keeps bounded proposals and images for
its own `replay` and `evaluate` operations by `provider_run.run_id`.

## Abstention

Each observation records `input_quality` (mean luma, Laplacian sharpness and
`dark`, `overexposed` or `blurred` flags) and typed `abstentions`, each with the
next information that would answer it: `no_candidate` (another viewpoint or a
memory query), `low_quality_input` (light, glare or a steadier view),
`geometry_unavailable` with the depth reason (`no_depth_calibration`,
`depth_out_of_sync`, `calibration_mismatch`, `too_few_valid_depth_samples`,
`incoherent_depth`, ...; a depth observation) and `provider_unavailable` on
fallback frames. `ambiguous_identity` is Knowledge association's outcome, not a
per-frame result. An abstention is never resolved by lowering a threshold: an
untracked profile needs a selected study.

## Calibration studies and learned concepts

`scripts/calibrate-perception-profile.py MANIFEST --model-name NAME --candidates
CANDIDATES.json --output REPORT` freezes the corpus hash, splits, objective
(macro F1 over labels), regression rule (no label F1 or recall, and no condition
with at least five scenes, may drop by more than 0.02) and a seeded paired
bootstrap over scene groups. It searches the candidates on development, then
evaluates the current profile and the best candidate once on held-out. It
selects only when the lower 95% bound of the held-out macro-F1 gain is above
zero and nothing regresses. The content-addressed record, with per-label and
per-condition results for every candidate, goes to
`~/.ros/lekiwi/perception/studies/<evidence_id>.json`; apply a selection with
`parameters_json` and that `evidence_id`, and revert with the recorded
`revert.revision`.

`scripts/learn-perception-concept.py MANIFEST` teaches a new category from
byte-exact views, such as Knowledge appearance-gallery region observations and
their source artifacts, without retraining the detector. Taught positives and
negatives become exemplar embeddings from the provider's encoder. Held-out
positives and negatives calibrate the match threshold so that no held-out
negative is admitted. Views that cannot be separated are refused, because that
concept needs a reviewed dataset and a trained detector instead. The concept
(name, prompts, encoder identity, exemplars, match) is stored by revision under
`~/.ros/lekiwi/perception/concepts/`. Name it in `set_query` as
`concept_revisions`: the provider proposes regions for its prompts and keeps
only those that match its exemplars, labelled with the concept name and its
similarity evidence. Only the remote recognizer can match concepts.

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
| `/robot/perception/<camera>/fallback/{detections,source_image}` | The same contracts for frames the configured fallback recognizer served; recorded under its own `model_sha256`. |
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

`--split development`, `--profile NAME`, `--profile-json` and `--blue-fraction`
support operating-point selection before held-out evaluation; the calibration
study script automates that protocol. Freeze the selected values before opening
the held-out results. Reusing a held-out set to tune a model invalidates it.
Non-OpenCV models automatically run in the installed perception environment.
Scenes may carry `conditions` tags; the report counts them and lists the hard
conditions (`dark`, `overhead`, `occlusion`, `transparent`, `reflective`, `blur`,
`similar_instances`) the corpus does not cover yet. These are accuracy results
on real models; the unit tests use controlled backend outputs and prove only
that every label, profile and concept crosses the system intact.

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
