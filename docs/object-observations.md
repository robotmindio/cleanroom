# Stationary object observations

This is the ROS foundation for implementation slices 0–1. It detects cup
candidates and measures the fraction of blue pixels in each candidate's
interior region. The ROS collector does not identify a permanent object, resolve
a user goal or authorize motion. Calibrated metric results are optional, as
described below. Multiple cups remain separate candidates; an empty result
means no detector candidates in that capture.

## Install and launch

Build the ROS package using the repository build procedure, then install the
one pinned model and start perception alongside the existing camera stack:

```bash
scripts/install-object-model.py
scripts/object-perception.sh
```

The installer verifies the exact upstream model size and SHA-256 and preserves
an existing verified copy. It never downloads weights during ROS startup or
installs a different OpenCV/NumPy. All parameters live in
`config/object_perception.yaml`; perception is explicitly started and has no
motor publisher, service or action client. Model artifacts stay outside Git.

The recognition node reuses the Apache-licensed
[OpenCV Zoo YOLOX implementation](https://github.com/opencv/opencv_zoo/tree/47534e27c9851bb1128ccc0102f1145e27f23f98/models/object_detection_yolox).
Its CPU backend is a baseline to evaluate, not a claimed site-qualified model.
The vendored wrapper includes a compatibility fix for OpenCV versions without
batched nonmaximum suppression; it still suppresses boxes separately by class.

## ROS interfaces

| Interface | Contract |
| --- | --- |
| `/robot/perception/detections` | Standard `vision_msgs/Detection2DArray`, original capture header; axis-aligned boxes in source pixels and class confidence. Detection track IDs remain empty because no persistent tracker exists. |
| `/robot/perception/source_image` | Canonical `sensor_msgs/Image`, exactly the same capture header; original pixels without overlays. |
| `/robot/observations/events` | Small String JSON containing schema, immutable observation ID and persisted sequence. A wake-up hint; query is authoritative. |
| `/robot/observations/query` | `QueryObservations`: bounded JSON cursor/ID query; metadata only, including `stream_ready`, database epoch and cursor-gap indication. |
| `/robot/observations/evidence` | `GetObservationEvidence`: ID-only lookup returning JPEG MIME type, SHA-256 and at most 512 KiB base64. No filesystem path accepted. |
| `/robot/observations/status` | `std_srvs/Trigger`: success requires fresh synchronized captures and a healthy latest ingestion. Historical records never establish live readiness. |

The collector uses ROS `message_filters.TimeSynchronizer`, matching the source
image and standard detections by their exact stamps. It is reusable with another
ROS detector: remap its `image` and `detections` inputs and configure the actual
model hash/camera ID. No detector-specific code is in the collector or store.
Freshness is checked against the capture stamp at ingestion and status time.
The existing `/robot/capabilities` catalog advertises these read interfaces;
its observation `available` field means service discovery, while the status
service and query `stream_ready` field establish capture freshness.

Example query:

```json
{"after_sequence":0,"limit":20}
```

Subsequent queries carry the returned `epoch` and `next_sequence` as
`after_sequence`. A changed epoch requires a new initial query. ID queries use
`{"observation_ids":["<64 lowercase hex characters>"],"limit":20}` instead of
a cursor. Inputs are capped at 4 KiB; metadata responses at 64 KiB. The format
preserves all candidate boxes, scores and `blue_fraction`; that fraction is a
fixed HSV measurement, not a calibrated recognition probability.
`latest_sequence` reports the persisted head so a new live consumer can begin
there without reading all historical pages. Historical pagination uses
`next_sequence`; it is distinct from a freshness or completion check.

Every observation declares `grounding: image_only`, `position: null`, source
stamp/frame, camera ID, model hash, dimensions and evidence hash. The capture
identity hashes camera/frame/stamp. A replay returns the existing sequence;
different content under the same capture identity is rejected. Metadata and
JPEG are written in one private SQLite transaction, avoiding file/DB orphan
races. Journals are partitioned by ROS domain and simulated-time mode, and a
process lock prevents competing writers.

Retention evicts oldest observations at seven days, 100,000 records or the
configured payload quota. SQLite page allocation is additionally bounded by
`max_bytes`; transaction journals can temporarily add disk use. Evidence that
has been evicted returns an explicit not-found/expired error. These records can be
pinned by admitted Fiber goals through the pin service described below.
Unpinned historical observations remain immutable while retained.

## Recorded evaluation

Use existing ROS tooling to record camera input for replay on an **isolated ROS
domain**, with no motor/control topics in the recording. Use `ros2 bag record`
and `ros2 bag play`; do not create another recorder or replay framework. Select
the original RGB topic; perception consumes standard images and detections.
For simulated-time playback start the perception script with
`use_sim_time:=true` and play with the rosbag clock. Never replay a sensor bag
into the live robot graph.

For held-out recognition evaluation, prepare a local manifest of independent
scene setups, split before tuning. Images stay outside Git:

```json
[
  {"scene_id":"setup-001","split":"heldout","image":"frames/setup-001.jpg",
   "cups":[{"bbox":[120,80,60,100],"blue":true}]},
  {"scene_id":"setup-002","split":"development","image":"frames/setup-002.jpg","cups":[]}
]
```

Boxes are `[x,y,width,height]` in original pixels. Label every visible cup and
its blue attribute, including distractors and ambiguous multiple matches.

The evaluator validates images and boxes in both splits before inference. It
rejects repeated file content and identical decoded pixels, including copies
with different filenames or lossless encodings. This prevents exact-image
leakage between development and held-out sets. It cannot establish independence
between adjacent video frames: use separately arranged scene setups and split
before tuning. Each report includes per-scene predictions, expected blue-cup
boxes, error counts and processing time so failures can be inspected. An image
changed after preflight invalidates the evaluation.

```bash
source scripts/setup.bash
scripts/evaluate-object-recognition.py /path/to/scenes.json \
  --output "$HOME/.ros/lekiwi/recognition-evaluation.json"
```

The evaluator shares production inference and color measurements. It rejects
duplicate scene IDs and invalid boxes, verifies model hashes, matches labeled
targets once at IoU ≥0.5, and records true/false positives, misses, confidence
intervals, processing latency and artifact hashes. Exit zero requires all
proposed initial gates: at least 80 held-out scenes (20 positive, 20 negative,
10 ambiguous), precision ≥98%, recall ≥90%, and p95 processing ≤1.5 seconds.
The ≥0.35 blue-fraction decision is an initial evaluation threshold to validate
on the development split. This benchmark cannot qualify metric position or
physical task success, and synthetic ROS fixtures do not establish its accuracy.

## Next gate

The package build, 138 selected unit/ROS/release/qualification tests, isolated
CTest registration, and repository Python/shell checks passed during development.
A saved Astra image also passed the complete pinned-model ROS replay on domain
232; five local inference runs took 0.37–0.45 seconds on this workstation. This
checks execution only and is not the recorded-scene accuracy evaluation.

The software foundation has unit and isolated ROS service tests. The controlled
recorded-scene corpus and accuracy acceptance are outstanding; a single existing
camera image can check inference execution but cannot close that gate. Complete
this evaluation before implementing Fiber goal completion or moving search.
Metric grounding follows measured calibration/time/frame acceptance. See the
[implementation plan](robot-intelligence-implementation-plan.md).

## Goal evidence and metric grounding

The completed Fiber consumer lives in `fiber-core`'s company plugin, under
`robot/`, with the authenticated `/robot` API and Fiber UI `/robot` page. See
[implementation report](robot-intelligence-report.md) for deployment order.
Generic goal APIs cannot complete a robot-owned goal; the verifier re-reads ROS
metadata, requires two fresh distinct unambiguous captures, checks pinned JPEG
hashes and atomically commits the result with the goal status.

`/robot/observations/pin` accepts `PinObservationEvidence` JSON with a stable
`request_id` and 1–20 immutable `observation_ids`. Repeating the same request is
idempotent; changing its IDs fails. Pins reserve evidence within existing
storage quotas for the configured retention interval. Quota pressure rejects
new captures instead of evicting reserved evidence. An ID query with
`pin_request_id` returns the actual reservation expirations. Expired pins are
pruned; no pin can request an arbitrary path or extend retention indefinitely.

Perception is integrated into bringup. `config/object_perception_deployment.yaml`
sets its release default, disabled initially. Its calibration path is relative
to the tracked package config directory; an empty path produces image results.
The existing install/deploy scripts consume these defaults through bringup.
The standalone perception wrapper remains useful for replay and diagnostics.

Metric mode requires a reviewed measured YAML/JSON calibration, schema 1,
`validated: true`, `depth_registered: true`, `camera_id`, `optical_frame`,
`rgb_size`/`depth_size`, measured `rgb_k` (9 values), `rgb_p` (12), `rgb_r` (9),
`rgb_d` (distortion), and `rgb_to_depth` (3×3 raster correspondence). Registration
is provided by the existing Astra ROS driver (`depth_registration: true`), not
inferred from a homography between different optical viewpoints. Include
measured `camera_uncertainty_m`, `map_uncertainty_m` and `clock_uncertainty_sec`.
Their gates are 0.10 m, 0.20 m and 0.02 s respectively. Do not copy synthetic
unit-test calibration values onto hardware.

The collector matches depth and CameraInfo within 50 ms, checks dimensions,
frame identity and all measured intrinsics, and uses `image_geometry` to project
a robust registered depth sample from blue pixels. Accepted depths are
0.55–3.0 m. The result estimates a **visible surface**, with uncertainty; it is
not a grasp pose or permanent object identity. Missing or incompatible geometry
leaves the image observation intact. Map positions require capture-time TF and
a capture after the current approved map epoch began. Historical TF failure
leaves a camera position with an explicit map-unavailable reason.

`/robot/map_context` publishes the validated map-manifest hash, epoch and
activation time. `/robot/navigation/odometry` provides a read-only cumulative
travel counter and epoch. Stale samples, gaps and resets invalidate travel
accounting. Optional navigation `deadline_unix_sec` and `max_travel_m` bounds
are persisted by the existing task gateway and enforced locally. They cancel
only the admitted owned task, retain pending termination status, and preserve
existing safety/ownership handling and deployment re-arm behavior.
