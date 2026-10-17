# Robot intelligence implementation and strategic completion report

2026-10-14. Scope: the original plan's blue-cup deliverable and software slices
0–4. Slice 5 is the separately qualified expansion backlog. No Astra agent was
used for implementation. No production policy, deployment or robot motion was
changed during this work.

## Delivered

| Stage | Implementation | Acceptance status |
| --- | --- | --- |
| Contracts and fixtures | Schema 1 observations, strict camera/model/clock/geometry contracts, immutable evidence IDs, typed Fiber requests/profiles/runs, rejection and replay fixtures. | Software checks pass; physical corpus remains open. |
| Recognition and evidence | Pinned OpenCV Zoo YOLOX, standard `vision_msgs`, exact source synchronization, blue attribute, bounded SQLite/JPEG store, cursor epochs, authenticated on-demand evidence and quota-bounded pin receipts. | CPU inference and ROS replay work; recognition accuracy is unqualified. |
| Complete stationary journey | Existing Fiber Goal/Execution/subject ledger and Temporal integration; retry-safe admission, one active goal per robot, observe→clarify→verify, owned cancellation, admin override and dashboard/MCP callers. | Fixed software journey, persistence, API, verifier and UI tests pass. Deployed canary remains open. |
| Scoped completion policy | Generic status changes cannot complete robot-owned goals. The trusted verifier re-reads concrete ROS metadata, pin receipts and JPEG hashes, checks live policy/revision/cancellation/termination and atomically commits evidence plus status. Planning/plan writes retain existing behavior. | Contract and race-fence tests pass; profile promotion requires normal release authority. |
| Metric grounding | Existing Astra driver registration, ROS camera geometry and capture-time TF; calibration hash, strict measured intrinsics/registration/time gates, robust depth samples, camera/map uncertainty and map epoch checks. | Synthetic geometry/failure tests pass; no hardware calibration is claimed. Defaults remain image-only. |
| Bounded attended search | At most five surveyed views, last-sighting ranking, existing Nav2 planner/gateway, stable owned task IDs, nonrefundable route reservations, measured odometry, local deadline/travel enforcement, candidate-triggered cancellation and fresh post-stop verification. | Software ownership, cancellation, telemetry and deadline tests pass; moving trials remain unqualified and search is disabled. |
| Release integration | Tracked disabled perception deployment settings, bringup inclusion, ROS install/dependency declarations, exact Fiber allow-lists, disabled profile example, operator UI and deployment/rollback guide. | Reviewable changes prepared; compatible ROS must deploy before Fiber. |

The world catalog keeps bounded sighting facts, provenance, surveyed places and
freshness/map compatibility. It does not infer permanent named-object identity.
An old sighting may rank existing approved views; it cannot satisfy a new goal.
Catalog saturation is explicit rather than silently discarding facts.

## Policy decision implemented in source

Adopt **automatic verifier-backed completion only for explicitly admitted robot
objectives**. Each goal freezes robot/site/model/calibration/map/policy/scopes
and has a separate status owner. Existing company goals and generic plan writes
retain their current policies. Every physical mutation still passes Fiber's
existing action grants, L1/L2 handling, durable approvals and idempotency.

Cancellation is persisted before reconciliation. A queued physical submission
is rechecked at the actual policy stage. A lost reply, missing task or accepted
cancel never proves termination. The robot locally enforces requested deadline
and travel bounds through its existing task cancellation path. No automatic
disarm or new global motion-state policy was added; deployment re-arm is intact.

An administrator override records actor/reason/revision and remains distinct
from sensor success. Withdrawal disables admission or revokes a policy revision;
existing owned work reconciles rather than being forgotten or resubmitted.

## Strategic epics still required

| Priority / epic | Concrete remaining work | Definition of done / owner |
| --- | --- | --- |
| P0 — E9 recognition qualification | Collect permitted held-out single, negative and ambiguous cup scenes across approved lighting/viewpoints; run the shipped evaluator against the pinned release. | At least 80 independent held-out scene groups for screening, ≥20 positives, ≥20 negatives, ≥10 ambiguous; precision ≥98%, recall ≥90%, p95 ≤1.5 s. Qualification additionally requires lower 95% scene-group confidence bounds to meet the precision/recall targets, with zero false unique successes for both generic and blue cups. Perception/site owner accepts statistical report and known limits. |
| P0 — E4 metric qualification | Measure RGB/depth registration, intrinsics, optical transform, clock error and camera/map uncertainty with independent references; commit the reviewed calibration and hashes. | Proposed camera error ≤0.10 m, map error ≤0.20 m and clock uncertainty ≤0.02 s; invalid geometry never yields a metric success. Calibration/site owner. |
| P0 — E10/E11/E14 stationary release | Bind the actual Fiber tenant/robot/private connector, promote disabled perception/profile via the release process, check authenticated UI evidence, duplicate/reconnect/restart/cancel canaries and retention policy. | Repeatable stationary goal with inspectable authoritative evidence; unauthorized or fabricated evidence cannot close it; deployment identities recorded. Fiber/release/product owners. |
| P0 — E1/E2/E4/E7/E8/E12/E13 moving acceptance | Qualify the chosen map and 3–5 surveyed views; run attended blocked-route, stale perception, map/odom reset, restart, approval timeout and cancellation trials with independent power stop. | Correct scoped findings; bounded measured travel/time; native termination proof and unchanged local safeguards across the scenario matrix. Robot/site/release owners. |
| P1 — E10 world/catalog operations | Qualify usefulness of the bounded sighting catalog, saturation handling and stale-map presentation against real site usage. Establish evidence access/retention ownership before persistent household collection. | Operator can distinguish recent, stale and incompatible-map sightings; saturation and expiration are visible. Product/site owners. |
| P1 — E3/E5 qualified inspect/approach | Add only after geometry and stopping gates: approach envelope, viewpoint/visibility criteria and separate skill outcome predicates. | Independent collision/clearance and inspection acceptance; locating a cup alone grants no manipulation readiness. Motion/product owners. |
| P2 — E6 grasp/carry/place and broader vocabulary/sites | Introduce separately qualified capabilities and distinct grasp, retention and placement evidence; expand recognizer/site profiles using measured datasets. | Each capability has its own physical acceptance and verifier. No trajectory or detector claim substitutes for object-in-hand or placement proof. Manipulation/perception owners. |

Sequence: recognition qualification → stationary release → measured metric
release → attended search release → inspect/approach → manipulation. Software
for later stages exists behind explicit configuration so these gates can be
completed individually without inventing evidence or enabling an unqualified stage.

## Verification and practical limits

Verification recorded on the implementation branches: **255 ROS tests**,
**179 Fiber tests**, and **17 UI/i18n tests** passed. Strict mypy checks for the
robot modules/API, targeted Ruff/ESLint, UI TypeScript, repository-configured
ament_flake8 and three registered perception CTest targets passed.

The ROS package builds locally. Tests exercise standard messages and real ROS
services on an isolated graph, storage retention/quota/pins/replay, calibrated
geometry and capture-time TF failures, odometry resets, gateway deadline and
admission bounds, bringup defaults and tracked launch snapshots.

Fiber checks cover strict typing, existing goals/connectors/MCP regressions,
tenant isolation, CAS/revocation/cancellation, authoritative evidence/pins,
Temporal sandbox loading, a complete fixed stationary workflow using real
persistence, and dashboard retry/reconnect/i18n behavior. Transport fixtures and
synthetic geometry are software verification, not physical accuracy evidence.

No live cluster, production database, robot or camera was modified. The required
held-out dataset, measured calibration and attended/deployed acceptance evidence
were unavailable. Promotion remains disabled until those specific epics pass.
See [object observations](object-observations.md) and the adjacent Fiber
`fiber-core/docs/ROBOT-GOALS.md` for runtime contracts and deployment order.


## Recognition qualification follow-up — 2026-10-15

The live Astra RGB topic has a publisher and supplied a fresh capture. A bounded
read-only probe using the pinned CPU model processed one live capture in
approximately **0.746 seconds**; its age at receipt was **0.170 seconds**.
This checks the actual input/inference path. It is one sample, is not p95 latency,
and provides no recognition accuracy estimate without ground-truth labels.
No robot motion, live perception enablement or goal-policy promotion occurred.

No labeled held-out scene manifest was found in the inspected project and ROS
artifact directories. Recognition acceptance therefore remains open. The next
required input is a permitted labeled dataset, or physically arranged stationary
scenes for capture and independent labeling: single blue cup, no blue cup,
multiple blue cups and representative distractors/lighting/viewpoints.

The evaluator now rejects repeated image bytes and identical decoded pixels
across both dataset splits, validates development-set image/box boundaries too,
pins the evaluated manifest/image content and rejects image changes during a
run. Reports include per-scene predictions, reference boxes, error counts and
processing times. Exact duplicate detection does not establish independence
between adjacent video frames; scene setups and split provenance still require
review. Dataset-integrity regressions use synthetic unit fixtures and do not
count toward the real recognition corpus.


The follow-up observation/evaluation suite passes **37 tests**; targeted Ruff
and repository-configured ament_flake8 checks pass. On the inspected live ROS
graph, the existing task submit/get/cancel services are present, while the new
observation and navigation-accounting services are absent. Fiber PR #5402 has
merged and ROS PR #80 remains open. Source merge status does not establish a
compatible deployed consumer. The stationary canary must follow recognition
acceptance and the compatible ROS/Fiber release, with no ad-hoc production
feature enablement used to bypass these gates.


### User-arranged cup visibility trial

A user-arranged cup was inspected while the robot remained stationary. Three
fresh 640×480 Astra captures produced zero cup detections; visual inspection
showed that the cup was outside that camera's view. The front camera (320×240)
showed a transparent cup close enough to crop its rim/base; the wrist camera
(352×288) showed a partial view. This establishes a camera-coverage limitation
for the current Astra-only perception default, not evidence that the object was
absent from the robot's surroundings.

The pinned model also produced zero accepted cup detections on the saved front
and wrist views at the unchanged 0.50 threshold. A separate diagnostic pass on
the front image scored cup at 0.127 and vase at 0.407. These rejected scores do
not establish a reliable cup classification. RGB conversion and letterboxing
were checked against the pinned upstream OpenCV Zoo demo; no preprocessing
mismatch was found. No runtime threshold or camera selection was changed.

This is one scene with correlated captures, not an independent held-out corpus.
Its color ground truth remains unconfirmed. The next trial needs a fully visible
cup in the selected camera and a blue positive example for the scoped blue-cup
deliverable. Camera coverage/source selection must be qualified before promoting
a profile. Private images and detailed capture reports remain in the local ROS
qualification directory, outside Git; only aggregate findings are recorded here.

### Repositioning readiness and recovery findings

The requested backward repositioning exposed a broken LAN control route. The
existing compute-service installer now selects the reachable tailnet address
and the active release workspace (`lekiwi_ws/current`). Selecting the parent
workspace initially loaded its obsolete simulation installation; this was
corrected before sending any motion command. Real driver feedback, wheel
odometry and all three camera streams were then observed. The relayed connection
still produced intermittent stale-feedback faults and is not qualified for motion.

Base permission remained denied because the measured shoulder lift was about
0.037 rad outside the accepted travel-stow reference (0.020 rad tolerance).
MoveIt planned a bounded correction, but the motor host rejected execution:
its saved August calibration differs from compute's September SO-101 mapping.
Joint feedback showed no correction, and no base command was sent. Neither
calibration was overwritten. Restarting the motor service cuts torque, so
physical arm support must be confirmed before that recovery step. The collision
monitor was also not active while navigation waited for map readiness.

Add a **P0 split-runtime readiness epic** ahead of moving acceptance: establish
the authoritative measured arm mapping on both computers, check calibration
agreement before admitting trajectories, preserve active-release workspace
selection during service recovery, and qualify a connection that continuously
meets existing freshness limits. Resolve collision-monitor availability for
guarded manual positioning when map initialization is pending. Acceptance needs
fresh actual feedback, confirmed travel stow, active collision protection and a
measured bounded reposition followed by a verified stop. These findings do not
justify widening tolerances, disabling interlocks or changing goal policy.


### Bounded physical repositioning follow-up

The saved compute SO-101 arm mapping was validated with the existing calibration
reader, copied atomically to the stopped motor host with its previous mapping
backed up, and read back for agreement. The motor service restarted successfully.
This synchronizes an existing mapping; it is not a new physical calibration.
MoveIt then planned and executed the small travel-stow correction successfully;
measured shoulder lift reached -1.7952 rad against the -1.7967 rad reference.

A finite backward command through `/cmd_vel_manual`, the existing mux and Nav2
collision monitor produced **0.0380 m backward wheel-odometry displacement**.
The client required fresh driver/permission/scan/odometry feedback, an active
collision monitor, transformed rear clearance and bounded distance/heading.
It paused on stale inputs, sent zeros afterward and confirmed fresh zero wheel
velocity. Relayed-network delays remain an operational limitation; this one
positioning operation does not qualify moving search or stopping performance.

Fresh post-stop frames arrived from all three cameras with source ages
0.097–0.138 s. At the unchanged 0.50 confidence threshold, the pinned model
returned no cups (CPU inference 0.435–0.603 s per frame). Visual inspection showed
a table leg ahead and no cup in these views. These latest frames differ from the
earlier cup scene; no current cup finding or recognition success is claimed.

A subsequent bounded viewing-angle check turned approximately **4.25°** according
to wheel odometry, with 2.7 mm translation, then stopped at its deadline as
freshness/clearance gates interrupted further motion. Its requested 14.3° view
was not reached. This partial check is not a completed object search.


### Patterned-mug physical trial — 2026-10-17

The user supplied a mug in the test environment. Initial live images showed a
cream mushroom-patterned mug with a blue rim: cropped in front and visible from
above in the wrist camera. The default Astra view faced elsewhere. Five guarded
backward positioning steps accumulated **0.1993 m by wheel odometry**. Two small
turns adjusted the front view. Each operation reached its bounded target and
confirmed a stop; the final front images show the whole mug, handle and base.
Final telemetry showed ARMED, travel stow, no supervisor faults, normal collision
monitor state and zero wheel velocities. No thresholds or motion guards changed.

The pinned recognizer returned **zero accepted cups in three final frames**,
captured at least 0.5 s apart with receipt ages 0.031–0.058 s. Maximum cup anchor
scores were **0.328, 0.297 and 0.301**, below the unchanged 0.50 gate. An earlier
fully visible view reached 0.437, also rejected. The RGB conversion and
letterboxing match the [pinned upstream demo](https://github.com/opencv/opencv_zoo/blob/47534e27c9851bb1128ccc0102f1145e27f23f98/models/object_detection_yolox/demo.py).
This establishes a concrete false-negative case after resolving framing; it
is not a successful detector finding or verifier-backed goal completion.

Final saved-frame CPU inference took 1.096–1.985 s; earlier captures in this
session took up to 4.375 s while workstation load was high. These are diagnostic
measurements, not deployment p95 acceptance. Frames were processed by the shared
`detect()` implementation in a probe; this does not demonstrate the deployed
node's end-of-inference freshness gate. All images, hashes, exact source stamps,
per-frame scores and bounded-motion reports remain in the private local ROS
qualification directory. Adjacent frames and repositionings remain **one scene**.

**Next E9 work:** retain this labeled development failure, compare recognizer
and threshold choices against an independently held-out corpus, qualify the
front/wrist coverage needed for near objects, and measure latency in the actual
service resource allocation. The isolated mug score does not justify lowering
the production gate. The physical viewpoint stage is complete for this scene;
recognition acceptance and compatible stationary goal deployment remain open.


## Robustness implementation — 2026-10-17

All five software stages from the recognition review are implemented. The model
comparison selected RT-DETRv2-R18 at 384×384 / 0.50 as the development operating
point. In all three retained mug frames it detected the cup at 0.693–0.706;
YOLOX at 0.50 detected none. Warm CPU processing was about 0.89 s; the first
measured frame took 6.76 s. RT-DETR 640 and Grounding DINO exceeded the latency
budget; NanoDet missed this mug. These are development comparisons of one scene,
not independent acceptance samples or a reason to promote the model.

| Stage | Complete software | Evidence still required |
| --- | --- | --- |
| Evaluation | Separate generic-cup detection, blue-attribute and generic/blue unique-goal outcomes; grouped splits, duplicate protection, screening and confidence-bound qualification. | Independent, provenance-reviewed held-out scene groups across lighting, negative and ambiguous cases. |
| Operating point and camera contract | Pinned artifacts and isolated CPU runtime; full operating-point identities; explicit front/wrist/Astra sources, frames and calibration fingerprints. Obsolete weight-only constants and duplicate package installation removed. | Per-camera coverage and latency under the deployed resource allocation. |
| Bounded fresh inference | Standard ROS messages, latest-only per-camera inputs, fair scheduling, separate receipt/inference callbacks, capture/completion freshness, clock-reset invalidation, diagnostics. | Sustained deployed load, camera failures and overload acceptance. |
| Stationary goal verification | Capture-time odometry and wrist-joint proof; same-candidate confirmation within two seconds; authoritative camera/model/evidence checks. Generic cups require explicit profile opt-in; blue remains the default. | Compatible ROS/Fiber tenant canaries and the stationary physical acceptance matrix. |
| Multi-camera and geometry | One store/API, per-camera required readiness, correctly bound depth and opt-in robust generic-cup surface sampling. | Independent measured calibration, metric error bounds and surveyed moving-search trials. |

The compatible producer must precede the consumer rollout. Profiles freeze the
new camera and full model fingerprints, color permission, confirmation window
and policy revision. Legacy observations without stationary/binding proof stay
historical. Explicitly revoke incompatible admitted policy revisions. The
software changes preserve existing goal ownership, atomic completion, scoped
authority, Nav2 safeguards and deployment re-arm; no autonomous motion-state
policy was added. Production perception and example profiles remain disabled.

The remaining epics are qualification and rollout: **P0 independent recognition
qualification**, **P0 stationary tenant release**, **P0 physical metric
calibration**, then **P0 attended moving-search acceptance**. Expanded vocabulary,
approach and manipulation remain separate product capabilities. Complete each
applicable acceptance gate before promoting its stage; image-only recognition
does not establish a grasp target or prove absence outside inspected views.

### Final software verification

The compatible ROS package builds. **200 targeted ROS tests pass**, including
perception, immutable storage/services, camera/geometry boundaries, launch
validation, all bringup snapshots and service installation. The Fiber change
passes **27 robot tests**, strict typing on all five robot modules, **three UI
tests**, TypeScript and targeted ESLint. Scoped Ruff and shell checks pass.

A standard ROS bag of front image/CameraInfo and read-only joint/odometry input
was replayed through the installed detector and collector on isolated domain
210. The real query service returned cup observations with the expected full
model/camera fingerprints and capture-time stationary proof. An evidence-service
JPEG matched both its returned and immutable metadata SHA-256. The status service
returned unavailable after playback stopped, including with the simulation clock
paused. Repeated playback exercises clock resets. Freshness rejections under
workstation load remain visible; the replay is an integration check, not a
latency benchmark or a real-goal canary. Simulated records cannot satisfy real
goals. No robot motion or production deployment occurred in this robustness
implementation. Private raw captures, bag, labels and result reports are retained;
disposable session scripts were removed.

Final lifecycle checks also received twelve diagnostics updates and four evidence
events across repeated clock resets on isolated domain 211. A regression prevents
diagnostics from taking the ROS clock lock while holding the frame lock. Under
this overloaded host, detector shutdown still required the
existing ROS launch SIGKILL escalation after SIGINT/SIGTERM; clean detector exit
under deployed resource allocation remains part of lifecycle acceptance. The
collector exited cleanly. A further regression retains 30-Hz Astra depth and
CameraInfo across the full allowed inference-age window, preventing valid metric
samples from being evicted before delayed detections arrive.


### Live verification follow-up — 2026-10-17 12:27 UTC

**Operational acceptance has not passed.** Earlier test/replay success did not
establish the deployed robot-to-Fiber goal journey. The live graph was running
the older release with no observation producer. A temporary read-only producer
canary was launched using this repository's script and selected configuration,
then stopped; production startup configuration remains disabled.

The live query and evidence services worked: fresh front-camera observations,
correct model/camera fingerprints, `ros_system_time`, stationary proof and
matching JPEG SHA-256. However, visual review showed the mug in a very dark image
and the detector returned **zero cups**. Mean grayscale was 2.55/255 and 99.89%
of pixels were below 32/255. This is a retained live development failure, not a
successful finding or proof of absence. No threshold/preprocessing change or
verified goal completion is claimed. The raw evidence and metadata are private
under `~/.ros/lekiwi/qualification/live-cup-verification-20261017.*`.

The idle detector and a fresh active recorded-stream diagnostic run exited
cleanly on SIGINT at the current lower host load. The prior forced-termination
result remains valid for its heavy-load conditions; a general shutdown deadlock
has not been established. Captured active stacks showed Torch convolution during
inference. Neither normal-load shutdown nor successful unit tests resolve the
heavy-load lifecycle acceptance gap. Diagnostic processes were stopped.

The next recognition stage is to retain this dark-scene failure, qualify the
required lighting/exposure envelope and evaluate any proposed operating-point
change against independent scenes. The compatible deployed Fiber goal journey
also remains unverified. Software implementation is delivered; the real robot
capability must not be described as complete or accepted.
