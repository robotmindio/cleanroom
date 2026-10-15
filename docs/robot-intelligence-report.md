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
| P0 — E9 recognition qualification | Collect permitted held-out single, negative and ambiguous cup scenes across approved lighting/viewpoints; run the shipped evaluator against the pinned release. | Proposed ≥80 held-out scenes, ≥20 positives, ≥20 negatives, ≥10 ambiguous; precision ≥98%, recall ≥90%, p95 ≤1.5 s. Perception/site owner accepts statistical report and known limits. |
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
