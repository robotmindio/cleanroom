# Robot intelligence implementation plan

Proposed 2026-10-14. This is a delivery plan for E9–E14 in
[product readiness](product-readiness.md), grounded in this checkout and a
read-only inspection of the adjacent Fiber source. It proposes APIs, policies
and thresholds; none is installed or physically accepted by this document.

Implementation status: software for slices 0–4 is implemented across cleanroom
and the adjacent Fiber repository, using existing ROS navigation, camera
registration, TF and Fiber governance. See the [implementation report](robot-intelligence-report.md)
for delivered contracts, verification and remaining acceptance epics.
Recognition accuracy, measured metric calibration, deployed canaries and attended
moving-search acceptance remain open. Both perception and robot profiles are
disabled by default. Slice 5 remains the explicitly separate expansion backlog.

## First deliverable

Deliver **“Find the blue cup and show me where it is” from a stationary robot**.
A user submits the request through Fiber, receives a durable goal ID, and sees
either a fresh image with the matching cup highlighted, multiple candidates to
choose from, or a precise unresolved result. The first result may locate the
cup within the image: “The blue cup is here in the Astra view; metric location
is unavailable.” It must not imply a room or metric pose that was not measured.
After calibration passes, the same result gains a measured camera-relative
position and, when localization is valid, a position on the approved site map.

Completion requires two distinct fresh captures, a resolved target, accessible
evidence and a deterministic verifier decision. User reconnection must recover
the same goal and evidence. Repeating a request with its original idempotency
key must not create another run. This slice includes recognition, minimal world
state, existing Fiber goals, a fixed observe/resolve/verify workflow and the
operator result view. A detector demo alone is not the deliverable.

The next release adds **bounded attended search among surveyed observation
poses**, using existing managed navigation. Manipulation follows separately
after E5 physical qualification. Neither stationary observation nor moving
search changes arming policy, adds automatic disarming or adds inhibition
markers. Preserve the deployment script's existing verified-deployment re-arm.

## What exists and what the plan must connect

| Inspected source | Existing behavior and implementation consequence |
| --- | --- |
| `lekiwi_rmf/camera_relay.py`, `astra_cloud_filter.py`; `launch/pi_astra.launch.py` | Canonical RGB/depth transport preserves capture headers. Use these topics; do not use camera/NVR display timestamps as capture time. |
| `config/astra_pro.yaml`, `config/astra_cloud_filter.yaml` | Astra requests 640×480 RGB and 320×240 registered depth; image transport is capped independently at 2 Hz, compact cloud at 10 Hz with stride 8. Equal optical frame names and registration flags do not prove calibrated pixel correspondence. |
| `launch/slam.launch.py` | SLAM has approximate synchronization with a 0.35-second maximum interval. This is not an accepted object-grounding tolerance. Front-camera projected depth is not interchangeable with measured Astra depth. |
| `lekiwi_rmf/task_gateway.py`, `task_journal.py`, `srv/*Task.srv` | Navigate/explore already have durable IDs, one-task admission, action ownership, pause/cancel/restart handling and conservative unknown outcomes. Keep this execution boundary. |
| `lekiwi_rmf/map_bundle.py` | Approved bundles have map identity and hashed artifacts. Link metric observations and approved viewing poses to that identity; a ROS frame named `map` alone is insufficient. |
| Fiber `src/fiber_core/goals/repository.py`, `db/model_domains/learning.py` | Tenant-owned goals/subgoals, 32,000-byte plan bound, plan compare-and-swap and separate progress already exist. Goal kinds are objective/key_result/workstream; do not invent an unsupported `robot` kind. |
| Fiber `src/fiber_core/goal_layer.py`, company `goals/{layer,blackboard,context,writeback}.py` | One installed goal-layer provider; company plans normalize to schema v2 and accept extra fields leniently. Add a validated robot extension within this provider. Agent-authored plan JSON is not trusted execution authority. |
| Fiber company `routers/goals.py`, `temporal/build_activities.py` | Ordinary top-level creation/closure is operator-managed; subgoal closure has a separate route. Trusted build finalization already changes goal status through the repository. A scoped, trusted robot completion path fits this pattern, but must enforce stronger evidence conditions. |
| Fiber `db/query_domains/subjects.py`, `db/model_domains/subjects.py` | Tenant-owned JSON documents with revision CAS, idempotent attempt claims and durable budgets already exist. Reuse them for a small world catalog and run records. Their helpers commit separately: do not assume two helper calls form an atomic transition. |
| Fiber integrations `connectors/rosbridge.py`, `routes.d/robot.yaml` | Named task operations and generic incoming `rosbridge.message` events exist. Current workflow ends at admission. There is no inspected robot observation query, evidence retrieval, semantic ingestion or sensor-backed completion operation. These are necessary implementation work. |

Fiber paths below are relative to `/home/nex/robotmind/fiber-core`. Its deployed
plugin set, role grants, credentials, UI repository and artifact service were
not verified. No suitable general household-image store was established by the
bounded source inspection; the plan therefore specifies a working bounded
evidence path rather than assuming one exists. The shared knowledge-client
subject-reference grammar accepts open-world keys, so `robot:<id>:…` references
can use current subject persistence without a grammar migration.

## Ownership and module changes

Keep perception beside ROS sensors and execution; keep semantic meaning,
user goals and orchestration in Fiber. Use one selected model, one perception
node and one fixed workflow. Add no graph database, separate goal scheduler,
general planner, fleet coordinator or model-backend framework.

| Owner/repository | Concrete work |
| --- | --- |
| Perception / cleanroom | New `lekiwi_rmf/object_perception.py`: validate camera inputs, run selected recognition, associate short tracks, ground usable depth and publish observation summaries. New `object_observations.py`: bounded observation/evidence persistence and queries. Split only the ROS-free parsing/geometry functions needed for direct replay tests. |
| ROS integration / cleanroom | New `srv/QueryObservations.srv`, `srv/GetObservationEvidence.srv`; versioned JSON payloads following the existing task-service pattern. Add launch/config/install entries in `launch/bringup.launch.py`, `config/object_perception.yaml`, `CMakeLists.txt`, `package.xml` and tracked deployment/profile scripts as needed. Add perception readiness to the capability response without changing motion capability availability semantics. |
| Integration / Fiber integrations plugin | Extend `connectors/rosbridge.py` with named read operations `rosbridge.observations.query` and `rosbridge.observations.evidence`, bounded responses and exact allow-lists. Update `routes.d/robot.yaml` and manifest/role declarations. Implement validated event ingestion and reconnect catch-up. |
| Robot domain / Fiber company plugin | Small `robot/` package for strict contracts, world queries, find workflow activities and deterministic verification; a scoped `routers/robot.py` for goal admission/cancellation/result/evidence. Extend the existing goal-layer context/validation, rather than registering a competing provider. Register activities through the existing plugin mechanism. |
| Persistence / Fiber core where necessary | Reuse goals and subject-document APIs. Add only the shared transactional helper needed to atomically fence robot run transitions and goal status. Keep core goal-model migrations in core's Alembic chain if a later implementation genuinely requires schema changes; initial records fit existing storage. |
| Operator product / Fiber UI owner | Submit a goal, see observation/search progress, choose between candidate thumbnails, cancel, reconnect and inspect timestamped evidence. Render verified completion distinctly from admission or operator override. Locate the actual UI owner/repository before its implementation PR; it was outside this review. |
| Site/release integrators | Pin camera calibration, model artifact, map/viewing-pose bundle, inference runtime, policy revision and evaluation evidence in tracked release inputs. Complete E1/E4 before moving-search acceptance. |

No goal metadata needs to be injected into navigation parameters. Fiber stores
the foreign-key-like link to the existing task ID; the ROS task journal remains
the authority on physical execution. Stationary observation steps have an
observation request ID and **no managed motion task**. The skill description
must distinguish a read service from a managed motion capability.

## Contracts and storage

### Observation and evidence

Publish versioned, bounded metadata on `/robot/observations/events`; expose
authoritative retrieval at `/robot/observations/query`. Use `request_json` /
`response_json` plus explicit success/error fields in the new query service.
The strict query schema includes robot/camera scope, observation IDs or a
producer-epoch/cursor, optional capture-after boundary and a capped result
limit. Reject incompatible combinations and unknown fields. A query can return
zero valid observations; that is distinct from camera/inference failure.

An observation contains:

- `schema_version`, `observation_id`, producer epoch and monotonic sequence,
  robot/camera IDs and capture identity. Re-delivery retains the same ID.
- Original RGB and depth ROS stamps, clock domain, capture-time UTC mapping
  with measured clock uncertainty, receive/inference times, frame IDs and
  image dimensions. A repeated cached frame is one capture, not new evidence.
- Class, per-attribute values/scores, bounding box in original RGB pixels,
  optional mask, selected model/weights/preprocessing hashes and calibration ID.
  Scores are model outputs; they are not automatically calibrated probabilities.
- Track ID scoped to producer epoch, candidate object ID if association is
  supported, and association ambiguity. A track ID is not permanent identity.
- Grounding status (`image_only`, `camera_position`, `map_position`, `invalid`),
  explicit invalid reasons, measured 3D point and uncertainty when available,
  contributing depth count/spread, transform provenance and map epoch/version.
  Unavailable position fields are null; never substitute zero coordinates.
- Evidence reference and hash, availability/expiry, and crop coordinates.
  Retain the unannotated source image for assessment; generate overlays without
  overwriting it. Household images and free text stay out of routine logs.

Use a separate private SQLite observation spool under the persistent ROS data
directory, partitioned by robot/domain/simulation, following task-journal
locking/migration conventions. It owns immutable selected observations and
bounded evidence files, **not goals or the semantic world**. Live inference
uses a newest-frame queue and a short RAM ring. Persist at most one representative
record per track per second plus selected verification captures; images are
persisted on goal use, not as unrestricted continuous video. The query can
freeze captures from the ring using a stable request ID; a lost reply retrieves
the original frozen selection. Return `evidence_expired` if its retention has
elapsed, rather than relabel a new image with the old observation ID.

Proposed first limits: 4 KiB query input, 100 observations per response, 20
detections per frame, 64 KiB per metadata response; last 30 seconds in RAM;
7 days/100,000 records and 2 GiB for local selected evidence, whichever limit
is reached first. Active-run evidence is pinned within the same hard quota;
if the quota cannot retain required evidence, return an explicit storage
failure and do not claim completion. Test atomic file writes, orphan cleanup,
disk-full behavior and restart recovery. Retention is configured in tracked
files, with deletion/tombstone behavior visible to Fiber.

For the first deployable evidence path, fetch a selected image by immutable ID
using `/robot/observations/evidence` and the named Fiber connector operation.
Cap an encoded response at 512 KiB; reject path arguments and oversized images.
Fiber serves it to the authenticated UI through its tenant-checked evidence
route, verifying the hash and content type. This is a low-rate, on-demand path,
not a video stream. Extend connector response-size limits only for this named
operation. If an established private artifact service is confirmed during
implementation, use it behind the same opaque evidence-reference contract;
do not introduce a new public camera server or embed base64 in goals/events.

### Minimal semantic world

Fiber owns one bounded versioned site catalog in a subject document under
`robot:<robot_id>:site:<site_id>`, key `robot_world_v1`, using existing revision
CAS. Start at at most 100 candidate objects and 20 operator-approved places /
viewing poses; enforce serialized size and stop admitting additions when full.
This bounded scan is adequate for one site. Move to indexed relational records
only when actual catalog size or query consumers require them.

Each object record holds an ID, label/attributes, identity confidence,
last-seen observation references, last-known pose with map/calibration identity,
freshness, and visibility/contradiction state. Each place holds an operator
name, provenance, approved bundle version and zero or more validated base
viewing poses. Preserve user naming separately from inferred spatial relations.
Store `observed_at` and approved place membership first; do not infer rooms
from free-form model text.

Association uses temporal continuity and measured spatial/appearance evidence;
merge only when it is unambiguous. Two blue cups remain separate candidates.
After a long occlusion or producer restart, return a possible match rather than
assert permanent identity. A new conflicting location is recorded with its
evidence; stale/contradicted records are not silently overwritten as certainty.
One missed detection does not prove absence or movement. Map replacement
invalidates previous map positions/viewing poses unless a reviewed map-to-map
transform is explicitly supplied. Preserve image evidence and last-known facts.

Persist only selected/changed facts and goal evidence in Fiber. Generic
rosbridge events are wake-up hints: the importer validates the integration's
tenant/robot identity, deduplicates by observation ID, reads authoritative
metadata through the query operation and advances a durable cursor. Reconnect
backfills within the spool's retention; expired cursors produce a visible gap.
Never trust tenant or robot identity asserted inside incoming JSON. Events,
labels and OCR-like scene text are data, never instructions or tool grants.

### Goal, plan and execution linkage

Use the existing Fiber goal with `kind=objective` (or an existing parent's
`workstream`) and a small `robot` extension inside company plan schema v2.
Strictly validate this extension before execution even though generic plan
writes remain lenient. Keep within the existing 32,000-byte plan cap.

Trusted admission freezes a robot run specification into a subject document
`robot:goal:<goal_id>`, key `robot_run_v1`. Include:

- Goal/run IDs, tenant/robot/site, parent if any, original request and validated
  target selector (`class=cup`, `color=blue`), resolved object ID or unresolved
  candidate set, and resolution/clarification history.
- Plan revision and input hash, workflow ID, allowed skills, location mode
  (`image` or `metric`), success predicate and policy/verifier versions.
- Original deadline, view/attempt/travel limits, allowed area and motion mode;
  consumed effort, cancellation generation and current step/revision.
- Each step's stable ID, exact admitted parameters, idempotency key, managed
  task ID where applicable, task status snapshot and observation/evidence IDs.
- Verification result, reason, actor, evaluated-at time and immutable evidence
  references. Agent-authored narrative cannot overwrite these measured fields.

The agent-facing plan is a proposal/summary; the trusted admitted specification
owns execution bounds. A proposed plan change must validate and create a new
revision. It cannot enlarge the admitted scope/budget without an authorized
goal change. Pin old step/task links when revising; never derive a fresh task
ID merely because a process restarted or a plan summary changed.

Reuse Temporal's durable workflow state and subject attempt claims. Derive a
task ID once from run ID + stable step ID + admitted attempt, store it before
calling `rosbridge.tasks.submit`, then submit exactly those parameters. On a
lost reply, query/resubmit **the same ID** according to the existing idempotent
contract. Reconcile `unknown`; never turn it into permission for a new task.
Persist counters before dispatch and use non-refundable claims for search
limits; avoid the ledger's fingerprint-based budget reset feature here.

Serialize run ownership per robot using the existing durable workflow/claim
pattern. Initially a competing goal gets `robot_busy`; do not build a priority
queue or preempt an unrelated goal. The gateway remains the final one-active-
motion-task arbiter. A robot-goal cancel always names its owned task ID; never
use the empty-ID convenience operation from a goal coordinator.

## Recommended goal policy change

**Allow authorized robot requests to create scoped goals and let a trusted
verifier automatically complete them when their admitted evidence predicate
passes.** Requiring an operator to manually mark every correctly verified
find request done would leave the intended product journey unfinished.
Keep ordinary company goals under their current policy. This changes Fiber
goal policy, not ROS motion permissions or arming behavior.

Add named robot goal admission/cancel/result operations and an internal
verifier completion activity. Admission binds the authenticated caller,
tenant, configured robot and immutable run specification. A model cannot
obtain completion authority by adding `robot=true` to a generic plan. The
verifier uses a dedicated server identity/action grant, loads the observations
itself and evaluates the pinned deterministic predicate. Do not expose a
general-purpose “set goal done” tool to the planning model.

The completion predicate requires the admitted target resolution, distinct
post-request captures, freshness and identity/attribute thresholds, a usable
image region (and qualified pose if metric completion was requested), intact
accessible evidence, and no unresolved contradictory candidate. A moving run
also requires confirmed termination of its owned motion task. `unknown`, stale,
ambiguous, unavailable or wrong-map evidence never passes. The success result
states the observation time and supported location mode, rather than promising
the cup is still there indefinitely.

Implement status + verifier result as one fenced transaction: check tenant,
trusted run binding, expected run/plan/policy revision, current active state
and cancellation generation; persist the verdict/audit reference and set the
existing goal status to `done`. Current `update_goal_status` alone only checks
tenant and ID, so a new shared conditional transition helper is needed. Do
not compose separately committing subject-document and goal helpers and call
that atomic. Replayed identical completion returns the saved result.

Cancellation and completion contend on the same revision/row lock. A cancel
recorded first prevents later completion, requests cancellation of owned work
and remains `canceling` in run state until termination is confirmed. A valid
completion committed first remains done; later cancel returns its terminal
result. Unknown termination remains visible and blocks replacement work.
Run states can express ambiguity, not-found and canceling while using the
existing core statuses (`active`, `paused`, `done`, `abandoned`); document that
mapping in the API and retain the detailed reason in the run record.

Pin policy version/hash at admission and include it in every verdict. Ordinary
policy updates affect new runs; an explicit revocation blocks future completion
and dispatch under the revoked policy pending revalidation. It does not refund
search budgets or silently change an in-flight task's physical policy. Human
clarification may resolve a candidate but cannot convert stale evidence into a
fresh sighting. Human overrides must be labeled `operator_override`, with actor
and reason, and excluded from sensor-verified success metrics.

Implementation reach: company `routers/goals.py`, new robot routes/activities,
`goals/{context,blackboard,writeback}.py`, plugin registrations, manifest/action
grants, core goal transition helper, and API/UI projections. Trace and test
both generic goal update routes and agent writeback so they cannot silently
bypass robot completion rules; preserve documented administrator override.
Existing trusted build-finalization behavior must retain its intended scope.
Do not fix only the prompt at `goals/context.py`.

Migration: existing goals remain unchanged and gain no robot authority. Only
new trusted admission records opt into this policy; preexisting goals require
explicit adoption with validated scope. Rollout starts with stationary-only
role grants. Test unauthorized/cross-tenant creation and closure, forged run
bindings, generic status bypass, stale revisions, replay, policy revocation,
simultaneous cancel/complete and administrator override audit.

## Camera, clock and metric grounding gates

Start with Astra RGB as the single observation path because its measured depth
can support the next slice. Use front RGB as a separately evaluated fallback
only if the selected Astra view cannot see the demonstration area. Do not make
wrist-camera reliability a dependency of initial find-object work.

Before any metric claim, measure actual published image dimensions, encodings,
CameraInfo K/D/R/P, rectification/registration behavior and camera serials.
Check finite nonzero intrinsics, image-size agreement and depth units; reject
zero/NaN/Inf/out-of-range values. The configured RGB/depth size difference
requires a calibrated mapping between rasters. Do not assume resize-by-two is
correct because the dimensions have a two-to-one ratio. Do not index the
decimated/unorganized collision cloud with RGB pixel coordinates.

Use robust foreground depth from an eroded object mask when available; otherwise
use an evaluated interior region, rejecting mixed/background depth or inadequate
support. A median from a cup bounding box is not automatically the cup surface.
Declare the reported point's meaning (visible surface estimate, not grasp centre)
and combine measured depth, calibration and TF/localization uncertainty. Reject
grounding when that uncertainty exceeds the goal's tolerance.

Proposed starting gates: RGB/depth capture separation ≤50 ms, measured device /
compute clock uncertainty ≤20 ms, source age ≤2 seconds at ingestion, and TF
lookup at the **capture stamp**, not latest TF. These are engineering targets
to validate on the existing 2 Hz transport. Independent RGB/depth throttling
may produce too few matching pairs even when driver sync is enabled. Measure
accepted-pair rate; if necessary pair before compression/throttling at the USB
host in `astra_cloud_filter.py` rather than relaxing tolerance silently. That
transport change must also preserve SLAM and existing camera tests.

Resolve camera→base→map at capture time with recorded transform provenance.
If the historical transform is missing or localization is invalid, retain
image/camera evidence and withhold the map pose. A camera-mounted-on-arm path
later needs time-matched joint/TF history. Keep mount/calibration offsets in
`urdf/lekiwi.urdf.xacro`; leave vendored CAD geometry intact.

Record real/simulated clock domains and startup epochs. A backward ROS-time
jump invalidates pairing/freshness buffers. An uncertain clock mapping cannot
be repaired by replacing the capture time with receipt time. Local monotonic
durations and durable Temporal deadlines enforce budgets; ROS simulation time
must never reset a real physical search deadline. For an evolving SLAM map,
assign a map-session epoch and invalidate metric targets on a localization
reset or discontinuous map correction. The first moving release should use
an approved fixed map and surveyed viewing poses to avoid silently tracking
semantic coordinates through unconstrained map changes.

## Model selection and evaluation

Do not choose a named model from reputation or assume a VLM's confidence proves
recognition. First inventory actual compute host CPU/GPU/VRAM, existing inference
runtime, redistribution/license constraints and allowed image-processing
locations. No new cloud image transfer is assumed. Check candidate official
model documentation/licenses at selection time and pin exact weights/runtime.

Benchmark at most two realistic candidates: a compact cup detector with an
evaluated color-attribute rule, and one recognizer that directly supports the
needed attributes if the first fails. A VLM may help resolve descriptions or
propose candidates; it does not produce unmeasured coordinates or its own
success authority. Choose the simplest candidate meeting held-out requirements,
then implement only that backend. An extra dependency is justified by the
selected inference engine, not by speculative future model switching.

Build a site dataset from actual compressed camera frames: propose 200 distinct
scene setups, with 120 for development/threshold selection and 80 held out.
Split by scene/session/object arrangement, not neighboring video frames. Include
blue cup present/absent, other blue objects, other cup colors, multiple blue
cups, glare, dark/light scenes, partial occlusion and empty/unusable frames.
Annotate class, color, instance where knowable, bounding region, visibility and
reference 3D points for the calibrated subset. Freeze the split before tuning;
report both counts and confidence intervals, not only a percentage.

Proposed go/no-go targets on visible in-scope targets: precision ≥98%, recall
≥90%, and capture-to-observation p95 ≤1.5 seconds on the deployed compute host
at the configured rate. Attribute/ambiguity tests must show no forced single
match when two indistinguishable cups are visible. Evaluate localization over
at least 30 independently measured placements spanning the declared range;
propose p95 camera-position error ≤10 cm and approved-map position error
≤20 cm. If map accuracy fails, ship the accepted image/camera result mode and
keep metric completion unavailable. These are proposed product gates, not
claims about current hardware or sufficient accuracy for grasping.

Measure inference CPU/GPU memory and latency while Nav2, SLAM and collision
processing run. New perception must not worsen existing guard/sensor deadlines;
use latest-frame dropping, explicit resource limits and the existing protected
robot service allocation. Saturation produces a perception-unavailable result,
not a backlog of apparently fresh detections.

## Delivery slices and acceptance

All numeric limits below are **proposed defaults to agree before collecting
release evidence**. They may be tightened after measurement; loosening a gate
requires a recorded product/physical justification.

| Slice / epics | Smallest reviewable increment | Exit evidence |
| --- | --- | --- |
| 0: contracts and fixtures / E9–E14 | Freeze request/result/observation/run schemas; inventory host/runtime, gather permitted stationary frames, pin target vocabulary and retention; stub-free replay harness around real frames. | Versioned fixtures include single match, no match, ambiguity, missing depth, stale time, wrong map and malformed input; owners accept scope/defaults. |
| 1: stationary perception / E9 | Selected recognizer, source validation, observation spool, query/evidence services and recorded replay. Image-only results are complete and explicit. | Held-out recognition/latency gates pass; two source captures remain distinguishable; evidence survives a node restart; zero metric poses emitted on invalid calibration. |
| 2: complete stationary user journey / E10/E11/E12/E14 | Fiber ingress, bounded catalog, goal admission/policy, fixed observe→resolve→verify workflow, result/clarification UI and durable recovery. | “Find the blue cup” returns inspectable image evidence; goal completion is verifier-backed; duplicate/reconnect/restart/cancel trials pass. This is the first user-facing deliverable. |
| 3: qualified metric result / E4/E9/E10 | Measured registration/intrinsics/extrinsics/time checks and map epoch support. Add metric result only where uncertainty gates pass. | Camera/map error targets pass against independent references; wrong/stale TF, map change and invalid depth fall back without false metric claims. |
| 4: bounded attended search / E1/E2/E4/E7/E8/E12/E13/E14 | Ordered approved viewing poses, navigate→observe loop, conserved search budgets and candidate-triggered owned cancellation. | Attended site scenario matrix and fault/recovery trials below pass, with applicable release evidence and unchanged local guards. |
| 5: later expansions / E3/E5 and then E6 | More vocabulary/sites, target search during geometric exploration, qualified inspect/approach; only then grasp/carry/place and their distinct outcome predicates. | Each new physical capability has separate acceptance. A located cup is not a graspable target; a successful trajectory is not a confirmed placement. |

Suggested stationary workflow: admit and pin specification → obtain observations
captured after admission → query world/candidates → request clarification if
needed → obtain a second verification capture after resolution → run predicate
→ atomically complete or return unresolved. Propose a 15-second stationary
deadline, one bounded observation retry, and final evidence no older than
5 seconds when verified. Require two distinct captures at least 250 ms apart.
Two frames are temporal corroboration, not statistically independent model
votes. Record the assessment time when evidence is later viewed historically.
If clarification outlasts the observation deadline, return `needs_clarification`
without completing the goal. A later user selection starts a newly authorized
observation attempt with a recorded budget and fresh captures; it never revives
expired evidence or silently resets a moving search's admitted limits.

For image-only mode, absence in the visible view yields “not found in the
current view,” not “there is no blue cup.” A request for a known instance with
insufficient identity evidence yields clarification/unknown. Initial natural
language parsing only fills the strict cup/color/place selector and mode; an
LLM cannot insert an arbitrary tool, URL, pose or success expression.

## Moving search workflow

Begin only after E1's current predictive obstacle-response acceptance and E4's
approved attended site/navigation evidence are available. The first moving
search uses an approved fixed map with 3–5 named viewing poses and one task at
a time. Fixed-map profiles disable explore but retain managed navigate when
RMF does not own navigation; this is enough to deliver target-driven search.

1. Observe at the current pose. If unresolved, rank approved poses by valid
   last-known location, coverage of the requested place and unvisited views.
   Use a deterministic order/tie-breaker. Never use an object's centre as the
   base goal or a language model's invented coordinates.
2. Before each dispatch, check goal/cancel revision, remaining budget, current
   map/view-pose version and motion capability availability. Local ROS guards
   still decide whether motion executes. Capability discovery alone does not
   establish physical permission or sensor freshness.
3. Submit managed navigate with stored task ID/parameters. Poll authoritative
   status while using events for responsiveness. Observe during transit only
   as a candidate cue; final verification occurs after confirmed task stop and
   a fresh stationary capture.
4. When a candidate appears, request cancellation of the owned task, poll for
   terminal proof, then inspect/verify. Cancellation acceptance is not a stop.
   Unknown termination blocks replacement/approach and escalates visibly.
5. At each completed view, record pose, capture interval, observations,
   occlusion/coverage limitations and result. A failed/unreachable view counts
   as attempted effort, not as inspected visibility. If a candidate moved,
   reacquire or choose another approved view within the original limits.
6. Stop the search workflow on verified success, user cancel, exhausted budget
   or unrecoverable uncertainty. Cancel any owned active motion and wait for
   confirmed termination before reporting stopped. Preserve partial coverage
   and distinguish `not_found_within_scope`, `ambiguous` and `unverifiable`.

Proposed search limits: five views, five minutes wall time from admission,
10 metres accumulated navigation travel in the qualified area, one retry of
a failed view, and no automatic budget renewal. Persist effort and the original
deadline across pause/restart/clarification. The goal deadline can expire while
the existing native action's active-time budget is paused; the coordinator then
requests task cancellation through the normal API. Deadline expiry does not
prove physical stopping. Derive travelled distance from the established
odometry source with jump/reset validation; when its accounting is unavailable,
do not grant more travel. Initially use conservative pre-dispatch route-length
checks plus measured travel accounting; reject an invalid/unavailable estimate.
Use Nav2's existing `ComputePathToPose` action for route estimates through a
bounded named Fiber connector operation with an exact action allow-list. Record
the start pose/map version and validate the returned path before measuring it;
this operation computes a path and does not dispatch motion. Include its
connector, timeout and invalid-path cases in slice 4 instead of assuming the
current navigate/explore catalog supplies route lengths.

These goal limits do not replace existing navigate/explore bounds. New motion
steps after a connectivity outage require reconciliation and current scope;
an already admitted task follows its existing local behavior. Describe that
behavior in the operator UI; do not imply a lost Fiber connection immediately
stops the robot. No autonomous change to arm/disarm policy is introduced.

After this works, integrate target cues with the existing Explore action in a
qualified mapping profile. Persist original radius/duration, inspected views
and map epoch; reuse owned cancellation on a candidate. Geometric frontier
exhaustion must never be reported as proof that an occluded object is absent.
General autonomous viewpoint generation and object approach remain later work.

## Validation and release gates

| Layer | Required checks |
| --- | --- |
| Pure perception/contracts | Box/mask bounds; NaN/Inf and oversized/nested JSON; depth units and insufficient support; mismatched CameraInfo/resolution; duplicate capture; RGB/depth skew; wrong/future clock; TF lookup failure; pose uncertainty; model/calibration hash mismatch. Use recorded inputs and small deterministic geometry fixtures. |
| Persistence and ingestion | Duplicate/out-of-order events, cursor gaps, queue pressure, file hash mismatch, disk full, retention, DB migration/restart, concurrent CAS, cross-tenant evidence access and invalid integration identity. Wrong or missing evidence cannot complete a run. |
| Fiber orchestration/policy | Crash before/after run persistence, task admission and response; repeated idempotency key; changed arguments with same key; stale plan write; task `unknown`; native replacement; cancellation/completion race; policy revocation; budget preserved across restart; generic goal-status bypass; ambiguous resolution. Use existing Temporal replay/activity testing patterns. |
| Integration and operator UI | Stationary live query through configured connector and default tenant; access-scoped image display; wrong robot/tenant rejected; clarification selection rechecked for freshness; reconnect returns same goal/run/task; admission and physical completion visibly distinct. |
| Motion regression | Existing task gateway/journal/ownership, navigation/exploration pause/cancel/recovery and profile tests; no raw cmd_vel or arm API path from planner; no changes to independent stop, collision guard or deployment re-arm. |
| Physical acceptance | Repeat declared target scenes at approved poses; independently measure location; observe canceled/blocked motion and restart under supervision; record release/config/calibration/site/model/policy identities with results. Software and simulation do not close these gates. |

Propose at least 40 live stationary requests, including 20 negative/ambiguous
or fault cases, and 30 attended search trials including visible, relocated,
occluded, absent, distractor and blocked-route cases. Require **zero false
verified successes** in either set; report sample counts and confidence bounds
because zero observed failures is not a guarantee. Propose ≥90% completion on
visible, uniquely identifiable in-scope positive scenes; p95 stationary response
≤10 seconds excluding human clarification, with every request terminal or
explicitly unresolved by its 15-second budget.

For moving trials require every dispatch within approved area/budget, every
verified success linked to fresh post-stop evidence and zero duplicate motion
tasks across recovery. Measure detection-to-cancel-request latency (proposed
p95 ≤1 second at 2 Hz input), cancellation-to-confirmed-stop latency, completion
time, path length and searched views. The accepted physical stop-time/distance
limit comes from E1/E4 measurements and must be fixed before trials; do not
invent an emergency-stopping guarantee from workflow polling latency.

Also report wrong-identity rate, clarification rate, not-found coverage,
grounding availability, stale rejection counts, loss/reconnect recovery and
CPU/GPU/ROS timing under load. Run a proposed two-hour stationary endurance
session and then an attended search endurance session sized to E8's approved
pilot target. Tests must show `navigate=succeeded` plus absent cup does not
satisfy the find goal. Later manipulation adds missed grasp, slip/drop and
wrong placement to this same verification discipline.

## Deployment and rollout

1. Add disabled-by-default perception/profile configuration, install entries
   and pinned inference dependencies/artifacts. Keep inference dependencies
   isolated from ROS's working OpenCV/NumPy/cv_bridge environment where the
   selected runtime would otherwise conflict. Use existing compute service
   resource controls and immutable release tooling.
2. Deploy compatible ROS query/evidence services before Fiber consumes them;
   discovery reports unavailable until present. Deploy Fiber schema validators,
   named operations, exact allow-lists, routes and read-only grants, then
   stationary goal admission/verification grants. Use established tenant
   configuration and GitOps routes, not manual post-startup ROS settings.
3. Validate host bindings and authenticated evidence delivery; rosbridge stays
   on its accepted private/tailnet topology. The local observation store is not
   a public image endpoint. Record retention/access choices before collecting
   persistent household imagery.
4. Run stationary canary requests, held-out replay and recovery trials against
   the exact release. Promote metric mode only after calibration gates pass.
   Enable moving search in a separately tracked approved site profile after
   E1/E4/E7 and operator acceptance; retain existing L1/L2 policy behavior for
   each physical task admission and never bypass a configured durable approval.
5. Rollback prevents new robot goal dispatch under the withdrawn profile and
   reconciles existing owned tasks via the established cancel/status path.
   Preserve task journals, observation records and run links. Keep old payload
   readers during the compatibility window; fail clearly on unknown schemas.
   Do not delete evidence or replay motion to make rollback look successful.

Acceptance artifacts should include source and installed release identities,
model/runtime hashes and license record, calibration/camera serials, map bundle,
policy version, dataset split/evaluation and physical trial report. Use E1's
reviewed safety-relevant fingerprint to determine which physical evidence is
affected; a documentation change alone should not force new braking trials.

## Work that can proceed together

After slice 0 freezes contracts, three implementation tracks can run in
parallel: perception/replay plus observation services; Fiber contracts/storage,
policy and mocked fixed workflow; and camera/site qualification with dataset
capture. The UI can use the same versioned fixtures while those tracks join.
Contract fixture changes are coordinated across both repositories. Join at
slice 2 for the stationary end-to-end release; join physical site evidence at
slice 4. Leave estimates in slices rather than dates until host/site/owners are
confirmed; do not make image-only delivery wait for manipulation or new sensors.

## Decisions and blockers

The implementation can start with the proposed contracts, fixture set and
stationary replay. The following facts cannot be established by source alone:

| Needed decision/evidence | Proposed default and effect |
| --- | --- |
| Actual inference host/runtime and permitted image-processing location | Local compute host; no cloud image upload. Inventory resources before choosing weights or promising latency. |
| Meaning of “the blue cup” | Class+blue attribute with clarification for multiple matches; persistent named-instance recognition is not assumed. User supplies representative objects/scenes. |
| Evidence retention and viewers | 7 days/2 GiB, authenticated tenant/operator access, selected still images only. Owner approval is needed before persistent collection in a household/site. |
| Image result versus metric requirement | Ship highlighted image first, explicitly stating location limits. Metric completion is enabled after measured calibration; grasp-quality precision is a later requirement. |
| Site and attended operating envelope | One approved fixed map, 3–5 surveyed observation poses, operator at independent power stop; no moving trials until E1/E4 acceptance. |
| Goal-policy authority and UI ownership | Adopt scoped verifier-backed robot goal completion as recommended above; confirm platform owner and actual UI repository. No live policy update is performed by this plan. |
| Physical stopping and localization thresholds | Site/safety reviewer fixes limits from the real robot before moving trials. Proposed perception tolerances do not replace physical qualification. |
| Deployed Fiber contracts and artifact service | Verify plugin versions, default tenant and grants, Temporal worker registration, connector topology and evidence-display path. Use the explicit on-demand evidence path if no existing artifact service is suitable. |

No live detector benchmark, camera calibration, hardware motion, deployment or
Fiber policy mutation was performed for this plan. Model performance, physical
timing and deployed permissions remain unmeasured. The inspection establishes
implementation entry points and reuse opportunities, not acceptance of the
assembled robot.
