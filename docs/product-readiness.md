# Product readiness and delivery epics

Assessment date: 2026-10-13. Scope: this ROS repository and its tracked
integration contracts, assuming one physical robot, one managed task at a time,
and an operator at the independent motor-power stop. A bounded read-only review
of the adjacent Fiber goal/connector code and camera deployment defaults informs
the intelligence epics below. External deployment, the assembled hardware and
the surveyed site are outside this source review.

## Product decision

The next user-visible capability should be **“find the blue cup and show me
where it is,” with fresh supporting evidence**. Start with stationary observation,
then add bounded search within a qualified attended site. This exercises object
recognition, world state, user goals, planning and verification before grasping
adds contact and payload risks. A repeatable attended navigation/exploration pilot
supplies the physical foundation for that moving search.

The repository has substantial execution, recovery and safety software. It does
not yet connect user intent to recognized objects, semantic locations or verified
physical outcomes. It also does not establish an unattended household robot,
a qualified manipulation product, automatic charging or production fleet operation.
Those outcomes require separate implementations, acceptance and integrations.

The immediate release concern is the current collision prediction model:
[the safety record](safety.md#physical-acceptance) explicitly says its physical
obstacle-response check is pending. The older braking evidence remains useful
within its recorded conditions; it does not qualify this changed behavior.

## Capability inventory

| User outcome | Evidence in this repository | Incomplete work |
| --- | --- | --- |
| Drive autonomously under supervision | Reviewed base stopping record: dry concrete, reported 200 g installed load, folded stow, 0.25 m/s and 0.40 rad/s | Current predictive obstacle-response check and binding reviewed evidence to safety-relevant release inputs |
| Ask the robot to explore and observe progress | Bounded Explore action; managed task journal; pause, cancel and restart reconciliation | Physical room-scale acceptance and deployed Fiber/UI completion, failure and reconnect trials |
| Understand task parameters before submitting | Live Explorer defaults and additive capability descriptions on this review branch | External clients must consume the contract; `available` means profile enabled and action server discovered, not physical permission |
| Recognize and locate a requested object | RGB streams, camera information and depth transport | Detector/recognizer, object identity, calibrated 3D grounding and a tested observation contract |
| Know objects, rooms and named destinations | Geometric occupancy map and fixed RMF graph vertices | Semantic world state, object/place identifiers, last-seen evidence and map-version handling |
| Set a durable user objective | Adjacent Fiber core has persisted goals/subgoals and an optional goal-layer plugin contract | Robot-specific success predicates, constraints, target resolution and linkage to physical task outcomes |
| Decide and verify how to achieve an objective | Nav2 plans paths; MoveIt plans arm trajectories; managed gateway exposes navigate/explore | Multi-step robot skill planning, target-driven search and independent outcome verification |
| Execute arm motion reliably | MoveIt and guarded trajectories; one loaded HOME cycle at 0.10 scaling | Measured workspace coverage, intrusion stopping, payload retention/endurance and wrist-camera reliability |
| Localize and navigate precisely | Wheel odometry, lidar/RGB-D SLAM, calibration tooling and map validation | Measured Astra calibration, site accuracy and an approved surveyed bundle |
| Operate unattended | Software checks and independent physical motor-power stop | Near-field/surround coverage, required physical sensing and a separately reviewed unattended scope |
| Run a fleet or recharge automatically | Free Fleet integration and bundle admission checks | Approved map, ownership architecture, physical battery source and charging interface/workflow |
| Reproduce a qualified installation | Immutable releases, pinned native fixes, CI and strict qualification runners | Final hosted CI, target service evidence, qualified GPU runtime observations and recovery measurements |

Sources: [task contract](robot-tasks.md), [exploration](exploration.md),
[safety](safety.md), [deferred acceptance](../DEFERRED.md),
[simulation qualification](../QUALIFICATION.md) and
[tracked fleet configuration](../config/fleet_config.yaml).
Software support and tests are evidence of implementation, not physical acceptance.

## Completed in this review branch

Reused the existing implementation from
[PR #78](https://github.com/robotmindio/cleanroom/pull/78), rather than creating a
second defaults mechanism:

- Explorer advertises its configured duration/radius to late subscribers.
- The capability catalog includes required fields, defaults, effective defaults,
  descriptions, examples and terminal states.
- Managed tasks retain requested and resolved parameters through restart and
  terminal results. Captured exploration bounds are sent on an unsent task's
  eventual dispatch, so later default increases cannot enlarge those tasks.

Completed additional admission fixes:

- Requests above known Explorer duration/radius limits are rejected before
  journal insertion, using the same limit validator as the Explorer.
- Resume requests with positive replacement bounds are rejected at the gateway.
- Excessively nested task JSON is rejected cleanly. Defaults messages also reject
  excessive size, nesting and numeric overflow without crashing the callback.
- Indexed the managed task contract and this report; corrected deferred mapping
  acceptance to match the implemented quota freeze instead of active DB rotation.
- Increased the isolated RMF ownership launch test's discovery window from one
  to five seconds after the inherited hosted run missed its peer. The assertion
  still requires the conflict exit code; production preflight settings are unchanged.

Compatibility limits remain explicit: when Explorer defaults have not been
received, effective exploration parameters are `null` and the native Explorer
resolves/enforces its bounds at dispatch. Saved native tasks retain their own
bounds. Old journal rows remain queryable. Capability discovery does not assert
the sensors are fresh or that starting a task will succeed. These changes add no
new autonomous arm/disarm policy and grant no physical acceptance.

## Delivery epics

Priority means release order: P0 blocks the corresponding safety claim; P1
completes the attended pilot and initial find-object capability; P2 expands the
product after that pilot. Object/goal epics E9–E14 are core product work and can
start with read-only perception while physical qualification proceeds.
Owners are proposed roles, not assigned people. Relative sequence is given
instead of dates because hardware, site and staffing availability are unknown.

### E1 — Bind acceptance evidence to the release (P0)

**Outcome:** a reviewer can establish which deployed behavior each physical
claim actually covers. Current validation checks geometry, limits, stow and
required fault flags, but only requires nonempty revision/sensor/date strings;
it does not read or hash `evidence_report`
([validator](../lekiwi_rmf/safety_acceptance.py)). Release sealing hashes source
and installed artifacts, which proves integrity, not physical applicability.

**Work:** define a reviewed fingerprint of safety-relevant code, configuration,
calibration and hardware; pin evidence artifacts; explicitly record unchanged
evidence reuse; perform the predictive collision-model obstacle trials and
affected fault cases. Add release validation and regression cases against that
approved evidence format. An exact source SHA alone would unnecessarily demand
new braking trials after a documentation change.

**Done:** missing/tampered evidence and changed relevant inputs fail release
qualification; unchanged evidence reuse has reviewer approval; the deployed
predictive model has measured obstacle response within declared limits.
No changes to runtime arming policy are part of this epic.

**Owner/dependency:** release maintainer and safety reviewer, with attended
hardware trials. Tracks [#6](https://github.com/robotmindio/cleanroom/issues/6)
and [#12](https://github.com/robotmindio/cleanroom/issues/12).

### E2 — Complete the operator task journey (P1)

**Outcome:** an operator can submit, see progress, cancel, reconnect and receive
the robot's final outcome. The existing workflow ends at admission; workflow
success is not physical task success.

**Work:** use the named Fiber operations and persisted status as the outcome
authority; display pause reasons and distinguish admission, cancellation request
and confirmed termination. Verify configured tenant, allow-lists and role grants.
Exercise optional L2 approval/denial/timeout and notification delivery when that
policy is enabled. Read the additive capability defaults; show unknown defaults
explicitly. Verify retries preserve task identity across client reconnects.

**Done:** deployed trials cover success, native failure, busy admission,
cancellation, connection loss and gateway/action-server restart without duplicate
motion or false completion. Physical navigation/exploration runs stay in the
accepted attended scope. If L2 is enabled, timeout never admits motion.

**Owner/dependency:** Fiber product/integration owner and ROS maintainer; E1
and the bounded physical site trials in E4. The source contract improvement is
complete on this branch; the external journey is not verified here.

### E3 — Qualify physical protection for broader operation (P0 before unattended use)

**Outcome:** obstacles and hardware failures are detected within a separately
approved operating envelope. The current LD06 mask leaves 100 degrees blind;
low obstacles are undetected. Astra freshness is not near-field base coverage.

**Work:** measure/remedy surround and low/overhanging obstacle coverage; add and
qualify contact, IMU and battery signals where required; review independent stop
design and stalled host/process behavior. Bench-qualify servo voltage,
temperature/current/load thresholds before promoting warnings into motion gates.
Track hardware bridges and policy in repository configuration once measured.

**Done:** installed sources and production requirements agree; tests cover the
reviewed surface, load and failure envelope; stopping/coverage and independent
energy removal have physical evidence. Any unattended claim requires its own
explicit scope review. Software-only dummy signals cannot close this epic.

**Owner/dependency:** hardware integrator and safety reviewer; E1 evidence
contract. Tracks [#6](https://github.com/robotmindio/cleanroom/issues/6).

### E4 — Establish site perception, localization and mapping (P1)

**Outcome:** the robot repeatedly reaches site destinations and safely explores
within measured accuracy and task bounds.

**Work:** calibrate Astra RGB/depth intrinsics, registration, scale and optical
centre; revalidate camera geometry and wheel/yaw scales. Measure room-scale drift
and destination error against independent references. Survey a map at 0.05 m or
finer, fit reference coordinates and validate clearance. Verify live quota
freezes, then offline DB closure/archive/reopen without losing the map.

**Done:** a reviewer approves site-specific accuracy criteria and measured
results; bounded physical exploration covers obstacles, pauses, cancellation
and recovery; an approved bundle pins every map/graph/fleet/report artifact.
The synthetic 0.5 m development bundle remains a development fixture.

**Owner/dependency:** perception/site integrator; E1 for changed behavior.
Tracks [#8](https://github.com/robotmindio/cleanroom/issues/8).

### E5 — Deliver one qualified manipulation task (P2)

**Outcome:** a defined object/payload can be manipulated repeatedly within a
measured workspace. Joint trajectory support alone does not deliver pick/place.

**Work:** first stabilize wrist USB and trace intermittent feedback gaps;
measure arm/cable stow and calibration envelopes, review CAD/SRDF collisions,
establish real depth workspace coverage and intrusion stopping. Qualify payload
attachment, retention, repeated cycles and thermal endurance. Select one object
and task, then implement only its missing perception/execution path and managed
task contract after requirements and physical limits are established.

**Done:** repeatable physical success against declared task criteria, accepted
scaling and payload, plus cancel, preemption, tolerance, telemetry-loss,
restart and intrusion trials. Newly launched RViz and `move_group` agree on
defaults. No household manipulation claim follows from one HOME cycle.

**Owner/dependency:** arm/perception integrator and product owner; E1, E3
workspace protections, E4 calibration, E9/E10 target grounding and E12/E14 skill
execution/verification. Tracks
[#7](https://github.com/robotmindio/cleanroom/issues/7).

### E6 — Complete controlled fleet operation and charging (P2)

**Outcome:** one navigation owner executes fleet jobs on an approved site;
charging becomes a separate qualified workflow.

**Work:** qualify Free Fleet/RMF against the surveyed bundle, including lane
clearance and task interruption; inventory external adapters and resolve
cross-domain/ownership architecture. The current startup guard is not a
distributed ownership lease. Keep charging disabled until battery measurement,
charger hardware, docking criteria and a reviewed charging interface exist.

**Done:** supervised fleet dispatch/recovery trials show one owner and correct
coordinates. Charging is complete only after physical dock, charge termination,
undock and failure-response acceptance; navigation to a vertex named `charger`
does not establish charging.

**Owner/dependency:** fleet/site integrator; E4 approved bundle, E3 battery
work for charging. Tracks [#8](https://github.com/robotmindio/cleanroom/issues/8).

### E7 — Qualify deployment and control access (P1)

**Outcome:** an exact release can be installed, restarted, rolled back and
operated through approved identities on the target hosts.

**Work:** decide and test isolated DDS versus DDS Security; enforce accepted
tailnet/proxy and ZMQ policies; provision/rotate credentials outside Git.
Verify services, device permissions, cgroups and target restart/shutdown.
Retain hosted CI for the final revision, revision-bound MoveIt shutdown evidence,
and GPU simulation runtime observations alongside strict runner results.

**Done:** unauthorized motion attempts fail in the deployed topology; target
service/recovery trials and hosted CI pass; simulation qualification has reviewed
scan/depth, mux, obstacle, RViz and native heartbeat-loss observations. A green
local test suite alone does not close this epic.

**Owner/dependency:** deployment/network administrator; E1 and provisioned
hosts/credentials/GPU. Tracks [#9](https://github.com/robotmindio/cleanroom/issues/9),
[#10](https://github.com/robotmindio/cleanroom/issues/10) and
[#11](https://github.com/robotmindio/cleanroom/issues/11).

### E8 — Make recovery and endurance measurable (P1)

**Outcome:** a pilot has a declared recovery-time and operating-duration target
and evidence that it meets them.

**Work:** measure host outages, feedback gaps, wrist reconnects and compute
pressure at their source boundaries. Replace the fixed 60-second driver startup
respawn delay with bounded retry/reset behavior after declaring the recovery
target; test outage recovery without changing arming semantics. Exercise repeated
jobs, map growth, log storage and thermal behavior on the deployed stack.

**Done:** the agreed pilot endurance run completes without unexplained task
loss, duplicate dispatch or resource exhaustion; outage recovery meets the
declared target; failure and restart evidence identifies the deployed revision.

**Owner/dependency:** runtime maintainer and robot integrator; E2/E7 and target
hardware. Tracks [#4](https://github.com/robotmindio/cleanroom/issues/4), plus
physical feedback/camera work in [#7](https://github.com/robotmindio/cleanroom/issues/7).

### E9 — Recognize objects and ground their locations (P1)

**Outcome:** distinguish the requested object from other visible objects and
report where the observation places it, with uncertainty and evidence.

**Current gap:** this repository transports images/depth and builds geometric
obstacle clouds; it has no tracked object-recognition node, semantic detection
message or model configuration. A MoveIt collision cloud does not identify a
cup. The adjacent camera chart's defaults configure no detector; that NVR's
potential detection support is not a deployed robot perception pipeline.

**Smallest delivery:** select a small declared set of site objects and attributes,
an available compute host and a representative evaluation set. Use one detector
or recognizer and one observation path; choose the model from measured accuracy
and latency instead of adding several model backends. Publish timestamped
observations containing label/attributes, image region, confidence, camera/frame
identity and evidence reference. Add depth-based position, uncertainty and TF
into a versioned map only after intrinsics, registration and time alignment pass.
Missing depth permits an image observation, never a fabricated metric pose.
Distinguish class recognition (“a cup”) from instance identity (“this blue cup”).

**Done:** a held-out site set covers absent targets, similar objects, occlusion,
lighting and multiple matches; declared false-positive, recall and latency
targets pass. Physical position error passes a predeclared threshold. Unknown,
ambiguous and unlocalized observations remain explicit and cannot become
grasp/navigation targets by assumption.

**Owner/dependency:** perception owner; E4 calibration for metric grounding.
Image-only evaluation can start immediately; no detector/model is installed by
this report.

### E10 — Maintain a semantic world state (P1)

**Outcome:** answer “where was the blue cup last seen?” and resolve named places
without treating an old sighting as a current fact.

**Smallest delivery:** retain objects, named places and observations with stable
IDs, map/site version, last-seen time, pose/frame, uncertainty and provenance.
Start with only the relations the find-object task needs, such as observed-at
and operator-confirmed room membership. Separate the persistent last-known
location from current visibility; mark an observation stale, moved, contradicted
or unknown. Keep manually named places separate from automatically inferred ones.
Reuse existing platform storage where it supports these queries; keep camera
rates out of durable knowledge storage and retain bounded evidence references.

**Done:** same-class objects keep distinct identities where evidence permits;
moving an object updates its location; conflicting sightings remain explainable;
map replacement never reuses an old pose silently. Queries disclose observation
age and coordinate frame. Retention and access rules cover captured household
images before persistent collection.

**Owner/dependency:** perception/platform owner; E9 observations and E4 map
identity. Geometric SLAM maps remain the navigation source; semantic facts are
linked to them rather than substituted for collision data.

### E11 — Turn user intent into durable robot goals (P1)

**Outcome:** an objective survives reconnects and can state exactly what physical
evidence will count as success, failure or a need for clarification.

**Existing infrastructure:** adjacent Fiber core already persists tenant-owned
goals and subgoals in `src/fiber_core/goals/repository.py`. Its optional
`goal_layer.py` delegates plan meaning and agent context to a trusted plugin;
the checked-in company plugin supplies such a layer. The cleanroom task journal
persists action execution state. Neither inspected contract supplies robot
object/location resolution or sensor-proven objective completion.

**Smallest delivery:** extend the established Fiber goal layer with a robot goal
containing the requested outcome, resolved target IDs or unresolved description,
site/robot scope, success evidence, constraints, time/search budget and parent
goal. Link the goal to its plan revision and each existing managed task ID.
Handle priority, cancellation and conflicting objectives through one owner while
retaining the robot's one-active-task admission rule. Resolve “the blue cup” and
“the kitchen” through E10; ask for clarification when several targets fit or no
approved place exists. The model may propose targets, but must not invent their
coordinates or turn incomplete observations into achieved goals.

**Done:** goals survive restarts, retries do not duplicate tasks, cancellation
propagates to owned work and waits for confirmed termination, and ambiguous
references require resolution. Goal completion requires the declared evidence;
an accepted command or a completed Fiber workflow cannot satisfy it alone.

**Owner/dependency:** Fiber goal/plugin owner and robot product owner; E2
execution linkage and E10 semantic references. Reuse Fiber persistence and
policy; do not add a competing goal scheduler to ROS.

### E12 — Compose bounded robot skills into plans (P1)

**Outcome:** achieve a resolved goal using available actions, with explicit
preconditions, failure reasons and bounded recovery.

**Current gap:** Nav2 and MoveIt solve geometric motion problems. The managed
catalog currently exposes only navigate/explore. There is no inspected robot
task planner that chooses observe, search, approach, inspect and verify steps
against a semantic goal.

**Smallest delivery:** attach input types, prerequisites, expected effects,
failure outcomes and completion evidence to the existing capability contract.
Implement one fixed find-object workflow: observe, resolve candidate, search
if required, approach an approved observation pose and verify. Persist step/task
links in Fiber. Check current observations and capability readiness before each
step; bound retries and total search effort. Replan when the object moves or a
route fails; surface uncertainty or ask the user when the budget expires.
Keep physical trajectory generation and motion guards in ROS. An agent selects
admitted skills rather than producing unguarded motor commands.

**Done:** the worked workflow handles missing perception, an unavailable skill,
stale targets, blocked routes, duplicate requests and process restart. Failed
preconditions prevent that step from dispatching; each completed step has
observable evidence. A generalized planner or learned skill library is not
required for the first find-object delivery.

**Owner/dependency:** Fiber workflow owner and ROS capability maintainer;
E9–E11 and E2. E5 later adds qualified grasp/place skills with their own contracts.

### E13 — Search and inspect for a target (P1)

**Outcome:** find a requested object within a declared region/budget, or report
what was actually searched and why the target remains unresolved.

**Current gap:** existing exploration visits geometric observation targets and
reports mapped area/reachability. It has no semantic target, object-found stop
condition or accounting for where a particular object could be seen.

**Smallest delivery:** let Fiber use E10 last-known locations and E9 detections
to choose approved observation poses. Reuse navigate/explore with the same
motion limits; add only the target-search coordination the current actions lack.
Track inspected views, visibility/occlusion and remaining search budget. A
candidate triggers owned-task cancellation with confirmed termination before
any replacement approach. Keep the robot outside the object's collision volume;
an object's centre is not a valid base destination or grasp pose. Exhausting
geometric frontiers cannot prove the object is absent behind occlusions.

**Done:** physical attended trials cover visible, relocated, occluded and absent
targets, similar distractors and bounded search exhaustion. Results distinguish
confirmed sighting, ambiguous candidate and “not found within searched area.”
No unbounded roaming or unnoticed budget renewal occurs after recovery.

**Owner/dependency:** perception/task owner; E4 qualified navigation, E9–E12,
and E1 physical evidence for any affected behavior.

### E14 — Verify outcomes and evaluate the whole task (P1)

**Outcome:** the robot can substantiate “I found it” and eventually “I placed it,”
including detecting when execution succeeded but the requested outcome did not.

**Smallest delivery:** define success predicates using fresh observations of the
target and requested state. Persist observation/task/goal evidence links and
check them after each relevant action. Keep the verifier separate from an
agent's unsupported claim of success. A trajectory result can prove the arm
finished moving; manipulation success needs evidence of the object state.
Build a small reproducible site scenario set and recorded-frame replay for
recognition failures. Record goal completion, false-success rate, clarification
rate, time, search effort and failure/recovery causes; choose release thresholds
before trials and keep physical measurements separate from offline results.

**Done:** negative cases include stale images, wrong same-class object,
ambiguous detections, dropped perception and a completed action with an unmet
goal. Every achieved goal has inspectable fresh evidence; unverifiable outcomes
remain unresolved or failed. Manipulation adds missed grasp, drop and placement
error when E5 is implemented.

**Owner/dependency:** robot product/perception owner; E9–E13 and E1 evidence
discipline. When inference fails and no other fresh evidence can verify the
outcome, the goal remains unresolved; unsupported agent narration cannot close it.

## Minimal goal-to-action contract

These are proposed responsibilities for implementation, not new working APIs:

| Layer | Owns | Required link/evidence |
| --- | --- | --- |
| Perception | Timestamped image/object observations and measured pose uncertainty | Observation ID, source camera/frame, capture time, model/calibration identity |
| Semantic world state | Object/place identity and last-known facts | Observation references, site/map version, freshness and contradictions |
| Fiber goal layer | Desired outcome, constraints, policy and clarification | Goal ID, resolved target, success predicate and budget |
| Fiber workflow | Bounded steps, recovery and durable coordination | Goal/plan revision to managed task IDs; evidence for each step |
| ROS task gateway | Admission, ownership and persisted execution outcome | Existing task ID/state/result and confirmed termination |
| Nav2/MoveIt and guards | Physical paths, trajectories and local limits | Native results, live safety permissions and accepted physical scope |
| Outcome verifier | Whether the requested state is actually observed | Fresh target evidence linked to the goal, including unresolved outcomes |

For “find the blue cup,” completion means a sufficiently certain fresh sighting
of the resolved target and an evidence-backed location; image-only completion
must say metric location is unavailable. Finding two matching cups prompts
clarification. A later “bring it to me” request adds a confirmed recipient/place,
qualified grasp/carry/place skills and delivery verification. It is a different
completion condition and cannot be satisfied by the find-object result.

## Execution sequence and completion metrics

1. Start E9 with stationary recorded/live observations, E10 with bounded object
   and named-place records, and E11 using Fiber's existing goals. This makes
   recognition and goal resolution assessable without introducing new motion.
2. In parallel, close E1's predictive-response and evidence-applicability gaps
   before treating the current release as physically qualified. Complete E2,
   E4, E7 and E8 for the attended site pilot. Track physical task
   success, destination error, cancellation-to-confirmed-stop time, outage
   recovery time and sustained operating duration. Agree thresholds before
   collecting trials; this review invents no physical performance targets.
3. Combine E12/E13 with E14 to deliver the bounded find-object task. Release
   only after negative scenarios show that ambiguous/stale observations cannot
   produce false success or speculative motion targets.
4. Complete E3 for broader safety claims. Add E5 grasp/carry/place using the same
   goal/evidence chain; deliver E6 fleet/charging increments only when their
   physical dependencies pass.

Keep [DEFERRED.md](../DEFERRED.md) as the authoritative external acceptance
checklist; this report supplies prioritization and product outcomes.
Open issues #13–#16 describe upstream compatibility compromises (rosdep retries,
unset-variable setup, NumPy environment separation and shell-specific setup),
not unfinished user features. Their stated removal conditions remain unmet;
removing those workarounds would regress installation.

## Assessment limits

Reviewed entry points, tracked profiles, managed task persistence/admission,
exploration limits, release and CI definitions, acceptance validation and existing
acceptance/deferred records. This is a product readiness review, not a full
line-by-line security audit. The adjacent Fiber goal repository/plugin contract,
rosbridge connector/routes and camera chart defaults were inspected as source;
their deployed configuration and any other perception services were not audited.
Absence findings about recognition/planning concern this ROS repository and
these inspected integration paths, not every RobotMind service. No live motion, hardware modification, external
Fiber policy update or deployment was performed. Shared-memory recall was
unavailable because its refresh credential was missing; repository sources were
used instead. Branch validation results and the review link are recorded in the
delivery message; local tests do not close the physical or hosted-CI gates above.
