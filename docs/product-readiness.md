# Product readiness and delivery epics

Assessment date: 2026-10-13. Scope: this ROS repository and its tracked
integration contracts, assuming one physical robot, one managed task at a time,
and an operator at the independent motor-power stop. External Fiber deployment,
the assembled hardware and the surveyed site are outside this source review.

## Product decision

The next deliverable should be a repeatable **attended navigation and exploration
pilot at one surveyed site**. The repository has substantial execution,
recovery and safety software. It does not yet establish an unattended household
robot, a qualified manipulation product, automatic charging or production fleet
operation. Those outcomes require separate acceptance evidence and integrations.

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
completes the attended pilot; P2 expands the product after that pilot.
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
workspace protections and E4 calibration. Tracks
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

## Execution sequence and completion metrics

1. Close E1's predictive-response and evidence-applicability gaps before
   treating the current release as physically qualified.
2. Complete E2, E4, E7 and E8 for the attended site pilot. Track physical task
   success, destination error, cancellation-to-confirmed-stop time, outage
   recovery time and sustained operating duration. Agree thresholds before
   collecting trials; this review invents no physical performance targets.
3. Complete E3 for broader safety claims. Deliver one bounded E5 task and
   E6 fleet/charging increments only when their physical dependencies pass.

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
line-by-line security audit. No live motion, hardware modification, external
Fiber policy update or deployment was performed. Shared-memory recall was
unavailable because its refresh credential was missing; repository sources were
used instead. Branch validation results and the review link are recorded in the
delivery message; local tests do not close the physical or hosted-CI gates above.
