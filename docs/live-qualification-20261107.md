# Native qualification observations — 2026-11-07

These observations apply to deployed release
`bbdf3863a973c13d270d143c16fabc0bab7cbe28`. They supplement the
[acceptance backlog](../DEFERRED.md); they do not extend physical acceptance.
Private source artifacts are retained under
`~/.ros/lekiwi/qualification/20261107-completion/` on the compute host.

## Data recovery

With fresh `travel_stow` confirmed and `lekiwi-stack.service` stopped, the
release's `scripts/robot-data.py` completed backup, restore, separate verify
and permission audit against the real domain-0 paths. All four stores were
included: task journal, observations, spatial memory and arm calibration.
Previous files were retained as `.pre-restore-*` copies.

Restore and separate verification reported `healthy: true`. SQLite integrity
and supported schemas passed, indexes were rebuilt, and all **16,557**
observation records matched their retained image hashes, with zero mismatches
and `visual_evidence: verified`. Permission audit returned no findings.
Compute restarted and `stack_readiness --exploration --armed` passed, including
another check while the isolated simulation was running. The Pi motor service
was not restarted and arm torque was not cut.

Evidence: `data-drill/result.json`, `data-drill/readiness.log` and
`readiness-under-simulation.log`. This closes the repeated recovery drill,
not the latency and saturation measurements still required by issue #102.

## Arm workspace gate

The canonical workspace-gate test inserted a temporary MoveIt collision object
at the wrist while the physical arm remained stowed. MoveIt reported contacts
with five CAD collision links. The arm action was rejected, measured joint
change was zero, and the object was removed. Permissions recovered and fresh
joint states confirmed `travel_stow` afterward. Base odometry excursion was
0.3 mm. Evidence: `workspace-gate/`.

This tests the software gate against an explicit planning-scene intrusion;
it does not establish physical obstacle coverage, stopping distance or payload
retention.

## Transport authentication

Read-only state requests with the registered CURVE client passed before and
after the negative tests. Plaintext and newly generated, unregistered CURVE
clients received no state response. A TLS 1.3 connection to the sensor bridge
without a client certificate was rejected with
`TLSV13_ALERT_CERTIFICATE_REQUIRED`. TLS 1.2 was disabled; its protocol-version
rejection is not client-authorization evidence. No motor commands were issued.
Evidence: `network-negative-auth.json` and its bounded test script.

These observations do not establish firewall enforcement, rosbridge
application authorization, tailnet peer policy or credential rotation.

## Stationary perception limit

The pinned optional YOLOX wrist pilot supplied two distinct, fresh,
non-degraded cup observations before admitting goal
`16962f82-3ce4-583b-9d7b-d393eb90ae53`. Retrying the same request returned the
same goal. The goal remained `starting` and then finished `unverifiable`;
it is **not a positive end-to-end acceptance**. The goal was confirmed
terminal, the pilot profile disabled and the base returned to its recorded
anchor. Evidence: `fiber-yolox-centered/` and `yolox-live-position/result.json`.

No detector score, input-quality, freshness or physical safety threshold was
relaxed. One successful scene does not qualify a general recognition model.

## Stationary perception follow-up on release 5791f96

The later compute/Pi deployment uses
`5791f96c3e49ec1eff01d29cabe50a231ca730d7`; its tree is identical to main's
`ce088ac19f339eac1ef19f45d72d926ce2e9f2f2`. Fresh armed readiness and
`travel_stow` passed before the tests below. No physical motion was required.
Follow-up artifacts are private under
`~/.ros/lekiwi/qualification/20261107-resume/`.

The JPEG-mode goal `4592a753-6ca3-5fb9-98f8-757a6569c8c6` finished
`unverifiable`. Its first workflow task started after 48 ms; five verification
activities ran within 16.54 seconds. The last four checks each retained only
one fresh qualifying capture. This failure was not a workflow queue delay.
The native source supplied the admitted camera, model and profile identities,
stationary captures, no input-quality flags, and cup scores above 0.5.

Both installed endpoints already support native SHA-256 receipts. The owning
Fiber integration was still configured for JPEGs. The tracked
`config/fiber_native_evidence_receipts.json` patch was applied through the
owning integration API with its complete configuration preserved. Read-back
confirmed that only `observation_evidence_format` changed. Credentials were not
logged or committed. Capture selection, two-second freshness, two-capture
requirement, 0.25-second separation and the 15-second goal budget stayed intact.

Goal `50e95a6d-e05f-5bff-a160-2c9dd4ce3bd7` then completed `done/found`, with
verifier version 3, 10.57 seconds after admission. The two cup captures were
0.604 seconds apart and 1.557/0.953 seconds old at verification. Both original
JPEGs were subsequently downloaded through the goal evidence endpoint; their
independently computed SHA-256 digests matched the immutable goal evidence.
The same request returned the same goal on retry. Evidence:
`fiber-native-retry/result.json`, `admitted-0.json`, and its two retained JPEGs.

The intended series of three successes remains incomplete. A preceding goal's
first workflow task timed out and retried at 75.82 seconds; the next goal after
the success also exceeded the pilot's wait. Both were canceled, their API
states confirmed terminal, and the pilot profile disabled. Temporal's server
reported persistence deadlines, while read-only PostgreSQL inspection observed
WAL write/sync waits. These observations identify a separate server-side
latency problem; they do not establish its storage root cause or qualify the
recognizer across scenes. No timeout or safety gate was relaxed to count a
success.

## Simulation follow-up on release 5791f96

The canonical runner completed all **105/105 CTests** in 507.35 seconds against
the exact sealed release. Its combined summary has no required-check failures:
source/build provenance, clean tree, dependency/static checks, EGL 1.5/OpenGL
4.6 renderer, isolated clean MoveIt shutdown, live scan/depth, active collision
monitor, guarded command topology and MoveIt parameters all passed. Evidence:
`20261107-resume/simulation/evidence/summary.json` and `commands/ctest.log`.
The summary correctly remains `NOT_QUALIFIED`: evidence collection does not
grant administrator acceptance.

A separately launched RViz MotionPlanning panel showed velocity scaling 0.50
and acceleration scaling 0.40. A temporary wrist collision object was visible
in its Scene Objects list and returned explicit contacts from MoveIt. The
object was removed afterward. Evidence under `simulation-manual/` includes
`rviz-main.png`, `rviz-obstacle.png`, and `scene.json`.

Execution-interruption acceptance remains incomplete. Before injecting the
object, the workspace gate intermittently reported stale state-validity results
and the supervisor had latched motion denial. The host showed about 45% CPU
scheduling pressure; this is a capacity observation, not a proven root cause.
The 0.15-second validity lease was not widened.

The manual checks also exposed a qualification-script defect: it copied every
joint from `/joint_states` into the arm trajectory. Simulation includes wheel
joints, so that request was rejected as malformed even with a clear arm path.
The initial rejection result was withdrawn. The repository script now uses
its existing complete-snapshot validator to select precisely `ARM_JOINTS`,
requires a complete finite stamped sample, and measures only arm displacement.
Regression coverage exercises mixed/reordered wheel-and-arm samples and rejects
missing arm joints before changing the scene or sending a goal.

With the corrected six-joint request, the simulator accepted a clear-path goal,
but a stale workspace lease withdrew permission before measurable motion. No
collision-interruption success is claimed. The source artifacts retain both
the invalid-request diagnostic and the corrected unsuccessful attempt. The
separate first helper's expectation of a hardware driver-state topic, absent
from this simulation, is also retained as a failed test rather than acceptance.

## Later timer recovery and dark-scene rejection

A separate Temporal GitOps rollout at 10:19 UTC included RobotMind commit
`fe1c4378c`: the native timer scheduler now gives active recovery tasks priority
over retention cleanup while retaining eight workers, twenty SQL connections
and a positive cleanup share. The installed ConfigMap read back weights
`high: 100`, `low: 90`, `preemptable: 1`. This was an independently delivered
server change, not a robot safety-policy change.

On the resulting server, real goal `9ce84ec2-6a16-5313-ab79-b76f5ac11395` started
its workflow task after 37 ms and its first verification after 741 ms. Five
checks returned `input_quality_unusable` before the 15-second observation
budget expired. The workflow completed `unverifiable` after 16.047 seconds,
including terminal persistence. Two fresh, stationary, identity-matched dark
captures were required before admission. The pilot profile was disabled after
the test. This demonstrates timely execution and rejection of unusable input
in this trial; it does not replace three positive recognitions.

The current wrist view no longer supplies eligible YOLOX cup observations.
Eight bounded viewpoints were tested with the canonical navigation guard;
retreat and forward studies reached maximum excursions of approximately
12.2 cm and 5.2 cm respectively, and returned to their anchors. Neither lowered
recognition or quality thresholds. The GPU standard profile recognized the cup
in a retained capture at score 0.519; its reduced-resolution `near_field`
profile did not. Subsequent live GPU captures carried `dark` and `blurred`
quality flags and were correctly ineligible. Private evidence is under
`20261107-temporal-tail/`, especially `fiber-dark-negative/timeline.json`,
`gpu-profiles.json`, and both viewpoint `result.json` files.

The disabled `config/fiber_perception_wrist_gpu_canary.json` binds the existing
GPU standard operating point to the wrist camera, fixes the class to cup and
retains the 15-second goal and two-second evidence limits. It is a controlled
test profile, not a default deployment or general scene qualification.

The provider client also had a misleading failure message: its own deadline
alarm closes the socket, which can surface as `RemoteDisconnected`. It now
identifies that deadline explicitly while retaining the existing unavailable
error contract. Timed-out POSTs are still not replayed, and the next request
reconnects. All 85 remote-provider tests passed, including the stricter timeout
and trickling-response assertions.

## Steady watchdog and interruption follow-up

The supervisor now publishes permission on a steady timer and preserves each
stamped input's ROS acquisition age when converting it to monotonic time.
A clock-progress requirement uses the existing 0.5-second permission lease;
repeated inputs cannot keep the measured safety decision healthy while ROS
time is frozen or moves backwards. No lease, geometry or actuator limit was
widened. All 92 focused tests and the exact staged release `6462f06`'s
105/105 native CTests passed (550.53 seconds); its sealed-release verification
also passed.

A separate domain-231 simulation on that release demonstrated interruption
after actual motion. A six-joint trajectory requested 0.02 rad shoulder pan;
feedback and 0.001144 rad measured motion preceded insertion of the temporary
wrist collision object. MoveIt then reported contacts, the supervisor withdrew
arm permission and the accepted action aborted. There was no blocked workspace
or false permission between sending the goal and inserting the object.
The object was removed afterward. Private evidence is
`20261107-temporal-tail/simulation-priority/probe-pan.json`.

This controlled trial ran inside a user systemd scope with CPUWeight 1000,
while the physical compute service was paused and its independent recovery
timer remained armed. Physical motor services stayed running. The wrapper
stopped its simulator and restored the physical compute service afterward.
This observation does not isolate CPU contention as the cause of earlier
failures or establish a hard real-time guarantee.

The separate wrist-roll trial remains unsuccessful: its accepted trajectory
ended with a goal-tolerance violation before the required positive displacement.
The test's 0.003-rad tolerance was stricter than the default; no gain, tolerance
or model change is justified by that result alone. A later retry outlived the
finite simulation fixture and found no action server, so it supplies no further
actuator evidence. The passing shoulder interruption is distinct from wrist
tracking qualification and does not grant administrator acceptance.

## Wrist-roll physics tracking correction

A renderer-free domain-232 extension of the existing physics smoke requested
0.10 rad wrist roll together with shoulder pan and gripper motion. With the
original derivative gain 0.4, the wrist reached only 0.0249 rad and violated
the unchanged 0.05-rad goal tolerance. Extending only the diagnostic goal
allowance from two to eight seconds still failed (0.0176 rad), so additional
waiting did not resolve the observed tracking error.

Changing only the simulated wrist-roll derivative gain to 0.02 reached
0.099925 rad; repeating with the original two-second goal allowance reached
0.099889 rad. Both runs passed the existing base, shoulder and gripper checks.
The tracked physics smoke now includes this wrist target and checks its actual
feedback with the same 0.05-rad tolerance. Proportional/integral gains, effort
limits, safety leases, CAD geometry and hardware motor configuration are
unchanged. These are simulator results, not physical wrist qualification.
Private experiment logs are under
`20261107-temporal-tail/wrist-physics/`.

## Combined-run recovery-fixture finding

The deployed `9ca8e75` combined runner passed 104/105 CTests; the gateway's
replacement-server test timed out while waiting for its initial running task,
before replacing any server. Other required runner checks passed. This failed
attempt remains under `20261107-temporal-tail/simulation-final/`.

That test disabled result transport before admission and assumed exactly one
initial send. A legitimate startup safety pause can require a confirmed stop
and resume before the task under test is running. The fixture now establishes
that the current goal is executing before losing its result transport, cancels
the already pending result request as part of that injected loss, and compares
against the sends observed at the fault boundary. It waits for the current
goal's completion rather than any earlier cancellation. An additional case
forces a startup permission loss and confirmed stop/resume before the same
replacement-server/no-replay assertions. Production recovery and leases are
unchanged.

The run also reported gateway-executor teardown races. Its fixture now stops
and joins the spin loop before destroying the executor's guard condition,
including at restart. All 102 gateway tests passed with unhandled-thread
warnings treated as errors; no such warnings were suppressed.
