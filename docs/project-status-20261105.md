# Project completion and outstanding work — 2026-11-05

This is a current inventory of documented acceptance gaps, open GitHub issues
and missing product capabilities. It replaces the **current-status** interpretation
of the October 13 readiness assessment; that assessment remains useful history.
It is not a claim that every latent software defect has been found.

## What “complete” currently means

The repository's explicit first product is **“find the blue cup and show me where
it is” from a stationary robot**, through Fiber. The
[implementation plan](robot-intelligence-implementation-plan.md#first-deliverable)
requires a durable goal, resolved target or explicit ambiguity, two distinct
fresh captures, accessible evidence and deterministic verification. An image-only
answer is valid when it explicitly discloses the absence of metric grounding.
Reconnects must retain the goal/evidence; retries must not create duplicate work.

The next delivery adds bounded attended search at surveyed viewpoints. Metric
location requires calibration and localization evidence. Manipulation follows
as a separately qualified capability.

There is **no approved complete specification for an unattended household
robot** in the inspected repository. General household tasks, arbitrary objects,
human interaction, unattended endurance and charging have no single agreed
release contract. The present staged plan cannot establish those requirements
on the product owner's behalf. In particular, passing the cup demo does not
complete a general mobile manipulator.

## Implemented versus accepted

| Capability | Present implementation/evidence | Still required |
| --- | --- | --- |
| Managed robot tasks | Durable journal, admission/idempotency, cancellation, owned navigation, pause/recovery and action reconciliation. | A complete deployed operator scenario matrix, restart/endurance and failure acceptance. |
| Image recognition | Pinned remote GPU recognizer and local fallback, latest-frame scheduling, source-age rejection, evidence storage/hash receipts and explicit ambiguity/quality outcomes. | Independent site corpus and accuracy/quality/latency acceptance. Positive scene-specific cup canaries do not qualify a general category detector. |
| Stationary Fiber objective | Software contracts, observe/resolve/verify workflow and evidence-backed completion are implemented. Earlier live scene-specific canaries are retained in machine qualification evidence. | Repeatable acceptance across positive, negative, ambiguous, stale, reconnect, cancellation and policy cases for the deployed tenant/profile/UI. |
| Spatial/world memory | Timestamped sightings, provenance, map revisions and capture anchors exist. | Real loop closure, relocalization, map merge/relink and capture/graph race acceptance; stable instance identity cannot be assumed from a class label. |
| Metric localization | RGB-D/TF grounding and map/uncertainty gates exist; invalid calibration leaves observations image-only. | Measured Astra intrinsics, depth scale/registration/extrinsics, independent position error and approved site map. |
| Active object search | Search action, view selection, bounded travel/time, cancellation and coverage bookkeeping exist. Unsupported camera orientations are rejected. | Live moving-search qualification, measured detectability and **3D** occlusion/coverage. `allow_search` remains disabled in tracked defaults. |
| Arm/manipulation | Guarded MoveIt trajectories, stow/torque-hold recovery, skill registry/training/execution infrastructure exist. | Measured workspace and intrusion stopping, payload/cable/collision acceptance, trained qualified skill artifacts and outcome verification. `config/robot_skills.yaml` contains no registered skills. Grasp/carry/place is not delivered. |
| Runtime and transport | Immutable split releases, service hardening, mTLS sensor bridge, CURVE motor transport, local DDS and a native sleep-inhibitor companion exist. | Sustained transport/endurance, saturation/outage and data-restore acceptance. Service readiness is not uninterrupted sensor freshness. |
| Fleet/charging | RMF/Free Fleet integration and map/ownership admission exist. | Surveyed approved bundle, distributed ownership/site architecture and fleet trials. Battery/charger/dock hardware and charge workflow remain separate missing capabilities. |

The intelligence implementation and follow-ups are described in
[the implementation report](robot-intelligence-report.md). Its dated trial results
must not be generalized beyond their recorded scene, hardware and conditions.

## Transient transport robustness

Previously all sensor priorities shared one TLS/TCP stream. Priority scheduling
could choose the next message, but could not undo TCP head-of-line blocking after
a lost packet or a queued camera frame.

The transport change uses three independent authenticated client sessions on the
existing port: scan/control `0–1`, depth/metadata `2–5`, and preview images
`6–7`. Each has a unique ROS bridge node and a disjoint import allow-list; compute
has no listener or discovery. Zenoh 1.5 restores only one link after a multi-link
peer session closes, so separate clients preserve isolation through reconnect.
Background reconnect retries range from 100 ms to 1 s, including cold startup
before the device exists.

`test/test_zenoh_tls.py` invokes a real two-domain ROS/mTLS probe when the pinned
native bridge is installed. It checks startup without the device, one imported
writer per topic, camera and depth delays of 800 ms without blocking scan,
all-link interruption with unchanged acquisition timestamps, and reconnection
of all three clients after their TCP connections are forcibly closed. The probe
checks bounded fresh-data recovery. This is repeatable software fault injection;
it does not establish a physical Wi-Fi outage or stopping-distance SLA.

The motion rule remains: stale safety data stops motion; torque is held according
to the existing runtime policy; fresh inputs permit recovery. Neither stale
frames nor invented timestamps may keep motion enabled. A complete radio outage
can still stop the robot. Robustness means isolating unrelated traffic, bounded
recovery and preserving task identity/budgets through the interruption.

## All currently open GitHub issues

The issue list was read from GitHub on 2026-11-05: **12 open issues** after closing #10 for deployed revision `3d7de958`. A row is not closed merely because part of its software exists.
Detailed acceptance remains in [DEFERRED.md](../DEFERRED.md).

| Issue | Remaining closure evidence / work |
| --- | --- |
| [#6 — physical safety](https://github.com/robotmindio/cleanroom/issues/6) | Qualify the current predictive collision model and affected faults; measure low/overhanging/blind-zone coverage and arm-workspace coverage; review independent stop and motor diagnostics. Bumper, IMU and real battery sensing are absent. Existing acceptance is restricted attended operation, not full-hardware or unattended acceptance. |
| [#7 — physical arm](https://github.com/robotmindio/cleanroom/issues/7) | Review assembled CAD/SRDF and folded stow/cable envelope; measure intrusion stop and trajectory cancel/preemption/tolerances/restart with accepted payload; qualify retention/endurance; resolve wrist USB disconnects and remaining source-feedback gaps. |
| [#8 — calibration/map/RMF](https://github.com/robotmindio/cleanroom/issues/8) | Measure cameras/Astra/wheel/yaw/odometry; qualify quota freeze and offline map close/archive/reopen; survey and approve a ≤0.05 m map with pinned RMF graph/report and clearance/envelope validation. Resolve deployment ownership/domain architecture; keep charging unqualified. |
| [#9 — security deployment](https://github.com/robotmindio/cleanroom/issues/9) | Retain the actual network isolation, tailnet/proxy authorization, firewall/CURVE and credential lifecycle decision, plus deployed unauthorized-client rejection tests. mTLS/CURVE implementation alone does not prove the whole site access policy. The optional RMF bridge now binds only localhost:7448 with mutual TLS and an explicit authenticated fleet client, with discovery disabled. Qualify that configuration on an approved map before enabling fleet operation or extending it across hosts. |
| [#11 — GPU simulation acceptance](https://github.com/robotmindio/cleanroom/issues/11) | Run the strict qualification runner on a qualified GPU/EGL host for the exact release, then review its scan/depth, mux, obstacle, RViz and native actuator-heartbeat observations. Renderer-free unit/physics tests do not close the graphical/runtime checklist. |
| [#12 — deployed measurements](https://github.com/robotmindio/cleanroom/issues/12) | Assemble the exact-revision sensor rates/ages, guarded base and camera/link loss, gripper endpoint/return, wrist travel, RViz/driver and simulation-response evidence into formal acceptance artifacts. Existing scattered trials are not the complete package. |
| [#14 — upstream setup compatibility](https://github.com/robotmindio/cleanroom/issues/14) | ROS/venv setup scripts must themselves source cleanly under `set -u` before removing the local nounset toggle. This is upstream compatibility debt; retaining the deliberate workaround is not a missing robot capability. |
| [#88 — active search](https://github.com/robotmindio/cleanroom/issues/88) | Live found-outside-initial-view and absent-object trials; detector-derived object-size/height limits; 3D occlusion; coverage reprojection checked on live SLAM optimization. Only then promote moving search. |
| [#91 — spatial memory](https://github.com/robotmindio/cleanroom/issues/91) | Real/replayed loop closure, localization restart, map-session merge, relinked anchors and capture/graph race with stated positional uncertainty. |
| [#98 — image quality](https://github.com/robotmindio/cleanroom/issues/98) | Collect ≥20 independent scene groups per listed hard condition on robot cameras; choose luma/blur abstention limits and report per-condition recall. Engineering defaults are not a measured calibration. |
| [#102 — latency and restore](https://github.com/robotmindio/cleanroom/issues/102) | Record every operation's p50/p95/p99 under normal and saturated camera/GPU load and provider outage/fallback; replace derived budgets with measured ones; The stopped-service backup/restore/verify drill passed on November 5: healthy databases, 28,086 matching evidence records and no mismatches; prior databases were preserved and armed readiness restored. The load/outage latency matrix and measured budgets remain open. |
| [#148 — intermittent CI receipt test](https://github.com/robotmindio/cleanroom/issues/148) | Identify and fix the held-inference busy-receipt timeout under coverage, with a reproducer. Forty isolated one-CPU coverage repetitions passed without reproducing the fault. Pre-cleanup thread/state diagnostics now retain evidence if it recurs; neither instrumentation nor a green retry establishes the root cause. |

## Release and recovery evidence completed on November 5

Issue #10 is closed for deployed revision `3d7de958bfb7d04fc979844612280ff2b4b7472a`:
105 compute and 16 device CTests passed, both release seals and target services
were verified, selected-install MoveIt teardown exited zero, and
[hosted CI passed](https://github.com/robotmindio/cleanroom/actions/runs/42024450388).
Three real two-second sensor-bridge interruptions withdrew permissions, held
motor torque and recovered armed readiness in 1.67–1.78 seconds. A short steady
capture still included one 369 ms scan; this is not an uninterrupted-radio SLA.

The real offline backup/restore drill preserved the replaced stores and verified
all 28,086 observation images against their recorded hashes. Restore hardening
also has regressions for crash WAL recovery, retained owner locks, duplicate
recovery-copy names and malformed manifests. Private artifacts are under
`~/.ros/lekiwi/qualification/20261105-remaining-fixes/`.

This host's service-account renderer probe passed EGL 1.5 / OpenGL 4.6 using AMD
radeonsi. The simulation qualification runner now allows 1,800 seconds for the
whole suite (measured ~800 seconds); individual test deadlines are unchanged.
The native simulator exposed a stale scan mask: the September 26 body sectors
predated the September 30 -90 degree lidar calibration. Forty native raw scans
were all rejected with the old mask and all valid after rotating those same
sectors +90 degrees. The corrected mask retains the same 71 degree width and
reaches; the physical mask and all freshness limits are unchanged. The renderer
scan check now uses the supervisor's validity checks and rejects negative
infinity, out-of-range and blind scans instead of reporting false success.
Full runtime qualification with the corrected installed configuration remains
required, including mux, obstacle, RViz and native-heartbeat observations.

## Acceptance gaps beyond individual issue titles

* **E1 evidence binding:** the reviewed safety-relevant code/configuration,
  calibration and hardware fingerprint must bind accepted evidence. The older
  acceptance revision and nonempty evidence-path fields do not prove that every
  later behavior was physically reviewed.
* **Recognition release:** independent positive/negative/ambiguous/hard-condition
  corpus, pinned model scope, declared accuracy and latency targets, and explicit
  scene-specific versus category acceptance.
* **Operator journey and governance:** the deployed tenant/profile/connector,
  authoritative evidence display, approval/revocation, same-instance ambiguity,
  idempotency and cancel/reconnect/restart trials must be accepted end to end.
* **Endurance and recovery:** declare and measure operating duration, outage
  recovery, thermal/log/database limits, USB recovery and repeated goals without
  lost/duplicate work. Successful launch is not endurance acceptance.
* **Manipulation product:** select an object/task and accepted load, train/register
  the skill, ground grasp/approach geometry and verify actual retention/placement.
  A successful arm trajectory is not evidence that an object was grasped.
* **Expanded product requirements:** unattended operation, general household
  skills, arbitrary object vocabulary, multi-site/fleet behavior and autonomous
  charging need explicit scope and separate hardware/acceptance contracts.

## Completion order and physical limits

1. Finish fault-injected transport isolation/recovery and sustained deployed
   sensor/perception measurements, then restore/endurance evidence.
2. Qualify the stationary objective across the complete scene and operator
   failure matrix, including independent recognition and evidence verification.
3. Establish measured metric calibration and an approved map/viewpoint bundle.
4. Complete affected stopping/intrusion tests and attended moving search, including
   3D coverage and spatial-memory revision behavior.
5. Deliver one trained, physically qualified grasp/place task with independent
   object-state verification; expand vocabulary/tasks/sites only after that gate.
6. Define and separately accept any unattended, fleet or charging product.

The current 30 cm motion authorization does not provide the corridor required
to repeat the recorded ~0.39 m stopping trials at 0.25 m/s. Source changes cannot
create missing physical sensing, independent scene labels, measured calibration
or approval of a larger operating envelope. These are concrete acceptance
dependencies, not permission to relax existing motion/freshness protections.
