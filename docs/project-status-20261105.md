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

The transport change uses the installed Zenoh 1.5 support for three authenticated
priority-scoped links on the existing port: scan/control `0–1`, depth/metadata
`2–5`, and preview images `6–7`. Compute uses peer mode with **no listener and
no discovery**: client mode treats the endpoints as failover alternatives and
opens only one link. Both peers allow three links. Background reconnect retries
are bounded from 100 ms to 1 s; the node can survive an unavailable peer.

`test/test_zenoh_tls.py` invokes a real two-domain ROS/mTLS probe when the pinned
native bridge is installed. It delays encrypted camera traffic for 800 ms,
requires scan/depth to remain fresh, then delays all links and checks data loss
and recovery with unchanged acquisition timestamps. The legacy configuration
fails the independent-link check. This is repeatable software fault injection;
it does not establish a physical Wi-Fi outage or stopping-distance SLA.

The motion rule remains: stale safety data stops motion; torque is held according
to the existing runtime policy; fresh inputs permit recovery. Neither stale
frames nor invented timestamps may keep motion enabled. A complete radio outage
can still stop the robot. Robustness means isolating unrelated traffic, bounded
recovery and preserving task identity/budgets through the interruption.

## All currently open GitHub issues

The issue list was read from GitHub on 2026-11-05: **13 open issues** at this
inventory point. A row is not closed merely because part of its software exists.
Detailed acceptance remains in [DEFERRED.md](../DEFERRED.md).

| Issue | Remaining closure evidence / work |
| --- | --- |
| [#6 — physical safety](https://github.com/robotmindio/cleanroom/issues/6) | Qualify the current predictive collision model and affected faults; measure low/overhanging/blind-zone coverage and arm-workspace coverage; review independent stop and motor diagnostics. Bumper, IMU and real battery sensing are absent. Existing acceptance is restricted attended operation, not full-hardware or unattended acceptance. |
| [#7 — physical arm](https://github.com/robotmindio/cleanroom/issues/7) | Review assembled CAD/SRDF and folded stow/cable envelope; measure intrusion stop and trajectory cancel/preemption/tolerances/restart with accepted payload; qualify retention/endurance; resolve wrist USB disconnects and remaining source-feedback gaps. |
| [#8 — calibration/map/RMF](https://github.com/robotmindio/cleanroom/issues/8) | Measure cameras/Astra/wheel/yaw/odometry; qualify quota freeze and offline map close/archive/reopen; survey and approve a ≤0.05 m map with pinned RMF graph/report and clearance/envelope validation. Resolve deployment ownership/domain architecture; keep charging unqualified. |
| [#9 — security deployment](https://github.com/robotmindio/cleanroom/issues/9) | Retain the actual network isolation, tailnet/proxy authorization, firewall/CURVE and credential lifecycle decision, plus deployed unauthorized-client rejection tests. mTLS/CURVE implementation alone does not prove the whole site access policy. |
| [#10 — release qualification](https://github.com/robotmindio/cleanroom/issues/10) | Retain service verification, dependencies, a clean revision/install-bound MoveIt shutdown probe and hosted CI for the deployed runtime revision. Native teardown fixes and previous clean probes exist; bind the final evidence before closing. |
| [#11 — GPU simulation acceptance](https://github.com/robotmindio/cleanroom/issues/11) | Run the strict qualification runner on a qualified GPU/EGL host for the exact release, then review its scan/depth, mux, obstacle, RViz and native actuator-heartbeat observations. Renderer-free unit/physics tests do not close the graphical/runtime checklist. |
| [#12 — deployed measurements](https://github.com/robotmindio/cleanroom/issues/12) | Assemble the exact-revision sensor rates/ages, guarded base and camera/link loss, gripper endpoint/return, wrist travel, RViz/driver and simulation-response evidence into formal acceptance artifacts. Existing scattered trials are not the complete package. |
| [#14 — upstream setup compatibility](https://github.com/robotmindio/cleanroom/issues/14) | ROS/venv setup scripts must themselves source cleanly under `set -u` before removing the local nounset toggle. This is upstream compatibility debt; retaining the deliberate workaround is not a missing robot capability. |
| [#88 — active search](https://github.com/robotmindio/cleanroom/issues/88) | Live found-outside-initial-view and absent-object trials; detector-derived object-size/height limits; 3D occlusion; coverage reprojection checked on live SLAM optimization. Only then promote moving search. |
| [#91 — spatial memory](https://github.com/robotmindio/cleanroom/issues/91) | Real/replayed loop closure, localization restart, map-session merge, relinked anchors and capture/graph race with stated positional uncertainty. |
| [#98 — image quality](https://github.com/robotmindio/cleanroom/issues/98) | Collect ≥20 independent scene groups per listed hard condition on robot cameras; choose luma/blur abstention limits and report per-condition recall. Engineering defaults are not a measured calibration. |
| [#102 — latency and restore](https://github.com/robotmindio/cleanroom/issues/102) | Record every operation's p50/p95/p99 under normal and saturated camera/GPU load and provider outage/fallback; replace derived budgets with measured ones; execute backup/restore/verify with services stopped and verified visual evidence. Transport separation is one improvement, not completion of this matrix. |
| [#148 — intermittent CI receipt test](https://github.com/robotmindio/cleanroom/issues/148) | Identify and fix the held-inference busy-receipt timeout under coverage, with a reproducer. Recent green runs and the separate driver recovery-window fix do not establish that root cause. |

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
