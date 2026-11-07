# Deployment and remaining acceptance work — 2026-11-07

Compute and the Pi run sealed release
`7f51fed145694a51aa13afdb0b613069653bb5c6`, merged by
[PR #174](https://github.com/robotmindio/cleanroom/pull/174). At deployment its
tree equalled main `3334807`. The canonical deployment confirmed fresh
`travel_stow` before torque cuts and finished armed. Fresh stow and armed
exploration readiness passed afterward. Documentation changes do not extend
the accepted physical operating scope.

## Delivered corrections

- [#169](https://github.com/robotmindio/cleanroom/pull/169): identify provider
  deadline expiry without replaying timed-out POSTs; retain a disabled wrist
  GPU canary with pinned identities.
- [#170](https://github.com/robotmindio/cleanroom/pull/170): publish safety
  permissions on a steady timer, preserve acquisition age and deny motion
  when ROS time stops progressing. Existing leases and limits remain intact.
- [#171](https://github.com/robotmindio/cleanroom/pull/171): correct only the
  simulated wrist-roll derivative gain. Native physics reaches the requested
  0.1 rad target within the existing tolerance and allowance.
- [#172](https://github.com/robotmindio/cleanroom/pull/172) and
  [#173](https://github.com/robotmindio/cleanroom/pull/173): establish the task
  fixture's current execution before injecting recovery faults, configure
  cancellation holds before admission, and join fixture executors before
  destroying their ROS entities. Retain transition evidence on failures.
- [#174](https://github.com/robotmindio/cleanroom/pull/174): prevent a NaN joint
  position from passing the pre-torque-cut stow check. Reject malformed joint
  arrays and invalid timestamps or CLI limits; retain the 0.02 rad tolerance
  and shared 50 ms clock-skew allowance. The previous NaN acceptance was
  reproduced without publishing synthetic hardware data or commanding motion.

The final release passed **105/105 native compute tests** in 814.17 seconds,
**16/16 Pi tests** in 104.37 seconds, 112 targeted stow/calibration/deployment
tests, Ruff and the full hosted CI including coverage and CodeQL. Hosted
evidence is [run 42382368019](https://github.com/robotmindio/cleanroom/actions/runs/42382368019).
The preceding `0e24a06` combined simulation runner also passed all 105 tests
and every required runtime check, in 942.90 seconds for CTest. Its summary
correctly remains `NOT_QUALIFIED`; a passing runner does not grant acceptance.

The previously delivered RobotMind owner-lookup indexes are deployed. Actual
production lookup plans used the indexes, with execution times of
0.222–0.908 ms. A later dark-scene objective began its first workflow task in
37 ms and correctly rejected unusable evidence. This does not establish a
universal end-to-end latency guarantee. The native evidence-receipt integration
and one independently hash-verified cup success are documented in
[the live observations](live-qualification-20261107.md).

## Current measurements and limits

A read-only 60-second sample after the final deployment observed every expected
stream. Header ages are acquisition-to-receipt ages, not execution latency.

| Source | Rate | Header age p99 | Maximum receipt gap |
| --- | ---: | ---: | ---: |
| Joint states | 14.71 Hz | 201 ms | 302 ms |
| LD06 scan | 9.97 Hz | 130 ms | 289 ms |
| Astra depth points | 9.24 Hz | 135 ms | 236 ms |
| Front RGB | 4.97 Hz | 137 ms | 324 ms |
| Wrist RGB | 4.97 Hz | 129 ms | 329 ms |
| Astra RGB | 1.86 Hz | 97 ms | 588 ms |

The final Astra sample contained 329 finite points, at optical Z distances
0.463–1.398 m. A live cloud is not proof of calibrated scale, arm coverage or
near-field base protection. All three RGB samples were `dark` and `blurred`:
mean luma was 0.84 front, 0.23 wrist and 3.17 Astra. These frames cannot qualify
recognition or make the physical measuring tape readable.

Telemetry TCP recorded retransmissions and a sender retransmission timeout of
209 ms; the Tailscale route was direct and answered a probe in 5 ms. This is
compatible with intermittent transport stalls, not proof of each stall's
cause. The observed maximum source-stamp gap in joint samples was 202 ms;
skipped samples can also originate after acquisition. Simultaneous source and
receive measurements are still required before attributing every gap to the
motor bus, network or scheduler.

An additional `0e24a06` simulated arm trial aborted before intrusion after a
state-validity result exceeded its unchanged 150 ms lease. It is a failed
interruption trial, despite earlier successful trials after actual motion.
The accompanying mux assertions passed, but five false permission samples
confounded the guarded-output expiry observation. The earlier independent mux
trial with 76 continuously true permission samples remains evidence for its
own revision. Do not replace these distinctions with a blanket manual pass.

Canonical RViz's accessibility model included a dynamically added collision
object and removed it afterward. Window captures still showed a blank list;
this does not demonstrate loss of the MoveIt scene. No speculative plugin
patch was deployed. A diagnostic RViz that survived SIGTERM was subsequently
stopped through the repository launcher and its PID verified absent.

Private source artifacts are retained under
`~/.ros/lekiwi/qualification/20261107-temporal-tail/`, including
`stow-final-deploy.log`, `stow-stage-*-final.log`, `final-sensors/collect.json`,
`simulation-serial-final/evidence/summary.json`, its failed `probe-wrist.json`,
and `canonical-ui-reader/accessible-{add,remove}.json`.

## Two-camera inference measurement

The canonical perception launcher ran the unchanged remote configuration with
front and wrist sources on `7f51fed`, with no robot goals. After both cameras
published, the observer targeted another 90 seconds. Total wall time including
startup and cleanup was 104.68 seconds; its temporary service ended inactive.
It published 152 fresh results, rejected two late answers and published seven
fallback results. The final diagnostics contained the following distributions:

| Stage | Samples | p50 | p95 | p99 |
| --- | ---: | ---: | ---: | ---: |
| Sensor age | 512 | 37 ms | 71 ms | 81 ms |
| Queue | 154 | 4 ms | 252 ms | 405 ms |
| Inference | 154 | 481 ms | 541 ms | 1,975 ms |
| Network | 145 | 254 ms | 314 ms | 535 ms |

The measured network p95 exceeded its derived 300 ms budget. Late answers
remained ineligible. These aggregate distributions do not separately qualify
a controlled fallback outage, skill execution, navigation or task latency.
Dark frames do not qualify recognition. The reviewed measurement summary and
model identities are retained in
[perception-latency-20261107.json](evidence/perception-latency-20261107.json);
the global budgets remain unchanged.

## Open issue inventory

GitHub still has the following **12 open issues**. Partial implementation or
one passing scene does not satisfy their completion conditions.

| Issue | What remains |
| --- | --- |
| [#6](https://github.com/robotmindio/cleanroom/issues/6) | Missing physical sensing and independent stop review; coverage and affected braking/fault acceptance. The full braking fixture exceeds the authorized 30 cm radius. |
| [#7](https://github.com/robotmindio/cleanroom/issues/7) | Measured arm/cable envelope, physical intrusion stopping, payload retention/endurance, USB substitution tests and source-boundary timing for joint gaps. |
| [#8](https://github.com/robotmindio/cleanroom/issues/8) | Measured Astra/camera/wheel calibration, surveyed approved site/RMF bundle, RTAB-Map quota/archive/reopen trials and site ownership acceptance. |
| [#9](https://github.com/robotmindio/cleanroom/issues/9) | DDS isolation/security and deployment-network authorization/rotation decisions. CURVE, mTLS and motor firewall evidence do not establish DDS or rosbridge authorization. |
| [#11](https://github.com/robotmindio/cleanroom/issues/11) | Reviewed simulation-server checklist, including stable manual fault observations and administrator acceptance. |
| [#12](https://github.com/robotmindio/cleanroom/issues/12) | Final formal evidence package, physical gripper endpoints/return and remaining guarded physical fault trials. |
| [#14](https://github.com/robotmindio/cleanroom/issues/14) | Upstream ROS setup still fails under `set -u` with `AMENT_TRACE_SETUP_FILES: unbound variable`; retain the existing wrapper. The current venv activation alone passed. |
| [#88](https://github.com/robotmindio/cleanroom/issues/88) | Live moving-search acceptance, measured detectability and 3D occlusion. Coverage reprojection exists but lacks live optimization acceptance; unsupported optical roll is rejected. Search remains disabled. |
| [#91](https://github.com/robotmindio/cleanroom/issues/91) | Verified real loop-closure/restart/merge/relinked-anchor and capture/graph-race evidence. |
| [#98](https://github.com/robotmindio/cleanroom/issues/98) | Independent hard-condition corpus and measured input-quality limits. Current three objects and four approved cup boxes do not provide that corpus. |
| [#102](https://github.com/robotmindio/cleanroom/issues/102) | Stage latency distributions for normal load, controlled saturation and fallback outage; replace derived budgets only with representative measurements. The 16,557-record data restore/hash drill is complete. |
| [#148](https://github.com/robotmindio/cleanroom/issues/148) | Busy-receipt timeout's deterministic cause. Repeated native/coverage runs have not reproduced it; failure diagnostics remain in the fixture. |

The stationary cup objective still needs its complete positive, negative,
ambiguous and hard-condition/operator matrix. Metric grounding and an approved
site precede moving search. A grasp/place product additionally needs a trained,
registered skill and independently verified physical retention and placement;
the qualified skill registry is empty. Unattended operation, fleet deployment
and charging require their own hardware and acceptance scope. They are not
established by the corrections or measurements above.
