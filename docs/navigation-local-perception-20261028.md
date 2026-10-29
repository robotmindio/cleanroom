# Lidar clock recovery and local recognition

Initial runtime candidate: `1b8ae73`, based on deployed `f6598a1` plus the data backup fix
from PR #115. The experimental perception default described below was removed during review;
the final PR retains the existing perception selection and startup policy.

The October 26 journal shows `slam_cloud` failing in `Time.from_msg(scan.header.stamp)`
with negative seconds, concurrently with native controller, planner and collision
monitor failures. The LD06 SDK subtracts unsigned wall-clock packet timestamps.
A backward adjustment can underflow that subtraction and create impossible beam
stamps. A packet-stream regression reproduces this using the real SDK parser:
it fails on the original source and passes after `0005-clock-reset.patch`.
Backward and repeated clock values discard the incomplete revolution; valid
complete scans resume. Patched SDK CMake/ctest `lidar_clock_jump` passes.

The canonical scan filter also rejects negative, nonfinite and unrepresentable
acquisition times before publishing to native consumers, preserving valid
clockwise negative time increments. Discarded scans retain no invented timestamp;
the existing sensor freshness gates remain in force. Nav2 processes use the
standard launch respawn option and existing lifecycle supervision.

The initial candidate selected local RT-DETR without a remote fallback for a
front-camera trial. That trial has not established a qualified replacement for
the open-vocabulary default, so review removed the selection change. Production
perception startup remains disabled pending qualification. GPU model pins, collision geometry, speed limits, goal
tolerances and the two-second perception freshness limit are unchanged.

Validation before rollout: 28 scan-filter tests, 145 broader scan/launch/provider/
profile tests, 13 launch snapshots, native clock regression, repository lint and
Git diff checks passed. The diagnostic planner returned paths for 0.10, 0.20
and 0.30 m goals without commanding motion. This is not evidence of successful
physical navigation. Subsequent runtime and physical results follow below;
private inputs and outputs are retained under
`~/.ros/lekiwi/qualification/20261028-navigation-rtdetr/`.


## First rollout and additional fixes

The split deployment of `1b8ae7382226` passed all 16 device and 105 compute
release tests, then reported operationally ready and armed. A controlled idle
controller-process loss recovered controller, planner and collision-monitor
lifecycle states to active in 14.918 s. Wheel-odometry translation during that
test was 0.000443 m.

Local RT-DETR first exposed a separate clock issue: 59 frames were rejected,
with some capture stamps about 17 ms in the future. The machines used different
Ubuntu NTP endpoints with approximately 245–293 ms measured network delay.
The tracked time installer selects the shared [Cloudflare anycast time service](https://developers.cloudflare.com/time-services/ntp/)
and [32–64 second polling](https://www.freedesktop.org/software/systemd/man/256/timesyncd.conf.html),
instead of the previous 34-minute maximum. Applying commit `40d006a`'s clock
configuration through the repository script on both hosts reduced the reported
NTP network delay to 2.4–7.0 ms. No freshness gate was relaxed. The 73 installer
checks passed, including idempotence and service fingerprints.

The repeated stationary front-camera run recorded 103 fresh outputs, no late
outputs and no invalid input ages. Inference p50/p95/p99 was
0.284/0.318/0.354 s; queue p95 was 0.394 s and sensor-age p95 0.050 s.
Its profile contained only local `rtdetr_v2_r18` as primary, with no fallback.
This validates timing for that run, not recognition accuracy or full #98 acceptance.

The first managed 0.10 m goal returned native success after about five seconds,
but the independent client TF sample measured 0.03196 m position error against
the configured 0.03 m tolerance. Its final path contained the current position,
not the requested target. The lattice planner's same-cell shortcut returns the
start position when both coordinates fall in one 0.05 m cell. The new native
regression reproduces this and also shows that shortcut accepting blocked
sub-cell endpoints and blocked intermediate rotations.

`0004-exact-subcell-goals.patch` preserves the exact goal and checks the rectangle's
translation and rotation sweep using the existing collision checker. The isolated
real-plugin regression fails before and passes after the patch for exact arrival,
occupied endpoint, swept rotation collision, free rotation and cancellation.
The test links only the lattice DSO because loading both the generic and lattice
DSOs duplicates the upstream search-table statics. The robot was returned slowly
near its starting pose after the failed trial.

## Corrected planner rollout and physical trials

The repository split-deployment script installed `528ddaffaa6e` on both hosts.
All 16 device and 105 compute test groups passed; the script verified fresh
`travel_stow`, activated the release and confirmed operational readiness and
armed torque. This release also contains the final no-step chrony LAN policy;
it supersedes the earlier timesyncd proposal above. See
[clock synchronization verification](clock-sync-20261028.md) for the installer,
77 installer tests, measured synchronization estimates and remaining limits.

Six managed navigation tasks alternated between a point 0.10 m ahead and the
initial map pose. Each task succeeded within 30 s. A separate client sampled
map-to-base TF 0.7 s after task completion; all six samples satisfied the
unchanged 0.03 m / 0.15 rad tolerances. Position errors in task order were
0.02307, 0.02865, 0.02438, 0.02200, 0.02474 and 0.02301 m; maximum yaw error
was 0.04044 rad. Wheel-odometry displacement from the initial pose stayed below
0.07915 m and the live guard recorded no health faults. These are short-range
map/odometry checks, not independent metrology or whole-room qualification.

A repeated stationary front-camera run on this release recorded 141 fresh
local RT-DETR outputs, zero late outputs, zero invalid ages and zero rejected
or overwritten frames. Inference p50/p95/p99 was 0.283/0.334/0.398 s;
queue p95 was 0.362 s and sensor-age p95 0.050 s. The local-only profile had
no fallback and retained the pinned model identity. The temporary perception
service was stopped after measurement.

The remote CI coverage run exposed a test race: the simulated first inference
automatically resumed after five seconds while the test thread could still be
sending frames. The fixture now waits for explicit release and confirms the
latest frame was received before releasing inference. All 13 perception tests
pass with coverage locally; repository Python style checks pass. This changes
test synchronization, not production freshness or scheduling behavior.

The reviewed source retains the prior open-vocabulary primary/local-fallback
selection and disabled perception startup. Recognition accuracy, independent
scene coverage and full object-search acceptance remain unqualified; the
three-image development sample is not an independent corpus. The existing
search coverage policy still requires fresh stationary observations.

## General lattice endpoint follow-up

Repeating after the reviewed `69db27d` restart exposed the remaining general-path
case. A 0.10 m / zero-yaw request received an initial path ending at 0.05 m /
-pi/2 yaw. The task exhausted the 30 s trial deadline and was canceled to a
confirmed terminal state. Guarded manual return restored its pre-trial wheel
pose without health faults. The earlier six successful tasks therefore did not
establish reliable general paths after restart.

`GridBased` search tolerance is now zero instead of 0.20 m. A real-plugin
regression showed zero tolerance alone still returning a discretized endpoint.
`0005-exact-lattice-endpoints.patch` shares the existing collision-checked exact
finish between same-cell and general paths, after smoothing. A final connection
that collides is rejected. Arrival tolerances, footprint, unknown-space policy
and motion limits are unchanged. All six native regression cases, all 12 actual
planner route cases with exact final position/orientation checks, and 68 launch
checks pass. GitHub's full build/test/lint and coverage run also passed.

The split deployment installed `ee7cbeac2ff7` on both hosts after all 16 device
and 105 compute groups passed. Live parameter reads confirmed both planner
tolerances at zero and controller arrival tolerances at 0.03 m / 0.15 rad;
planner, controller and collision monitor were active. Perception startup
remained disabled, with the reviewed primary/fallback selection retained.

All six new managed 0.10 m out-and-back tasks passed after this restart, each
in 4.78–5.93 s. Settled client-TF position errors were 0.02014, 0.02423, 0.02361,
0.02411, 0.02348 and 0.02421 m. Maximum final yaw error was 0.01636 rad;
maximum wheel-yaw excursion was 0.02151 rad, without a recovery spin. Wheel
translation stayed within 0.07914 m of the initial pose. No health faults were
recorded. Final checks confirmed operational readiness, armed torque, fresh
`travel_stow`, all five Pi services active and the temporary perception service
stopped. These remain short-range map/odometry trials; independent physical
metrology, whole-room navigation and recognition accuracy remain unqualified.

### Confined short-goal replanning

On the corrected floor map, an oblique 5 cm goal had a collision-free direct
sweep, but lattice-heading discretization produced a roughly 40 cm route.
`0006-short-omni-connections.patch` first checks the exact straight translation
and shortest yaw sweep for omni goals within two map cells (10 cm at the tracked
resolution). It uses the existing footprint/unknown-space checks and cancellation;
a blocked sweep falls back to ordinary lattice planning. Other motion models and
longer routes retain lattice search. Native tests cover seven short goals,
mid-connection cancellation, and a blocked direct sweep with a valid detour.
The short-path regression fails on the preceding binary and passes with the patch.
