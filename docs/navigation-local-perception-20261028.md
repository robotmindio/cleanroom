# Lidar clock recovery and local recognition

Runtime candidate: `1b8ae73`, based on deployed `f6598a1` plus the data backup fix
from PR #115. Physical qualification is in progress; this is not acceptance.

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

Recognition configuration now selects local RT-DETR with no remote fallback,
using the tracked front camera. Production perception startup remains disabled
pending qualification. GPU model pins, collision geometry, speed limits, goal
tolerances and the two-second perception freshness limit are unchanged.

Validation before rollout: 28 scan-filter tests, 145 broader scan/launch/provider/
profile tests, 13 launch snapshots, native clock regression, repository lint and
Git diff checks passed. The diagnostic planner returned paths for 0.10, 0.20
and 0.30 m goals without commanding motion. This is not evidence of successful
physical navigation. Runtime rollout and managed navigation/perception trials
are pending; private inputs and outputs are retained under
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
near its starting pose after the failed trial. Final goal-fix rollout and repeated
managed navigation are pending.
