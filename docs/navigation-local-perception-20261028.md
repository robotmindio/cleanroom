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
physical navigation. Runtime rollout and managed navigation/perception trials
are pending; private inputs and outputs are retained under
`~/.ros/lekiwi/qualification/20261028-navigation-rtdetr/`.
