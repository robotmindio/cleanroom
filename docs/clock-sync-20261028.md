# Split-host clock synchronization — 2026-10-28

`scripts/install-time-sync.sh compute|device` replaces the timesyncd/Cloudflare
proposal in PR #116. Lenovo serves NTP to the robot LAN; robot-1 follows
`andres-Lenovo.local` with interleaved mode every 4–8 seconds. Both roles slew
corrections without stepping the clock. The installer masks initial package
startup until this policy is installed. The role-specific configuration is
tracked and included in both service-installation fingerprints.

## Live verification

Applied commit `59870d2` using only the time installer on both hosts.
Chrony 4.5 is enabled at boot. The selected device source is
`192.168.110.202`, reached from `192.168.110.229`; `ntpdata` confirms
interleaved mode, kernel receive timestamps and daemon transmit timestamps.
One observed peer round-trip delay was 1.54 ms. The device selected compute at
11:14:42 UTC, approximately one minute after starting chrony.

Collected 21 snapshots at approximately 15-second intervals from
2026-10-28T11:14:12Z to 2026-10-28T11:19:13Z. After first reporting normal
synchronization, the device stayed synchronized in all 18 remaining snapshots.

| Device chrony metric | Observed value |
| --- | ---: |
| Final last offset relative to compute | +0.471 ms |
| Final RMS offset | 0.397 ms |
| Largest absolute last offset among synchronized snapshots | 2.681 ms |
| Synchronized snapshots with absolute last offset below 1 ms | 16/18 |

These are chrony's estimates, sampled during initial convergence, rather than
an independent measurement of physical clock error. Source sample offsets,
remaining clock correction and tracking last offset describe different stages
of the estimation/correction process. Five minutes on Wi-Fi establishes
functionality, not a guaranteed worst-case bound. Compute still uses external
Ubuntu NTP sources with roughly 250 ms round-trip delay; relative LAN alignment
does not establish submillisecond UTC accuracy.

A separate 30-second ROS subscription received 150 `/scan` messages: zero
negative ages; minimum/median/maximum ages were 18.40/107.34/194.99 ms.
These ages include acquisition and transport latency; they are not a direct
clock-offset measurement.

Repeated the installer on each host: chrony start times remained 11:13:12 UTC
(compute) and 11:13:42 UTC (device). Compute stack start time remained
10:57:37 UTC; motor-host and Astra start times remained 10:57:36 UTC.
No robot-service restart, disarm or clock step was requested.

## Repository verification and limits

77 service-installation tests pass, including fresh package installation,
repeat installation, both roles, no step policy, shared LAN reference and
fingerprint changes. ShellCheck, Ruff, ament_flake8 and `git diff --check` pass.
The full remote build/test CI is pending at the time of this report.

No reboot, suspend/resume or network-outage qualification was performed. Large
cold-start clock errors converge gradually because startup also omits clock
steps. The device holds over without an independent public source when
compute is absent; an unsynchronized compute clock is not advertised as a
valid local reference. Hostname and subnet changes require editing the tracked
profiles; see `docs/real-robot.md`.

## Pending branch inventory

After fetching origin and checking all remote branches against GitHub PR
history, the only branches with distinct patches outside main and without a
merged PR are:

- `fix/navigation-local-perception` — PR #116: LiDAR timing guards, Nav2
  respawn, local perception qualification, and this clock configuration.
- `fix/base-height-guard` — PR #33: physical base/sensor height and chassis
  collision guard. Its PR records pending physical/RViz verification.
- `dependabot/pip/config/pip-f08cc43eaa` — PR #83: perception Python dependency
  updates.

Historical braking and task-recovery branches have merged PRs, including
squash merges. Individual commit ancestry or `git cherry` alone can therefore
misclassify their old commits as pending work. No branches were deleted or
unrelated PRs merged by this change.
