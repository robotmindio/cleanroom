# Autonomous exploration follow-up

Current checkpoint: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md),
17 November 11:07 MYT / 03:07 UTC.

PRs #190 and #191 are merged. Main 5f0b79f is clean and has the qualified e74683f
source tree. e74683f is deployed on compute and Pi after canonical 109/109 + 16/16,
sealed inventory checks, all five hosted checks, fresh stow gates, readiness and
rearm. Previous is 9503b92. The native mapper enforces 0.15 s sync.

VL805 kernel DMA repair survived a guarded reboot and restored Astra, lidar and
wrist camera. Final state: ARMED/travel_stow (.012322 rad error), fresh front,
wrist/Astra and scan/depth, clear collision monitor, all resources free. The
supported pose needed no more manual nudging; MoveIt returned it to nominal.

Generation 56 exercised fixed mapping initialization and succeeded with no
reachable observation targets: 0 targets visited and exactly 0 m odometry travel.
This verifies startup/termination; useful base-motion/exploration/search acceptance
remains open for the observed map. Keep all footprint, stow, collision and freshness
limits. Do not claim room coverage or map growth. Reach/skills policy stays off.

No own temporary process, motion task, recorder, archive or staging job remains
active. Backups, receipts, raw trial traces and sealed releases are preserved.
