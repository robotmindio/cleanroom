# Autonomous exploration follow-up

Current checkpoint: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md),
17 November 00:07 MYT / 16 November 16:07 UTC.

PRs #186–#189 merged. Main b8984e4 has the same tree as candidate 9503b92.
The native synchronization fix passed all five hosted checks, complete native
build and packaged-mapper ABI/parameter probes. Canonical compute 107/107 passed
in 806.34 s; sealed inventory verification passed. Actual device qualification
and deployment of 9503b92 remain pending because the Pi is unreachable.

The previous fa91885 release was deployed to both hosts after 106/106 + 16/16.
Compute still selects it; Pi pointers/markers are last verified before the outage.
Its mapper still ignores approx_sync_max_interval; the merged fix is not live.
Bounded Explore generation 54 logged 0.251153 m odometry travel and 0.165840 m net,
visited zero targets, then canceled with stop confirmed and resources released.

Final read-only probe: no joints/images, invalid sources, arm/base permissions
false. Current arm posture/torque cannot be verified. Do not cut torque without
fresh SRDF travel_stow. User power/Wi-Fi and nearby-object clearance checks are
pending. Keep collision/freshness/stow limits and the tailnet motor guard intact.

Next: restore connection, check Pi disk/readiness, stage and verify device 9503b92,
then canonical deployment from the exact qualified source with fresh stow checks.
Verify the launched mapper's actual 0.15 value, repeat cadence/scene/localization
checks, then another bounded physical trial if conditions permit.

No own temporary process or motion task remains active; Fiber resources are free.
Compute robot services remain active. Source branches and this checkpoint are
committed/pushed. Keep WIP until the physical/deployment follow-up is completed.
