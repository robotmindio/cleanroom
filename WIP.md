# Autonomous exploration follow-up

Current checkpoint: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md),
17 November 10:35 MYT / 02:35 UTC.

The robot runs qualified 9503b92 on both hosts, armed in travel_stow. Native 0.15
sync is live. VL805 kernel repair is persistent, Astra/lidar recovered, and #190
merged after all five checks (main 2ab33a0). Guarded MoveIt returned the supported
pose to nominal without further manual adjustment. All strict safety gates remain.

PR #191 at e74683f fixes exploration's localization->mapping startup-frame race.
Source is clean/pushed/locked. Canonical Pi qualification passed 16/16, sealed and verified. Compute
lekiwi-stage-mapping-start-e74683f.service is running the 109-suite matrix.
All five hosted checks passed. Reproduction and targeted recovery cases pass, but final
full qualification/seals/merge/deploy are still pending. Do not restart staging
or modify selected/sealed releases. Prepared deploy wrapper is not executed yet.

The generation-55 trial stopped before base movement; stop/resources confirmed.
No own movement task/recorder/archive process remains active. A verified 92-file
journal backup enabled 2.1G safe vacuum; Pi has space for the candidate. Current
map snapshot is valid and retains all 56 earlier node IDs. A guarded Pi reboot verified automatic DMA=40 controller startup and recovered
the wrist camera too. Normal rearm succeeded; all three camera feeds, scan,
stow and permissions are fresh/healthy. Runtime remains 9503b92.
