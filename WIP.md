# Autonomous exploration follow-up

Current checkpoint: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md),
17 November 10:16 MYT / 02:16 UTC.

The robot runs qualified 9503b92 on both hosts, armed in travel_stow. Native 0.15
sync is live. VL805 kernel repair is persistent, Astra/lidar recovered, and #190
merged after all five checks (main 2ab33a0). Guarded MoveIt returned the supported
pose to nominal without further manual adjustment. All strict safety gates remain.

PR #191 at e74683f fixes exploration's localization->mapping startup-frame race.
Source is clean/pushed/locked. Canonical compute and Pi staging are active under
lekiwi-stage-mapping-start-e74683f.service and
lekiwi-stage-device-mapping-start-e74683f.service, with matching private logs.
Hosted CI is running. Reproduction and targeted recovery cases pass, but final
full qualification/seals/merge/deploy are still pending. Do not restart staging
or modify selected/sealed releases. Prepared deploy wrapper is not executed yet.

The generation-55 trial stopped before base movement; stop/resources confirmed.
No own movement task/recorder/archive process remains active. A verified 92-file
journal backup enabled 2.1G safe vacuum; Pi has space for the candidate. Current
map snapshot is valid and retains all 56 earlier node IDs. Wrist camera JYU2C USB
is still absent (-71); front GENERAL, Astra and lidar work.
