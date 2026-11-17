# Autonomous exploration follow-up

Current checkpoint: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md).

PRs #192/#193 are merged. Main af9e42e has the deployed ffc9566 tree. Both roles
passed canonical qualification (110/110 compute, 16/16 Pi), seals and hosted CI.
Exploration now observes and uses measured local steps when no global target is
reachable, including an occupied global starting footprint, without weakening
collision, stow, freshness or region limits.

Physical acceptance: existing map 2.43 m accumulated travel, one target reached;
separate empty database 3.68 m, two targets reached, known area 1.02 to 13.99 m².
Both trials ended with confirmed stops and free resources. The primary map was
restored with its raw hash preserved during the fresh-map trial and all 107 prior
node IDs retained. This is bounded exploration proof, not full-room coverage.

Runtime is ARMED in fresh travel_stow (.009253 rad error), primary map active.
Two physical issues remain: persistent front-left StopZone returns inside the
footprint, and the wrist JYU2C camera electrically disappeared from USB during
movement. Front/Astra/lidar/motor remain present. User was asked to inspect the
wrist connection and the front-left obstruction; answer pending. Keep the robot
still and all safety layers active. Do not ask for manual stow nudging.

One fresh-start lifecycle RPC timeout was preserved; another empty-database cold
start passed with the same readiness criteria and read-only polling paced at 4 Hz.
Consider bounding repository readiness query frequency separately; no runtime
configuration was changed for that diagnostic.

Broader find/search coverage, calibration and trained arm skills remain open;
reach/skills availability stays off. No own movement task, recorder, staging or
fresh-map unit remains active. Preserve locked releases, backups and raw evidence.
