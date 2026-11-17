# Current rollout checkpoint — 17 November 2026

PRs #190/#191/#192/#193 are merged; main af9e42e matches deployed qualified source
ffc9566 on compute and Pi. Previous is 0aa796b. Canonical qualification passed
110/110 compute and 16/16 Pi, both seals verified, all five hosted checks green.
The canonical deployment used fresh stow gates, readiness and normal rearm.

The zero-target false completion and occupied-start retry loop are fixed. Explore
acquires new views under owned Nav2 goals and can advance/reverse 15 cm through
measured laser free space before retrying global routes. All previous safety and
task bounds remain active. No vendored CAD or arm/default-armed policy changed.

Physical trials: generation58 traveled 2.43 m, reached one target and stopped at
the configured region margin; generation59 used a separate empty database,
traveled 3.68 m, reached two targets and grew known area from 1.02 to 13.99 m².
The fresh-map trial was canceled after acceptance, with stop confirmed/resources
free. The primary database stayed unchanged while testing the fresh map and was
restored with all 107 previous node IDs retained. Full-room coverage is not claimed.

Final runtime: primary service active, ARMED, fresh travel_stow error .009253 rad.
Front/Astra and lidar are present; Pi kernel1068/VL805 DMA40 remains correct.
Hardware follow-up remains: the wrist camera disconnected from USB during the
trial, and lidar returns inside the front-left footprint keep StopZone active.
User input about the connection/obstruction is pending; hold still with all gates
active. Do not report collision-clear or three healthy cameras in this checkpoint.

One initial cold-start lifecycle timeout was recorded. A second truly empty map
booted with identical runtime configuration and rate-limited read-only readiness
queries. See the handoff for exact IDs, failed attempts, manifests and raw traces.
Broader search, calibration and trained skills remain pending; reach/skills stay off.
