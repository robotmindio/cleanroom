# Base speed acceptance

Completed: PR #42 provides bounded manual 0.20/0.30 m/s stages in a 1 m
fixture, fixed raw-LiDAR centering, single nominal/fault selection and production
restoration. Angular test rates are provisional at 0.40/0.60 rad/s. Production
Nav2 and acceptance remain 0.03 m/s and 0.06 rad/s.

Verified: 283 focused checks, Ruff, compilation and diff validation. The first
200 g forward stop at 0.03 m/s passed; evidence is in
`docs/base-speed-measurements-20261008.json`. No higher-speed trial has run.
Clean main `1dda4248ba83aee928930b8814ba7b04b21fae34` deployed and re-armed.
Release checks passed 67/67 compute and 16/16 Pi. Live production limits are
0.03 m/s and 0.06 rad/s; stow and base permission were true. New continuous
sequence and motion-guard checks passed 32 tests, Ruff and compilation.

The 0.20 stage stopped during reverse positioning, before its high-speed pulse:
StopZone was occupied. Independent excursion stayed below 6 mm and production
was restored. LiDAR detected 71 reference points inside the test stop zone,
including a cluster about 0.34 m beyond the left footprint edge. See
`docs/base-speed-stage-020-positioning-20261008.json` and
`.benchmarks/base-speed-qualification/obstacle-map.png`. The robot slice had
zero swap. The operator must clear or identify these detected objects.

Next: resolve the occupied stop zone, then run the authorized continuous
0.20/0.30 sessions with 200 g and folded stow in system `lekiwi.slice`. Use
`.benchmarks/base-speed-qualification/run-qualified-trial.bash --stage 0.20
--attended-sequence`, detached with logs. The runner reuses the deployed test
driver/profile through `LEKIWI_WS=/home/nex/lekiwi_ws/current`; only the attended
test orchestration comes from this task branch. Production runtime files are
unchanged. Check zero slice swap and cancel exploration. The operator has
authorized both continuous sessions while remaining at the power stop.

Pending: 0.20/0.30 measurements, measured angular proposal and stop geometry,
five target trials in each direction plus moving faults, updated acceptance
and tests, final PR/deployment and live speed/base-permission verification.
Stop the ramp on failed bounds; do not raise production limits without evidence.
Remove this file before the acceptance PR.
