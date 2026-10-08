# Base speed acceptance

Completed: PR #42 provides bounded manual 0.20/0.30 m/s stages in a 1 m
fixture, fixed raw-LiDAR centering, single nominal/fault selection and production
restoration. Angular test rates are provisional at 0.40/0.60 rad/s. Production
Nav2 and acceptance remain 0.03 m/s and 0.06 rad/s.

Verified: 283 focused checks, Ruff, compilation and diff validation. The first
200 g forward stop at 0.03 m/s passed; evidence is in
`docs/base-speed-measurements-20261008.json`. No higher-speed trial has run.
Deployment of clean main `1dda4248ba83aee928930b8814ba7b04b21fae34` is running as
`lekiwi-base-qualification-deploy-42.service` in the user manager. Device release
checks passed 16/16. Log: `.benchmarks/base-speed-qualification/deploy-42.log`.

Next: finish verified deployment, run the confirmed 0.20 m/s forward trial
with 200 g payload and folded stow in the system `lekiwi.slice`. Use
`.benchmarks/base-speed-qualification/run-qualified-trial.bash --stage 0.20
--direction forward`, detached with logs. Check zero slice swap and cancel
exploration. The existing confirmation requirement remains per moving trial
unless the operator explicitly authorizes the pending continuous sequence.

Pending: 0.20/0.30 measurements, measured angular proposal and stop geometry,
five target trials in each direction plus moving faults, updated acceptance
and tests, final PR/deployment and live speed/base-permission verification.
Stop the ramp on failed bounds; do not raise production limits without evidence.
Remove this file before the acceptance PR.
