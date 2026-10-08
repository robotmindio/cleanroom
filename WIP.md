# Base speed qualification

Completed: added `--direction` for one nominal braking trial without retries
or a final return; resume now rejects changed speed/load/stopping profiles.
Production limits and acceptance remain 0.03 m/s and 0.06 rad/s.

Verified: 74 focused tests passed; Ruff and compilation passed. Both deployed
releases are `b979c91`; compute uses `lekiwi.slice` with zero swap. The stationary
metrology preflight had no supervisor faults, 140 independent captures and a
0.300 s maximum capture gap. See `docs/base-speed-preflight-20261008.json`;
raw artifacts are in this worktree's `.benchmarks/base-speed-qualification/`.
Exploration cancellation succeeded with no active goals.

One 200 g forward stop passed at the current 0.03 m/s command: ground speed
0.02773 m/s, conservative stopping bound 22.433 mm plus 20 mm allowance,
and conservative receive-time stopping bound 0.790 s. There were no feedback
interruptions or supervisor faults. This is one nominal trial, not a new
worst-case latency or increased-speed acceptance. See
`docs/base-speed-measurements-20261008.json`; source run is
`.benchmarks/onboard-braking/20261008-214041`. Services and torque were retained.
Resumed runs now anchor their independent boundary at the stopped test center;
the same 74 tests passed after this correction.

Next: obtain fresh operator-at-motor-power-stop confirmation for one reverse
trial, including a slow return to the original center. Run
`scripts/test-onboard-braking.py --payload-g 200 --direction reverse
--resume .benchmarks/onboard-braking/20261008-214041`
as a detached user service, sourcing the current ROS installation and this
worktree's Python modules. The shared systemd environment expands variable
references in inline commands before Bash sources ROS; use a Bash script file
for sourcing, Python paths and execution. Each later moving trial needs fresh
explicit operator confirmation; use the first run's center when resuming.

Pending: current-speed braking/fault measurements, a tracked higher-speed
qualification path, incremental ramp, measured rotation proposal and stopping
geometry, full target acceptance, final PR, deployment from clean updated main,
live limit/base-permission verification. Existing bounded test speed guards cap
requests at production limits and the test area is too small for 0.30 m/s.
Do not raise Nav2 limits or mark target acceptance without physical evidence.
Remove this file before the final PR.
