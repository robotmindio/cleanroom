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

Next: obtain operator-at-motor-power-stop confirmation, unchanged folded stow,
dry clear floor and current payload grams. Run one forward trial at 0.03 m/s
using `scripts/test-onboard-braking.py --payload-g <grams> --direction forward`
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
