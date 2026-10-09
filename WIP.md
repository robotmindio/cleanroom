# Remaining 0.25 m/s physical proof

PR #56 merged as 8ad34cb. Eleven qualifying nominal stops (forward 5, reverse 5, left 1) and all four linear moving faults are preserved in docs/base-speed-stage-025-corridor-progress-20261009.json with 56 artifact hashes. Worst fault stop is 0.272961 m; maximum stop-time upper bound is 1.524829 s. Production remains 0.03 m/s / 0.06 rad/s and old stow. All 349 focused checks, Ruff and diff checks passed.

The deployed bounded rotation geometry is being updated from square zones to 16-sided polygons enclosing the same protected circle. Positioning now checks the stopped position and heading after a turn. No arm command is authorized. Continuous attended test authorization remains recorded, with 200 g, folded pose, dry concrete, motor-power-stop operator and the supplied 1 m forward corridor.

Latest cumulative resume and actual stopped pose: .benchmarks/onboard-braking/20261009-200204, pose [0.1345528, 0.0131504, 1.3281833]. This return-only attempt timed out 6.5 cm short of the center; it contains all passing evidence. No qualifying trial ran. Depth fault source is 20261009-195426. Earlier insufficient-speed or interrupted left trials and right setup failure remain excluded.

Deployment 8ad34cb finished and verified robot armed; all 67 compute and 16 Pi CTests passed. All eight moving faults now passed, preserved in docs/base-speed-stage-025-all-faults-20261009.json. Angular independent ground speeds were 0.478971-0.485757 rad/s; worst angular fault stop 0.170674 m, with stop-time upper bound 1.358223 s. Right and CW nominal attempts stopped within budget but were excluded at 0.219679 m/s and 0.441861 rad/s respectively.

Latest cumulative resume is .benchmarks/onboard-braking/20261009-201736, actual stopped pose [0.1941753, -0.0126175, -0.3676016]. Eleven nominal stops and all eight faults qualify. The observer supports --nominal-duration 2.0 within unchanged stage boundaries to allow steady terminal speed; reused stops must still independently cover 90% speed under identical load/stop/error conditions. 356 focused tests passed, plus Ruff and diff checks.

Next: run the remaining CW, CCW, right and left batches with --nominal-duration 2.0 and center [0.25, 0, 0], placing the pulse start at the original corridor position. Nineteen nominal stops remain. Only complete 30/8 evidence permits the coupled production config/acceptance update, clean-main deployment and live limit/base-permission verification. Remove WIP before PR/final delivery.
