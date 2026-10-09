# Remaining 0.25 m/s physical proof

PR #56 merged as 8ad34cb. Eleven qualifying nominal stops (forward 5, reverse 5, left 1) and all four linear moving faults are preserved in docs/base-speed-stage-025-corridor-progress-20261009.json with 56 artifact hashes. Worst fault stop is 0.272961 m; maximum stop-time upper bound is 1.524829 s. Production remains 0.03 m/s / 0.06 rad/s and old stow. All 349 focused checks, Ruff and diff checks passed.

The deployed bounded rotation geometry is being updated from square zones to 16-sided polygons enclosing the same protected circle. Positioning now checks the stopped position and heading after a turn. No arm command is authorized. Continuous attended test authorization remains recorded, with 200 g, folded pose, dry concrete, motor-power-stop operator and the supplied 1 m forward corridor.

Latest cumulative resume and actual stopped pose: .benchmarks/onboard-braking/20261009-200204, pose [0.1345528, 0.0131504, 1.3281833]. This return-only attempt timed out 6.5 cm short of the center; it contains all passing evidence. No qualifying trial ran. Depth fault source is 20261009-195426. Earlier insufficient-speed or interrupted left trials and right setup failure remain excluded.

Next: wait for the detached lekiwi-025-circular-zone-deploy-2003 service and verify its deployed-and-armed completion. Run remaining angular faults with --faults-only, then --direction-sequence right, rotation_cw, rotation_ccw and left, from the latest stopped reference. Nineteen nominal stops and four angular faults remain. Only complete 30/8 evidence permits the coupled production config/acceptance update, clean-main deployment and live limit/base-permission verification. Remove WIP before PR/final delivery.
