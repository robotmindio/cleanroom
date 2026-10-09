# Remaining 0.25 m/s corridor qualification

Completed: five independently measured forward stops at 0.25 m/s command with 200 g payload, held folded arm and unchanged 0.02 rad stow tolerance. Ground speeds 0.230074–0.239606 m/s; worst conservative stop 0.165177 m, modeled uncertainty 0.036126 m, stop-time upper bound 1.088423 s. All pass the 0.06 m / 1.6 s profile and 90% speed coverage without feedback interruption. Latest resumable source: .benchmarks/onboard-braking/20261009-150027. Fixed raw-LiDAR center [0.2, 0, 0]; final measured pose approximately [0.1915, -0.00088, -0.01578].

PR #54 (https://github.com/robotmindio/cleanroom/pull/54) contains the startup-report fix, excluded attempts and five forward stops with immutable raw-artifact hashes. 324 focused tests, Ruff and diff checks passed. PR remains open. This branch starts from its head; rebase onto main after merge if needed.

Motion jobs inactive; managed compute stack restored. Both machines deploy 9e441b3, with matching test-only stage profile. Production still uses old accepted stow and 0.03 m/s / 0.06 rad/s caps; stow can deny production base permission. Never command the arm or widen tolerance.

Physical fixture currently permits 1 m forward travel from the supplied starting front position, rather than clearance in all directions. Forward-only tests used --center 0.2 0 0 and returned to that fixed center. Use corridor mode for remaining directions: move to the same longitudinal pulse start, then rotate in place so each body-axis translation follows the saved forward axis. All existing native stop/slowdown, stow, depth/scan freshness, driver boundary and lease gates remain enforced. No arm commands. Continuous attended authorization remains in force; operator at motor-power stop, payload 200 g, dry floor.

Moving linear scan-loss 20261009-151628 passed: conservative stop 0.229842 m, uncertainty 0.052499 m, stop-time upper bound 1.333522 s. The run returned to center and is the latest resumable source. Corridor runner changes pass 330 focused tests, Ruff and diff checks.

Next: command/depth/telemetry linear faults, remaining nominal directions with --corridor, then angular faults. Seven moving faults and 25 nominal stops remain. Stop on any unqualified measurement. Rotation stage cap 0.50 rad/s remains provisional. Full passage is required before production stow/SRDF, acceptance, speed limits, stopping zones and exploration/test bounds are updated together. Then clean-main deploy and live controller limits/base-permission verification. Remove WIP before the next PR/final delivery.
