# Remaining 0.25 m/s physical proof

Production is still 0.03 m/s / 0.06 rad/s on clean main 8ad34cb. The last deployment passed 67 compute and 16 Pi CTests and verified re-arm. No arm motion is authorized. Continuous attended sequence authorization covers the 200 g folded payload and dry 1 m forward corridor.

27 nominal stops qualify: forward 5, reverse 5, left 2, right 5, CW 5, CCW 5. All eight moving faults passed. Nominal rotation target is 0.40 rad/s; angular faults passed at the higher 0.50 request. Three left stops remain. Previous incomplete-speed or interrupted trials remain excluded.

The latest left trial 20261009-210701 lost single-view scan registration during stop observation and remains excluded. Original fixed geometry plus the stationary pulse view replays its actual final 62 raw scans with zero registration failures. Recovery-only source .benchmarks/base-speed-qualification/fixed-atlas-recovery-20261009-2116 preserves the 27/8 evidence and actual pose [0.2472005561, -0.0626711052, -4.2700055756]. It adds no qualifying trial.

The observer now locks a static union of the original fixed reference and the validated stationary pulse view. The bounded anchor error is added to stop uncertainty and subtracted from the independent center radius. Geometry, speed, freshness and uncertainty gates remain unchanged. All 368 focused checks passed, plus Ruff and diff checks.

Next: resume that recovery source for left at center [0.375, 0, pi], duration 2.2 s, rotation target 0.4. After all 30/8 pass, update the measured report, acceptance, folded stow, production limits/zones, exploration margin and affected tests together; then PR, clean-main deployment and live verification. Remove this WIP before final PR.
