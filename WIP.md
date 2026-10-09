# 0.25 m/s qualification

Completed: held folded reference captured without arm commands, retaining 0.02 rad tolerance; full CAD tolerance envelope checked. PRs #51–53 merged. Main 9e441b3 deployed and verified, robot re-armed; 67 compute/16 Pi checks and GitHub CI pass. Test-only 0.25 stage reserves 0.06 m uncertainty and 1.6 s stop time. Production remains 0.03/0.06 with old stow acceptance.

Reporting fix passed 324 focused checks, Ruff and diff checks; committed/pushed ac054ac. Pi connectivity restored on matching deployed source. Saved-reference return 20261009-143942 failed before motion after physical repositioning; fresh stationary reference analysis accepted 216 captures, zero rejections. The original reference can also recover the measured shifted pose with unchanged fitting gates (226 captures; 0.01145 m stationary excursion), but was not used for further motion.

Fresh nominal 20261009-144604: conservative stop 0.097186 m, uncertainty 0.025625 m, stop bound 0.772077 s all pass. Ground speed 0.219504 m/s and terminal guarded command 0.0875 m/s fail speed coverage after CollisionMonitor slowdown. No trial counted. Scan returns in forward corridor roughly 0.55–0.60 m beyond front footprint. Sequence stopped and restored managed compute stack. Clearance is now specified as 1 m forward from the current front footprint; all future motion is restricted to that corridor. Full acceptance remains outstanding.

Passing forward-only trial 20261009-145518 used --center 0.2 0 0, positioning/return along the supplied forward corridor. Ground speed 0.235458 m/s, conservative stop 0.120550 m, modeled uncertainty 0.030489 m, time bound 0.791070 s; no interruption. Returned to fixed center x≈0.189 m. This is the first eligible trial under 6 cm/1.6 s.

Active: lekiwi-025-forward-corridor-repeats runs the four remaining forward trials with the same reference/center; log .benchmarks/base-speed-qualification/025-forward-corridor-repeats.log. Stop on any exclusion. Await corridor width and rear clearance for planning other body directions/rotation within that corridor.

Next: inspect forward repeats; preserve eligible evidence. Fit the remaining six-direction/fault tests to actual corridor dimensions using existing gates and measured stopping reserves. Do not launch the default all-direction sequence based on forward-only clearance.

Pending: complete target evidence, propose measured rotation limit, update production stow/SRDF, acceptance, Nav2 limits/all-direction guards/exploration/test bounds together; reviewable PR, clean-main deploy and live controller limits/base permission. Old production stow presently can deny base permission and SLAM startup. Do not raise production limits before full acceptance. Remove WIP before PR/final delivery.
