# 0.25 m/s qualification

Completed: held folded reference captured without arm commands, retaining 0.02 rad tolerance; full CAD tolerance envelope checked. PRs #51–53 merged. Main 9e441b3 deployed and verified, robot re-armed; 67 compute/16 Pi checks and GitHub CI pass. Test-only 0.25 stage reserves 0.06 m uncertainty and 1.6 s stop time. Production remains 0.03/0.06 with old stow acceptance.

Reporting fix passed 324 focused checks, Ruff and diff checks; committed/pushed ac054ac. Pi connectivity restored on matching deployed source. Saved-reference return 20261009-143942 failed before motion after physical repositioning; fresh stationary reference analysis accepted 216 captures, zero rejections. The original reference can also recover the measured shifted pose with unchanged fitting gates (226 captures; 0.01145 m stationary excursion), but was not used for further motion.

Fresh nominal 20261009-144604: conservative stop 0.097186 m, uncertainty 0.025625 m, stop bound 0.772077 s all pass. Ground speed 0.219504 m/s and terminal guarded command 0.0875 m/s fail speed coverage after CollisionMonitor slowdown. No trial counted. Scan returns in forward corridor roughly 0.55–0.60 m beyond front footprint. Sequence stopped and restored managed compute stack. Await physical corridor clearance/repositioning; full acceptance remains outstanding.

Next: after corridor clearance/repositioning, start a fresh reference/center at the confirmed 1 m clear position. Run fresh forward nominal, eight moving faults, then five nominal stops per direction at 0.25/0.50 under the continuous attended authorization. No eligible trials yet under the new 1.6 s profile.

Pending: complete target evidence, propose measured rotation limit, update production stow/SRDF, acceptance, Nav2 limits/all-direction guards/exploration/test bounds together; reviewable PR, clean-main deploy and live controller limits/base permission. Old production stow presently can deny base permission and SLAM startup. Do not raise production limits before full acceptance. Remove WIP before PR/final delivery.
