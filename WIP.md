# 0.25 m/s qualification

Completed: held folded reference captured without arm commands, retaining 0.02 rad tolerance; full CAD tolerance envelope checked. PRs #51–53 merged. Main 9e441b3 deployed and verified, robot re-armed; 67 compute/16 Pi checks and GitHub CI pass. Test-only 0.25 stage reserves 0.06 m uncertainty and 1.6 s stop time. Production remains 0.03/0.06 with old stow acceptance.

Latest attempt 20261009-142120 received zero scans, joint or independent pose samples; no motion trial ran. Pi network unreachable, Tailscale reports offline. Attended sequence stopped and restored managed compute stack. Reporting fix retains original failure with null missing pose/radius and cleanup.

Next: restore Pi connectivity, verify matching source/services and sensor feedback, then return to original center using reference run 20261009-135905. Start fresh forward nominal, eight moving faults, then five nominal stops per direction at 0.25/0.50 under the continuous attended authorization. No eligible trials yet under the new 1.6 s profile.

Pending: complete target evidence, propose measured rotation limit, update production stow/SRDF, acceptance, Nav2 limits/all-direction guards/exploration/test bounds together; reviewable PR, clean-main deploy and live controller limits/base permission. Old production stow presently can deny base permission and SLAM startup. Do not raise production limits before full acceptance. Remove WIP before PR/final delivery.
