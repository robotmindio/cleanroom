# Exploration field validation

Completed: depth-stream permission handling, sparse-cloud retention, navigation acknowledgement and measured-start checks, native full-body lattice collision checks and backward escape, and controller path-heading configuration.

Verified: all 73 compute and 16 device release checks passed; cd70aef6384fb09aca67d3f5fcee02d84fdbdb34 deployed on both hosts. Live parameters and the loaded patched planner library match. Bounded exploration trials moved 1.05 m, 0.35 m and 0.40 m, paused/resumed on collision detection, and ended stationary at their explicit duration limits. Collision detection itself did not terminate exploration.

Implemented follow-up: use a fresh Nav2 costmap service snapshot when selecting goals; preserve soft inflation as a cost; retry occupied starts without rejecting untried targets; require one second of clear collision status before resuming; preserve active time during pauses and across checkpoint recovery. Mapping quotas still apply during a stop. The 97-case recovery matrix and five final focused regressions passed.

The follow-up passed all 73 compute and 16 device checks. Deployment readiness was blocked by the existing 514 MiB map exceeding the old 512 MiB storage quota; the tracked default is now 16 GiB (the compute host has 36 GiB free; the Pi does not run mapping), preserving the map and four-hour session limit.

Pending: qualify and deploy the quota adjustment, then verify live target completion and pause/resume behavior. Previous trials completed no target; nearby human presence explains the earlier repeated front-right stops and does not justify a self-mask change. Collision guards remain intact. Remove this file before final delivery.
