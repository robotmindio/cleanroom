# Exploration field validation

Completed: depth-stream permission handling, sparse-cloud retention, navigation acknowledgement and measured-start checks, native full-body lattice collision checks and backward escape, and controller path-heading configuration.

Verified: all 73 compute and 16 device release checks passed; cd70aef6384fb09aca67d3f5fcee02d84fdbdb34 deployed on both hosts. Live parameters and the loaded patched planner library match. Bounded exploration trials moved 1.05 m, 0.35 m and 0.40 m, paused/resumed on collision detection, and ended stationary at their explicit duration limits. Collision detection itself did not terminate exploration.

Implemented follow-up: use a fresh Nav2 costmap service snapshot when selecting goals; preserve soft inflation as a cost; retry occupied starts without rejecting untried targets; require one second of clear collision status before resuming; preserve active time during pauses and across checkpoint recovery. Mapping quotas still apply during a stop. The 97-case recovery matrix and five final focused regressions passed.

Pending: qualify and deploy the follow-up, then verify live target completion and pause/resume behavior. Previous trials completed no target; nearby human presence explains the earlier repeated front-right stops and does not justify a self-mask change. Collision guards remain intact. Remove this file before final delivery.
