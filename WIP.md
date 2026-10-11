# Exploration field validation

Completed: depth-stream permission handling, sparse-cloud retention, navigation acknowledgement and measured-start checks, native full-body lattice collision checks and backward escape, and controller path-heading configuration.

Verified: all 73 compute and 16 device release checks passed; cd70aef6384fb09aca67d3f5fcee02d84fdbdb34 deployed on both hosts. Live parameters and the loaded patched planner library match. Bounded exploration trials moved 1.05 m, 0.35 m and 0.40 m, paused/resumed on collision detection, and ended stationary at their explicit duration limits. Collision detection itself did not terminate exploration.

Pending: no trial completed an observation target. The latest trial recorded five unreachable targets; native planner logs contain occupied-start and lethal-goal rejections. Nearby human presence explains the earlier repeated front-right stops; no self-mask change is justified. Collision guards remain intact.

Next: reproduce the remaining start/goal collision rejections with the measured pose and map, correct the proven navigation cause, and verify target completion plus an attended hand-withdrawal trial. Duration currently remains an explicit wall-clock limit, including pauses. Remove this file before final delivery.
