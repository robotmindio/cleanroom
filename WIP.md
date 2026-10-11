# Exploration field validation

Completed: depth-stream permission handling, sparse-cloud retention, navigation acknowledgement and measured-start checks, native full-body lattice collision checks and backward escape, and controller path-heading configuration.

Verified: all 73 compute and 16 device release checks passed; cd70aef6384fb09aca67d3f5fcee02d84fdbdb34 deployed on both hosts. Live parameters and the loaded patched planner library match. Bounded exploration trials moved 1.05 m and 0.35 m, paused/resumed on collision detection, and ended stationary.

Pending: neither trial completed an observation target. Repeated LiDAR returns straddle the front-right body boundary; dark cameras do not establish whether an external object or a self-return causes them. A physical corner check has been requested. Collision guards remain intact.

Next: establish the source of those returns, correct any proven sensor or navigation issue in tracked configuration, and verify target completion plus an attended hand-withdrawal trial. Remove this file before final delivery.
