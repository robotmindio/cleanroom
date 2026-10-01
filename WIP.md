# Reliability follow-up

Completed: Pi-integrated timestamped base pose, compute alignment across dropped
packets/restarts, pose-only EKF, native SLAM covariance policy, posture-dependent
FCL resting contacts in MoveIt/RViz, increased service scheduling priority.

Verified: 186 initial, 166 follow-up and 62 collision/service Python checks;
native build and resting-contact check; live MoveIt folded release/return (max
final error 0.0093 rad). Finite route passed both Nav2 goals, stayed inside a
13.3 cm measured radius, returned within 5.2 mm, and produced a visual closure
(21718--21825) plus three proximity recognitions. Feedback gaps paused/resumed.

Runtime installed on both machines. The live map dictionary was recovered by
RTAB-Map's native recovery and a clean launcher shutdown: 13,885 words, zero
missing references. Fixes now signal the launcher once and remove unnecessary
Wi-Fi-triggered service kills; they still need the compute unit refreshed.

Fresh RViz now declares the collision plugin before constructing its scene;
live loader reports LeKiwiRestFCL and the MotionPlanning panel loads correctly.
Offline graph rebuilding was rejected: it fragmented the connected map, so the
production database was preserved. Historical constraints still reject some
candidate closures; calibration/map accuracy need independent physical evidence.

Next: build/sync the final RViz/shutdown fixes, refresh the compute unit after
administrator authentication, then finish cold-start and finite hold checks.

Pending: compute administrator authentication for changed service configuration;
independent floor scale and confirmation of surface/payload for physical braking
acceptance; room lighting for further visual qualification. Never set acceptance
validated from wheel odometry alone.
