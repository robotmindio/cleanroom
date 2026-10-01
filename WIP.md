# Reliability follow-up

Completed: Pi-integrated timestamped base pose, compute alignment across dropped
packets/restarts, pose-only EKF, native SLAM covariance policy, posture-dependent
FCL resting contacts in MoveIt/RViz, increased service scheduling priority.

Verified: 186 initial and 166 follow-up Python checks; native build and resting-
contact check; live MoveIt folded release/return (max final error 0.0093 rad).

Runtime installed on both machines. The live map dictionary was recovered by
RTAB-Map's native recovery and a clean launcher shutdown: 13,885 words, zero
missing references. Fixes now signal the launcher once and remove unnecessary
Wi-Fi-triggered service kills; they still need the compute unit refreshed.

Next: finite navigation/SLAM test within the authorized 30 cm center radius;
verify fresh RViz plugin configuration and persisted dictionary after restart.

Pending: compute administrator authentication for changed service configuration;
independent floor scale and confirmation of surface/payload for physical braking
acceptance. Never set acceptance validated from wheel odometry alone.
