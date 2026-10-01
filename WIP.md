# Reliability follow-up

Completed: Pi-integrated timestamped base pose, compute alignment across dropped
packets/restarts, pose-only EKF, native SLAM covariance policy, posture-dependent
FCL resting contacts in MoveIt/RViz, increased service scheduling priority.

Verified: 186 targeted Python checks; native build and resting-contact check.

Next: deploy to both machines; verify finite arm, navigation and sensor-gap tests
within the authorized 30 cm center radius; diagnose/reprocess stored graph.

Pending: compute administrator authentication for changed service configuration;
independent floor scale and confirmation of surface/payload for physical braking
acceptance. Never set acceptance validated from wheel odometry alone.
