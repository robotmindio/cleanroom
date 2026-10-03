# Physical braking verification

Completed: bounded optical runner, visible-camera preflight, CollisionMonitor readiness, capture-time speed, zero-command recovery, smaller RViz-visible collision polygons, and measured 0.05 m/s translation cap.

Verified: dry concrete, 0 kg payload, modeled 8 degree marker tilt; 31 algorithm-eligible forward stops and 31 reverse stops at 0.05 m/s in 20261003-180606. Worst swept bounds 23.70/21.96 mm. These remain uncalibrated characterization. Ninety-six affected software checks passed. Folded arm remains stationary, ARMED and MoveIt-valid; all ten Nav2 nodes active.

Camera repaired. Full-resolution detection in a tracked region retains all three markers; 23 regression checks pass. Run 20261003-190656 completed thirty left repetitions, six eligible right stops and initial rotations before a 6 cm optical guard stopped the next pulse. Background feature matching shows less than 1 pixel camera drift. Approximately 2 cm of wheel-only return error accumulated across repetitions. The runner now calibrates the body-to-camera response with two small orthogonal moves and closes its return on the optical pose; 24 regression checks pass. Next run will test only right and rotations.

Fixed startup configuration: empty and one-node SLAM databases now start without the new-session closure gate; populated maps retain relocalization gating. Deployed stationary probe confirms false and a seed with 271 nonzero 3D features. Growth and loop closure remain unverified; one stationary seed is expected below the 40 mm keyframe threshold. Production database preserved.

Latest software root: domain 206 discovery UDP 58901 collided with Foxglove's ephemeral socket. Test domains changed to 40–84. Runtime fixes deployed as d5e1d3b; final 57/57 CTests and 29 qualification/motion checks pass. All test runners exited; production restored.

Latest run 20261003-193625 uses projective marker-plane rectification and optical returns: 31 eligible forward and reverse stops at 0.05 m/s, 14 left, one right and one rotation each at 0.10 rad/s. A 0.20 rad/s clockwise stop consumed 43.7 mm plus 10 mm uncertainty, exceeding the 50 mm budget; production rotation cap is reduced to 0.10 rad/s. Stationary startup waits no longer dilute the terminal attained-speed check. Pulses extend to 30 mm without relaxing the optical guard or 90% speed threshold.

Next: deploy and finish left/right/clockwise/counterclockwise repetitions. Finish navigation and outstanding fault tests, restore production and close all runners.

Pending: independent floor/lens/timing calibration, four directions' repetitions, outstanding applicable fault cases, verified loop closure/navigation, and intermittent shared network stalls. Physical acceptance remains false. Details: docs/physical-verification-20261003.md.
