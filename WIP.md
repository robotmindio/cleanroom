# Physical braking verification

Completed: bounded optical runner, visible-camera preflight, CollisionMonitor readiness, capture-time speed, zero-command recovery, smaller RViz-visible collision polygons, and measured 0.05 m/s translation cap.

Verified: dry concrete, 0 kg payload, modeled 8 degree marker tilt; 31 algorithm-eligible forward stops and 31 reverse stops at 0.05 m/s in 20261003-180606. Worst swept bounds 23.70/21.96 mm. These remain uncalibrated characterization. Ninety-six affected software checks passed. Folded arm remains stationary, ARMED and MoveIt-valid; all ten Nav2 nodes active.

Current blocker: measurement image/tape/floor shifted abruptly at 18:12:48, leaving markers outside the image. An operator question is pending to restore and secure the fixed camera. No further motion until the reference is usable.

Fixed startup configuration: empty and one-node SLAM databases now start without the new-session closure gate; populated maps retain relocalization gating. Deployed stationary probe confirms false and a seed with 271 nonzero 3D features. Growth and loop closure remain unverified; one stationary seed is expected below the 40 mm keyframe threshold. Production database preserved.

Latest software root: domain 206 discovery UDP 58901 collided with Foxglove's ephemeral socket. Test domains changed to 40–84. Runtime fixes deployed as d5e1d3b; final 57/57 CTests and 29 qualification/motion checks pass. All test runners exited; production restored.

Next: resume only left/right/clockwise/counterclockwise repetitions once the rig is stable. Finish navigation and outstanding fault tests, restore production and close all runners.

Pending: independent floor/lens/timing calibration, four directions' repetitions, outstanding applicable fault cases, verified loop closure/navigation, and intermittent shared network stalls. Physical acceptance remains false. Details: docs/physical-verification-20261003.md.
