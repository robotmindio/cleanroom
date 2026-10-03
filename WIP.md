# Physical braking verification

Completed: bounded optical runner, visible-camera preflight, CollisionMonitor readiness, capture-time speed, zero-command recovery, smaller RViz-visible collision polygons, and measured 0.05 m/s translation cap.

Verified: dry concrete, 0 kg payload, modeled 8 degree marker tilt; 31 algorithm-eligible forward stops and 31 reverse stops at 0.05 m/s in 20261003-180606. Worst swept bounds 23.70/21.96 mm. These remain uncalibrated characterization. Ninety-six affected software checks passed. Folded arm remains stationary, ARMED and MoveIt-valid; all ten Nav2 nodes active.

Current blocker: measurement image/tape/floor shifted abruptly at 18:12:48, leaving markers outside the image. An operator question is pending to restore and secure the fixed camera. No further motion until the reference is usable.

Completed root fix: fresh SLAM startup previously required a loop closure after its first node. Empty and one-node databases may now grow, while populated maps retain relocalization gating. Production database preserved.

Next: deploy the seed/preflight fixes, verify isolated fresh-map startup, then resume only left/right/clockwise/counterclockwise repetitions once the rig is stable. Finish navigation and outstanding fault tests, restore production and close all runners.

Pending: independent floor/lens/timing calibration, four directions' repetitions, outstanding applicable fault cases, verified loop closure/navigation, and intermittent shared network stalls. Physical acceptance remains false. Details: docs/physical-verification-20261003.md.
