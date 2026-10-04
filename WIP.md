# Physical acceptance: measurement qualification failed

Completed: 185 eligible nominal stops across all six directions, ten moving linear/angular fault cases, repeated sensor cases, whole-ROS crash, sixteen Nav2 goals and fresh-map spatial closure. All applicable functional fault flags are now true; counters, distances and declared limits are populated. Physical acceptance remains false.

Additional live checks: real bottle blocks an otherwise authorized command; temporary MoveIt wrist collision rejects an arm goal with zero joint change; authenticated motor-error and exact-duplicate telemetry injections withdraw permission at rest. The motor-error test found normal mode overriding arm health. That override is removed, deployed and retested; driver feedback-gap holding remains intact. No named pose or production map was changed.

Verified implementation: 3ecbba672ad9, 57/57 CTests including nine launch tests (163.52 s); 72 targeted measurement/torque checks. The additional motor-diagnostic and duplicate-telemetry checks passed while translating and rotating. Test host/injections are finite and restore production.

The bottle was removed and the ruler is now visible; its confirmed width is 7 mm. Those prerequisites are complete. A sensitivity check of the narrow, blurred floor reference cannot establish the declared 10 mm measurement error: one-pixel corner changes produce 18.27–46.70 mm floor estimates for a 35 mm marker-plane displacement. This is a calibration failure, not another unperformed fault test.

Next: obtain a resolved floor reference with two known dimensions (for example the existing checkerboard-8x6-25mm.pdf placed in view, with the camera fixed), bound the error for the recorded camera geometries, then review validated:true. Retain 0.04 m/s / 0.08 rad/s and 5 cm clearance. See docs/physical-verification-20261003.md and .benchmarks/physical-rig/floor-reference-sensitivity.json. Production is restored; no commissioning or monitoring client remains active.
