# Physical acceptance: final measurement prerequisites

Completed: 185 eligible nominal stops across all six directions, ten moving linear/angular fault cases, repeated sensor cases, whole-ROS crash, sixteen Nav2 goals and fresh-map spatial closure. All applicable functional fault flags are now true; counters, distances and declared limits are populated. Physical acceptance remains false.

Additional live checks: real bottle blocks an otherwise authorized command; temporary MoveIt wrist collision rejects an arm goal with zero joint change; authenticated motor-error and exact-duplicate telemetry injections withdraw permission at rest. The motor-error test found normal mode overriding arm health. That override is removed, deployed and retested; driver feedback-gap holding remains intact. No named pose or production map was changed.

Verified implementation: 7c3ef989cdab, 57/57 CTests including nine launch tests; 151 targeted checks. Test host/injections are finite and restore production.

Next: remove the bottle for moving motor-diagnostic/replay stop measurements, and place readable floor-ruler graduations in the external camera view to independently bound the declared 10 mm error. Latest image shows no ruler. Only then review validated:true, deploy and verify production base permission. Retain the current 0.04 m/s / 0.08 rad/s limits and 5 cm clearance. Evidence: docs/physical-verification-20261003.md.
