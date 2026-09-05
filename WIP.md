# Robot description consistency

Completed:
- Confirmed actual arm is SO-101. Corrected its base placement in LeKiwi by matching the original shoulder centre and physical pan/lift axes; reusing the SO-100 base origin displaced it ~79 mm sideways.
- RobotSkin lidar mount is identical in pinned/standalone sources. Both sensor brackets now attach to the upper plate's top surface.
- Connected the validated native wrist-flex part in the official mesh frame, converted from millimetres to metres.
- Corrected Astra direction, body envelope and nominal optical centre; fixed RViz description durability.
- Added source/output manifests to reject stale CAD exports; model-source.json records the source revision automatically.
- Added raw arm state publication and fresh/finite capture independent of old offsets. Capture preserves directions and backs up calibration, without restarting services or changing torque.
- Generated the three-view SO-101 zero-pose reference; removed legacy folded-pose instructions.
- Removed test-only MoveIt collision overrides and stale folded-pose exemptions. Replaced the oversized wrist sphere with a mesh-enclosing box (+4 mm), removed the non-physical tool-frame sphere, and included the Astra body in collision checking.

Verified:
- 84 focused Python tests, ROS package build, six installed CTest suites and the real-driver/fake-host test including raw joint publication.
- Native wrist fidelity; shoulder centre/axis alignment and propagation of assembly edits; Xacro semantics; chassis/native/accessory/reference/manufacturing checks.
- Managed install, live robot_state_publisher and RViz description matched after deployment of baa45d7. RViz UI shows the corrected reference geometry; live TF is incomplete while the robot host is offline.
- 41 focused model/calibration/simulation tests pass after the collision changes; wrist-envelope clearance is checked against all native wrist/servo mesh vertices.
- MoveIt plan/execute passed without test-only exemptions before adding the Astra collision body. The full final model correctly rejects its nominal overlaps (see blockers below); do not claim final end-to-end acceptance.

Unresolved:
- Astra's nominal mounting location overlaps the lidar body and shoulder envelope. Measure the physical Astra mounting-screw midpoint and lidar centre relative to the upper plate before correcting source placement. Do not disable moving-arm/camera collision checks to make the test pass.
- Gazebo physics smoke intermittently fails because the one-shot arm trajectory does not reach the native watchdog. It passed with diagnostic Gazebo topic subscribers attached, but fails unobserved. Temporary diagnostics and an unsuccessful PublishRaw experiment were removed; only the useful action-result error reporting remains. Reproduce with test_test_simulation_physics_launch.py. No physical actuator changes were made.
- robot-1 is offline on Tailscale (last seen 2026-09-05 17:00 Kuala Lumpur); host-health and SSH time out. Calibration requires reconnection and a supported, disarmed reference pose.

Next:
- Resolve the measured sensor locations, re-export/vendor/rebuild, then rerun MoveIt and Gazebo acceptance. Managed startup has auto_arm_on_startup=false. Latest collision qualification changes are installed but the running publisher/RViz still use the earlier deployed description; restart only after placement is resolved.
- User must hold the disarmed arm in the reference pose for capture, then verify directions and multiple poses. Existing elbow/wrist offsets produce out-of-range readings; do not clamp them or overwrite calibration from an unknown pose.
- Physically confirm sensor placement and measure Astra optical-centre correction; verify scan/cloud directions against known objects.
- Remove this file when physical calibration/validation and final delivery are complete.
