# Robot description consistency

Completed:
- Operator identified the Astra's existing left-edge holes in a second photo. Moved the bracket to CAD (-100, 0, 7) mm, yaw +90 degrees, and corrected its screw spacing to 40 mm. Camera now faces left (ROS +Y), 8 degrees down; both screw centres are checked against the plate contours.
- Operator confirmed forward (+X) is the arm/fixed-camera side. Applied a fixed half-turn to the SO-101 mount while preserving its shoulder centre; reference pose now extends outward. Calibration values are unchanged.
- Found the old Pi case still exported as Bottom-V2-v3/Top-V2-v2. Removed both and placed the RobotSkin lidar on the former rear Pi screw pair plus the adjacent grid row. All four fasteners are verified against actual upper-plate contours.
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
- After moving the Astra to its physical side mount, all 25 focused model tests and the installed real-driver/fake-host MoveIt plan/execute test pass, including the Astra collision body with no test-only exemptions. Build and CAD hole/semantic/manifest checks pass. This is offline qualification, not physical calibration acceptance.
- Installed, live robot_state_publisher and restarted RViz descriptions match the Astra side-mount export (LeKiwi 60dbfac); reference render checked. The attempted desktop screenshot was black, so this reload has no visual RViz confirmation. Motor services and calibration were unchanged.

Unresolved:
- A read-only full-chain check found the fixed camera's current optical +Z points approximately (0, 0.174, 0.985) in base_link, not forward. Verify the lens axis in its CAD geometry and correct the ROS wrapper transform before qualifying camera overlays; the side containing this camera remains the operator-defined +X direction.
- Gazebo physics smoke intermittently fails because the one-shot arm trajectory does not reach the native watchdog. It passed with diagnostic Gazebo topic subscribers attached, but fails unobserved. Temporary diagnostics and an unsuccessful PublishRaw experiment were removed; only the useful action-result error reporting remains. Reproduce with test_test_simulation_physics_launch.py. No physical actuator changes were made.
- robot-1 is offline on Tailscale (last seen 2026-09-05 17:00 Kuala Lumpur); host-health and SSH time out. Calibration requires reconnection and a supported, disarmed reference pose.

Next:
- Astra mounting is resolved and rebuilt (LeKiwi 60dbfac). Continue with the fixed-camera optical-frame check, Gazebo command delivery and physical sensor/arm calibration once the robot is online.
- Latest arm/lidar correction (LeKiwi 4278583): build, 25 focused tests and four installed CTest suites passed. Source, live publisher and RViz description match (56 links). Reloaded only robot_state_publisher's description from the tracked installed Xacro and restarted RViz, without restarting motor services or changing calibration. Regenerated the reference image.
- Do not rely on the earlier auto_arm_on_startup=false note: current bringup defaults true, the running launch has no explicit override, and the offline driver could not be queried. No auto-arm setting or torque state was changed by this correction.
- User must hold the disarmed arm in the reference pose for capture, then verify directions and multiple poses. Existing elbow/wrist offsets produce out-of-range readings; do not clamp them or overwrite calibration from an unknown pose.
- Physically confirm sensor placement and measure Astra optical-centre correction; verify scan/cloud directions against known objects.
- Remove this file when physical calibration/validation and final delivery are complete.
