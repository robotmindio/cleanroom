# Robot description consistency

Completed:
- Confirmed actual arm is SO-101. Corrected its base placement in LeKiwi by matching the original shoulder centre and physical pan/lift axes; reusing the SO-100 base origin displaced it ~79 mm sideways.
- RobotSkin lidar mount is identical in pinned/standalone sources. Both sensor brackets now attach to the upper plate's top surface.
- Connected the validated native wrist-flex part in the official mesh frame, converted from millimetres to metres.
- Corrected Astra direction, body envelope and nominal optical centre; fixed RViz description durability.
- Added source/output manifests to reject stale CAD exports; model-source.json records the source revision automatically.
- Added raw arm state publication and fresh/finite capture independent of old offsets. Capture preserves directions and backs up calibration, without restarting services or changing torque.
- Generated the three-view SO-101 zero-pose reference; removed legacy folded-pose instructions.

Verified:
- 84 focused Python tests, ROS package build, six installed CTest suites and the real-driver/fake-host test including raw joint publication.
- Native wrist fidelity; shoulder centre/axis alignment and propagation of assembly edits; Xacro semantics; chassis/native/accessory/reference/manufacturing checks.
- Source and checkout-local installed descriptions matched before the final axis correction; repeat on the managed install after deployment.

Next:
- Install/restart and compare live publisher, MoveIt and RViz with the generated model. Managed startup has auto_arm_on_startup=false.
- User must hold the disarmed arm in the reference pose for capture, then verify directions and multiple poses. Existing elbow/wrist offsets produce out-of-range readings; do not clamp them or overwrite calibration from an unknown pose.
- Physically confirm sensor placement and measure Astra optical-centre correction; verify scan/cloud directions against known objects.
- Remove this file when physical calibration/validation and final delivery are complete.
