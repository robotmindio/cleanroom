# Robot description consistency

Completed:
- Corrected Astra orientation from its CAD +Y-facing saddle to the ROS +X camera convention (8 degrees downward).
- Made RViz's robot-description subscription transient-local for late startup.
- Added read-only vendor snapshot verification and preflight of all mesh inputs before overwriting outputs.
- Documented the CAD export → committed snapshot → vendor → ROS build → consumer restart workflow.

Verified:
- 33 focused model, sensor, simulation-description, and vendor tests passed; changed Python files passed repository lint.
- Before changes, source, installed package, live robot_state_publisher, and RViz parameters all contained 58 links / 57 joints; all mesh paths resolved.
- Vendored CAD matches LeKiwi commit 05951401134ebb789caf86aa192aa68f2ad8bffd byte for byte.
- Live late-subscriber experiment received zero descriptions with volatile durability and one with transient-local durability.
- Repository build script passed into the checkout-local install (managed `/home/nex/lekiwi_ws` install unchanged). Initial direct build hit stale user-local Protobuf cache; the repository script rebuilt successfully with its normal cache/tool isolation.

Pending / next:
- Confirm physical arm variant with user. Export unconditionally replaces the original LeKiwi arm with official SO-101, including a different base and gripper; do not select the hardware model from assumption.
- Resolve calibration against the confirmed model. Live elbow was 5.364 rad (limits ±1.69), wrist flex -3.069 rad (limits ±1.65806). Saved zero offsets exist; do not hide errors by clamping displayed states or overwriting calibration from an unknown pose.
- Measure/confirm sensor mounts. Current lidar is on the lower plate at CAD (75,75,12) mm; its cylinder extends to z=51 mm and the upper plate starts at z=50 mm. Photo appears to place it further outboard. Confirm plate and mounting holes before changing CAD source.
- Astra visual is an approximate box whose origin is currently the saddle contact point; optical-centre translation and body placement need physical dimensions. Direction fix is verified, physical placement is not.
- Install/deploy final model and inspect newly launched RViz and MoveIt after hardware choices/calibration are resolved. Running managed stack has not been restarted or reconfigured by this task.
- Remove this file when the entire task is delivered.
