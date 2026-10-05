# Onboard camera verification — 2026-10-05

These finite checks used the robot's front, wrist and Astra cameras. No external
measurement camera was used and no motion command was sent. They supplement the
existing physical base acceptance; they do not qualify payload or new speeds.

## Results

- Front and Astra colour views show the environment without gripper occlusion.
  Their calibration dimensions match their images. Registered Astra depth is
  available, with 25–29% valid pixels in this scene.
- A 15-second stationary sample provided 31 Astra RGB-D frames. Of the initial
  ORB features, 111 had measured depth between 0.6 and 4 m. Native OpenCV PnP
  produced 27 relative poses with 67–85 inliers. Maximum apparent translation
  was 6.280 mm. This is a stationary repeatability check, not an absolute
  calibration or a replacement for the accepted stopping measurement error.
- MoveIt's planning service accepted paths from the actual joint state to the
  unchanged `travel_stow`, a short shoulder/elbow release, and unchanged `home`.
  Planned durations were 1.218, 1.211 and 6.722 seconds respectively. Physical
  execution remains to be checked.
- The current shoulder pan is 0.164176 rad; wrist roll is -0.039893 rad. These
  differ from `travel_stow` by more than the configured 0.02 rad. The pose is
  collision-valid, but base permission is consequently false.
- `slam_cloud.require_arm_stowed` also suppresses the fused SLAM cloud in this
  state. On restart, the front-depth readiness gate therefore keeps RTAB-Map and
  Nav2 waiting. Fresh Astra images do not by themselves satisfy that gate.

## Corrected wrist preview rate

The Pi produced 238 raw and 239 compressed wrist frames in eight seconds while
the compute side received only one. The device bridge explicitly capped the
wrist topic at 0.1 Hz. Revision `967e7c5` changes that cap to 2 Hz; image priority
and the existing lidar/depth delivery settings remain unchanged.

The normal split deployment completed and both deployed revision markers match
`967e7c569dd78ef02a83aeb8c332350f09992a16`. All five device services and the compute
service are active. The subsequent eight-second check received:

| Stream | Frames |
| --- | ---: |
| Wrist | 16 |
| Front | 16 |
| Astra colour | 16 |
| Astra depth | 15 |
| Astra RGB-D | 16 |
| Joint states | 80 |
| Lidar | 42 |

The last wrist image was 0.296 seconds old. Driver state is ARMED, arm permission
is true, and no supervisor faults were reported. Base permission remains false
for the posture described above. Four targeted bridge tests passed.

The kernel also recorded actual wrist USB disconnect/re-enumeration events
before deployment. These are separate from the preview cap; their physical
cause has not been identified. No such event was present in the post-deployment
kernel interval checked. The existing camera supervisor recovers the stream.

## Evidence and remaining live tests

Local artifacts under `.benchmarks/onboard-verification/`:

- `20261005-155238/`: initial robot camera images and joint/MoveIt snapshot.
- `20261005-155951-stationary/result.json`: stationary PnP and planning results.
- `deployment.log`: normal split deployment output.
- `20261005-160424/`: post-deployment images and stream/permission snapshot.

Before live execution, the outstanding question is whether the 30 cm area is
still clear and the operator is beside the physical motor-power stop. Then:
execute the saved travel posture through MoveIt, check that SLAM and Nav2 start,
and run short unloaded arm/base movements with onboard visual observations.
Preserve the production database and return to the saved travel posture. No
test client or monitoring process was left running.
