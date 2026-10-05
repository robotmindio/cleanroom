# Onboard camera verification — 2026-10-05

Finite unloaded tests used the robot's front, wrist and Astra cameras, registered
depth and lidar. No external measurement camera was used. All commanded base
movements stayed inside the requested 30 cm area; the largest observed radius
was 8.02 cm. These checks supplement the existing physical acceptance. They do
not qualify added payload, higher speeds or accuracy across the whole room.

## Arm and MoveIt: passed

The actual stored SRDF targets were executed through MoveIt's `/move_action`.
Neither `home` nor the compact navigation default `travel_stow` was changed.

| Movement | Execution time | Maximum joint error at completion |
| --- | ---: | ---: |
| Initial return to `travel_stow` | 2.908 s | 0.00921 rad |
| `home` | 8.023 s | 0.01688 rad |
| Return from `home` to `travel_stow` | 7.754 s | 0.01689 rad |

All three actions returned status 4 and MoveIt error code 1. The configured
controller tolerance is 0.02 rad. Driver logs recorded holding the arm goal
through motor-feedback gaps during the initial stow and HOME movements; those
executions resumed and completed. Base permission was withdrawn while the arm
was unfolded and restored when it reached stow.

The final posture was collision-valid, with no reported contacts and a largest
stow error of 0.00769 rad. The arm remained folded before the production restart
and after the navigation tests.

## Navigation and SLAM: passed in the tested area

Four Nav2 goals on a fresh temporary map succeeded: 10 cm forward and return,
then 6 cm lateral and return. The robot also completed rotations to ±0.30 rad
and returned toward its starting pose.

- Maximum wheel-odometry radius: 7.15 cm; independent Astra visual radius:
  8.02 cm. Native visual odometry reported no tracking losses.
- There were 159 dual-view RGB-D arrays, all with two views. The mapper used
  registered Astra RGB-D and front RGB with depth projected from the measured
  fused lidar/Astra range cloud. The wrist camera supplied arm observations.
- The saved map contained 11 nodes with 169–232 finite measured 3D features per
  node. A spatial return recognition linked nodes 81 and 1 with 49 visual
  inliers.
- An isolated restart using unchanged production mapping settings recognized
  node 81 with 46 inliers and posterior 0.631. The normal appearance threshold
  remained 0.11; no recognition threshold was lowered in production.

The earlier production database had 111 nodes but produced only 4–8 candidate
matches and no accepted localization from the robot's current position. The
user confirmed that it was built elsewhere in the same room. That result does
not establish database corruption; the current view lacked a verified match to
the saved observations. The startup setting requiring a loop closure before
adding a new session consequently kept rejecting new locations.

After the user's authorization to choose the better map, the tested 11-node map
was manually installed at `/home/nex/.ros/lekiwi_rtabmap.db`. SQLite integrity
was checked, the stack was stopped with the arm folded, and the replacement
was atomic. The old 111-node database was removed. No automatic deletion or
startup reset was configured.

The restarted **normal production service** localized against that map. Its
graph contains a global closure between nodes 81 and 91. Two further Nav2 goals
(8 cm forward and return) both succeeded with the accepted production limits
of 0.03 m/s and 0.06 rad/s. The independent camera measured 7.08 cm of forward
movement. Maximum observed radius in this production test was 6.14 cm visually
and 4.88 cm in wheel odometry. A later spatial recognition had 64 visual inliers;
the graph had grown to 13 nodes. No supervisor faults or tracking losses were
recorded during this production test.

## Measurement limits and unresolved causes

The initial custom fixed-reference ORB/PnP observer produced an isolated 2.208 m
translation during a rotation. Neighbouring visual poses were centimetres apart
and wheel displacement was below 1 mm. It stopped that test, but its outlier is
not a valid physical displacement measurement. It was replaced by the existing
native RTAB-Map RGB-D odometry, with no wheel guess, automatic tracking reset or
competing TF publisher.

The wheel and native visual headings differed by approximately 4.84° at the end
of the first route. This short experiment cannot separate visual drift from
wheel slip or calibration error, so no wheel scale was changed. Native visual
odometry also has not been calibrated to the external rig's accepted absolute
measurement uncertainty. These results do not replace braking measurements.

The Pi kernel recorded real wrist-camera USB disconnect/re-enumeration events
at 16:01, 16:02 and 16:15. Their physical cause remains unidentified. No further
Pi USB disconnect was recorded during the arm and base exercises beginning at
16:20. The camera supervisor recovered the stream; this is separate from the
preview-rate correction below. The current map covers the small tested area,
not a fully surveyed room. Added payload remains qualified at **0 g**.

## Wrist preview correction and final production state

The Pi produced about 30 wrist frames/s while the compute bridge received only
one in eight seconds: `config/zenoh_device.json5` capped the preview at 0.1 Hz.
Revision `967e7c5` raised that cap to 2 Hz. Normal split deployment succeeded;
both runtime revision markers match
`967e7c569dd78ef02a83aeb8c332350f09992a16`. Four targeted bridge tests passed.

The final eight-second production check received 82 joint states, 41 lidar scans,
17 wrist images, 17 front images, 16 Astra colour images, 15 depth images and 15
Astra RGB-D pairs. Driver state was ARMED, both arm/base permissions were true,
`arm_stowed=true`, `bounded_base_test=false`, and supervisor faults and latched
faults were empty. All five device services and the compute service were active.
All action clients, visual observers and captures ended; normal production
services remain running.

## Repeatable check and evidence

`scripts/test-onboard-navigation.py` reuses the finite navigation runner and the
native RGB-D observer. It runs four short Nav2 goals and rotations on a temporary
map, saves all robot camera views, and restores production without changing its
database or acceptance. It rejects unavailable/lost tracking, nonfinite poses,
old/future capture timestamps and an early 16 cm visual boundary before another
manual command. Its guard check and the existing motion checks passed: **38
tests**. The complete route was exercised with the initial equivalent runner;
the new observer class was also used for the two normal production goals.

Run only with the arm already in `travel_stow` and the attended 30 cm area clear:

```bash
source scripts/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/test-onboard-navigation.py
```

Local artifacts under `.benchmarks/onboard-verification/`:

- `20261005-155951-stationary/`: initial stationary repeatability/planning check.
- `deployment.log`: preview-rate deployment.
- `20261005-162014-arm-travel_stow/`, `20261005-164046-arm-home/`, and
  `20261005-164129-arm-travel_stow/`: arm actions, encoder traces and camera views.
- `20261005-162537-base/`: rejected fixed-reference PnP outlier.
- `20261005-163103-base/`: native visual observer rotation check.
- `20261005-163629-base/`: four navigation goals, rotations, graph and new map.
- `20261005-165624-map-restart/`: successful reload with production defaults.
- `manual-map-replacement.json`: authorized one-time replacement record.
- `20261005-171327-production-navigation/`: two normal production Nav2 goals,
  native camera measurements and the relocalized graph.
- `20261005-171435/`: final camera, permission, joint and MoveIt snapshot.
