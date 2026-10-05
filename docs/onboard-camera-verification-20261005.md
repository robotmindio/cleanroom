# Onboard camera verification — 2026-10-05

**Latest follow-up:** runtime revision `1937e3923ce8` is deployed. Arm posture
and the final lidar turn/scan-mask check passed. The latest visual SLAM/Nav2
recheck is blocked by dark images; wrist USB disconnection cause remains
unidentified. Earlier successful camera/navigation tests are recorded below.

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
of the first route. That experiment did not separate visual drift from
wheel slip or calibration error; the subsequent calibration below uses an
independent lidar observer as well. Native visual
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
Revision `967e7c5` raised that cap to 2 Hz. That split deployment succeeded;
both runtime revision markers then matched
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

## Follow-up root fixes

An independent native RTAB-Map ICP observer used `/scan`, without wheel guesses,
TF publication or automatic tracking resets. The completed three-turn baseline
provided 198 source-timestamp-aligned wheel/lidar pairs: lidar yaw gain was
**1.08453** relative to wheel yaw, with 0.241° regression residual RMS over
32.95°. Independent RGB-D also measured larger turns than the wheel estimate.
The shared yaw scale was corrected from **0.90 to 0.976** in
`lekiwi_rmf/odometry.py`, used by the Pi integrator and compute driver. Saved
calibration overrides remain supported. Both deployed instances used 0.976.

The device had a saved 5 GHz Wi-Fi profile at priority 10 while its active
2.4 GHz fallback had priority 50. The existing tracked switch script now reuses
the matching saved profile and sets its priority above the fallback (51 here),
preserving native timed rollback. Automatic channel selection uses an empty
setting; NetworkManager rejected the former numeric zero when selecting band
`a`. The Pi joined 5 GHz at 5785 MHz, with observed transmit rates of 433.3 Mbit/s.
This corrects profile selection; it does not establish that every transport gap
has disappeared.

The finite observer checked cached poses before processing queued ROS callbacks.
The shared navigation tick now drains callbacks and validates feedback before
publishing nonzero manual commands. The onboard guard reuses the existing
bounded zero-command pause while fresh visual tracking recovers. Capture-age
bounds and the early 16 cm visual/18 cm wheel boundaries remain unchanged.
A live interrupted turn recovered from three transient visual tracking losses;
its subsequent stop exposed the separate scan-mask error below.

Two hundred stationary raw lidar scans in `travel_stow` found 34 body returns
in 14 scans at **210.9–213.8°**, ranges **0.136–0.195 m**. Their transformed
coordinates were x=-0.064 to -0.028 m, y=-0.158 to -0.111 m, inside the chassis.
The first self-mask sector started at 214°, so these returns reached
CollisionMonitor's one-point StopZone and caused intermittent false stops.
Its start is now **210°**, with the same 0.24 m range bound. The entire extended
sector lies inside the folded footprint; exterior obstacle returns and the
accepted stopping clearance remain unchanged. Targeted motion, scan-mask and
safety tests passed: **95 tests**.

An eight-second deployed check received 39 filtered scans with no StopZone
points. A subsequent return rotation exposed four additional affected scans:
233.1° at up to 0.202 m and 340.2–341.8° at 0.214–0.220 m. The first sector now
ends at 234°. The last sector's reach is split at 342° into 0.226 m and 0.218 m,
keeping all masked endpoints within the original footprint. The same 95 tests
passed again. Normal split deployment completed at revision `1937e3923ce8`,
with verified re-arm and the compact arm posture preserved.

### Current visual verification blocker

The interrupted routes left approximately 39.8° of accumulated rotation,
measured by independent lidar. A finite two-step return measured 37.2° of
physical rotation toward the initial view, with a source-aged lidar guard and
unchanged collision monitoring. The mapper still did not accept a global
closure, so the next production Nav2 check stopped before issuing any goal.
The existing map was preserved.

At 18:26, the front camera's mean brightness was **0.31/255** and Astra's was
**4.48/255**; both had **zero ORB features**. The frontal camera's automatic
exposure was active. Depth still supplied valid ranges over 34.5% of the raster,
and all camera streams were arriving. These images cannot support the visual
relocalization or independent camera motion verification. Earlier successful
Nav2 tests remain historical evidence; the final production recheck is **not
passed** under these lighting conditions. It needs usable illumination first.

The additional yaw comparisons reduced the baseline error but showed remaining
gain variation between interrupted routes. They do not establish one exact
wheel-to-ground scale under every turn, traction condition or scan match.
No visual matching threshold, accepted speed or stopping budget was relaxed.

### Final finite lidar check: passed

After the final mask deployment, left/right/return turns to ±0.15 rad completed
in normal production. The independent source-aged lidar observer recorded no
tracking losses and a maximum radius of **6.03 mm**; wheel radius was 1.31 mm.
No filtered scan point entered StopZone during the check. Across 153 paired
source timestamps, lidar/wheel yaw gain was **1.02655**, with residual RMS
**0.218°**, compared with the initial gain 1.08453. The remaining local scale
difference is 2.65%; this is not a claim of exact odometry on every surface.
Transient `scan: stale` permission gaps still occurred; zero commands held the
base until permission recovered, and all three turns completed. The final state
was ARMED, arm stowed, both permissions true, no supervisor or latched faults.
The camera observer remained lost in the dark images and was not used as a
physical displacement measurement for this check.

The final eight-second snapshot received 84 joint states, 42 lidar scans,
17 front images, 17 wrist images and 16 Astra RGB-D pairs. MoveIt reported a
valid arm state with no contacts, the arm remained stowed, and all six normal
services were active. Both installed revision markers matched
`1937e3923ce8ace75d391ba60b8b72968c3f4ae0`. Front/Astra images were still too
dark (mean 0.45/2.68, with 0/1 ORB features), and global localization remained
unaccepted. All finite observers, captures and motion clients were closed.

Follow-up local evidence:

- `20261005-172749-rotation-comparison/`: completed baseline yaw comparison.
- `rotation-followup-fits.json`: source-time fits for interrupted comparisons.
- `usb-root-cause.log`, `usb-camera-service.log`, `camera-query-ioctls.log`:
  kernel/capture correlation and health-query work.
- `self-mask-edge-measurement.json`: 200 raw stationary scans.
- `20261005-181803-stopzone-scan.json`: deployed first-edge check.
- `20261005-182341-rotation-comparison/`: return-heading recovery and the
  remaining mask-edge measurements.
- `20261005-182032-production-navigation/`: production localization timeout
  before any Nav2 goal was sent.
- `20261005-182612/`: dark camera images, calibration, fresh depth and valid arm.
- `20261005-183250-rotation-comparison/`: completed final lidar/scan-mask check.
- `20261005-183546/`: final arm, camera, permission and localization snapshot.
- `final-mask-tests.log`, `final-deployment.log`: 95 checks and verified split
  deployment of `1937e3923ce8`.

### Wrist USB: physical cause remains unresolved

Kernel USB disappearance preceded capture-supervisor restarts. The camera was
stationary and connected during the failures. It occupies its own USB bus,
separate from the Astra hub, and no competing capture process was found. Pi
throttling was zero and the log contained no undervoltage/overcurrent event.
A controlled suspend/resume and camera-service restart preserved the USB device
identity and did not reproduce the failure. Those observations do not exclude
a short local power or connector fault, or a camera/controller firmware fault.
Separating those possibilities requires a cable/port/camera substitution or an
electrical measurement; software observations have not identified which one.

The supervisor's repeated health query used `v4l2-ctl --all`, unnecessarily
reading hardware controls. It now uses `--info`: a traced invocation issued
31 ioctls rather than 119. This reduces polling work but is **not proof** that
the USB disconnect cause is fixed. No new USB disconnect was recorded after
16:15 during the finite follow-up tests. Runtime deployment and final movement
evidence are recorded below.
