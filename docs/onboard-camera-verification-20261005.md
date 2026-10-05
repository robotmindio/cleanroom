# Onboard camera verification — 2026-10-05

**Verification closed for initial attended trials:** unloaded arm motion and
local Nav2/SLAM tests can start within the existing 30 cm test area, at the
accepted 0.03 m/s and 0.06 rad/s base limits on dry concrete, with the operator
at the physical motor-power stop. The wrist USB fault and standalone camera
translation drift remain open. See [the closeout](#verification-closeout-for-initial-trials)
for the verified scope and evidence.

**Latest follow-up:** runtime revision `1937e3923ce8` is deployed. After the wrist
camera connector was tightened, the finite arm and production Nav2 checks
passed. Wrist USB still disconnected at 20:25:10 and recovered in about five
seconds; tightening did not resolve it. Independent Astra translation drift
also remains unresolved; the follow-up measurements are recorded below.

Finite unloaded tests used the robot's front, wrist and Astra cameras, registered
depth and lidar. No external measurement camera was used. All commanded base
movements stayed inside the requested 30 cm area; the initial series' largest
observed radius was 8.02 cm. These checks supplement the existing physical
acceptance. They do not qualify added payload, higher speeds or accuracy across
the whole room.

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

### Dark-interval visual verification blocker (18:26–18:35)

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

## Illuminated recovery and current production verification (18:57–19:08)

The new snapshot measured 835 front-camera and 909 Astra ORB features, with
mean brightness 98.58 and 39.45 respectively. No database reset or replacement
was performed. A finite ±0.15 rad orientation search recovered the saved view:
RTAB-Map accepted the global link **1–6741 with 20 visual inliers**, followed by
spatial recognitions against saved nodes 80 and 81. Production registration
thresholds and mapping settings were unchanged.

Two Nav2 goals on the active production map (8 cm forward and return) both
returned status 4. Native onboard RGB-D odometry measured **5.74 cm** of forward
movement. Maximum radius was **9.84 cm visually** and 6.34 cm in wheel odometry,
within the attended 30 cm area. Camera tracking losses and supervisor fault
samples were both zero during that route. All **65 dual-view arrays had two
views**; the graph had grown to **45 nodes with 87–241 image features per
node**. These are local checks with the accepted unloaded speed profile, not a
room-wide survey or qualification of added payload.

MoveIt then executed the stored HOME target and returned to travel_stow:

| Movement | Execution time | Maximum joint error |
| --- | ---: | ---: |
| HOME | 8.500 s | 0.01841 rad |
| travel_stow | 7.617 s | 0.01382 rad |

Both actions returned status 4 / MoveIt error code 1, below the 0.02 rad
controller tolerance. HOME encountered a stale state-validity result and a
joint-feedback gap; the driver held the goal and resumed it successfully.
The return to stow recorded no supervisor faults. Base permission was withdrawn
while unfolded and restored at stow.

### Wrist USB recurrence: recovered, cause not resolved

At **19:01:07**, during the orientation search with the arm stowed, the kernel
recorded device 4-1 disconnecting (address 11), URB status -19, and immediate
re-enumeration at address 12. The capture process then reported ENODEV; the
supervisor detected it at 19:01:08 and restarted wrist capture at 19:01:12,
about **five seconds** after the disappearance. Front and Astra services stayed
active. No undervoltage or overcurrent event was logged, and Pi throttling was
zero. This recurrence confirms that the health-query hardening did not remove
the underlying USB fault. Cable/connector, local camera power and firmware
remain unseparated; a cable substitution in the same port is the next
controlled physical comparison.

The final eight-second snapshot received 84 joint states, 42 scans, 17 wrist
images, 17 front images and 16 Astra RGB-D pairs. All image source ages were
0.43–0.58 s. MoveIt reported a valid state with no contacts; the driver was
ARMED, the arm stowed, both permissions true, with no current/latched faults.
The mapper retained an accepted closure (last ID 7020). All finite test clients,
captures and independent observers ended; normal production services remain
active.

Local evidence:

- `20261005-185751/`: restored lighting and valid arm snapshot.
- `20261005-190055-rotation-comparison/`: view search and accepted global link.
- `20261005-190255-production-navigation/`: two production Nav2 goals, camera
  measurements, graph and dual-view counts.
- `20261005-190520-arm-home/`, `20261005-190537-arm-travel_stow/`: MoveIt actions
  and before/after onboard camera views.
- `usb-recurrence-1901.log`: kernel disappearance and capture recovery sequence.
- `20261005-190834/`: final production snapshot.

### Earlier wrist USB hardware isolation

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
the USB disconnect cause is fixed. The earlier finite follow-up did not reproduce
the disconnect; the subsequent 19:01 recurrence is recorded above.

## Tightened camera connector: finite follow-up (20:18–20:32)

The wrist camera remained on the same Pi port with the same cable after its
connector was tightened. The initial arm, Nav2 and small-turn checks through
20:23 recorded no USB disconnect or capture restart, retaining USB address 12.
However, at **20:25:10**, before the next commanded turn and with the arm still
stowed, the kernel recorded another disappearance and enumeration as address
13. ENODEV followed in the capture process. The supervisor restarted capture at
20:25:14 and image conversion resumed at 20:25:15, about **five seconds** later.
Pi throttling remained zero and no undervoltage/overcurrent was logged.
Tightening the connector did **not** resolve the intermittent USB fault.

A 90-second stream check spanning both arm movements and the Nav2 route
received 180 wrist images. Maximum source-frame interval was **0.528 s** and
maximum reception interval **0.513 s**; no reception gap exceeded one second.
Front and Astra streams also had no reception gap over one second. Joint states
arrived 895 times, with maximum source interval **0.137 s**.

MoveIt executed unchanged HOME and travel_stow successfully, without supervisor
fault samples. HOME took **8.137 s**, maximum joint error **0.01841 rad**;
travel_stow took **7.424 s**, maximum error **0.01382 rad**. Both were inside the
0.02 rad controller tolerance. Base permission returned after folding.

Two production Nav2 goals, 8 cm forward and return, both returned status 4. There
were zero supervisor fault samples and zero independent visual tracking losses;
all **53 RGB-D arrays contained both mapping views**. Maximum wheel radius was
**5.13 cm**, while the standalone camera observer reported **14.49 cm**. Its
13.65 cm forward estimate differs substantially from wheel odometry, so it is
not an accurate physical distance measurement in this scene.

A ±0.15 rad left/right/return sequence used independent source-aged lidar
tracking as the motion guard. Lidar maximum translation radius was **7.03 mm**,
while the camera observer reported **14.57 cm**. There were no tracking losses,
motion pauses or unmasked StopZone points. A controlled repeat using native
`Reg/Force3DoF=true` still produced **11.93 cm** of camera drift versus **11.32 mm**
in lidar. That experiment did not establish a correction and was reverted.
Production SLAM already uses that planar constraint; remaining camera drift
cannot be attributed solely to unrestricted roll/pitch. These tests have not
isolated image/depth calibration, scene geometry and visual-estimator error.

The last movement ended with the arm stowed, driver ARMED, both motion
permissions true and no current/latched faults. All finite clients and observers
ended. The existing production map and normal services were preserved. A cable
substitution on the same port remains the next physical isolation step; this
run did not identify the faulty component. A final eight-second snapshot after
recovery received 17 wrist frames, with latest source age 0.645 s. MoveIt
reported a valid folded state with no contacts; both permissions remained true
and current/latched faults were empty.

Local evidence under `.benchmarks/onboard-verification/`:

- `20261005-201849/`, `20261005-202325/`: camera, arm-validity and health snapshots.
- `20261005-203149/`: fresh wrist images and valid stow after USB recovery.
- `20261005-201957-arm-home/`, `20261005-202010-arm-travel_stow/`: MoveIt results.
- `20261005-202054-production-navigation/`: Nav2, map and dual-view measurements.
- `20261005-202206-rotation-comparison/`: independent lidar and camera comparison.
- `20261005-202511-rotation-comparison/`: reverted planar-observer experiment.
- `cable-tightened-streams.json`, `cable-tightened-usb-events.log`: source-frame
  intervals and kernel/service evidence after tightening the connector.

## Verification closeout for initial trials

Observations collected at 21:30–21:37 close the initial stage for **attended, unloaded functional
trials** in the existing 30 cm area. It retains the measured acceptance record
from 2026-10-04; no stopping trial, fault result or payload rating was invented
or re-dated. Follow-up checks used the installed runtime and sent no motion
commands. Earlier finite arm and Nav2 execution results remain the functional
movement evidence.

| Component | Result and evidence |
| --- | --- |
| Physical acceptance | Installed schema-4 record validated against the configured stow, hardware, Nav2 footprint and clearance. Worst recorded stop distance plus uncertainty is 0.047712 m against the 0.05 m budget. |
| Base limits | Installed production limits match acceptance: 0.03 m/s linear and 0.06 rad/s angular. Added payload remains 0 kg; accepted surface is dry concrete. |
| Arm / MoveIt | Current folded pose is collision-valid with no contacts; arm permission is true. Stored HOME and travel_stow executions passed within 0.02 rad. MoveGroup and arm trajectory action servers are available. |
| Nav2 | Planner, controller, navigator, behavior server and CollisionMonitor are active. NavigateToPose is available; the preceding two production goals passed. |
| SLAM inputs and map | 16 dual-view arrays all contained both views; 44 fused clouds arrived, minimum 687 points. The saved graph returned 65 poses and 195 links. Front 2D RGB, Astra RGB-D and measured fused lidar/depth are the mapping inputs. |
| Current safety state | ARMED, arm stowed, both permissions true, no current/latched faults. All six normal compute/device services were active. |

The final eight-second snapshot received 84 joint states, 42 scans, 18 wrist
images, 17 front images and 16 Astra RGB-D pairs. Front/Astra images contained
782/884 ORB features; image source ages were 0.10–0.46 s. The installed acceptance,
production-safety, Nav2 and EKF YAML values matched the tracked configuration.

Two exceptions remain outside the completed functional scope:

- **Wrist USB reliability:** tightening did not prevent the 20:25 disconnect.
  Its camera is an arm preview, separate from the two fixed mapping views.
  Vision-dependent manipulation with that stream remains unverified.
- **Independent visual distance accuracy:** standalone Astra odometry drifted
  during the small turns. Production EKF uses `/wheel/odometry` only; the
  diagnostic RGB-D odometry is not one of its inputs. Production SLAM still
  uses visual registration alongside lidar, so room-scale localization accuracy
  and precision placement remain unverified.

Initial trials therefore cover empty-arm pose/motion checks and supervised
local navigation/mapping. Base travel uses the accepted folded travel_stow.
Payload and higher speeds require new physical measurements; a larger route
requires a scope extension and further navigation verification. The operator
maintains the 30 cm test area during normal production operation. All finite
clients ended; the robot remained
folded, ARMED and quiet with normal services running.

Evidence under `.benchmarks/onboard-verification/`:

- `20261005-213055/`: current posture, camera freshness and healthy permissions.
- `verification-closeout-acceptance.json`: installed acceptance and speed checks.
- `verification-closeout-runtime.json`: action/lifecycle readiness, mapper inputs
  and saved map graph. These observations do not replace the recorded physical
  fault and stopping measurements.
