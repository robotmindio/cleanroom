# Robot verification follow-up — 2026-10-06

Runtime revision **`60c518b8ddd3`** was deployed successfully to compute and the
Pi using `scripts/deploy-split.sh`, including its verified re-arm. Initial
attended trials can continue at the existing unloaded acceptance limits:
**0.03 m/s, 0.06 rad/s, dry concrete, 0 g added payload**, inside the authorized
30 cm area. This is a local functional verification, not room-scale accuracy
or additional payload qualification.

## Root causes corrected

| Issue | Finding and correction |
| --- | --- |
| Laser requests TF in the future | LD06 stamped a completed sweep at publication and added positive ray intervals. The pinned SDK already supplies acquisition timestamps. The patch uses those timestamps and preserves range ordering: the configured reversed array starts at the newest ray with a negative interval; the other direction starts at the oldest ray with a positive interval. |
| RViz indexed-map sampler warning | Ogre validated a newly linked program before assigning its two sampler types to different texture units. Both initially used unit zero. The vendor patch removes that premature validation when reading the linker log, retaining actual linking and error checks. |
| Invalid Astra depth metadata | The device's factory query returned NaN for all calibration fields. Upstream's inverted validity predicate copied invalid intrinsics into depth CameraInfo. The patch rejects invalid factory data, publishes consistent finite defaults and explicitly reports that these defaults are uncalibrated. It also corrects principal-point offsets and preserves valid factory distortion. |
| MoveIt validity requests expire during planning | The old RPC budget was 100 ms. Production now allows 250 ms per request and a 350 ms validity lease, below the unchanged 450 ms joint freshness limit. Validity is dated from the queried pose's acquisition, so delayed replies cannot renew an old pose. Expired replies remain ineffective. |
| Deployment compilation overloads compute | The compiler limit used total RAM although compute had roughly 2 GB available and heavy swap use. It now uses available RAM and selects one compiler below 8 GB. Existing native builds also use one compiler. |
| Native rebuild requires an unavailable build tool | The tracked builder bootstraps pinned `ament_cmake_vendor_package` in the workspace, avoiding a privileged package-install handoff. |

The camera supervisor's liveness probe now requests only the image header,
avoiding pixel-array serialization. A finite Pi comparison showed a small
CPU reduction; it does not establish the source of remaining telemetry gaps.

The native sources remain pinned: LD06 SDK
`cac5d3d4c15522c6126ef65cfa8a65b08531a66b`, Astra
`f7e71d9ce806e788cb48d8580aac2c778fba4214`, RViz 14.1.23
`feb01669f1297df2af755ce9cd2ed18083e7a8b2`, and the vendor build tool from
ament_cmake 2.5.6 `5741cff5b9f83253bf3521bd8f44108fde3504ad`.

## Arm and MoveIt

Unchanged SRDF `home` and `travel_stow` targets were executed through
`/move_action` using the RViz defaults of 0.50 velocity and 0.40 acceleration.

| Final deployed run | Action duration | Settled maximum joint error |
| --- | ---: | ---: |
| `home` | 3.881 s | 0.01381 rad |
| `travel_stow` | 4.093 s | 0.01229 rad |

Both actions returned status 4 / MoveIt error code 1, within the 0.02 rad
controller tolerance. Driver logs recorded holding and resuming through brief
feedback gaps; neither action aborted. A brief stale validity verdict still
occurred during the return. These observations do not prove all input gaps have
been eliminated.

The test client previously evaluated feedback immediately after the action
reply, before its own two-second settling observation. In the final stow run,
that older sample had 0.02302 rad error; fresh settled feedback had 0.01229 rad.
The verifier now evaluates the settled samples and retains both readings in
its evidence. This fixes a false test failure; it does not change controller
tolerance or servo calibration.

Final live checks reported collision-valid stow with no contacts, maximum stow
error 0.00769 rad, driver ARMED, and both arm/base permissions true. Base
permission was correctly false while the arm was unfolded at HOME.

## Production navigation and SLAM

Two Nav2 goals, 8 cm forward and return, succeeded on the retained production
database. A short manual correction returned near the starting wheel pose.
The normal production profile remained active (`bounded_base_test=false`).

- Maximum observed radius: 6.12 cm in wheel odometry and 8.53 cm in the
  independent Astra observer. Visual forward displacement was 9.83 cm.
- No supervisor faults were recorded during these Nav2 goals. The test paused
  briefly for fresh independent camera tracking, then resumed.
- The saved graph contained 75 nodes across seven sessions. Global links joined
  the newly deployed session to saved observations, including 23143–23683 and
  23143–23937; the finite run recorded up to 46 visual inliers.
- Production still uses front RGB with measured fused lidar/depth, plus Astra
  RGB-D. The wrist camera remains an arm observation stream. The production EKF
  uses wheel odometry, not the independent verification odometry.

A subsequent finite ±0.15 rad turn and return completed. No unmasked chassis
returns or collision-monitor pauses were observed. Independent lidar tracking
had zero losses and maximum radius 1.10 cm; standalone visual tracking had six
lost samples and maximum radius 2.78 cm. Its improved result in this run does
not establish an optical calibration or eliminate the earlier rotation drift.
All temporary visual/lidar observers and motion clients exited.

The active `/home/nex/.ros/lekiwi_rtabmap.db` was retained throughout. No reset,
automatic deletion, HOME change, travel-stow change, safety acceptance change
or added autonomous motion-state policy was introduced.

## Timing and interface checks

During 35 seconds including the final turns, 1,051 workspace verdicts were
clear, the largest observed validity round trip was **49.16 ms**, and 700
supervisor diagnostic samples contained no faults.

The final stationary 20-second check recorded:

| Source | Samples | Maximum acquisition age | Maximum receive gap |
| --- | ---: | ---: | ---: |
| `/joint_states` | 198 | 76.2 ms | 160.1 ms |
| `/scan` | 100 | 202.0 ms | 277.2 ms |
| `/camera/depth/points` | 186 | 116.4 ms | 233.7 ms |

There were zero supervisor fault samples. Both endpoint transforms were
available for all 85 checked scans. A separate clock comparison measured about
4 ms Pi/compute offset; the original roughly 85 ms future request was not
explained by clock skew.

RViz loaded the patched workspace Ogre library, rendered the map and restricted
geometry, showed all three camera views and ARMED state, and connected its
MotionPlanning panel with scales 0.50/0.40. The new log contained no sampler
conflict or future-extrapolation message. TF cache warm-up messages occurred
across stack restarts. The optional `/recognize_objects` server is absent; this
does not block the tested arm planning or execution.

The Pi had about 2.9 GB available out of 4 GB, load around 2 on four cores and
throttling flags `0x0`. Twenty ping packets were received, with 1.70–9.19 ms
round trips. These finite checks did not establish Pi resource exhaustion or
network loss as the cause of occasional feedback gaps.

## Regression checks

- 61 tests passed across arm workspace monitoring, camera supervision and
  service/build installation.
- The isolated ROS graph test passed collision denial, recovery and input
  silence, followed by clean node exit. It ran in ROS domain 42, separate from
  the physical robot.
- Native C++ harnesses passed LD06 timestamp/range-order tests in both
  directions and Astra invalid/valid calibration and CameraInfo consistency.
- An actual OpenGL experiment reproduced validation failure before sampler
  assignment and success afterward; the patched RViz rendering was checked
  separately. A private-symbol Ogre harness could not link and is not counted
  as a passing test.

## Remaining work

1. **Wrist USB hardware path:** the Pi kernel recorded another disconnect at
   10:23:57, before capture recovery. Tightening the connector has not resolved
   it. Cable/connector, local power and controller/firmware causes remain
   unseparated. Substitute the cable on the same port first, then compare port
   and camera hardware as needed; no logged undervoltage or overcurrent was
   found.
2. **Astra optical calibration:** factory values are invalid and no measured
   Astra calibration file was installed. Finite FOV defaults prevent NaN but
   are approximate. Measured RGB/depth intrinsics, registration and scale are
   needed before claiming accurate visual translation. The upstream SDK/device
   reason for the invalid factory query remains unseparated.
3. **Intermittent feedback gaps:** arm execution now survives the observed
   gaps, but brief holds still occurred. Their remaining origin is not isolated
   by these finite checks. Distinguishing host serial polling, transport and
   compute scheduling requires simultaneous source-boundary timing.

Added payload, higher speeds and full-room mapping accuracy remain outside
this verification. Existing physical acceptance remains unchanged.

## Local evidence

Ignored artifacts are under `.benchmarks/onboard-verification/`: deployment
`20261006-deploy-validity.log`, arm runs `20261006-104131-arm-home` and
`20261006-104141-arm-travel_stow`, navigation
`20261006-104200-production-navigation`, turns
`20261006-104258-rotation-comparison`, final snapshot `20261006-104408`, timing
`20261006-final-timing.json`, validity `20261006-validity-latency.json`, calibration
`20261006-calibration-result.json`, regression tests
`20261006-regression-tests.log`, and RViz `20261006-rviz-closeout.png`.

At closeout, compute and all five checked device services were active, the arm
was folded and the driver armed with both permissions true. Finite test and
observer processes ended. Normal robot services and the user-facing RViz stay
running; no agent monitoring mode remains.
