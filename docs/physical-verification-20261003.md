# Physical verification — 2026-10-03/04

**Stopping characterization completed in all six directions. Physical acceptance
remains false.** The operator confirmed 35 mm measures each detected black
marker square, excluding its white margin. Surface: dry concrete; added payload:
0 kg; arm: unchanged compact `travel_stow`. All maneuvers use the authorized
30 cm radius, independent camera tracking and finite test clients.

## Root causes corrected

- **Depth loss on the Pi:** kernel UDP buffers were capped at 212992 bytes,
  smaller than a complete raw cloud despite DDS requesting 8 MiB. The kernel
  recorded 794 receive-buffer drops in ten seconds. The device installer now
  persists 8 MiB maxima from `config/dds_socket_buffers.conf` before starting
  publishers. Compact obstacle clouds target 10 Hz; mapping images stay 2 Hz.
- **Shared receive gaps:** Pi was on 2.4 GHz while compute used 5 GHz. A tracked
  NetworkManager helper clones the saved connection, sets 5 GHz and installs a
  timed rollback before activation. The original connection remains available.
  Exact RF interference sources were not isolated.
- **Lidar self-returns:** ranges in the existing folded-body sectors exceeded
  the old mask. The same angular union is now split at 315 degrees to give the
  earlier sector sufficient range while staying within the measured footprint.
  Regression tests retain far returns and check the sector geometry. Rare
  transient StopZone detections have also occurred; their cause is not proven.
- **MoveIt perception gaps:** a second 5 Hz throttle discarded already bounded
  upstream frames. Removing it exposed a separate gate starvation: every new
  cloud replaced the pending frame before its joint TF became available. The
  gate now retains one frame until its transform arrives or the existing
  0.5-second wait expires. In successive 20-second probes, raw depth stayed
  below 0.16 s gaps; processed output went from 0.60 s gaps, then 1.00 s at
  the gate, to **0.211 s** after both fixes. The final probe had zero workspace
  denials. Rate zero disables the second throttle in
  [MoveIt 2.12.4's updater](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp#L182).
- **Test validity:** shared launch defaults now read production speed limits;
  startup stops when CollisionMonitor reduces the requested test speed. Sensor
  loss timing uses the final live capture rather than the earlier SSH stop
  request. Cleanup restores production even if the entire test ROS tree died.
- **Measurement:** full-resolution marker tracking, projective rectification of
  the tilted plane, fixed optical returns and terminal capture intervals replace
  decimated detection, affine geometry, drifting wheel-only returns and speed
  checks diluted by stationary startup. These fixes do not independently
  calibrate floor alignment, lens distortion or exposure timing.

Simultaneous 60-second source/receiver probes after the network and buffer fixes:

| Measurement | Pi | Compute |
| --- | ---: | ---: |
| UDP receive-buffer drops | 0 | 0 |
| Largest depth receive gap | 0.152 s | 0.173 s |
| Largest depth capture age | 0.022 s | 0.077 s |
| CPU busy | 32.8% | 13.2% |

Compute joint gaps were at most 0.150 s; filtered lidar gaps at most 0.219 s.
All 1200 supervisor samples reported only unvalidated physical acceptance.
This finite sample establishes improvement, not uninterrupted future operation.
Depth and arm-perception deadlines are now 0.5 s, with joint freshness at 0.45 s.

## Nominal braking

Corrected evidence: `20261003-193625`, `20261003-211714` and
`20261003-214655` under `.benchmarks/physical-braking/`.
Each direction completed thirty algorithm-eligible repetitions; counts below
include eligible exploration. Translation was 0.05 m/s and rotation 0.10 rad/s.

| Direction | Eligible stops | Worst swept bound | Stop-time upper bound |
| --- | ---: | ---: | ---: |
| Forward | 31 | 24.52 mm | 0.416 s |
| Reverse | 31 | 24.02 mm | 0.422 s |
| Left | 30 | 25.15 mm | 0.812 s |
| Right | 31 | 18.83 mm | 0.311 s |
| Clockwise | 30 | 17.42 mm | 0.204 s |
| Counterclockwise | 32 | 17.76 mm | 0.192 s |

Add the declared **10 mm uncertainty** to each bound. It is not independently
surveyed. Earlier affine/higher-speed exploration remains historical evidence,
not substituted into this corrected series. Clockwise 0.20 rad/s nominal braking
exceeded the 50 mm clearance including uncertainty.

## Fault-stop boundary

After transport correction, depth loss at 0.05 m/s measured 40.026 mm plus
10 mm uncertainty, exceeding the predeclared 50 mm budget. Lidar loss during
0.10 rad/s rotation measured 40.335 mm plus 10 mm, also exceeding it. Failed
runs remain recorded. Limits are **0.04 m/s and 0.08 rad/s**; the StopZone remains
56 x 54 cm, the folded footprint plus 5 cm on each side. MPPI, velocity smoothing,
manual driver, spin behavior, bounded launch and test profile share those caps.

At 0.04 m/s all five finite linear fault checks passed optically and functionally:

| Fault | Swept bound | Stop-time upper bound |
| --- | ---: | ---: |
| Lidar service loss | 30.48 mm | 0.615 s |
| Depth service loss | 32.60 mm | 0.683 s |
| Telemetry loss | 29.00 mm | 0.662 s |
| Compute driver suspension | 33.89 mm | 0.672 s |
| Motor-host restart | 26.73 mm | 0.532 s |

Add 10 mm to these distances. Evidence:
`.benchmarks/physical-optical-faults/20261003-235925/`.
Services and permissions recovered; the compact arm was retained. One short
workspace-permission interruption during recovery cleared within the runner's
bounded pause. No lost telemetry was presented as a changed arm pose.

At 0.08 rad/s the same five rotation fault checks passed:

| Fault | Swept bound | Stop-time upper bound |
| --- | ---: | ---: |
| Lidar service loss | 20.01 mm | 0.318 s |
| Depth service loss | 28.37 mm | 0.527 s |
| Telemetry loss | 25.23 mm | 0.491 s |
| Compute driver suspension | 25.97 mm | 0.504 s |
| Motor-host restart | 18.86 mm | 0.333 s |

Add the same declared 10 mm. Evidence:
`.benchmarks/physical-optical-faults/20261004-000448/`.

After both MoveIt perception fixes, the affected sensor-loss cases were repeated
at the final caps. All passed:

| Fault | Translation swept bound / time | Rotation swept bound / time |
| --- | ---: | ---: |
| Lidar loss | 33.89 mm / 0.725 s | 31.05 mm / 0.608 s |
| Depth loss | 34.50 mm / 0.734 s | 29.57 mm / 0.602 s |

Again add the declared 10 mm. Final sensor evidence:
`.benchmarks/physical-optical-faults/20261004-002413/` and `20261004-002549/`.
The repeat protects against earlier incidental workspace stops confounding the
sensor-fault bounds. Driver/host watchdog cases retain their recorded evidence.

## Whole ROS crash and navigation

At the final 0.04 m/s translation limit, the test killed and verified **31 owned
ROS processes**. The independent camera observer and Pi remained alive. Swept
stopping bound: **29.84 mm + 10 mm**, time upper bound **0.615 s**. Production
then re-armed with fresh telemetry and arm permission; the finite restored probe
confirmed 81 joint samples, unchanged folded pose, MoveIt validity and all ten
Nav2 lifecycle nodes active. Evidence:
`.benchmarks/physical-ros-restart/20261004-000626/`.
The functional ROS-restart flag is recorded; overall acceptance stays false.

The separate fresh-map run `20261004-000738` completed **12 Nav2 goals**, six
forward-and-return cycles with 8 cm requested goals and fixed optical returns.
Each forward goal produced **50.5–58.4 mm independently observed translation**,
consistent with Nav2's 30 mm goal tolerance. The largest wheel radius was
51.5 mm from that test center. All goals succeeded. The map saved **11 nodes**,
with 212–273 genuine nonzero 3D features per node, and accepted a **spatial
proximity closure from node 143 to node 1 with 35 visual inliers**. Global
appearance-only closure was not observed; a spatial closure is the actual result.
The saved graph contains that additional closure link. Both RGBD camera feeds
and the combined depth/lidar scan remained configured. The production database
was not overwritten by the test database.

That first route also exposed the short MoveIt perception interruptions fixed
above; it is not evidence that those interruptions never happened. A subsequent
four-goal route after both fixes completed successfully with **zero workspace
permission withdrawals**. Its two independently observed forward movements were
55.15 and 55.21 mm, maximum wheel radius 55.0 mm, and its fresh graph saved four
nodes. Evidence: `.benchmarks/physical-navigation/20261004-002217/`.

## Remaining qualification

The provisional marker-plane evidence does not establish an independently
bounded floor/lens measurement error. The acceptance record now contains the
185 eligible nominal stops and conservatively rounded distances, including
the larger measured moving fault bounds. It remains `validated: false`.
The declared 10 mm measurement error is still unverified: the floor ruler's
graduations are absent from the external camera image. Moving diagnostic and
duplicate-telemetry stopping distances also remain unmeasured; the bottle must
be removed before those two cases can attain the test speed.
Meaningful Nav2 movement, fresh-map growth and spatial closure were observed;
this small route does not establish whole-house accuracy or global relocalization. The production SLAM database is
retained; fresh test databases are separate artifacts.

## Evidence

Raw MJPEG, frame times, ROS feedback, measurement JSON and failure results are
under `.benchmarks/physical-braking/`, `.benchmarks/physical-optical-faults/`,
`.benchmarks/physical-ros-restart/`, `.benchmarks/physical-navigation/` and
`.benchmarks/physical-rig/`. The latter contains nominal aggregate statistics,
deployment logs and paired buffer/network timing probes. Earlier completed
full ROS crash verification killed and checked 31 owned ROS processes while an
independent camera measured the stop; it did not kill the Pi or camera observer.

## Final software and restored-runtime check

Runtime revision **c6afe474f597** was deployed to compute and Pi without operator
intervention. A complete CTest rerun passed **57/57 checks in 36.56 seconds**,
including all nine launch tests. The preceding run's sole failure was an obsolete
assertion requiring the former 3 Hz depth profile; it now checks the deployed
10 Hz profile. Targeted configuration/supervisor checks passed 53/53.

The final eight-second production probe received 81 complete joint samples with
zero position span on every arm joint. The driver was **ARMED**, compact stow
and arm permission true, MoveIt state validity true, and all ten Nav2 lifecycle
nodes active. The only remaining base-permission denial was **physical acceptance
not validated**; bounded commissioning mode was false. Live parameter queries
confirmed 0.04 m/s, 0.08 rad/s, 0.5 s perception freshness, and no second MoveIt
cloud throttle. The final photo confirms the compact fold and all three tags.

Artifacts: `.benchmarks/physical-rig/final-ctest-confirm.log`,
`final-production-runtime.log` and `final-production-camera.jpg`. Interrupted Pi
services are active, temporary fault firewall rules and rollback timers absent.
Finite test clients have exited; production is restored and no test monitor stays
running. Subsequent commits update this report only; the recorded runtime
revision identifies the tested deployment.

## Additional live gates — 2026-10-04

| Case | Actual input and response |
| --- | --- |
| External obstacle | Operator's bottle appears in Astra RGB and depth and the lidar StopZone. An otherwise permitted 0.04 m/s request was forced to zero; wheel translation stayed below 0.7 mm. This was a stationary blocking check. |
| Arm workspace | A temporary MoveIt collision object at the wrist produced FCL contacts, withdrew permission and rejected an arm goal. All six measured joints remained unchanged. This tests injected planning-scene geometry, not camera detection of that object. The object was removed. |
| Motor diagnostics | ERROR injected only into authenticated outgoing observations withdrew base permission but initially left arm permission true. The normal-mode override was the cause. Removing that override makes both capabilities obey their health gates; the driver's existing feedback-gap hold remains intact. |
| Duplicate telemetry | Exact copies of the last successful authenticated observation did not renew state leases. Driver/joint freshness expired and permission was withdrawn; normal fresh observations restored permission. |

The last two cases used the real servo host and driver **at rest**, preserving
servo registers and the local watchdog. Injection expired after four seconds;
a finite native test unit and independent rollback restored the production
host. Final repeat on revision `7c3ef989cdab`: motor-error permission response
upper bound 0.432 s, replay response upper bound 1.149 s, maximum joint change
0 rad in both cases; camera swept excursions below 0.4 mm. These response times
include the fault request and transport; they are not moving stopping times.
All applicable functional fault flags are now recorded as passed. The full
CTest suite after the correction passed **57/57 in 154.53 s**, including nine
launch checks. Targeted supervisor/driver/torque checks passed 151/151.

Evidence: `.benchmarks/acceptance-gates/20261004-083003/`,
`20261004-083219/`, `.benchmarks/physical-telemetry-gates/20261004-084631/`
and `.benchmarks/physical-rig/arm-health-fixed-ctest.log`.
