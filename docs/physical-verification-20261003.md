# Physical verification — 2026-10-03/04

**Physical acceptance completed: `validated: true`.** Scope: attended autonomous
base operation on dry concrete, 0 kg added payload, unchanged compact
`travel_stow`, and an operator at the independent motor-power stop.
Production limits are **0.03 m/s and 0.06 rad/s**. The StopZone remains 56 × 54 cm,
with 50 mm clearance outside the accepted 46 × 44 cm footprint.

## Measurement and exclusions

The operator measured the floor AprilTag's black square as **44 × 44 mm** and
the three body squares as 35 × 35 mm, excluding white margins. The fixed USB
camera records 1280 × 960 native MJPEG and capture/receive timestamps. Tracking
uses floor projective rectification, raised-marker magnification correction,
and multiple marker headings. The current primary marker is 69; marker 59
remains readable when the third square is obscured.

The declared **20 mm measurement error** leaves **30 mm for measured stopping**
inside the original 50 mm budget. Each accepted window is checked against 256
independent one-pixel floor-corner perturbations, sixteen floor-registration
subsets, observed stationary pairwise swept jitter, and a 1 mm dimensional
reserve. The largest accepted window's calculated error is **13.669 mm**.
This is a conservative model for this fixed rig, not a surveyed camera calibration.
The visible 7 mm ruler is an independent dimensional cross-check; its blurred
inch labels were not used as exact calibration measurements.

Swept distance bounds use translation plus rotation of a point up to 0.53 m
from the tracked marker, retaining raw frame excursions. Stop timing uses the
median of eight terminal observations and trailing three-frame medians; each
median keeps its newest receive timestamp, adding observation delay. This
removes isolated tag noise from the time estimate without filtering the raw
swept-distance bound. Sensor, telemetry and remote restart fault timing starts
from the last valid hardware capture; SSH request travel is recorded separately.
Whole-ROS timing starts at the verified local process kill.

Historical runs remain on disk. The old forward/reverse view could not be
registered reliably to the floor and was replaced. Six reverse windows lacked
an observed raw starting marker and were excluded; new stops complete the
required count. A rotation/lidar window had a calculated 20.069 mm error and
was replaced by a new window bounded at 11.488 mm. No failed recording was
rounded into a pass. Only the individually qualified windows listed in the
[evidence manifest](physical-acceptance-evidence-20261004.json) contribute.

## Nominal braking and accepted bounds

All **186 nominal stops** meet speed coverage, stopping distance, latency,
frame-gap and error requirements. Earlier nominal tests at 0.05 m/s / 0.10 rad/s
and later 0.04 m/s cover the lower final caps. At least thirty independently
qualified nominal stops remain in each direction. The bounds below also include
applicable moving faults and conservatively retain the larger original or
floor-reconstructed result. Angular fault maxima cover both rotation directions.

| Direction | Qualified nominal stops | Accepted swept bound, mm | Bound + 20 mm error, mm |
| --- | ---: | ---: | ---: |
| forward | 31 | 27.712 | 47.712 |
| reverse | 30 | 23.219 | 43.219 |
| left | 31 | 26.632 | 46.632 |
| right | 32 | 24.072 | 44.072 |
| rotation_cw | 30 | 27.596 | 47.596 |
| rotation_ccw | 32 | 27.596 | 47.596 |

The worst bound including error is **47.712 mm**, inside the 50 mm clearance.
The maximum qualified moving fault stop-time bound is **0.789 s**. The record
retains **1.149 s** as its maximum command-stop latency because the earlier
at-rest authenticated replay-denial check had that conservative upper bound;
the predeclared limit remains 1.5 s.

## Moving fault stops

Each of seven faults passed while translating and rotating at the final caps.
A separate whole-ROS crash also passed: **15 moving fault stops** in total.
Permissions were withdrawn, the base stopped, and current healthy telemetry
and arm permission were required for recovery. The host's watchdog remained
active throughout authenticated diagnostic/replay injections.

| Fault | Motion | Swept bound, mm | Stop-time upper bound, s |
| --- | --- | ---: | ---: |
| scan_disconnect | linear | 27.457 | 0.789 |
| depth_disconnect | linear | 27.712 | 0.741 |
| telemetry_loss | linear | 25.813 | 0.612 |
| compute_command_loss | linear | 22.636 | 0.650 |
| host_restart_stops_then_gated_rearm | linear | 8.518 | 0.177 |
| motor_diagnostic_fault | linear | 12.115 | 0.336 |
| telemetry_replay_or_duplicate | linear | 22.804 | 0.681 |
| scan_disconnect | angular | 26.046 | 0.524 |
| depth_disconnect | angular | 25.854 | 0.579 |
| telemetry_loss | angular | 26.126 | 0.603 |
| compute_command_loss | angular | 24.641 | 0.457 |
| host_restart_stops_then_gated_rearm | angular | 7.824 | 0.011 |
| motor_diagnostic_fault | angular | 15.792 | 0.298 |
| telemetry_replay_or_duplicate | angular | 27.595 | 0.579 |
| whole_ros_command_loss | linear | 21.973 | 0.739 |

Add the same 20 mm error to every swept bound. Full-ROS injection killed and
verified **31 owned processes**; the independent camera and Pi remained alive.
The production service was restored after every finite test. Native rollback
timers cover interrupted Pi injections and test firewall rules.

The higher-speed boundary is retained as failed evidence: 0.05 m/s sensor-loss
and 0.10 rad/s rotation cases exceeded the original budget; the final 44 mm
floor-reference lidar-loss test at 0.04 m/s measured **38.088 + 20 mm**, also
exceeding it. These observations justify the lower final caps without enlarging
the obstacle zone or the stopping-distance limit.

## Other applicable gates

- The independent physical motor-power stop removes actuator energy without ROS.
- A real bottle in the lidar StopZone blocked an otherwise authorized command;
  removing it cleared the block. This was stationary blocking, not a moving
  approach-distance test.
- A temporary wrist planning-scene obstacle produced FCL contacts, withdrew arm
  permission and rejected an arm goal with zero joint change. It was removed.
- Authenticated diagnostic ERROR and exact duplicate observations withdraw both
  capability permissions; old observations do not renew freshness leases.
- Unauthorized ZMQ clients are rejected; the DDS control plane is confined to
  loopback/bridge topology; rosbridge is disabled by tracked defaults.
- No bumper, IMU or battery-state monitor is fitted. Their tests are explicitly
  not applicable. Acceptance grants no unattended scope or added payload.

## Root fixes and navigation evidence

Pi DDS receive-buffer maxima are persisted at 8 MiB; obstacle clouds target
10 Hz and mapping images 2 Hz. Paired sixty-second probes after buffer and
5 GHz transport fixes recorded zero UDP drops, depth gaps 0.152 s on Pi and
0.173 s on compute, and CPU busy 32.8% / 13.2%. A later Wi-Fi disconnect/reconnect
restored direct tailnet transport and DNS; its precise wireless cause was not
proved. RGB JPEG quality is explicitly 70 and depth remains lossless.

MoveIt's redundant cloud throttle was removed, and its TF gate now keeps one
pending frame until transformed or expired. Final twenty-second output gaps
were at most 0.211 s. Normal hold mode now retains health gates instead of
unconditionally overriding permission; healthy inputs restore permission.
Deployments can transfer the already verified pushed Git revision over native
SSH when the Pi cannot fetch its origin. No named pose or CAD arm was changed.

The fresh-map route completed twelve meaningful Nav2 goals with 50.5–58.4 mm
observed forward travel. It saved eleven nodes with genuine 3D visual features
and a spatial proximity closure, 143 → 1, with 35 visual inliers. Four subsequent
goals passed with zero workspace withdrawals; later bounded repetitions also
completed their Nav2 goals. The depth, 2D camera and lidar feeds remain configured.
Global appearance-only closure and whole-house accuracy were not established by
a route restricted to 30 cm. The production SLAM database was preserved.

## Reproducible evidence and delivery

The [manifest](physical-acceptance-evidence-20261004.json) lists selected windows,
counts, fault bounds, reference metadata, file sizes and SHA-256 hashes. Original
MJPEG, camera timestamps, ROS feedback and measurements remain in `.benchmarks/`.
The repeatable registration, qualification and closure programs are in
`.benchmarks/physical-rig/`. The tracked acceptance file binds these results to
footprint, stow, hardware, payload, surface, revision and speed limits. A changed
captured stow invalidates it.

Software checks and the final native deployment/runtime result are recorded
below after delivery. No persistent measurement or test-motion client is left
running.
