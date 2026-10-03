# Physical verification — 2026-10-03

**Result: characterization completed partially; smaller indoor zones deployed;
physical acceptance remains false.**
The fixed measurement camera was retained. The three chassis AprilTag 36h11
markers (53, 69, 59) have a supplied black-edge side length of 35 mm.

## Recorded stopping trials

| Direction | Characterization trials | Worst swept-point bound | Longest receive-time stop bound |
| --- | ---: | ---: | ---: |
| Forward | 20 | 48.05 mm | 0.426 s |
| Reverse | 20 | 44.07 mm | 0.306 s |
| Left | 19 | 69.11 mm | 0.265 s |
| Right | 19 | 44.71 mm | 0.341 s |
| Clockwise | 4 | 62.86 mm | 0.261 s |
| Counterclockwise | 3 | 40.64 mm | 0.190 s |

These 85 trials include speed exploration and repeated exploration after runner
fixes. They are **not** thirty qualifying trials per direction at an accepted
speed. Commands reached 0.30 m/s and 1.20 rad/s. Collision slowdown reduced
commands to 35%; attained speed was recorded separately. Every recorded trial
was below the predeclared 80 mm budget including the declared 10 mm allowance.
That allowance has not been independently calibrated. The affine marker-plane
measurement does not establish floor-plane alignment or lens calibration.
Receive-time stopping bounds include camera transport delay; exposure clocks
were not independently synchronized.

## Physical blocker

Rotation brought persistent lidar returns into the rear-right StopZone:
x = -0.300 to -0.285 m, y = -0.296 to -0.274 m. All 25 stationary scans contained
points inside the zone. Native collision-monitor logs reported `StopZone`;
guarded velocity became zero despite nonzero return commands. A further bounded
retest recorded `StopZone` and zero guarded velocity. This sector is outside the
external camera view. Repetition and navigation goals could not be completed.

Replaying the marker observations against the original stationary reference
gave a largest estimated chassis-center bound of 0.135 m, with a 0.20 m
marker-offset allowance. This estimate inherits the plane-calibration limitation;
the authorized radius was 0.30 m.

## Software and runtime verification

- All 57 CTest checks passed; 122 affected unit checks passed.
- AprilTag refinement fixes missed detections. Quad decimation reduces detector
  cost; isolated malformed JPEG frames are recorded and skipped under the
  tracking lease. Ready ROS callbacks are drained after each spin.
- The runner keeps ROS alive during interrupt cleanup, sends zero, withdraws
  its commissioning lease, closes the camera and restores production.
- CI's first failure was Git ownership inside its container; the next failure
  was a missing Cyclone DDS runtime dependency. Both causes have tracked fixes.
- The folded arm remained stationary and MoveIt-valid. All ten Nav2 lifecycle
  nodes were active after restoration.
- Front RGB, Astra RGB/depth and lidar streams were fresh. Front RGB and Astra
  RGB showed scene features. No new loop closure was accepted; the existing
  111-node working graph was retained while registration remained unverified.

## Indoor zone revision and retest

After the fixture was moved, 31 stationary lidar scans contained no returns in
either the original stop zone or the proposed smaller zone. The closest observed
return was about 18 cm outside the folded footprint. The arm was still folded,
stationary and MoveIt-valid.

The tracked indoor configuration now uses a 56 x 54 cm stop zone: the unchanged
46 x 44 cm folded footprint plus 5 cm on every side. The slowdown zone is
70 x 68 cm, replacing the original 90 x 90 cm square. MPPI, velocity smoothing,
manual-driver limits and the physical test profile agree on 0.10 m/s translation
and 0.20 rad/s rotation. Both collision polygons are enabled in the saved RViz
configuration. These limits and the smaller clearance remain subject to physical
acceptance; no existing characterization was converted into qualifying evidence.

Stopping clearance now uses the largest body-point displacement from the
pre-stop pose: translation plus the rotation chord at the conservative body and
marker-offset radius. The earlier sum of frame-to-frame travel accumulated
stationary detector jitter. Raw path and residual rotation are still recorded.
The independent plane/distortion and timing-calibration limitations remain.

The attempted braking run failed before motion: at 16:55:32 the external camera
disconnected from USB; repeated address-enumeration failures (-71) left no device
or PipeWire source. A tracked preflight now opens that camera before stopping
production and includes GStreamer failure detail. It does not select the laptop
camera as a substitute.

A subsequent low-speed navigation test moved 2.1 cm before lidar, depth and joint
feedback became stale together. It did not complete the navigation route.
`arm_stowed=false` represented unverified stale joint feedback; the restored arm
readings were unchanged, and MoveIt still reported a valid pose. The runner now
reports that distinction instead of claiming the arm moved out of stow.
The maximum lidar age reported during the interruption was 2.7 seconds.

Two finite network probes recorded 2.5–3.3% packet loss and peak round-trip delay
of 0.44–0.85 seconds; compute received 53.5 Mbps during a five-second sample.
Both Wi-Fi power-save settings were off; Tailscale used its direct LAN path.
The Pi load was 1.3 across four cores, available memory 2.8 GiB and throttling
flags zero. These observations support investigating network contention rather
than relaxing the smaller guard's safety deadlines. Competing traffic has not
been isolated as a causal test, so its source is not established.

All 57 CTest checks and 94 targeted checks passed. Production was restored with
all ten Nav2 lifecycle nodes active, the arm valid and ARMED, and physical
acceptance as the only standing base-permission fault. No test monitor remained.

## Evidence and next step

Local evidence is under `.benchmarks/physical-braking/` in runs
`20261003-153223`, `20261003-153903`, `20261003-154351` and
`20261003-160631`. Each contains raw MJPEG, frame times, ROS feedback,
measurements and a result. Aggregated CSV, plot, lidar returns and restored
runtime checks are under `.benchmarks/physical-rig/`.

Reconnect the external camera while retaining its fixed view, resolve the
network interruption, then repeat the finite sequence at the new indoor limits.
New evidence is under `.benchmarks/physical-rig/indoor-*` and
`.benchmarks/navigation-test/`. Confirm actual surface and payload, establish the
measurement geometry and full-speed coverage, and complete remaining applicable
fault tests before populating qualifying measurements in
`config/safety_acceptance.yaml`. Its validation and trial counters were left
unchanged; the map database was preserved.
