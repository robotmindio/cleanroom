# Physical verification — 2026-10-03

**Result: characterization completed partially; physical acceptance remains false.**
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

## Evidence and next step

Local evidence is under `.benchmarks/physical-braking/` in runs
`20261003-153223`, `20261003-153903`, `20261003-154351` and
`20261003-160631`. Each contains raw MJPEG, frame times, ROS feedback,
measurements and a result. Aggregated CSV, plot, lidar returns and restored
runtime checks are under `.benchmarks/physical-rig/`.

Clear the rear-right lidar obstruction with the measurement camera fixed, then
repeat the finite sequence. Confirm actual surface and payload, establish the
measurement geometry and full-speed coverage, and complete remaining applicable
fault tests before populating qualifying measurements in
`config/safety_acceptance.yaml`. Its validation and trial counters were left
unchanged; the map database was preserved.
