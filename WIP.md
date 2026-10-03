# Physical braking verification

Completed: finite optical runner, full-resolution tag tracking, projective marker-plane rectification, fixed optical returns, collision-monitor readiness, capture-time speed checks, and zero-command bounded recovery. Shared indoor limits deployed as 6f6ce96: 0.05 m/s, 0.10 rad/s, 5 cm StopZone clearance. HOME and compact travel_stow unchanged.

Verified: dry concrete and 0 kg payload. Corrected run 20261003-193625 completed thirty forward and reverse repetitions after exploration. Worst eligible swept bounds 24.52/24.02 mm plus declared 10 mm uncertainty. Left has 14 eligible stops; right/CW/CCW one each. A 0.20 rad/s clockwise stop exceeded the clearance budget. Twenty-six motion regressions and 57/57 CTests pass. Deployment completed without operator intervention.

The operator repositioned the robot and confirmed 35 mm excludes white borders. All three markers now track (637/639 valid frames). Run 20261003-203732 moved less than 18 mm before self-returns inside the footprint triggered StopZone: measured 300–302 degree ranges were beyond old 0.165/0.192 m mask bounds; 200 stationary raw scans also found 278.1 degrees at 0.217 m, 1 mm outside its bound. The tracked existing sectors now retain at least 10 mm range margin, without extending masked angles or removing returns beyond the body.

Pending: floor/lens/timing calibration, remaining lateral/rotation repetitions, applicable fault cases, meaningful navigation and SLAM growth/closure. Intermittent shared scan/depth/joint gaps remain unresolved; simultaneous stationary source/receiver probes found healthy Pi sensing and no stalls or packet loss in that 15-second sample. Physical acceptance stays false; production SLAM database retained.

Next: with the camera view restored, run scripts/test-braking.py --directions left right rotation_cw rotation_ccw, then finish bounded navigation and fault verification. Evidence and historical findings: docs/physical-verification-20261003.md. No finite test runner remains active.
