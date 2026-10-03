# Physical braking verification

Completed: finite optical runner, full-resolution tag tracking, projective marker-plane rectification, fixed optical returns, collision-monitor readiness, capture-time speed checks, and zero-command bounded recovery. Shared indoor limits deployed as 6f6ce96: 0.05 m/s, 0.10 rad/s, 5 cm StopZone clearance. HOME and compact travel_stow unchanged.

Verified: dry concrete and 0 kg payload. Corrected run 20261003-193625 completed thirty forward and reverse repetitions after exploration. Worst eligible swept bounds 24.52/24.02 mm plus declared 10 mm uncertainty. Left has 14 eligible stops; right/CW/CCW one each. A 0.20 rad/s clockwise stop exceeded the clearance budget. Twenty-six motion regressions and 57/57 CTests pass. Deployment completed without operator intervention.

Pending: camera now shows only floor/support, not the robot or markers. Recover a fixed three-marker view before further movement. Confirm whether 35 mm measures the detected black edge; floor/lens/timing calibration is unverified. Remaining lateral/rotation repetitions, applicable fault cases, meaningful navigation and SLAM growth/closure are incomplete. Intermittent shared scan/depth/joint feedback gaps remain unresolved. Physical acceptance stays false; production SLAM database retained.

Next: with the camera view restored, run scripts/test-braking.py --directions left right rotation_cw rotation_ccw, then finish bounded navigation and fault verification. Evidence and historical findings: docs/physical-verification-20261003.md. No finite test runner remains active.
