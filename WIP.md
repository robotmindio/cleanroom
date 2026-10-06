# Production hardening

Scope: repair qualification, enforce motor-host motion limits, and stage split
releases before cutover. Keep the existing arm-hold and verified-deployment
re-arm behavior. No live deployment has been performed.

- Completed: qualification collection, test discovery/registration, and stale
  acceptance guidance corrected; motor actions enforce calibrated base/joint
  limits and uploaded trajectories must match the host calibration.
- Next: stage releases before the split deployment cutover.
- Verified: 56 qualification/braking/reload tests and 70 arm/motor-host tests
  passed; changed Python files pass Ruff.
- Pending: fresh build/integration checks and review of deployment fault paths.
