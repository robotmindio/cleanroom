# Production hardening

Scope: repair qualification, enforce motor-host motion limits, and stage split
releases before cutover. Keep the existing arm-hold and verified-deployment
re-arm behavior. No live deployment has been performed.

- Completed: qualification collection, test discovery/registration, and stale
  acceptance guidance corrected; motor actions enforce calibrated base/joint
  limits and uploaded trajectories must match the host calibration.
- Completed: staged split releases with full pre-cutover qualification, artifact
  manifests, retained previous installations, and stable service paths.
- Completed: device builds omit unused compute plugins/native overlays and run
  15 tracked role-specific test targets; CI covers this build variant.
- Next: complete the real staged build, then remove this checkpoint note and
  finalize draft PR #23. The staged vendor tooling, class loader, and rclcpp
  builds passed; live installed binary checksums are unchanged.
- Verified: 56 qualification/braking/reload tests and 70 arm/motor-host tests
  passed; changed Python files pass Ruff.
- Verified: fresh package build, all 61 CTest targets (nine launch tests), full
  Ruff/flake8/ShellCheck, both URDFs, package XML, and configuration/map YAMLs.
  Staging success/failure and activation regressions passed.
- Verified: device-only package build and all 15 tests passed; compute rebuild,
  sealed-release guards, migration settings, and MoveIt integration passed.
- Verified: real isolated device staging built sensor drivers, passed all 15
  tests, and sealed/verified its artifact manifest. The real compute native
  overlay and sensor drivers built successfully; package/qualification is next.
