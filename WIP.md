# Physical braking verification

Completed: finite camera runner, 35 mm marker specification, production-bounded test limits, camera/callback/interrupt fixes, and CI ownership/dependency fixes.

Verified: 57/57 CTests; 122 affected tests; 85 measured stopping trials, fresh sensors, stationary folded arm, active Nav2 and valid MoveIt pose. Details: docs/physical-verification-20261003.md.

Completed: smaller 56 x 54 cm stop zone, 70 x 68 cm slowdown zone, matching 0.10 m/s and 0.20 rad/s limits; RViz polygons; farthest-excursion stopping measurement and camera preflight.

Verified: 57 CTests and 94 affected tests; tracked zones active on both machines; arm remains folded, stationary and valid. The new position cleared the old StopZone returns.

Latest: camera reconnected. Shared readiness now waits for active CollisionMonitor. At 0.10 m/s reverse stopping plus the declared allowance exceeded 50 mm; translation is capped at 0.05 m/s. Decimation 2 detects all three visible markers. Dry concrete, zero added payload and marker tilt following the modeled Astra mount are confirmed.

Next: run the complete finite braking sequence using capture-time speed intervals and bounded zero-command recovery for feedback gaps. Interrupted/unattained trials are retained but excluded from repetition counts. Then test navigation/SLAM and remaining fault cases, restore production and stop the test runners.

Pending: thirty qualifying trials per direction, full-speed coverage, floor-plane calibration, actual payload/surface confirmation, remaining fault tests and verified SLAM registration. Physical acceptance remains false. Check CI for the new dependency fix.
