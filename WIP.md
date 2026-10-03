# Physical braking verification

Completed: finite camera runner, 35 mm marker specification, production-bounded test limits, camera/callback/interrupt fixes, and CI ownership/dependency fixes.

Verified: 57/57 CTests; 122 affected tests; 85 measured stopping trials, fresh sensors, stationary folded arm, active Nav2 and valid MoveIt pose. Details: docs/physical-verification-20261003.md.

Next: clear persistent rear-right StopZone returns without moving the measurement camera; rerun scripts/test-braking.py and complete remaining fault/navigation tests. Production is restored; no test monitor remains.

Pending: thirty qualifying trials per direction, full-speed coverage, floor-plane calibration, actual payload/surface confirmation, remaining fault tests and verified SLAM registration. Physical acceptance remains false. Check CI for the new dependency fix.
