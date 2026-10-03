# Physical braking verification

Completed: finite camera runner, 35 mm marker specification, production-bounded test limits, camera/callback/interrupt fixes, and CI ownership/dependency fixes.

Verified: 57/57 CTests; 122 affected tests; 85 measured stopping trials, fresh sensors, stationary folded arm, active Nav2 and valid MoveIt pose. Details: docs/physical-verification-20261003.md.

Completed: smaller 56 x 54 cm stop zone, 70 x 68 cm slowdown zone, matching 0.10 m/s and 0.20 rad/s limits; RViz polygons; farthest-excursion stopping measurement and camera preflight.

Verified: 57 CTests and 94 affected tests; tracked zones active on both machines; arm remains folded, stationary and valid. The new position cleared the old StopZone returns.

Next: reconnect the external measurement camera (USB enumeration error -71), investigate network interruption (up to 0.85 s RTT and packet loss), then repeat scripts/test-braking.py and bounded navigation. The live navigation retest stopped after 2.1 cm for simultaneous stale feedback. Production is restored; no test monitor remains.

Pending: thirty qualifying trials per direction, full-speed coverage, floor-plane calibration, actual payload/surface confirmation, remaining fault tests and verified SLAM registration. Physical acceptance remains false. Check CI for the new dependency fix.
