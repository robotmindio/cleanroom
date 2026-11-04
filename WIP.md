Gripper static tracking experiment

Completed: captured both free-jaw failures with production 689266f. The actual
servo Goal_Position stays at the planned endpoint while Present_Position stays
nine encoder counts short in either direction for the settling window. Current
is 6.5–13 mA, status is zero, temperature 38–39 C. Both goals abort at the existing
0.005 rad tolerance; the arm returned to travel_stow.

Verified: P=48 passed native 105/105 compute and 16/16 device tests, CI, and both
ordinary MoveIt free-jaw goals. With planner constraints tightened to 1e-6 rad,
both exact endpoint tests still failed at 0.0051/0.0053 rad (four counts) against
the unchanged 0.005 rad controller tolerance. No stable exact-endpoint acceptance.
All motor health stayed normal; arm returned to travel_stow. Preserve these
failures at qualification/20261104-gripper-tracking/p48-exact-endpoints.

Next: stage and qualify P=64 (from 16), I=0, D=32. Three counts of expected residual
error leave room for encoder rounding; this remains a prediction pending real
tests. Do not merge unless both exact endpoint tests and eight-second holds pass,
idle tracking stays stable, and existing motor protection stays healthy. Do not
widen trajectory tolerance or current/torque limits. Contact-force and complete
object-manipulation acceptance remain separate and unqualified.

Evidence: private qualification/20261104-gripper-tracking/post-689/{result,telemetry}.json.
