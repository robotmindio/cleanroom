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

Verified: P=64 passed CI, 105/105 compute and 16/16 device checks, then the exact
closing goal and eight-second hold. Exact opening still failed at 0.00515 rad:
the real servo goal was 0.998803 rad for a 1.0 rad request, adding 0.001197 rad
of truncation bias to three counts of tracking error. LeRobot's installed
RANGE_0_100 encoder uses int(), which confirmed the bias without motor I/O.

Implemented: the owned, unconnected bus is replaced before connect with a
gripper-only nearest-count encoder. Other motor conversions delegate unchanged.
56 unit tests pass. The actual installed LeRobot codec, using a nonexistent
port, emits count 2801 instead of 2800 and round-trips all 1458 encoder counts
exactly in both drive directions. No serial connection was made for this proof.

Next: stage and qualify this rounding correction with P=64, I=0, D=32. Do not
merge unless both exact endpoint tests and eight-second holds pass,
idle tracking stays stable, and existing motor protection stays healthy. Do not
widen trajectory tolerance or current/torque limits. Contact-force and complete
object-manipulation acceptance remain separate and unqualified.

Evidence: private qualification/20261104-gripper-tracking/post-689/{result,telemetry}.json.
