Gripper static tracking experiment

Completed: captured both free-jaw failures with production 689266f. The actual
servo Goal_Position stays at the planned endpoint while Present_Position stays
nine encoder counts short in either direction for the settling window. Current
is 6.5–13 mA, status is zero, temperature 38–39 C. Both goals abort at the existing
0.005 rad tolerance; the arm returned to travel_stow.

Next: stage and qualify P=48 (from 16), I=0, D=32. P=48 targets approximately three
counts of residual static error without integral contact windup. This prediction
is not hardware acceptance. Do not merge unless both strict endpoint tests pass,
idle tracking stays stable, and existing motor protection stays healthy. Do not
widen trajectory tolerance or current/torque limits. Contact-force and complete
object-manipulation acceptance remain separate and unqualified.

Evidence: private qualification/20261104-gripper-tracking/post-689/{result,telemetry}.json.
