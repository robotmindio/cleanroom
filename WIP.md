# Loaded acceptance installation

- Completed: installed reported 200 g arm/navigation checks, thirty qualified
  loaded stops and eight affected moving faults. Record validated against the
  production profile and written to config/safety_acceptance.yaml.
- Verified: final MoveIt state valid, ARMED/stowed, both permissions true,
  read-only torque state enabled, all camera and range streams present.
- Next: fast-forward deployment checkouts, install the tracked acceptance record
  on compute and Pi, compare file hashes and confirm motor-host PID unchanged.
- Do not restart motors, disable torque or change HOME/travel_stow/map.
- Remove this file after both installations are verified; finite clients exited.
