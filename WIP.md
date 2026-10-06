# Loaded stopping acceptance

- Completed: installed reported 200 g arm cycle, loaded rotations and production
  Nav2 forward/return. Reports in docs/*payload-verification-20261006.md.
- Verified: finite production runners preserve normal services and torque.
  Five regression checks pass for production preservation, independent scan
  geometry, measurement windows and optical terminal speed estimation.
- Root measurement fix: RTAB-Map correspondence residual covariance was being
  interpreted as pose uncertainty. Independent raw scans now use actual wall
  geometry covariance and removal of whole angular sectors; raw observations
  and modeled uncertainty are retained. The 50 mm / 1.5 s stop budgets and the
  existing declared 20 mm measurement error remain unchanged.
- Pending: five qualified loaded stops in each of six directions and eight
  affected moving fault cases, then register new 0.2 kg evidence and install
  the acceptance file on both hosts. Do not relabel historical unloaded trials.
- Keep torque on, preserve the production map and use the robot cameras.
  Keep movement within the authorized 30 cm radius. Finite test clients must
  exit after verification. Remove this file only when acceptance closes.
