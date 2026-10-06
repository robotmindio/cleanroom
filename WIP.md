# Loaded stopping acceptance

- Completed: reported 200 g arm cycle, loaded rotations and production Nav2
  forward/return. Reports: docs/arm-payload-verification-20261006.md and
  docs/base-payload-verification-20261006.md.
- Verified: production stopping runner records payload without restarting
  services; 2 regression tests passed.
- Pending: external USB camera currently excludes the robot; request to re-aim
  it at the chassis/floor markers has been sent. No stopping acceptance changed.
- Next: fresh camera calibration, six directions with five qualified loaded
  stops each, affected moving fault stops, then register measured 0.2 kg evidence
  in config/safety_acceptance.yaml and install on both hosts.
- Keep torque on; do not restart motor services. Keep motion inside 30 cm and
  preserve the production SLAM database. Remove this file when acceptance closes.
