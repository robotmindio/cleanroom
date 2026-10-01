# Reliability follow-up

Completed: Pi-integrated timestamped base pose, compute alignment across dropped
packets/restarts, pose-only EKF, native SLAM covariance policy, posture-dependent
FCL resting contacts in MoveIt/RViz, increased service scheduling priority.

Verified: 186 initial, 166 follow-up and 62 collision/service Python checks;
native build and resting-contact check; live MoveIt folded release/return (max
final error 0.0093 rad), folded pan +/-0.20 rad and return. All five moves
succeeded, including a 1.01 s joint-feedback gap and permission pause/resume.
Finite route passed both Nav2 goals, stayed inside a
13.3 cm measured radius, returned within 5.2 mm, and produced a visual closure
(21718--21825) plus three proximity recognitions. Feedback gaps paused/resumed.

Runtime installed on both machines. The live map dictionary was recovered by
RTAB-Map's native recovery and a clean launcher shutdown: 13,885 words, zero
missing references. Fixes now signal the launcher once and remove unnecessary
Wi-Fi-triggered service kills; they still need the compute unit refreshed.

Fresh RViz now declares the collision plugin before constructing its scene;
live loader reports LeKiwiRestFCL and the MotionPlanning panel loads correctly.
Offline graph rebuilding was rejected: it fragmented the connected map, so the
production database was preserved. Historical constraints still reject some
candidate closures; calibration/map accuracy need independent physical evidence.

Final runtime code is built on both machines; device units refreshed, production
stack restored. A clean shutdown took 5.24 s; MoveIt exited without escalation
with the longer grace period, and the mapper retained 20,109 dictionary entries
with no missing references. The cloud gate's real isolated ROS process also
exited zero after SIGINT. A finite hold showed zero encoder-resolved movement.

Next: after `sudo -v` on compute, run `scripts/deploy-split.sh` to install the
pending `KillMode=mixed` / Nice=-5 compute unit. Current global sudo timestamps
allow that authentication to be shared; restart permission alone does not allow
rewriting units. Deployment markers remain old until the full script succeeds.

Pending: compute administrator authentication for changed service configuration;
independent floor scale and confirmation of surface/payload for physical braking
acceptance; room lighting for further visual qualification. Never set acceptance
validated from wheel odometry alone.
