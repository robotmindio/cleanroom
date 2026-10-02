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

The full deployment of 32a8969 succeeded autonomously after sudo authentication;
the compute unit now uses KillMode=mixed and Nice=-5, and deployment markers agree.
The webcam confirms good lighting and the compact empty-gripper fold.

Deployed 47fdbef: acceptance now binds to tracked MPPI speed limits, the manual
driver uses those limits, and required base health is enforced in hold mode.
Depth now protects both base and arm. All 163 affected Python checks passed.
Normal hold mode keeps torque and recovers permission without new fault latching.

Final idle verification identified frequent false "joint state missing" events:
the arm workspace monitor rejected every future capture stamp, while Pi samples
occasionally arrive a few milliseconds ahead of compute. It now shares the
supervisor's bounded 50 ms source-clock tolerance and subtracts transport age
from its monotonic joint lease; stale data is not given a fresh full lease.

Live fault checks passed for lidar, depth and telemetry loss (4.9 cm maximum
wheel-odometry radius). Compute-driver SIGSTOP exercised the Pi command watchdog,
and motor-host restart stopped commands then recovered with fresh permission
(2.7 cm radius in that run). Webcam pairs were retained for each stop. These are
low-speed fault checks, not measured physical braking trials. Reports:
`.benchmarks/physical-acceptance/result.json` and
`.benchmarks/physical-acceptance-restarts/result.json`.

The remaining native failures have reproducible fixes. rclcpp's action removal
kept an expired weak registration, and class_loader unloaded the code needed
by weak control blocks still held by ROS executors. Pinned patches remove the
expired registration and retain plugin code until process exit; plugin objects
still undergo normal destruction. Three isolated MoveIt SIGINT trials exited
zero, and the native action regression passes. Nav2 now uses a tracked ten-second
lifecycle RPC deadline, shares discovery/response timing, honors success=false,
and receives its parameter file. Delayed, timed-out, refused and disappearing
service cases all pass; 98 affected Python checks pass. Deployment and production
restart verification are next.

Next: independent floor scale and confirmation of surface/payload and test speed
scope, then 30 stopping trials per direction and remaining fault tests (motor
diagnostics, replay, ROS restart, obstacle stop, arm workspace intrusion).
The authorized 30 cm commissioning tests are capped at 0.03 m/s and 0.20 rad/s;
production is faster. Never approve production speed or physical stopping
distance from slow wheel-odometry trials. Keep validated:false until physical
acceptance and independent map calibration have evidence.
