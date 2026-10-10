# Deferred acceptance and deployment work

This file is the single list of work that cannot be completed from this source
checkout without physical hardware, site measurements, deployment credentials,
an external service, or a qualified GPU host. None of the items below may be
marked complete from unit tests or loopback simulation alone.

The current physical acceptance record is validated for the restricted scope
listed below. Outstanding work limits expansion of that scope; simulation and
unit tests cannot grant or extend physical acceptance. Runtime capability gates
remain enforced in both recoverable and strict modes.
Each section names its completion condition and its tracking issue.

## Physical safety hardware and acceptance

Owner: robot integrator and safety reviewer at the deployment site.

Done when `config/safety_acceptance.yaml` holds a reviewed record with
`validated: true` for the deployed revision, including every stopping trial and
fault test, and the production profile requires every physical input it names.
Tracked in [#6](https://github.com/robotmindio/cleanroom/issues/6).

The accepted scope is attended autonomous base motion on dry concrete with the
installed reported 200 g load, at 0.25 m/s and 0.40 rad/s, with an operator at the
physical motor-power stop. The recorded software, stow, stopping measurements
and fault results are in `config/safety_acceptance.yaml` and its dated evidence
report. Bumper, IMU and battery monitoring are absent. The full-hardware items
below remain necessary before claiming those functions or expanding to
unattended operation.

- Install bumper/contact sensing, publish its real state on
  `safety/bumper_active`, and set `require_bumper: true` in
  `config/safety_production.yaml`.
- Qualify obstacle coverage for any expanded operating scope. The tracked LD06
  body mask is 100 degrees and the production minimum usable coverage is 4.3
  radians. The accepted trials do not establish full surround coverage or
  protection against low and overhanging obstacles. Review sensor mounting and
  self-occlusion physically before changing the tracked mask or coverage limit.
  The camera floor-scan fallback cannot establish surround protection.
  The Astra depth cloud cannot serve as a collision-monitor source either:
  it faces sideways/rearward and its nearest valid depth (~0.55 m) is beyond
  every StopZone and SlowdownZone point in its field of view, so a live capture
  put zero points in either zone. Low-obstacle stopping needs a near-field
  sensor or a camera remount; done when that sensor feeds `collision_monitor`
  and the attended braking acceptance passes with an obstacle below the LD06 plane.
- Measure the Astra Pro's optical-centre correction and prove that its
  `/camera/depth/points` cloud covers the arm workspace; the driver publishes
  the cloud, but coverage is not established.
- Provide stamped `/imu/data` and `/battery_state` from accepted physical
  sources, then set `require_imu` / `require_battery` true in
  `config/safety_production.yaml` and add the IMU back to `config/ekf.yaml`.
  Servo voltage alone is not a qualified battery state-of-charge
  source. `/hardware/diagnostics` comes from the driver's motor telemetry; its
  voltage, temperature, current and load thresholds still need the bench
  qualification in [docs/safety.md](docs/safety.md).
- Review the electrical and mechanical stop design, including the effect of a
  stalled process, severed network, motor-bus failure, payload shift and power
  fault. Software torque-off is not the independent E-stop.
- Predeclare reviewed maximum stop latency and distance. Then perform at least
  the configured minimum trials (currently five) in each of forward, reverse,
  left, right, clockwise rotation and counter-clockwise rotation on every
  accepted surface/payload combination, or more if the scope review requires it.
  Record worst distances, timing and measurement uncertainty.
- Fault-inject every applicable item required by `config/safety_acceptance.yaml`: scan,
  depth, diagnostics, telemetry loss
  and replay, host and ROS restart, unauthorized ZMQ, DDS and rosbridge policy,
  Nav2 obstacle stop, and arm-workspace intrusion stop.
- Confirm the enabled Nav2 StopZone contains the accepted footprint and the
  velocity-dependent FootprintApproach covers measured stopping and uncertainty.
  Update tracked parameters
  from the reviewed measurements; do not tune only in RViz.
- Record the exact software revision, sensor configuration, payload, surface,
  validation time, all six measured stow positions and all trial results before
  setting `validated: true`.

## Arm calibration, collision model and physical execution

Owner: arm integrator with the physical robot supported and the sweep area
cleared.

Done when the measured stow and calibration poses are committed, the collision
matrix review is recorded, and the physical MoveIt trials and arm-workspace
intrusion stop are in the acceptance record. Tracked in
[#7](https://github.com/robotmindio/cleanroom/issues/7).

- The real joint-zero calibration is measured. A manually positioned upright
  stow candidate is captured in both tracked safety files and is collision-free
  in MoveIt. Measure its physical arm and cable envelope against the base
  footprint before accepting it; the CAD check alone does not prove clearance.
- Find and record a physically collision-free calibration pose, then review
  the production CAD/SRDF collision matrix against the assembled robot. The
  loopback MoveIt test uses that production matrix, but simulated clearance is
  not evidence that it matches the assembled robot.
- With the real depth updater running, verify that a new obstacle keeps
  `/moveit/filtered_cloud` (the perception-liveness evidence of the arm
  workspace monitor) fresh, causes `/safety/arm_workspace_clear` to become
  false, and stops a guarded physical trajectory within the predeclared limit.
  The software monitor checks discretely; its physical stopping latency is not
  established by its launch test.
- Run real MoveIt plan-and-execute trials at the accepted scaling limits and
  payload, including cancel, preemption, path tolerance, goal tolerance,
  telemetry loss and restart. Finish every trial with explicit disarm.
- Recheck the newly launched RViz MotionPlanning panel as well as `move_group`
  after any RViz, MoveIt, desktop image or configuration revision.
- Qualify arm payload beyond the tested case. The installed reported 200 g
  completed one HOME cycle at 0.10 velocity/acceleration scaling; its holder
  mass and attachment location are unconfirmed, and higher speeds, other
  masses or placements, repeated cycles, retention and thermal endurance are
  untested.
- Isolate the intermittent joint-feedback gaps. Arm execution holds and
  resumes through them, but their origin (host serial polling, transport or
  compute scheduling) needs simultaneous source-boundary timing.
- Resolve the wrist camera's intermittent USB disconnects. The kernel drops and
  re-enumerates the device with the arm stationary; the capture supervisor
  recovers the stream in about five seconds. No undervoltage, overcurrent or
  throttling is logged, and tightening the connector did not help. Substitute
  the cable on the same port first, then the port and camera. Vision-dependent
  manipulation with the wrist stream is unverified until then.

## Physical calibration, mapping and RMF

Owner: site mapping and fleet integrator.

Done when an approved, SHA-256-pinned bundle of the real site (0.05 m or finer,
with RMF graph and approval report) is committed and selected by default, and
the calibration and RTAB-Map shutdown trials are recorded. Tracked in
[#8](https://github.com/robotmindio/cleanroom/issues/8).

- Revalidate front/wrist camera intrinsics, camera height/pitch, wheel scale,
  yaw scale and odometry against physical measurements.
- Calibrate the Astra Pro optically. Its factory calibration query returns NaN,
  so the driver publishes finite, explicitly uncalibrated default intrinsics.
  Measured RGB/depth intrinsics, registration and scale are needed before
  claiming accurate visual translation. Standalone RGB-D odometry drifts
  centimetres during small turns that lidar measures in millimetres; the
  production EKF uses wheel odometry only, but SLAM uses visual registration,
  so room-scale localization accuracy and precision placement are unverified.
- Run a controlled RTAB-Map session and verify quota-triggered orderly
  shutdown, SQLite/WAL closure, archive rotation and successful reopen.
- Survey the real cleanroom and produce a map no coarser than 0.05 m, an RMF
  navigation graph, fitted robot/RMF reference coordinates and an approval
  report. Pin every artifact and the acceptance report by SHA-256 in a new map
  bundle.
- The checked-in `cleanroom-development` bundle is intentionally unusable for
  deployment: it is unapproved and its 0.5 m occupancy resolution exceeds the
  0.05 m limit.
- Validate every graph vertex/lane against known free space and prove the
  tracked 0.32 m fleet envelope remains conservative for the real robot.
- Inventory and stop or adopt any externally started `free_fleet_adapter`
  before RMF testing; two adapters must never own the same Nav2 instance. The
  tracked `rmf_owner_guard` refuses to start this repository's adapter when
  it discovers the configured fleet's ownership nodes on the selected DDS
  graph. This is a bounded preflight, not a distributed lease: the site must
  still account for other domains, undiscoverable hosts and an adapter started
  after the preflight. The guard deliberately never kills or adopts a process.
- RMF currently uses DDS domain 0 because no tracked cross-domain bridge is
  configured. A different domain requires a deployment architecture decision
  and a separately tested bridge.
- Keep battery drain and automatic charging disabled until a real battery
  source, charger interface and accepted charging workflow exist.

## Network security and credentials

Owner: deployment/network administrator.

Done when the deployment records, with test evidence, how DDS is isolated or
secured, that rosbridge is loopback-only or behind an authenticated TLS proxy,
and either CURVE plus a firewall on `5555`-`5557/tcp` or an accepted
trusted-network decision. Tracked in
[#9](https://github.com/robotmindio/cleanroom/issues/9). What each port exposes
today is described once, in the README's
[Network security](README.md#network-security).

- Decide whether ROS 2 DDS will use a physically isolated control network or
  DDS Security. Provision identities, governance/permissions, secret storage,
  rotation and recovery; then prove an unauthorized participant cannot publish
  control topics or call motion services.
- Keep rosbridge disabled or loopback-only unless an authenticated TLS proxy,
  authorization policy and firewall are deployed and tested. The one built-in
  exception is `scripts/install-compute-services.sh --rosbridge-tailnet`, which
  binds it to the host's `tailscale0` address: WireGuard authenticates and
  encrypts the link, but rosbridge itself remains no authorization boundary, so
  decide which tailnet peers may command the robot.
- The motor host's ZMQ endpoints (`5555`-`5557/tcp`) accept any client unless
  CURVE is enabled. For remote motor control on an untrusted network, generate
  unique CURVE identities, transfer secret keys through an approved channel,
  restrict key permissions, pin `--bind-address` to the control interface,
  install firewall rules and prove unauthorized ZMQ clients are rejected.
- Do not copy private keys, tokens or site firewall secrets into this
  repository.
- Optional Hugging Face dataset upload still requires the `core-scripts`
  extra, an approved write token and a data-retention/privacy decision.

## Deployment and external package qualification

Owner: deployment image maintainer.

Done when the services are verified on the target hosts, the MoveIt shutdown
probe passes with a fixed package, and a green hosted CI run of the deployed
revision is retained. Tracked in
[#10](https://github.com/robotmindio/cleanroom/issues/10).

- Install the repository's device and compute systemd services with the actual
  non-root accounts, workspace paths, serial/camera groups and key locations.
  Verify restart, shutdown, cgroup ownership and all hardening directives on
  the target hosts.
- Ensure the system ROS interpreter has `python3-zmq`, and the deployed image
  has `moveit_ros_perception`, ShellCheck and the remaining rosdep-resolved
  dependencies. CMake omits ZMQ launch tests when its selected interpreter
  lacks pyzmq instead of emitting malformed missing-result failures. The
  tracked qualification runner checks the complete required test-name set and
  fails when those tests were omitted; an acceptance image must install the
  dependency and run them.
- Keep qualifying ROS Jazzy MoveIt shutdown with
  `scripts/moveit-shutdown-probe.py` after native package changes. The reproduced
  2.12.4 SIGSEGV came from weak action control blocks surviving plugin unload:
  first in `CallbackGroup`, then in the executor's wait set. Pinned rclcpp and
  class_loader fixes now remove expired action registrations and retain plugin
  code through normal object teardown. Three isolated SIGINT trials exited
  zero. The builder, pins and process-lifetime mapping limit are documented in
  `thirdparty/README.md`. The strict qualification runner still requires clean,
  revision- and selected-install-bound JSON. No signals or exit codes are masked.
- Run the GitHub Actions workflow on the final revision and retain its result.
  Local checks are not evidence that the hosted CI image and rosdep resolution
  are healthy.

## Qualified simulation-server acceptance

Owner: simulation-server administrator.

Done when `scripts/sim-qualification.py` writes a passing `summary.json` for an
exact revision on a qualified GPU host and its runtime checklist, with the
manual observations attached, has been reviewed. Tracked in
[#11](https://github.com/robotmindio/cleanroom/issues/11). The host
requirements and procedure are in [QUALIFICATION.md](QUALIFICATION.md).

## Evidence to collect on the deployed revision

Done when every measurement below has been taken on the deployed revision and
attached to the formal acceptance artifacts. Tracked in
[#12](https://github.com/robotmindio/cleanroom/issues/12).

Only measurements taken on the final deployed revision can support
`validated: true`. Collect these and copy the results into the formal acceptance
artifacts:

- Renderer-free simulation physics: base translation, shoulder-pan and gripper
  response to a `FollowJointTrajectory` goal.
- Gripper endpoint calibration and a return-command position check on the
  physical gripper.
- Camera and scan rates and header ages over a fixed interval.
- Guarded-base, camera-loss and link-loss behaviour: zero-velocity samples after
  disarm and the `DISARMED -> LINK_LOST -> DISARMED` state sequence.
- Wrist-roll travel, LAN endpoint reachability, RViz scaling and the installed
  driver.
