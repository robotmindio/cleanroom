# Deferred acceptance and deployment work

This file is the single list of work that cannot be completed from this source
checkout without physical hardware, site measurements, deployment credentials,
an external service, or a qualified GPU host. None of the items below may be
marked complete from unit tests or loopback simulation alone.

The repository remains deliberately default-deny while these blockers exist.
In particular, `config/safety_acceptance.yaml` must remain `validated: false`.
Each section names its completion condition and its tracking issue.

## Physical safety hardware and acceptance

Owner: robot integrator and safety reviewer at the deployment site.

Done when `config/safety_acceptance.yaml` holds a reviewed record with
`validated: true` for the deployed revision, including every stopping trial and
fault test, and the production profile requires every physical input it names.
Tracked in [#6](https://github.com/robotmindio/cleanroom/issues/6).

- Install and independently wire a hardwired E-stop that removes actuator
  energy without depending on ROS, the motor host, DDS, or the compute OS.
- Install bumper/contact sensing, publish its real state on
  `safety/bumper_active`, and set `require_bumper: true` in
  `config/safety_production.yaml`.
- Accept the LD06 as the production scan source: verify its 360-degree
  coverage, mounting-plane height and self-occlusion against the production
  profile's 6-radian scan requirement. The camera floor-scan fallback is not a
  360-degree scanner and cannot satisfy that requirement or reliably detect
  side, rear, low-contrast and overhanging obstacles.
- Once the RPi 5 table is fitted, run `scripts/lidar-self-mask.py` on the
  stationary robot with nothing else within 30 cm and copy its proposal into
  `config/lidar_self_mask.yaml`. The CAD puts the table legs across the LD06
  scan plane at roughly 90-145 laser-frame degrees and 0.08-0.21 m, which needs
  a second sector (the filter accepts a list).
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
  30 trials in each of forward, reverse, left, right, clockwise rotation and
  counter-clockwise rotation on every accepted surface/payload combination.
  Record worst distances, timing and measurement uncertainty.
- Fault-inject every item required by `config/safety_acceptance.yaml`: scan,
  depth, IMU, battery, diagnostics, bumper, independent E-stop, telemetry loss
  and replay, host and ROS restart, unauthorized ZMQ, DDS and rosbridge policy,
  Nav2 obstacle stop, and arm-workspace intrusion stop.
- Confirm the enabled Nav2 StopZone contains the accepted footprint plus the
  measured worst stopping distance and uncertainty. Update tracked parameters
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

- Measure the real joint-zero calibration and a mechanically safe, collision-
  checked stow pose. Replace the placeholder zero stow in
  `config/safety_production.yaml`; the accepted stow mapping must match it
  exactly.
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

## Physical calibration, mapping and RMF

Owner: site mapping and fleet integrator.

Done when an approved, SHA-256-pinned bundle of the real site (0.05 m or finer,
with RMF graph and approval report) is committed and selected by default, and
the calibration and RTAB-Map shutdown trials are recorded. Tracked in
[#8](https://github.com/robotmindio/cleanroom/issues/8).

- Revalidate front/wrist camera intrinsics, camera height/pitch, wheel scale,
  yaw scale and odometry against physical measurements.
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
- Done: the device zenoh bridge (`7447/tcp`) requires mutual TLS with the
  robot's private CA (`config/zenoh_device.json5`, `scripts/setup-zenoh-tls.sh`).
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
- Resolve the ROS Jazzy MoveIt 2.12.4 `move_group` SIGSEGV after SIGINT. The
  tracked `scripts/moveit-shutdown-probe.py` has isolated it from the driver,
  pyzmq, joint feedback and trajectory execution: after reaching readiness,
  `MoveItCpp` destruction reaches `TrajectoryExecutionManager`, `rclcpp::Node`
  and `CallbackGroup` destruction before exiting `-11`. This matches open
  upstream MoveIt issues
  [#3680](https://github.com/moveit/moveit2/issues/3680) and
  [#3721](https://github.com/moveit/moveit2/issues/3721). The strict
  qualification runner invokes the probe and requires revision- and selected-
  install-bound JSON with `clean_shutdown: true` and
  `move_group_exit_code: 0`; the current package therefore fails closed. Do
  not replace orderly teardown with SIGKILL or signal masking. A deployment
  image remains unacceptable until a fixed package passes this probe.
- Run the GitHub Actions workflow on the final revision and retain its result.
  Local checks are not evidence that the hosted CI image and rosdep resolution
  are healthy.

## Qualified simulation-server acceptance

Owner: simulation-server administrator.

Done when `scripts/sim-qualification.py` writes a passing `summary.json` for an
exact revision on a qualified GPU host and its runtime checklist, with the
manual observations below attached, has been reviewed. Tracked in
[#11](https://github.com/robotmindio/cleanroom/issues/11).

Requirements:

- Ubuntu 24.04 with this exact repository revision.
- A GPU exposed to the service account, including render-device permissions
  for a VM/container.
- A headless EGL OpenGL context version 3.3 or newer. The current Raspberry Pi
  exposes OpenGL 3.1 and cannot qualify Ogre2. The installed Ogre Vulkan path
  also has an unresolved `glslang::InitializeProcess` symbol and is not an
  accepted substitute.

Provision and test:

```bash
./scripts/install.sh --simulation
source scripts/setup.bash
scripts/sim-qualification.py \
  --build-dir build/lekiwi_rmf \
  --output-dir /absolute/path/outside/checkout/to/qualification-evidence
```

The runner remains an evidence collector and never grants qualification. It
requires a clean exact revision, a build and installed package bound to this
checkout, `validated: false`, ShellCheck, pyzmq in CMake's selected interpreter,
the complete expected CTest set, every test passing, orderly MoveIt shutdown
from that selected install and an EGL/OpenGL renderer of at least 3.3.
It retains every command result and writes `summary.json` even on failure. In
particular, retain its results for the renderer-free physics test and the
native actuator-failsafe fault-injection test. The native failsafe must zero
stale wheel targets and interrupt an arm trajectory on heartbeat loss. Those
tests launch Gazebo as a launch-managed process, use a fresh transport
partition per invocation, and hold a shared CTest resource lock so a stale or
parallel server cannot supply their evidence. See `QUALIFICATION.md` for the
evidence layout and review boundary.

Start the managed simulation with no external bridge:

```bash
scripts/sim-up.sh start_rosbridge:=false
```

In another sourced terminal, before issuing motion:

```bash
scripts/sim-scan-check.py --timeout 30
ros2 lifecycle get /collision_monitor
ros2 topic info -v /cmd_vel_safe
ros2 topic hz /camera/depth/points
```

Pass only if `/scan` has at least 180 ranges with usable values beyond
`range_min`, collision monitor is active, `/cmd_vel_safe` has the intended
publisher/subscriber topology, and the delayed/noisy depth cloud remains live.
An absent or all-minimum scan is a correct fail-closed stop, not permission to
bypass collision monitoring. Rerun the evidence collector with
`--collect-runtime` to retain bounded scan, lifecycle, topology, depth,
MoveIt-parameter and log-tail observations, then complete its generated
`runtime-checklist.md`; command success alone is not administrator review.

With a clear simulated room, check mux priority and stale-command stopping:

```bash
ros2 topic pub -r 5 --times 10 /cmd_vel_smoothed geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}}'
ros2 topic pub -r 5 --times 10 /cmd_vel_manual geometry_msgs/msg/Twist \
  '{linear: {y: 0.07}}'
ros2 topic echo /cmd_vel_muxed
ros2 topic echo /cmd_vel_safe
```

Manual input must preempt Nav2, guarded output must remain zero for an occupied
path, and both outputs must return to zero after publishers stop. Then launch
with `start_moveit:=true`, place an obstacle in the arm workspace, and verify
it appears in both `move_group` and a newly launched RViz MotionPlanning panel.
Confirm the arm-workspace gate withdraws permission and interrupts execution.

Fault-inject loss of the ROS omni controller/bridge and arm adapter heartbeat.
The Gazebo-native 250 ms failsafe must stop wheel demand and replace an active
arm trajectory with a measured-position hold. This supplements, but does not
represent, physical E-stop or braking acceptance.

Stop only the recorded process group:

```bash
scripts/ros-stop.sh
```

Retain the renderer output, test results, scan/depth evidence, collision-monitor
state, fault-injection result, RViz/move_group values and final 30 lines of
`~/.ros/lekiwi/sim-stack.log`. Store these beside the runner's `summary.json`;
the generated checklist remains incomplete until the manual mux, obstacle,
RViz and heartbeat-loss observations above are attached and reviewed.

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
