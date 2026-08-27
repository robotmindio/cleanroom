# Deferred acceptance and deployment work

This file is the single list of work that cannot be completed from this source
checkout without physical hardware, site measurements, deployment credentials,
an external service, or a qualified GPU host. None of the items below may be
marked complete from unit tests or loopback simulation alone.

The repository remains deliberately default-deny while these blockers exist.
In particular, `config/safety_acceptance.yaml` must remain `validated: false`.

## Physical safety hardware and acceptance

Owner: robot integrator and safety reviewer at the deployment site.

- Install and independently wire a hardwired E-stop that removes actuator
  energy without depending on ROS, the motor host, DDS, or the compute OS.
- Install bumper/contact sensing and publish its real state on
  `safety/bumper_active`.
- Provide full low-obstacle scan coverage. The monocular front-camera fallback
  is not a 360-degree scanner and cannot satisfy the production profile's
  6-radian scan requirement or reliably detect side, rear, low-contrast and
  overhanging obstacles.
- Install and calibrate a physical arm-workspace depth sensor that publishes
  `/camera/depth/points`. The current physical camera arrangement does not
  establish that coverage.
- Provide stamped `/imu/data`, `/battery_state`, and
  `/hardware/diagnostics` from accepted physical sources. Servo voltage alone
  is not a qualified battery state-of-charge source.
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

- Measure the real joint-zero calibration and a mechanically safe, collision-
  checked stow pose. Replace the placeholder zero stow in
  `config/safety_production.yaml`; the accepted stow mapping must match it
  exactly.
- Find and record a physically collision-free calibration pose, then review
  the production CAD/SRDF collision matrix against the assembled robot. The
  loopback MoveIt test uses narrowly scoped test-only contact exemptions and is
  not evidence that the production matrix is correct.
- With the real depth updater installed, verify that a new obstacle produces a
  freshly stamped octomap, causes `/safety/arm_workspace_clear` to become
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
  tracked `rmf_owner_guard` now refuses to start this repository's adapter when
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

- Decide whether ROS 2 DDS will use a physically isolated control network or
  DDS Security. Provision identities, governance/permissions, secret storage,
  rotation and recovery; then prove an unauthorized participant cannot publish
  control topics or call motion services.
- Keep rosbridge disabled or loopback-only unless an authenticated TLS proxy,
  authorization policy and firewall are deployed and tested. Rosbridge itself
  is not an authentication boundary.
- For remote motor control, generate unique CURVE client/server identities,
  transfer secret keys through an approved channel, restrict key permissions,
  install firewall rules and prove unauthorized ZMQ clients are rejected.
- Do not copy private keys, tokens or site firewall secrets into this
  repository.
- Optional Hugging Face dataset upload still requires the `core-scripts`
  extra, an approved write token and a data-retention/privacy decision.

## Deployment and external package qualification

Owner: deployment image maintainer.

- Install the repository's device and compute systemd services with the actual
  non-root accounts, workspace paths, serial/camera groups and key locations.
  Verify restart, shutdown, cgroup ownership and all hardening directives on
  the target hosts.
- Ensure the system ROS interpreter has `python3-zmq`, and the deployed image
  has `moveit_ros_perception`, ShellCheck and the remaining rosdep-resolved
  dependencies. CMake now omits ZMQ launch tests when its selected interpreter
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

Owner: simulation-server administrator. This section incorporates the former
server handoff.

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
./scripts/install-sim-host.sh
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

## Historical evidence that must be repeated

The deleted test handoff contained useful baselines, but they predate the
current acceptance schema and do not make `validated: true` legitimate:

- Renderer-free simulation previously measured `dx=0.198 m`, `dy=0`, shoulder
  pan `0.250 rad`, and gripper `0.287 rad` for a `0.300 rad` goal.
- A prior Pi rendered attempt exited 139 on unsupported OpenGL; an older Ogre1
  run observed collision monitor exit -11. Gazebo was the sole simulated
  joint-state source.
- The gripper's defective raw range `2045..2051` was replaced with endpoints
  `3325/1875` (1450 ticks); a return command measured `1.549 rad`.
- Historical camera evidence used 320x240 frames and recorded 90 scans in 20 s
  with header ages 0.028..0.364 s.
- Historical guarded-base/camera-loss/link-loss checks observed 35 zero-
  velocity samples after disarm and `DISARMED -> LINK_LOST -> DISARMED`.
- Historical wrist-roll, LAN endpoint, RViz scaling and installed-driver
  checks passed for an older revision.

Repeat the relevant evidence against the final deployed revision and copy the
new measurements into the formal acceptance artifacts rather than relying on
these baselines.
