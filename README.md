# LeKiwi ROS 2 + lidar SLAM + Open-RMF

A shared ROS 2 Jazzy stack for running a LeKiwi mobile manipulator in Gazebo or on real hardware. Nav2 and Open-RMF see the same robot interface in both modes.

The usual operator commands are indexed in [`scripts/README.md`](scripts/README.md).

```text
Open-RMF -> Free Fleet -> Nav2 -> cmd_vel_smoothed --+
                         ^                            |-> mux -> collision monitor -> /cmd_vel_safe
                         |                            |                              |-- Gazebo Harmonic
                         |                            `-> manual teleop              `-- LeRobot ZMQ host -> LeKiwi
                RTAB-Map lidar SLAM
                LD06 + Astra cloud + wheel odometry
```

The package includes:

- a Gazebo cleanroom and LeKiwi model;
- a metric Nav2 occupancy map and matching RMF navigation graph;
- a LeRobot-to-ROS driver for velocity, odometry, arm joints, and the front camera;
- MoveIt planning and execution for the five-joint arm on real hardware and
  the physics-actuated Gazebo arm;
- RTAB-Map lidar SLAM (ICP and proximity loop closure) on the merged LD06 scan and
  Astra obstacle-band cloud, using metric wheel odometry;
- a Free Fleet adapter connecting Nav2 to Open-RMF;
- a read-only Foxglove dashboard for robot, navigation, sensor, and safety telemetry;
- optional rosbridge WebSocket access for browsers and external applications;
- one installer for the supported development/runtime stack.

Safety-input status, motor-health diagnostics, and the physical qualification
procedure are documented in [docs/safety.md](docs/safety.md).

## Supported platform

- Ubuntu 24.04, `amd64` or `arm64`
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12

The installer supports Ubuntu 24.04 only and rejects any other platform. The robot and the workstation must run the **same** distro — ROS 2 does not guarantee cross-distro wire compatibility.

## Install everything

The installer uses `sudo` for apt packages, downloads pinned Free Fleet/RMF task sources, installs the Zenoh bridge in `~/.local/bin`, creates a Python virtual environment, installs LeRobot, and builds the workspace. It adds an idempotent, marked setup block to `~/.bashrc`; when zsh is the login shell or `~/.zshrc` already exists, it adds the same block there. The block remembers the chosen workspace unless `LEKIWI_WS` is already set. Remove that marked block to opt out of automatic setup.

```bash
chmod +x scripts/install.sh
./scripts/install.sh
source scripts/setup.bash
```

The default workspace is `~/lekiwi_ws`. To use another persistent location:

```bash
LEKIWI_WS=$HOME/robot_ws ./scripts/install.sh
export LEKIWI_WS=$HOME/robot_ws
source scripts/setup.bash
```

Rerunning the installer is safe. It stops if a managed source checkout contains local changes, except for the one verified LD06 build patch the installer itself reapplies.

Installed upstream components are pinned where compatibility matters:

| Component | Installed version |
| --- | --- |
| ROS/Gazebo/Open-RMF/RTAB-Map | Current Jazzy apt packages |
| LeRobot | 0.6.1 |
| Zenoh Python + ROS bridge | 1.5.0 |
| Free Fleet | Pinned commit from its Jazzy-supported branch |
| RMF task CLI | rmf_demos 2.3.0 |

Confirm the environment after installation:

```bash
ros2 pkg prefix lekiwi_rmf
ros2 pkg prefix free_fleet_adapter
ros2 pkg prefix rtabmap_slam
zenoh-bridge-ros2dds --version
python -c 'import numpy; print(numpy.__version__)'   # must be 1.x
"$LEKIWI_WS/.venv-lerobot/bin/python" -c 'import lerobot; print(lerobot.__version__)'
```

LeRobot lives in a second virtualenv, `.venv-lerobot`, and is deliberately absent
from the ROS environment: it requires `numpy>=2`, while ROS 2 Jazzy's compiled
extensions are built against numpy 1.26, and mixing them segfaults `rmf_adapter`.
`bringup.launch.py` runs the hardware driver against `.venv-lerobot` on its own.
See [HARDWARE.md](HARDWARE.md).

### Rebuild the complete robot model

From the `cleanroom` checkout, one command validates the sibling `LeKiwi` CAD
sources, refreshes the vendored Xacro and meshes, renders the reference image,
builds the ROS package, and runs every CTest:

```bash
./scripts/rebuild-all.sh
```

Set `LEKIWI_SOURCE` only when the source checkout is somewhere other than
`../LeKiwi`. The separate LeKiwi manufacturing audit covers recovery files that
do not feed the robot Xacro.

### Headless simulation server

Use the repository's simulation-only installer on a remote brain/server. It
installs the ROS/Gazebo stack and builds this workspace, but deliberately omits
the LeRobot motor/camera environment:

```bash
./scripts/install.sh --simulation
```

The server still needs a GPU whose driver exposes OpenGL 3.3 or newer to
headless EGL. See [DEFERRED.md](DEFERRED.md#qualified-simulation-server-acceptance)
for the validation sequence after it is provisioned.

## Simulation quick start

Source the environment in every new terminal:

```bash
source scripts/setup.bash
```

Launch Gazebo, lidar SLAM, and Nav2 through the managed renderer preflight:

```bash
scripts/sim-up.sh
```

The drivetrain, wheel-derived odometry, and simulated arm action also have a
renderer-free Gazebo acceptance test. It is safe to run on a headless Jazzy
builder after building the package:

```bash
ctest --test-dir build/lekiwi_rmf \
  -R '^test_test_simulation_physics_launch.py$' --output-on-failure
```

That test launches physics, spawns the same launch-time anisotropic-contact
SDF used by bringup, drives through `/cmd_vel_safe`, checks encoder odometry,
and completes an arm/gripper `FollowJointTrajectory` goal. Camera, depth, and
lidar rendering remain in the GPU-qualified acceptance below.

It refuses a host that cannot create a headless OpenGL 3.3 context, records
only its own process group, and writes `~/.ros/lekiwi/sim-stack.log`. Stop it
with `scripts/ros-stop.sh`; that script escalates only within the recorded
process group if an external Gazebo/ROS binary does not exit on SIGINT.

Test Nav2 only after `/scan` has real ranges and `/map` covers a clear nearby
goal (see `TROUBLESHOOTING.md`):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.3, y: 0.0}, orientation: {w: 1.0}}}}'
```

RMF is opt-in because it connects the robot to a fleet graph. It is only valid
with `localization:=amcl`, `slam_mode:=localization` and an approved map bundle; the checked-in
synthetic development bundle cannot start RMF. After installing a surveyed
bundle, start the managed simulation with its manifest:

```bash
scripts/sim-up.sh localization:=amcl slam_mode:=localization \
  start_rmf:=true map_bundle:=/absolute/path/to/site-v1.yaml
```

```bash
ROS_DOMAIN_ID=0 ros2 run rmf_demos_tasks dispatch_patrol \
  -p charger dropoff -n 1 --use_sim_time
```

On later runs, reuse the map database without adding new map nodes:

```bash
scripts/sim-up.sh slam_mode:=localization \
  rtabmap_database:=$HOME/.ros/lekiwi_rtabmap_sim.db
```

## Launch options

The arguments of `launch/bringup.launch.py`, arm planning and pose calibration,
arming and recovery, production safety prerequisites, camera sources and the
odometry scale are described in [docs/launch-options.md](docs/launch-options.md).

## Foxglove dashboard

`scripts/install.sh` installs Foxglove Desktop on the graphical workstation
and the ROS 2 Foxglove Bridge. Every normal bringup starts the read-only bridge
at `ws://127.0.0.1:8765`. On the same server where RViz runs, open the desktop
app already connected to it with:

```bash
scripts/foxglove.sh
```

Import [`config/foxglove-layout.json`](config/foxglove-layout.json) once
through **Layouts → Import from file**. Save the imported layout and later
opens show the dashboard immediately.

The layout has four views: **Overview** combines the robot, maps/costmaps,
plans, depth, scan, front camera, and safety diagnostics; **Navigation** is a
top-down Nav2 view; **Perception** groups RGB cameras and depth; and
**Telemetry & Health** shows diagnostics, safety permissions, battery,
measured/commanded velocity, motor diagnostics, and joints. Optional sensors
leave their panels empty rather than preventing the rest of the dashboard from
opening.

The bridge deliberately exposes no ROS client-publish or service capability,
so Foxglove is observational only. It is loopback-bound by default. For a
separate trusted workstation, configure `foxglove_address` to that machine's
Tailscale address through the same tracked launch/deployment configuration;
the launch validation rejects ordinary LAN exposure until authenticated TLS is
configured.

## WebSocket access with rosbridge

Rosbridge is separate from Zenoh: Zenoh remains the transport required by Free Fleet, while rosbridge exposes one ROS graph as JSON over WebSocket. It is disabled by default and binds to loopback when explicitly enabled.

```bash
scripts/sim-up.sh start_rosbridge:=true rosbridge_address:=127.0.0.1
```

For a local client, connect to loopback:

```text
ws://127.0.0.1:9090
```

`ws://ROBOT_IP:9090` is only appropriate when a separately managed
authenticated proxy deliberately exposes that endpoint.

Minimal roslibjs connection:

```js
const ros = new ROSLIB.Ros({url: "ws://ROBOT_IP:9090"});
ros.on("connection", () => console.log("connected"));
ros.on("error", console.error);
```

The default `rosbridge_domain:=0` exposes LeKiwi, camera, SLAM, and Nav2. To expose the separate RMF graph instead:

```bash
scripts/sim-up.sh start_rosbridge:=true rosbridge_address:=127.0.0.1 \
  rosbridge_domain:=55
```

Rosbridge has no authentication, authorization, or TLS. Keep it on loopback
for local tools. Do not bind it to a LAN address until it is behind a separately
managed authenticated TLS proxy and firewall policy. A client must use the
documented `/cmd_vel_manual` or navigation action; publishing directly to
`/cmd_vel_safe` bypasses the intended guard and is an unsafe protocol violation.
Rosbridge access is not a safety boundary.

On a compute host shared with Fiber through Tailscale, install the service with
`scripts/install-compute-services.sh --remote DEVICE_ADDR --rosbridge-tailnet`.
This binds rosbridge only to that host's authenticated `tailscale0` address.

The disposable `mode:=sim` test topology deliberately permits an unprotected
rosbridge endpoint. ZMQ is also permitted without CURVE for trusted device-LAN
deployments; CURVE remains available when that network cannot be trusted.

### Driving the robot from Fiber

[Fiber](https://github.com/robotmindio/robotmind/tree/main/fiber-core) talks to this
stack over the same port with its `rosbridge` connector, and
`fiber-core/examples/lekiwi/` is the worked wiring: a Nav2 goal built from a
webhook, an emergency stop, and `/rosout` errors arriving as alerts.

Two things matter when subscribing from outside. Throttle everything —
`/odom` publishes at 50 Hz and an unthrottled subscription puts 50 events a
second through whatever is listening. And do not subscribe to image topics;
pull a frame on demand instead, or every message carries a base64 payload.

Motion belongs on the `/navigate_to_pose` action. Manual velocity commands must use
`/cmd_vel_manual`; they are arbitrated with navigation then pass through the same
collision monitor. Publishing directly to `/cmd_vel_safe` is an unsafe protocol violation.

## Real robot

Bringing up the physical robot, `.env` configuration, the boot services, the
coordinated split deployment, the choice of `/scan` source and the calibration
and mapping sequence are described in [docs/real-robot.md](docs/real-robot.md).
Start with [HARDWARE.md](HARDWARE.md).

## Frames and interfaces

```text
map -> odom -> base_footprint -> base_link -> ld06_body -> laser
```

| Interface | Producer | Consumer |
| --- | --- | --- |
| `/cmd_vel_manual` | Teleop or approved external client | `cmd_vel_mux` |
| `/cmd_vel_smoothed` | Nav2 | `cmd_vel_mux` |
| `/cmd_vel_safe` | Collision monitor (intended sole guarded output) | Gazebo or `lekiwi_driver` |
| `/wheel/odometry`, `/odom`, `odom -> base_footprint` | Driver and robot-localization EKF | RTAB-Map and Nav2 |
| `/camera/front/image_raw` | Gazebo, local `v4l2_camera`, or the remote-camera relay | Camera-scan fallback, RViz, calibration |
| `/camera/front/camera_info` | Gazebo, calibrated `v4l2_camera`, or the relay | Camera-scan fallback, RViz, calibration |
| `/scan` | Gazebo, `free_space.py`, or the LD06 driver (`laser_source`) | Nav2 and `slam_cloud` |
| `/camera/depth/points` | Astra cloud filter (device or local) | MoveIt, `slam_cloud`, safety supervisor |
| `/slam/cloud` | `slam_cloud` (LD06 scan plus Astra obstacle band) | RTAB-Map |
| `map -> odom` | RTAB-Map or AMCL | Nav2 and Free Fleet |
| `/navigate_to_pose` | Nav2 action server | Free Fleet through Zenoh |
| `safety/driver_state` | `lekiwi_driver` | Safety supervisor |
| `safety/supervisor_state` | Safety supervisor | Operator/automation |
| `safety/base_motion_permitted` | Safety supervisor | Mux and driver interlock |
| `safety/arm_motion_permitted` | Safety supervisor | Driver/arm interlock |
| `/safety/arm`, `/safety/disarm`, `/safety/reset_fault` | Driver / supervisor | Explicit operator control |
| `ws://127.0.0.1:9090` | rosbridge when explicitly enabled | Local browser/external client |

By default nothing serves the checked-in PGM: RTAB-Map publishes `/map` itself, drawing the occupancy grid from the merged LD06 and Astra cloud while ICP scan matching and proximity loop closure correct pose drift. The robot starts at the origin of a map it has not seen yet, and the grid grows as it drives. `static_map:=true` puts the checked-in floor plan back on `/map` instead, and moves RTAB-Map's own grid to `/rtabmap/map`.

A live map and the checked-in RMF navigation graph do not agree on coordinates: the graph names points in the PGM's frame. Keep `start_rmf:=false` while mapping, or regenerate the graph against the map the robot draws.

### Immutable map bundles

RMF operation must use an approved bundle under `maps/bundles/`. A bundle pins
the occupancy YAML and image, navigation graph, and fleet configuration by
SHA-256. Validation also fits the RMF/robot reference coordinates, checks the
fleet footprint, and proves every graph vertex and lane has the configured
footprint of free, known map space. The checked-in
`maps/bundles/cleanroom-development.yaml` is intentionally a synthetic,
unapproved example and cannot start RMF.

Inspect a development bundle without treating it as deployable:

```bash
PYTHONPATH=. python3 scripts/validate-map-bundle.py \
  maps/bundles/cleanroom-development.yaml --allow-unapproved
```

The command still enforces map resolution, checksums, graph geometry, and
free-space consistency. It currently fails for the demonstration bundle's
0.5 m resolution; that is expected. An approved surveyed bundle must contain
`validated: true` and a SHA-256-pinned `artifacts.validation_report`. Launch
selects the map, graph, and fleet from that manifest; do not provide separate
artifact paths that disagree with it:

```bash
scripts/workstation-up.sh localization:=amcl slam_mode:=localization \
  start_rmf:=true map_bundle:=/absolute/path/to/maps/bundles/site-v1.yaml
```

Mapping is a separate, bounded activity. The session guard includes the
database and SQLite sidecars (`-wal`, `-shm`, and `-journal`) and exits with a
quota status when `rtabmap_mapping_max_bytes` or
`rtabmap_mapping_max_seconds` is reached. The launch then shuts down RTAB-Map
so the closed database can be archived safely. Do not rename an active SQLite
database. The repository-managed default database is rotated at startup;
explicit map databases are retained for export and must be managed by the
map-bundle workflow.

## Troubleshooting

Symptoms and fixes are collected in [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## Safety and current limits

- Real mode evaluates its safety inputs continuously; a one-shot readiness
  message is not a motion permit. With `LEKIWI_DISARM_ON_FAILURE=true` (strict
  mode) the supervisor is default-deny: missing or stale required inputs deny
  motion, and a runtime fault latches until the driver is disarmed and
  `/safety/reset_fault` is explicitly called. By default it reports them in
  `/diagnostics` without withholding motion.
- Keep a hardwired physical E-stop reachable and supervise every hardware run.
  The ROS E-stop topic and software torque cut are status/control interfaces,
  not substitutes for removing actuator energy independently of ROS.
- The production profile requires `/scan`, `/camera/depth/points`, `/odom`,
  `/joint_states`, `/hardware/diagnostics`, and `safety/driver_state`. It does
  not require `/imu/data`, `/battery_state`, `safety/bumper_active`, or
  `safety/estop_active`, because the shipped robot has no source for them. The supplied
  `config/safety_acceptance.yaml` is deliberately unvalidated; physical
  stopping and fault trials must populate it before production arming.
- The acceptance record is schema version 2. It remains invalid until it has
  reviewed limits, at least 30 trials in every translation/rotation direction,
  worst-case distances plus uncertainty, stop latency, traceable
  software/sensor/payload/surface details, and every fault test marked true.
  This includes independent E-stop behavior, unauthorized ZMQ rejection, and
  DDS/rosbridge isolation or authentication. At startup, the supervisor also
  requires the accepted footprint and padding to match both tracked Nav2
  costmaps and proves the enabled StopZone leaves at least the measured worst
  stopping distance plus uncertainty around that footprint.
- The driver arms only after receiving fresh healthy telemetry and supervisor
  permission. By default it re-arms itself after a host session change or link
  loss, with servo torque on throughout; in strict mode it disarms, cuts torque,
  and `/safety/arm` is an explicit operator action after inspection. An
  operator's `/safety/disarm` cuts torque through the motor host, aborts arm
  motion, and holds until `/safety/arm`.
- Network exposure (the ZMQ motor endpoints, the zenoh sensor bridge,
  rosbridge and DDS) and its protection are described in
  [Network security](#network-security).
- When commands cease, the motor-host watchdog stops the base and freezes the
  arm with torque on by default; in strict mode it cuts and verifies all servo
  torque and the ROS driver observes that cut and requires an explicit re-arm.
  This software mechanism is not a substitute for an E-stop. The camera floor
  scan fallback is supplemental and cannot see all side/rear, floor-coloured,
  low, or overhanging obstacles.
- Production MoveIt has an execution-time arm-workspace gate. It requires
  fresh complete joint state, a fresh `/moveit/filtered_cloud` (the MoveIt
  octomap updater's output, the perception-liveness evidence) and repeated
  successful `/check_state_validity` responses; collision, timeout or perception failure
  withdraws the arm lease. Its discrete check and software stop still require
  measured physical latency/intrusion acceptance, and the CAD collision matrix
  needs a measured collision-free calibration pose.
- Gazebo drives three wheel joints at the CAD-derived 120-degree layout. The
  launch-time `sim_sdf` conversion adds anisotropic `fdir1` contact friction,
  and encoder joint positions—not model ground truth—produce `/odom`.
  Commands have velocity, acceleration, jerk, wheel-rate, and ROS watchdog
  limits. A Gazebo-native 250 ms failsafe owns the final actuator topics:
  stale wheel traffic is forced to zero, and loss of the arm adapter heartbeat
  interrupts an active native trajectory with a measured-position hold. This
  is simulation fault containment, not a physical safety mechanism or E-stop.
- The simulator actuates all six arm/gripper joints and publishes a Gaussian-
  noisy depth cloud through a seeded latency/dropout stage. It still does not
  model the physical E-stop, bumpers, battery, motor thermal/current behavior,
  detailed omni rollers, or measured hardware braking. Physical and full
  rendered simulation acceptance remain separate test activities.
- Battery drain is disabled until a real battery state source and charging
  workflow have passed acceptance. RMF schedules mobile-base patrol and
  delivery; it does not schedule arm trajectories in this package.

## Network security

The one description of what the robot exposes on the network. Open deployment
work is tracked in [DEFERRED.md](DEFERRED.md#network-security-and-credentials).

| Port | Listener | Carries | Protection |
| --- | --- | --- | --- |
| `5555/tcp` | `lekiwi-host` | ZMQ motor commands | none; optional CURVE |
| `5556/tcp` | `lekiwi-host` | ZMQ observations and joint state | none; optional CURVE |
| `5557/tcp` | `lekiwi-host` | Torque arm, disarm and state | none; optional CURVE |
| `7447/tcp` | `lekiwi-zenoh` | Zenoh, export-only: `/pi/lidar/scan`, rate-limited compressed `/pi/camera/*`, compact `/camera/depth/points`, and the compressed Astra colour preview with `camera_info` | mutual TLS |
| `9090/tcp` | `rosbridge` (off by default) | JSON over WebSocket to one ROS graph | none; loopback or tailnet bind |

- The ZMQ listeners bind every interface by default and accept any client
  unless CURVE is configured. Anyone who can reach `5555` or `5557` can command
  the robot and cut torque, so keep them on a trusted robot network or behind a
  firewall. `--bind-address DEVICE_IP` pins them to one interface.
- The zenoh sensor bridge accepts only peers that present a certificate signed
  by the robot's private CA, and presents its own
  (`config/zenoh_device.json5`, `enable_mtls: true`); it refuses to start
  without its identity rather than fall back to plaintext.
  `scripts/setup-zenoh-tls.sh` creates the CA and both identities under
  `/etc/lekiwi/zenoh-tls`, and `scripts/install-compute-services.sh` runs it.
  Its allow-list is export-only, so no peer can publish into the device's DDS.
- Rosbridge has no authentication, authorization or TLS and is not a safety
  boundary. It is disabled by default and loopback-bound when enabled;
  `--rosbridge-tailnet` binds it to the host's WireGuard-authenticated
  `tailscale0` address, but every tailnet peer that reaches it can command the
  robot. See [WebSocket access with rosbridge](#websocket-access-with-rosbridge).
- This repository does not configure DDS security. ROS 2 discovery and the
  control graph are exposed to the local network unless the deployment
  isolates it; neither CURVE, zenoh TLS nor rosbridge secures DDS.
- The device installer's polkit rule lets the deployment user manage
  NetworkManager over SSH. NetworkManager passes polkit no connection details,
  so that user can change, delete and activate every system connection, and
  so can take the robot off its network.

To enable CURVE, generate the server, health, and driver identities once as the
device service account, then install both halves with a protected key directory.
Copy the driver secret and server public key to the compute host only through an
approved secret-transfer process:

```bash
"$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/generate-zmq-keys.py \
  "$HOME/.ros/lekiwi/curve"
sudo scripts/install-device-services.sh --bind-address DEVICE_IP \
  --curve-dir "$HOME/.ros/lekiwi/curve"
sudo scripts/install-compute-services.sh --remote DEVICE_IP \
  --curve-dir "$HOME/.ros/lekiwi/curve"
```

The last command is run on the compute host only after its key directory has
`clients/driver.key_secret` and `server.key`. Omit both `--curve-dir` options
to use the unauthenticated transport. CURVE does not configure a firewall.
