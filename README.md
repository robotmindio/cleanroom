# LeKiwi ROS 2 + visual SLAM + Open-RMF

A shared ROS 2 Jazzy stack for running a LeKiwi mobile manipulator in Gazebo or on real hardware. Nav2 and Open-RMF see the same robot interface in both modes.

```text
Open-RMF -> Free Fleet -> Nav2 -> cmd_vel_smoothed --+
                         ^                            |-> mux -> collision monitor -> /cmd_vel_safe
                         |                            |                              |-- Gazebo Harmonic
                         |                            `-> manual teleop              `-- LeRobot ZMQ host -> LeKiwi
                RTAB-Map visual SLAM
                front RGB + wheel odometry
```

The package includes:

- a Gazebo cleanroom and LeKiwi model;
- a metric Nav2 occupancy map and matching RMF navigation graph;
- a LeRobot-to-ROS driver for velocity, odometry, arm joints, and the front camera;
- MoveIt planning and execution for the five-joint arm on real hardware and
  the physics-actuated Gazebo arm;
- RTAB-Map monocular place recognition and loop closure using metric wheel odometry;
- a Free Fleet adapter connecting Nav2 to Open-RMF;
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

### Headless simulation server

Use the repository's simulation-only installer on a remote brain/server. It
installs the ROS/Gazebo stack and builds this workspace, but deliberately omits
the LeRobot motor/camera environment:

```bash
./scripts/install-sim-host.sh
```

The server still needs a GPU whose driver exposes OpenGL 3.3 or newer to
headless EGL. See [DEFERRED.md](DEFERRED.md#qualified-simulation-server-acceptance)
for the validation sequence after it is provisioned.

## Simulation quick start

Source the environment in every new terminal:

```bash
source scripts/setup.bash
```

Launch Gazebo, visual SLAM, and Nav2 through the managed renderer preflight:

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
in `slam_mode:=localization` with an approved map bundle; the checked-in
synthetic development bundle cannot start RMF. After installing a surveyed
bundle, start the managed simulation with its manifest:

```bash
scripts/sim-up.sh slam_mode:=localization start_rmf:=true \
  map_bundle:=/absolute/path/to/site-v1.yaml
```

```bash
ROS_DOMAIN_ID=0 ros2 run rmf_demos_tasks dispatch_patrol \
  -p charger dropoff -n 1 --use_sim_time
```

On later runs, reuse the visual database without adding new map nodes:

```bash
scripts/sim-up.sh slam_mode:=localization \
  rtabmap_database:=$HOME/.ros/lekiwi_rtabmap_sim.db
```

## Launch options

| Argument | Values | Default | Purpose |
| --- | --- | --- | --- |
| `mode` | `sim`, `real` | `sim` | Select Gazebo or the LeRobot hardware bridge |
| `headless` | `true`, `false` | `true` | Run Gazebo server-only with offscreen rendering; set false to open its GUI |
| `localization` | `visual_slam`, `amcl` | `visual_slam` | Select the sole `map -> odom` provider |
| `slam_mode` | `mapping`, `localization` | sim: `mapping`; real: `localization` | Extend or reuse the RTAB-Map database |
| `remote_ip` | IPv4/hostname | `127.0.0.1` | Address of the LeKiwi ZMQ host |
| `rtabmap_database` | file path | sim: `~/.ros/lekiwi_rtabmap_sim.db`; real: `~/.ros/lekiwi_rtabmap.db` | Visual map database |
| `publish_astra` | `true`, `false` | `true` | Start the tracked local Astra Pro as registered RGB-D; otherwise SLAM uses the front RGB camera plus scan |
| `hardware_config` | YAML path | `config/hardware.yaml` | Tracked hardware identities, including the required Astra serial when RGB-D is enabled |
| `camera_info_url` | ROS camera URL | `file://~/.ros/camera_info/lekiwi_front.yaml` | V4L2 front-camera calibration (not used by Astra Pro) |
| `wrist_camera_info_url` | ROS camera URL | `file://~/.ros/camera_info/lekiwi_wrist.yaml` | Optional wrist-camera calibration |
| `camera_source` | `local`, `remote` | `local` | Read the camera here, or decompress what the robot's Pi publishes |
| `camera_device` | V4L2 path | `/dev/video0` | Existing front V4L2 camera |
| `laser_source` | `auto`, `camera`, `ld06`, `none` | `auto` | Select camera fallback or LD06 on real hardware; Gazebo supplies `/scan` in sim |
| `lidar_source` | `local`, `remote` | `local` | Machine that opens the LD06 serial port; remote relays `/pi/lidar/scan` |
| `lidar_port` | serial path | CP2102 `/dev/serial/by-id/...` | LD06 device when `laser_source:=ld06` |
| `lidar_offset_x`, `lidar_offset_y`, `lidar_offset_z`, `lidar_offset_yaw` | metres, radians | `0.0` | Measured scan-frame correction from the CAD LD06 pose |
| `camera_height`, `camera_offset_x`, `camera_offset_y` | metres | `0.093`, `0.03`, `0.0` | Front-camera pose used by the camera scan |
| `camera_pitch`, `camera_yaw`, `camera_roll` | radians | `0.031`, `0.0`, `0.0` | Front-camera orientation used by the camera scan |
| `xy_velocity_scale` | float | `1.0` | Correction for reported and commanded translation |
| `yaw_velocity_scale` | float | `0.90` | Correction for reported and commanded rotation |
| `start_rmf` | `true`, `false` | `false` | Start Zenoh, RMF schedule, dispatcher, and fleet adapter |
| `rmf_domain` | integer | `0` | DDS domain used by RMF processes; validation currently requires `0` because no tracked cross-domain bridge is configured |
| `start_rosbridge` | `true`, `false` | `false` | Start rosbridge WebSocket and ROS API nodes |
| `start_moveit` | `true`, `false` | `false` | Start MoveIt arm planning and execution against the real or simulated action server |
| `rosbridge_address` | bind address | `127.0.0.1` | Interface exposed by rosbridge; keep loopback unless protected separately |
| `rosbridge_port` | TCP port | `9090` | WebSocket listening port |
| `rosbridge_domain` | integer | `0` | ROS graph exposed through rosbridge |

Only one localization mode should run. `visual_slam` publishes `map -> odom` through RTAB-Map; `amcl` publishes it from the fixed occupancy map.

### Arm planning

Start MoveIt with either robot mode. Open RViz separately when visual planning
is needed; `bringup.launch.py` does not start a desktop session. The following
starts the real arm action server, but does not move the robot:

```bash
# terminal 1
ros2 launch lekiwi_rmf bringup.launch.py mode:=real start_moveit:=true
# terminal 2, after the stack is running
scripts/rviz.sh
```

MoveIt executes through `/arm_controller/follow_joint_trajectory` in both
modes. In simulation, Gazebo's six-joint physics controller supplies actual
joint feedback and the adapter enforces the same trajectory limits and
tolerances as the hardware boundary. To exercise it, launch with
`mode:=sim start_moveit:=true`; the delayed depth cloud feeds MoveIt's octomap.
The host uses five motor read retries while the arm moves; override only after validating
your bus with `LEKIWI_READ_RETRIES`.

For a deliberate one-joint physical adjustment (useful for validation and setup), use
the bounded jog tool. It reads the current real joint state, accepts at most 0.1 rad per
command, enforces configured joint limits, and asks for confirmation before sending motion:

```bash
scripts/arm-jog.sh shoulder_lift +0.1
scripts/arm-jog.sh wrist_roll -0.1
```

RViz joint sliders remain preview-only; they never command the physical arm.


### Arm pose calibration

Run `scripts/calibrate.sh pose`. It starts the temporary host/driver needed to capture the
physical arm in the vendor CAD's **folded home pose** (the all-zero RViz model), saves
`~/.ros/lekiwi_arm_calibration.json`, tears that stack down, then starts the normal stack
when camera calibration is available. Do not use an upright arm for this capture: upright
is a nonzero CAD configuration. If a joint moves opposite in RViz, change only that
joint's `directions` value between `1` and `-1`, then restart.

### Recovery after motor power loss

Real hardware is default-deny. Startup arming is guarded by complete, fresh
telemetry and current permission from the continuous safety supervisor. A host
session change, explicit disarm, or stale/failed telemetry cuts torque,
cancels the interrupted trajectory, and remains disarmed after recovery.
Inspect the robot and then:

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
```

`/safety/state` reports the driver's `DISARMED`, `ARMED`, or `LINK_LOST` state.
The supervisor publishes `safety/supervisor_state`,
`safety/base_motion_permitted`, and `safety/arm_motion_permitted`, and latches
runtime faults until `/safety/reset_fault` is called while the driver is
disarmed and all inputs are healthy. `/safety/disarm` stops ROS commands and
waits for the motor host to confirm that it cut torque on all nine servos.
The host always restarts torque-off. `/safety/arm` holds each arm joint at its
measured position, sends zero wheel velocity, and requires an explicit fresh
arm request. The physical E-stop remains mandatory for any electrical,
mechanical, or process failure.

### Production safety prerequisites

Real mode loads `config/safety_production.yaml`. It denies base and arm motion
until the following current, stamped inputs are present and healthy:

| Input | Topic | Purpose |
| --- | --- | --- |
| Driver state | `safety/driver_state` | Motor-link and torque state |
| Full scan | `/scan` | Obstacle coverage and freshness |
| Depth | `/camera/depth/points` | Arm-workspace obstacles |
| Odometry | `/odom` | Base state |
| IMU | `/imu/data` | Base dynamics |
| Joint state | `/joint_states` | Arm feedback and stow interlock |
| Bumper | `safety/bumper_active` | Contact stop |
| E-stop | `safety/estop_active` | Independent emergency stop state |
| Battery | `/battery_state` | Voltage and charge limits |
| Motor health | `/hardware/diagnostics` | Servo/bus faults |
| Arm collision gate | `/safety/arm_workspace_clear` | Live MoveIt scene/state validity |

The repository ships `config/safety_acceptance.yaml` with `validated: false`.
It is an acceptance template, not proof of safety. A qualified hardware
procedure must record all-direction stopping trials, fault responses, software
revision, sensor configuration, and the measured stow pose before setting it
true. No physical stopping, E-stop, depth, or full production sensor acceptance
is implied by a passing software test.

### Where the camera comes from

The local real robot has three cameras: the existing front V4L2 camera, the
existing wrist V4L2 camera, and an ORBBEC Astra Pro. The Astra's pinned
OpenNI/UVC driver publishes synchronized, depth-registered RGB-D under
`/camera/astra/...`; its cloud is additionally exposed at
`/camera/depth/points` for MoveIt. RTAB-Map uses the Astra RGB-D pair when it
is enabled; otherwise it uses the front RGB camera plus `/scan`, which keeps
the remote-camera topology usable. Set the physical serial in the tracked
`config/hardware.yaml` before enabling Astra: an empty serial is rejected
instead of allowing the driver to claim an arbitrary compatible USB camera.
The full hardware installer also installs the camera's udev rule, so this
works from the managed service without an interactive permission fix.

The driver uses `astra_camera_optical_frame`; keep its physical mount
transform/calibration in `urdf/lekiwi.urdf.xacro` when the Astra mount is
measured, rather than adding a runtime TF. The checked-in zero offset is a
mounting placeholder and must be replaced with the measured Astra bracket
offset. Verify the depth cloud frame and its overlay in RViz before enabling
arm motion.

The repository host is started camera-less for ROS, so a delayed camera frame
cannot take the motor bus down. Direct LeRobot dataset/teleoperation mode may
still be camera-sensitive and should not be used as the ROS motor service.

The front/wrist V4L2 cameras are also the supported remote-camera topology:
frames are read by `v4l2_camera` on the machine where they are plugged in, then
relayed as below. Astra is local-only in this launch configuration.

With a Pi on the robot, `ros-cameras.sh` reads each USB camera there and publishes a
compressed `/pi/camera/...` stream. The workstation relays and expands it onto the
canonical `/camera/...` topics when `camera_source:=remote`; camera frames never pass
through the LeRobot motor host.

On a wired robot, use the default `camera_source:=local`; the existing
`camera_device` continues to identify the front V4L2 camera.

The known JYU2C wrist camera is auto-detected unless `LEKIWI_WRIST=none`. It publishes on `/camera/wrist/image_raw` for watching the gripper. Run `scripts/calibrate.sh wrist` **on the machine the wrist camera is plugged into** before using its `camera_info` for calibrated perception; navigation and RTAB-Map use only the front camera. In remote mode the camera node publishes its compressed wrist stream to the workstation. Both cameras share one USB 2.0 hub, so the wrist feed stays small.

### Odometry scale

LeRobot's kinematics assume a wheel 12.5 cm from the centre of rotation. Measure your robot — wheel centre to wheel centre, divided by √3 — and set `yaw_velocity_scale` to `0.125 / that`. Wheels 24 cm apart give 0.90, the default here. The factor corrects both what the base reports and what it executes, so a rotation Nav2 asks for is the rotation it gets.

Check translation against a printed checkerboard, which needs no measuring tools beyond the board itself:

```bash
ros2 run lekiwi_rmf odom_scale.py --axis linear
```

It drives a short leg and compares the distance the calibrated camera sees against the distance odometry claims. Rotation is better derived from the wheel measurement above: estimating orientation from a flat target viewed head-on is unreliable at small angles.

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

Before anything here, bring the robot up under plain LeRobot: motor IDs,
calibration, cameras, and keyboard teleoperation. See
[HARDWARE.md](HARDWARE.md). If teleoperation does not work there, nothing in
this section will.

Once the calibration steps below have been done once, the wired robot — motors on
the machine that runs ROS — comes up with one command:

```bash
scripts/up.sh
```

It starts the LeRobot host, waits for the nine servos to answer, and brings up the
ROS navigation stack; logs land in `~/.ros/lekiwi`. Real mode remains motion-
denied until the production safety supervisor has current healthy inputs and
`config/safety_acceptance.yaml` has been replaced by a validated physical
acceptance record. A successful launch alone does not mean the robot is safe
to move. On the 4 GB robot computer,
MoveIt and RViz are intentionally opt-in so they cannot starve camera safety. Run
`scripts/rviz.sh` from a workstation when visualisation is needed, and pass
`start_moveit:=true` only for an arm task. `scripts/ros-stop.sh` stops all of it.
With a robot computer holding the devices, its half runs there instead:

```bash
scripts/pi-up.sh                         # on the robot computer (motor host + cameras + LD06)
scripts/workstation-up.sh <PI_IP>         # on the workstation
```

`workstation-up.sh` starts the remote ROS stack and RViz; any following arguments go
to the ROS launch file, so
`scripts/workstation-up.sh 192.168.1.50 slam_mode:=localization` works. The Pi host
has to be up first — the driver gives up and exits if no host
answers on `5555/tcp`. Stop everything with `Ctrl-C`, or `scripts/ros-stop.sh`
from another terminal.

### Boot services

The bringup splits into services named for what they own rather than which
computer they run on. Every topology works: everything on the robot's
computer, devices there and compute on a desk machine, or a wired robot with
both on one desktop.

```bash
scripts/install-device-services.sh          # where motors and cameras plug in
scripts/install-compute-services.sh         # where the ROS stack should run
```

Both installers require a non-root service account. When run through `sudo`,
the invoking account is selected; a direct root invocation must name it. Use
these explicit options when the workspace or virtual environment is not under
the account's default paths:

```bash
sudo scripts/install-device-services.sh \
  --service-user "$USER" --workspace "$HOME/lekiwi_ws" \
  --lerobot-venv "$HOME/lekiwi_ws/.venv-lerobot"
sudo scripts/install-compute-services.sh \
  --service-user "$USER" --workspace "$HOME/lekiwi_ws"
```

For a split device/compute deployment, install the device unit on the motor
machine, then configure the compute machine with:

```bash
sudo scripts/install-compute-services.sh --service-user "$USER" \
  --workspace "$HOME/lekiwi_ws" --remote DEVICE_IP
```

The motor and torque endpoints bind to all interfaces (`0.0.0.0`) by default,
so any reachable server can use unauthenticated ZMQ. Do not expose ports 5555,
5556, or 5557 outside a trusted robot network: an unauthenticated client can
command the robot. CURVE remains an opt-in hardening layer.

Generate the server, health, and driver identities once as the device service
account, then install both halves with a protected key directory. Copy the
driver secret and server public key to the compute host only through an
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
to use the enabled unauthenticated transport. CURVE does not configure a
firewall or authenticate ROS 2 DDS.

The device side installs three units: `lekiwi-host.service` (the motor bus,
served on `5555/tcp` with observations on `5556/tcp` and torque safety on
`5557/tcp`) and `lekiwi-cameras.service` (the cameras, published by
`v4l2_camera` right where they are plugged in — never read by the motor host:
one reader per device, so a stalled camera frame cannot abort the motor bus).
`lekiwi-lidar.service` owns the LD06 serial port and publishes the private
device scan.
The compute side installs `lekiwi-stack.service`:

- without arguments it assumes the device side is this same machine and
  orders itself after the host, starting only once its ZMQ port answers. If a
  `lekiwi-cameras.service` is installed here too, the stack takes that
  service's compressed frames over loopback (`camera_source:=remote`) — v4l2
  allows one reader per camera, and the service already holds them;
- with `--remote <device-address>` it reaches a host on another machine;
  compressed frames stream from the device machine and relays in the bringup
  expand them into the same canonical topics, so nothing downstream can tell
  the topologies apart. It also relays the device LD06's `/pi/lidar/scan` as
  the sole `/scan` publisher by default.

Both installers are re-runnable when the split changes; keep the machines'
clocks roughly in sync (anything NTP-ish) since camera stamps now originate
on the device machine and RTAB-Map pairs them approximately. The stack keeps
retrying until the device half appears, however long the other machine takes
to boot; stop services with `systemctl`, not `scripts/ros-stop.sh`, which the
restart policy would simply undo. RViz is deliberately not a service — it
needs a desktop session — so run `scripts/rviz.sh` when you sit down at it.

All four units run as the selected non-root service account with no new
privileges, a private temporary directory, a read-only system tree, kernel and
control-group protection, and restrictive file creation permissions. Their
HOME remains writable for repository-managed ROS databases, logs, calibration,
and key material; serial and V4L2 devices remain visible because their dynamic
device paths are required by the host/camera services.

### Coordinated split deployment

The normal device and compute service installers seed a least-privilege
deployment sudo rule for their selected account. It permits only LeKiwi unit
start, stop, restart, and reset; it does not grant an arbitrary shell or root
commands. Run the service installers once after changing a unit template or
topology. Ordinary code/configuration deployments then run from the compute
checkout with one command and never prompt for a password:

```bash
scripts/deploy-split.sh DEVICE_IP
```

The initial service installation still needs an administrator authentication:
creating a passwordless privilege rule without one would be a privilege-escalation path.

The deployer first exits quickly when the requested revision, both built
workspaces, and all services are already current. Otherwise it fast-forwards
both clean checkouts to the same pushed commit, confirms torque-off, stops the
compute stack before the device host, rebuilds both service workspaces, and
starts the host, cameras, and LD06 before the compute stack. A temporary
auto-arm inhibit keeps the replacement driver disarmed
until revision, motor-health, and camera checks pass. Any failure leaves the
inhibit at `~/.ros/lekiwi/deploy-inhibit-auto-arm` and does not roll back or
resume a partially deployed robot. Inspect the failure and rerun the deploy;
remove that file manually only when abandoning the deployment after verifying
the robot is safe.

The two halves can also mix ownership: keep the device services running and
drive the stack by hand whenever you feel like it —

```bash
scripts/ros-start.sh camera_source:=remote   # frames come from the cameras service
scripts/rviz.sh                              # Ctrl-C on ros-start.sh stops it
```

— and `scripts/ros-stop.sh` knows about the units: an active
`lekiwi-host.service` or `lekiwi-cameras.service` is left alone and reported,
so stopping a manual stack never takes the boot services down with it.


To watch the robot, `scripts/rviz.sh` opens RViz on `config/lekiwi.rviz` — map,
costmaps, robot model, TF, the goal-pose tool, and a panel for each camera. The
camera panels depend on the saved dock layout in that file; the header of
`scripts/rviz.sh` explains what not to touch.

For camera frames without RViz, one window per publishing camera:

```bash
scripts/cameras.sh
```

To drive the base by hand:

```bash
scripts/teleop.sh
```

Keys go to the terminal running it, and only while that terminal has focus.
Teleoperation publishes `/cmd_vel_manual`; Nav2 publishes `/cmd_vel_smoothed`.
The mux arbitrates them before collision monitoring, but drive or send goals,
not both at once.

Arrows drive: up and down for forward and back, left and right to strafe, `1` and `2`
to turn, space to stop, `9` and `0` for slower and faster. Those keys send the same
character on every layout, so a Dvorak or Latin American keyboard needs no setting.

To drive it from RViz instead, click **2D Goal Pose**, then press on the map where the
robot should end up and drag before releasing to set which way it should face.

### Choosing what publishes /scan

Nav2's obstacle layer and RTAB-Map both read `/scan`, and `laser_source` decides who
produces it on the real robot. In simulation Gazebo always provides it and this argument
does nothing.

`laser_source:=camera` forces the camera fallback. `laser_source:=auto` (the default) uses
the LD06 only when its known CP2102 by-id path is present; otherwise it uses the camera. The
camera fallback needs no extra hardware: the floor is flat, so every
floor pixel is at a known distance, and the first pixel that stops looking like floor is an
obstacle.

It only means anything once the camera's geometry is measured. Lay the printed 8x6
checkerboard flat on the floor in view of the camera and run:

```bash
ros2 run lekiwi_rmf free_space.py --ros-args -p calibrate:=true \
  -r image:=/camera/front/image_raw -r camera_info:=/camera/front/camera_info
```

It prints and saves the camera height and pitch in `~/.ros/lekiwi_launch_calibration.conf`.
Future `scripts/up.sh` launches use them automatically; an explicit launch argument overrides
the saved value:

```bash
scripts/up.sh laser_source:=camera
```

Watch the LaserScan in RViz before trusting it. It reads a uniform floor: patterned tiles,
hard shadows and reflections come back as obstacles, an object the colour of the floor comes
back as nothing, and it measures where things touch the floor, so a table is as far away as
its legs. Wrong height or pitch puts phantom walls in the costmap.

`laser_source:=ld06` replaces all of that guesswork with a real LDROBOT LD06 on its
RobotSkin base -- see HARDWARE.md for the mount, port and permissions. The
normal startup scripts detect its stable `/dev/serial/by-id` device themselves.

The standard device installer starts `lekiwi-lidar.service` on the robot host,
and the standard compute installer relays its private scan as the sole `/scan`
publisher. No LD06-specific installation or launch flag is needed.

Its 12 m range makes the camera trick redundant, which is why the two are
mutually exclusive. `laser_source:=none` is rejected in real mode: production
navigation must retain either camera or LD06 obstacle sensing. It is useful
only for non-motion diagnosis where the real driver is not launched.

The rest of this section walks through bringing up the real robot and the
calibration each piece depends on.

### 1. Start the LeRobot host

The computer attached to the motors and cameras needs LeRobot with the `lekiwi` extra. If it is a separate Raspberry Pi, copy this repository there and install the Pi side:

```bash
./scripts/install-pi.sh
```

It needs a 64-bit image with Python 3.12+. Use Ubuntu 24.04 when the Pi publishes cameras to this Jazzy workstation; Raspberry Pi OS Trixie can run the LeRobot host only. Then, on the Pi:

```bash
scripts/pi-up.sh
```

Keep the host watchdog enabled. Ports `5555/tcp`, `5556/tcp`, and `5557/tcp`
must be reachable from the ROS computer only through the configured protected
control interface; do not expose them to an untrusted network. Check the
repository health handshake before starting ROS:

```bash
"$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/host-health-check.py \
  --host 127.0.0.1
```

### 2. Calibrate the front camera

Calibration is mandatory for visual SLAM. Use the exact camera resolution, lens focus, and mounting that will be used in operation. The example target is an 8-by-6 inner-corner checkerboard with 25 mm squares.

Start the bridge without RMF. AMCL may wait for `/scan` during this calibration-only run; that does not prevent camera publication.

```bash
ros2 launch lekiwi_rmf bringup.launch.py \
  mode:=real remote_ip:=192.168.1.50 \
  curve_client_secret_key_file:=/secure/path/driver.key_secret \
  curve_server_public_key_file:=/secure/path/server.key \
  localization:=amcl publish_camera:=true start_rmf:=false
```

In another sourced terminal:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/front/image_raw camera:=/camera/front
```

Save/commit the calibration in the calibration window. The expected default file is:

```text
~/.ros/camera_info/lekiwi_front.yaml
```

Verify it before moving on:

```bash
ros2 topic hz /camera/front/image_raw
ros2 topic echo /camera/front/camera_info --once
```

The camera matrix `k` must not be all zeroes.

### 3. Calibrate wheel odometry

Place LeKiwi at the RMF charger pose `[-4.0, -2.5, 0.0]`. Measure commanded versus actual straight-line travel and rotation, then tune these driver parameters if needed:

```text
xy_velocity_scale
yaw_velocity_scale
```

Accurate wheel scale matters: monocular images do not provide absolute metric scale by themselves.

### 4. Build the visual map

Use a new database filename for the first mapping run:

```bash
ros2 launch lekiwi_rmf bringup.launch.py \
  mode:=real remote_ip:=192.168.1.50 \
  curve_client_secret_key_file:=/secure/path/driver.key_secret \
  curve_server_public_key_file:=/secure/path/server.key \
  localization:=visual_slam slam_mode:=mapping \
  rtabmap_database:=$HOME/.ros/lekiwi_cleanroom.db
```

This is the mapping procedure, not a safety bypass: the current production
profile still denies motion until its required health inputs and physical
acceptance record are installed. Do not weaken the supervisor just to map;
use a reviewed mapping configuration and retain the hardwired E-stop.

Drive slowly around the complete route and return to previously viewed areas from similar angles so RTAB-Map can close loops. Avoid motion blur, blank walls, changing illumination, and moving the arm through the front camera view.

Stop with `Ctrl-C`; RTAB-Map persists the database at the configured path.

### 5. Operate from the saved map

Start from the charger pose and switch RTAB-Map to localization mode:

```bash
ros2 launch lekiwi_rmf bringup.launch.py \
  mode:=real remote_ip:=192.168.1.50 \
  curve_client_secret_key_file:=/secure/path/driver.key_secret \
  curve_server_public_key_file:=/secure/path/server.key \
  localization:=visual_slam slam_mode:=localization \
  rtabmap_database:=$HOME/.ros/lekiwi_cleanroom.db
```

Then dispatch Nav2 goals or RMF tasks with the same commands used in simulation.

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
| `/camera/front/image_raw` | Gazebo, local `v4l2_camera`, or the remote-camera relay | RTAB-Map |
| `/camera/front/camera_info` | Gazebo, calibrated `v4l2_camera`, or the relay | RTAB-Map |
| `/scan` | Gazebo, `free_space.py`, or the LD06 driver (`laser_source`) | Nav2 and RTAB-Map |
| `map -> odom` | RTAB-Map or AMCL | Nav2 and Free Fleet |
| `/navigate_to_pose` | Nav2 action server | Free Fleet through Zenoh |
| `safety/driver_state` | `lekiwi_driver` | Safety supervisor |
| `safety/supervisor_state` | Safety supervisor | Operator/automation |
| `safety/base_motion_permitted` | Safety supervisor | Mux and driver interlock |
| `safety/arm_motion_permitted` | Safety supervisor | Driver/arm interlock |
| `/safety/arm`, `/safety/disarm`, `/safety/reset_fault` | Driver / supervisor | Explicit operator control |
| `ws://127.0.0.1:9090` | rosbridge when explicitly enabled | Local browser/external client |

By default nothing serves the checked-in PGM: RTAB-Map publishes `/map` itself, drawing the occupancy grid from the camera-derived `/scan` while its visual place database corrects pose drift. The robot starts at the origin of a map it has not seen yet, and the grid grows as it drives. `static_map:=true` puts the checked-in floor plan back on `/map` instead, and moves RTAB-Map's own grid to `/rtabmap/map`.

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
ros2 launch lekiwi_rmf bringup.launch.py mode:=real \
  slam_mode:=localization start_rmf:=true \
  map_bundle:=/absolute/path/to/maps/bundles/site-v1.yaml
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

### `ros2 topic list` hangs and never returns

Orphaned nodes from an earlier bringup are still holding DDS participants. `ros2 launch`
shuts its nodes down on SIGINT to the whole process group; killing the launcher alone
leaves around twenty nodes running. Those orphans keep talking to each other, so a new
stack still works while introspection dies silently. Stop a run with `Ctrl-C`, or from
another terminal:

```bash
scripts/ros-stop.sh
```

### RTAB-Map floods the log with `Not found word N (dict size=M)`

Mapping restarted on top of a database from an earlier session whose visual dictionary no
longer matches. Every loop closure is then rejected with `Not enough features in images
(old=0)`. Delete the database or point `rtabmap_database:=` at a fresh path.

### The machine runs out of memory during a long mapping run

RTAB-Map's working memory lives in RAM. `rtabmap_wm_nodes` (default 300) caps how many
nodes stay resident; the rest move to the database and return when the robot comes back
near them. Lower it on a small machine, raise it where memory allows — a larger working
memory recognises places sooner.

### RTAB-Map database or old crash archives consume disk

Before every repository-managed real-hardware launch, the default
`~/.ros/lekiwi_rtabmap.db` is rotated once it exceeds 512 MiB. Its SQLite sidecars move with
it, so a fresh database cannot replay an old WAL. Automatic `stale-*` and `corrupt-*` archives
are retained for at most 14 days, three sessions, and 1.5 GiB combined (including sidecars).
An explicit `rtabmap_database:=...` is never rotated or deleted; use it for a map that must be
kept. The same policy runs from both `scripts/up.sh` and the systemd `scripts/ros-start.sh` path.
If the stack is already running, `scripts/rtabmap-db-maintenance.py --prune-only` safely applies
only the automatic-archive retention policy; it never opens or moves the active database.

### Installer reports a ParaView/VTK conflict

Ubuntu's `python3-paraview` conflicts with the `python3-vtk9` package required by RTAB-Map through PCL. If you do not need the existing ParaView installation, remove it and rerun the installer:

```bash
sudo apt-get remove paraview python3-paraview
./scripts/install.sh
```

### Driver rejects missing calibration

Visual-SLAM mode deliberately fails if the real camera calibration is missing or empty. Run camera calibration and pass the correct `camera_info_url`.

### RTAB-Map receives no synchronized data

Check all three inputs:

```bash
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/front/camera_info
ros2 topic hz /odom
```

Also verify the static camera transform:

```bash
ros2 run tf2_ros tf2_echo base_footprint front_camera_optical_frame
```

### pytest fails with `PluginValidationError`

A newer `pytest` in `~/.local` shadows the one ROS's `launch_testing` plugins
expect. Disable plugin autoloading for the unit tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test -q
```

### Free Fleet does not discover `lekiwi_1`

Confirm that the bridge is running and Nav2 provides the action server:

```bash
pgrep -af zenoh-bridge-ros2dds
ros2 action list | grep navigate_to_pose
ROS_DOMAIN_ID=0 ros2 node list
```

### A rosbridge client cannot connect

Confirm that rosbridge is listening and uses the intended ROS domain:

```bash
ros2 node list | grep -E 'rosapi|rosbridge'
ss -ltn | grep ':9090'
```

Remote clients require an explicitly permitted firewall rule for TCP 9090 and
an authenticated proxy; a raw unauthenticated WebSocket is not supported for
robot control.

### Localization jumps or closes false loops

Recalibrate the camera and wheel scale first. Then remap with slower motion and more distinctive views. Repetitive cleanroom walls are difficult for feature-based place recognition.

## Safety and current limits

- Real mode uses a default-deny, continuously evaluated safety supervisor. A
  one-shot readiness message is not a motion permit. Missing or stale required
  inputs deny motion; a runtime fault latches until the driver is disarmed and
  `/safety/reset_fault` is explicitly called.
- Keep a hardwired physical E-stop reachable and supervise every hardware run.
  The ROS E-stop topic and software torque cut are status/control interfaces,
  not substitutes for removing actuator energy independently of ROS.
- The production profile requires `/scan`, `/camera/depth/points`, `/odom`,
  `/imu/data`, `/joint_states`, `/battery_state`,
  `/hardware/diagnostics`, `safety/bumper_active`, and
  `safety/estop_active`, plus `safety/driver_state`. The supplied
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
- The driver may arm only during guarded startup after receiving fresh healthy
  telemetry and supervisor permission. A host session change, disarm, or link
  loss never restores torque automatically; `/safety/arm` is then an explicit
  operator action after inspection, while `/safety/disarm` cuts torque through
  the motor host and aborts arm motion.
- Rosbridge is disabled by default and loopback-bound when enabled. It has no
  authentication or TLS. Keep it on loopback, or put a separately managed
  authenticated proxy and firewall in front of it. The motor command and
  torque endpoints also default to loopback, but support the explicitly
  configured unauthenticated device LAN transport.
- This repository does not configure DDS security. ROS 2 discovery and the
  control graph are residual network exposure unless the deployment supplies
  DDS security or network isolation; neither rosbridge nor CURVE secures DDS.
- The motor-host watchdog cuts and verifies all servo torque when commands
  cease; authenticated telemetry makes the ROS driver observe that cut and
  require an explicit re-arm. This software mechanism is not a
  substitute for an E-stop. A front monocular floor scan is supplemental and
  cannot see all side/rear, floor-coloured, low, or overhanging obstacles.
- Production MoveIt now has an execution-time arm-workspace gate. It requires
  fresh complete joint state, a freshly stamped octomap and repeated successful
  `/check_state_validity` responses; collision, timeout or perception failure
  withdraws the arm lease. Its discrete check and software stop still require
  measured physical latency/intrusion acceptance, and the CAD collision matrix
  needs a measured collision-free calibration pose.
- Gazebo drives three wheel joints at the CAD-derived 120-degree layout. The
  launch-time `sim_sdf` conversion adds anisotropic `fdir1` contact friction,
  and encoder joint positions—not model ground truth—produce `/odom`.
  Commands have velocity, acceleration, jerk, wheel-rate, and ROS watchdog
  limits. A Gazebo-native 250 ms failsafe now owns the final actuator topics:
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
