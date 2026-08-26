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
- MoveIt planning and execution for the five-joint arm on real hardware;
- RTAB-Map monocular place recognition and loop closure using metric wheel odometry;
- a Free Fleet adapter connecting Nav2 to Open-RMF;
- optional rosbridge WebSocket access for browsers and external applications;
- one installer for the supported development/runtime stack.

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

## Simulation quick start

Source the environment in every new terminal:

```bash
source scripts/setup.bash
```

Launch Gazebo, visual SLAM, and Nav2:

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim \
  rtabmap_database:=$HOME/.ros/lekiwi_sim.db
```

Test Nav2 only after `/scan` has real ranges and `/map` covers a clear nearby
goal (see `TROUBLESHOOTING.md`):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.3, y: 0.0}, orientation: {w: 1.0}}}}'
```

RMF is opt-in because it connects the robot to the configured fleet graph. Add
`start_rmf:=true` to the launch above, then dispatch an RMF patrol from another sourced terminal:

```bash
ROS_DOMAIN_ID=0 ros2 run rmf_demos_tasks dispatch_patrol \
  -p charger dropoff -n 1 --use_sim_time
```

On later runs, reuse the visual database without adding new map nodes:

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim \
  slam_mode:=localization rtabmap_database:=$HOME/.ros/lekiwi_sim.db
```

## Launch options

| Argument | Values | Default | Purpose |
| --- | --- | --- | --- |
| `mode` | `sim`, `real` | `sim` | Select Gazebo or the LeRobot hardware bridge |
| `headless` | `true`, `false` | `true` | Run Gazebo server-only with offscreen rendering; set false to open its GUI |
| `localization` | `visual_slam`, `amcl` | `visual_slam` | Select the sole `map -> odom` provider |
| `slam_mode` | `mapping`, `localization` | `mapping` | Extend or reuse the RTAB-Map database |
| `remote_ip` | IPv4/hostname | `127.0.0.1` | Address of the LeKiwi ZMQ host |
| `rtabmap_database` | file path | `~/.ros/lekiwi_rtabmap.db` | Visual map database |
| `camera_info_url` | ROS camera URL | `file://~/.ros/camera_info/lekiwi_front.yaml` | Real front-camera calibration |
| `wrist_camera_info_url` | ROS camera URL | `file://~/.ros/camera_info/lekiwi_wrist.yaml` | Optional wrist-camera calibration |
| `camera_source` | `local`, `remote` | `local` | Read the camera here, or decompress what the robot's Pi publishes |
| `camera_device` | V4L2 path | `/dev/video0` | Front camera when `camera_source:=local` |
| `laser_source` | `auto`, `camera`, `ld06`, `none` | `auto` | Select camera fallback or LD06 on real hardware; Gazebo supplies `/scan` in sim |
| `lidar_port` | serial path | CP2102 `/dev/serial/by-id/...` | LD06 device when `laser_source:=ld06` |
| `camera_height`, `camera_offset_x`, `camera_offset_y` | metres | `0.093`, `0.03`, `0.0` | Front-camera pose used by the camera scan |
| `camera_pitch`, `camera_yaw`, `camera_roll` | radians | `0.031`, `0.0`, `0.0` | Front-camera orientation used by the camera scan |
| `xy_velocity_scale` | float | `1.0` | Correction for reported and commanded translation |
| `yaw_velocity_scale` | float | `0.90` | Correction for reported and commanded rotation |
| `start_rmf` | `true`, `false` | `false` | Start Zenoh, RMF schedule, dispatcher, and fleet adapter |
| `rmf_domain` | integer | `0` | DDS domain used by RMF processes; must match the robot graph unless a cross-domain bridge is configured |
| `start_rosbridge` | `true`, `false` | `true` | Start rosbridge WebSocket and ROS API nodes |
| `start_moveit` | `true`, `false` | `true` | Start MoveIt arm planning and execution (real hardware only) |
| `rosbridge_address` | bind address | `0.0.0.0` | Interface exposed by rosbridge |
| `rosbridge_port` | TCP port | `9090` | WebSocket listening port |
| `rosbridge_domain` | integer | `0` | ROS graph exposed through rosbridge |

Only one localization mode should run. `visual_slam` publishes `map -> odom` through RTAB-Map; `amcl` publishes it from the fixed occupancy map.

### Arm planning

Start MoveIt with the real robot. RViz opens with the **MotionPlanning** display;
select the `arm` group, then plan and execute normally:

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=real
```

MoveIt executes through `/arm_controller/follow_joint_trajectory`. Gazebo keeps
the unactuated arm rigid, so arm execution is deliberately unavailable in `mode:=sim`.
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

The driver automatically arms once at startup, but only after it receives complete, fresh motor
telemetry. It begins with a zero command, so arming alone does not make the robot move.
After stale or failed telemetry it keeps publishing measured joint state, cancels the interrupted
trajectory, and remains disarmed after recovery. An operator must inspect the robot and explicitly
arm it before sending a new motion command:

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
```

`/safety/state` reports `DISARMED`, `ARMED`, or `LINK_LOST`. A power-loss recovery always needs
a fresh navigation or MoveIt command; an interrupted trajectory is aborted and is never replayed.
`safety/disarm` stops ROS commands, but cannot remove servo torque: LeRobot's ZMQ client does not
expose torque control, so use the physical emergency stop for that.

### Where the camera comes from

The cameras are read by `v4l2_camera` nodes on whichever machine they are plugged into, not relayed through the LeRobot host. The host aborts the whole robot — motor control included — when a camera frame arrives more than half a second late, and USB webcams do that regularly.

With a Pi on the robot, `ros-cameras.sh` reads each USB camera there and publishes a
compressed `/pi/camera/...` stream. The workstation relays and expands it onto the
canonical `/camera/...` topics when `camera_source:=remote`; camera frames never pass
through the LeRobot motor host.

On the wired variant everything is local, so `camera_source:=local` with `camera_device` pointing at a `/dev/v4l/by-id/...` path.

The known JYU2C wrist camera is auto-detected unless `LEKIWI_WRIST=none`. It publishes on `/camera/wrist/image_raw` for watching the gripper. Run `scripts/calibrate.sh wrist` **on the machine the wrist camera is plugged into** before using its `camera_info` for calibrated perception; navigation and RTAB-Map use only the front camera. In remote mode the camera node publishes its compressed wrist stream to the workstation. Both cameras share one USB 2.0 hub, so the wrist feed stays small.

### Odometry scale

LeRobot's kinematics assume a wheel 12.5 cm from the centre of rotation. Measure your robot — wheel centre to wheel centre, divided by √3 — and set `yaw_velocity_scale` to `0.125 / that`. Wheels 24 cm apart give 0.90, the default here. The factor corrects both what the base reports and what it executes, so a rotation Nav2 asks for is the rotation it gets.

Check translation against a printed checkerboard, which needs no measuring tools beyond the board itself:

```bash
ros2 run lekiwi_rmf odom_scale.py --axis linear
```

It drives a short leg and compares the distance the calibrated camera sees against the distance odometry claims. Rotation is better derived from the wheel measurement above: estimating orientation from a flat target viewed head-on is unreliable at small angles.

## WebSocket access with rosbridge

Rosbridge is separate from Zenoh: Zenoh remains the transport required by Free Fleet, while rosbridge exposes one ROS graph as JSON over WebSocket. It starts by default and binds on every IPv4 interface (`0.0.0.0`).

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim start_rosbridge:=true
```

Connect a rosbridge-compatible client such as roslibjs to the robot's address:

```text
ws://ROBOT_IP:9090
```

Minimal roslibjs connection:

```js
const ros = new ROSLIB.Ros({url: "ws://ROBOT_IP:9090"});
ros.on("connection", () => console.log("connected"));
ros.on("error", console.error);
```

The default `rosbridge_domain:=0` exposes LeKiwi, camera, SLAM, and Nav2. To expose the separate RMF graph instead:

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim \
  rosbridge_domain:=55
```

Rosbridge has no authentication, authorization, or TLS. Any network client that can reach port
9090 can publish to
`/cmd_vel_manual`, which reaches the collision-monitor path and can cause motion
once the driver has armed. Use this configuration only on a trusted, access-controlled LAN. For
an untrusted network, put an authenticated TLS proxy and firewall policy in front of port 9090.

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
ROS navigation stack; logs land in `~/.ros/lekiwi`. On the 4 GB robot computer,
MoveIt and RViz are intentionally opt-in so they cannot starve camera safety. Run
`scripts/rviz.sh` from a workstation when visualisation is needed, and pass
`start_moveit:=true` only for an arm task. `scripts/ros-stop.sh` stops all of it.
With a robot computer holding the devices, its half runs there instead:

```bash
scripts/pi-up.sh                         # on the robot computer (motor host + cameras)
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

The device side installs two units: `lekiwi-host.service` (the motor bus,
served on `5555/tcp`) and `lekiwi-cameras.service` (the cameras, published by
`v4l2_camera` right where they are plugged in — never read by the motor host:
one reader per device, so a stalled camera frame cannot abort the motor bus).
The compute side installs `lekiwi-stack.service`:

- without arguments it assumes the device side is this same machine and
  orders itself after the host, starting only once its ZMQ port answers. If a
  `lekiwi-cameras.service` is installed here too, the stack takes that
  service's compressed frames over loopback (`camera_source:=remote`) — v4l2
  allows one reader per camera, and the service already holds them;
- with `--remote <device-address>` it reaches a host on another machine;
  compressed frames stream from the device machine and relays in the bringup
  expand them into the same canonical topics, so nothing downstream can tell
  the topologies apart.

Both installers are re-runnable when the split changes; keep the machines'
clocks roughly in sync (anything NTP-ish) since camera stamps now originate
on the device machine and RTAB-Map pairs them approximately. The stack keeps
retrying until the device half appears, however long the other machine takes
to boot; stop services with `systemctl`, not `scripts/ros-stop.sh`, which the
restart policy would simply undo. RViz is deliberately not a service — it
needs a desktop session — so run `scripts/rviz.sh` when you sit down at it.

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

Keys go to the terminal running it, and only while that terminal has focus. Teleoperation
and Nav2 both write `/cmd_vel`, so drive or send goals, not both at once.

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

`laser_source:=ld06` replaces all of that guesswork with a real LDROBOT LD06 on the mast --
see HARDWARE.md for the mount, port and permissions. Give the bringup the serial device,
preferably by its stable `/dev/serial/by-id` name:

```bash
scripts/up.sh laser_source:=ld06 lidar_port:=/dev/serial/by-id/usb-...-if00-port0
```

Its 12 m range makes the camera trick redundant, which is why the two are mutually
exclusive: pick one, or `laser_source:=none` to run without obstacle detection (RViz still
shows everything else; Nav2 just plans against walls it cannot see).

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

Keep the host watchdog enabled. Ports `5555/tcp` and `5556/tcp` must be reachable from the ROS computer; do not expose them to an untrusted network.

### 2. Calibrate the front camera

Calibration is mandatory for visual SLAM. Use the exact camera resolution, lens focus, and mounting that will be used in operation. The example target is an 8-by-6 inner-corner checkerboard with 25 mm squares.

Start the bridge without RMF. AMCL may wait for `/scan` during this calibration-only run; that does not prevent camera publication.

```bash
ros2 launch lekiwi_rmf bringup.launch.py \
  mode:=real remote_ip:=192.168.1.50 \
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
  localization:=visual_slam slam_mode:=mapping \
  rtabmap_database:=$HOME/.ros/lekiwi_cleanroom.db
```

Drive slowly around the complete route and return to previously viewed areas from similar angles so RTAB-Map can close loops. Avoid motion blur, blank walls, changing illumination, and moving the arm through the front camera view.

Stop with `Ctrl-C`; RTAB-Map persists the database at the configured path.

### 5. Operate from the saved map

Start from the charger pose and switch RTAB-Map to localization mode:

```bash
ros2 launch lekiwi_rmf bringup.launch.py \
  mode:=real remote_ip:=192.168.1.50 \
  localization:=visual_slam slam_mode:=localization \
  rtabmap_database:=$HOME/.ros/lekiwi_cleanroom.db
```

Then dispatch Nav2 goals or RMF tasks with the same commands used in simulation.

## Frames and interfaces

```text
map -> odom -> base_footprint -> mast -> front_camera_link -> front_camera_optical_frame
```

| Interface | Producer | Consumer |
| --- | --- | --- |
| `/cmd_vel` | Nav2 | Gazebo or `lekiwi_driver` |
| `/odom`, `odom -> base_footprint` | Gazebo or `lekiwi_driver` | RTAB-Map and Nav2 |
| `/camera/front/image_raw` | Gazebo, local `v4l2_camera`, or the remote-camera relay | RTAB-Map |
| `/camera/front/camera_info` | Gazebo, calibrated `v4l2_camera`, or the relay | RTAB-Map |
| `/scan` | Gazebo, `free_space.py`, or the LD06 driver (`laser_source`) | Nav2 and RTAB-Map |
| `map -> odom` | RTAB-Map or AMCL | Nav2 and Free Fleet |
| `/navigate_to_pose` | Nav2 action server | Free Fleet through Zenoh |
| `ws://ROBOT_IP:9090` | rosbridge | Browser/external clients |

By default nothing serves the checked-in PGM: RTAB-Map publishes `/map` itself, drawing the occupancy grid from the camera-derived `/scan` while its visual place database corrects pose drift. The robot starts at the origin of a map it has not seen yet, and the grid grows as it drives. `static_map:=true` puts the checked-in floor plan back on `/map` instead, and moves RTAB-Map's own grid to `/rtabmap/map`.

A live map and the checked-in RMF navigation graph do not agree on coordinates: the graph names points in the PGM's frame. Keep `start_rmf:=false` while mapping, or regenerate the graph against the map the robot draws.

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

Remote clients require an explicitly permitted firewall rule for TCP 9090.

### Localization jumps or closes false loops

Recalibrate the camera and wheel scale first. Then remap with slower motion and more distinctive views. Repetitive cleanroom walls are difficult for feature-based place recognition.

## Safety and current limits

- Use a physical emergency stop and supervise every first hardware run.
- Rosbridge is disabled by default. If enabled, it has no authentication or TLS and must remain loopback-only unless a separately managed authenticated proxy restricts access.
- The LeRobot watchdog stops the base when commands cease; this is not a substitute for an E-stop.
- Gazebo uses native planar velocity control. Omniwheels are visual geometry, not roller-contact physics.
- Monocular floor segmentation is a conservative supplemental obstacle source, not a substitute for lidar/depth sensing; it cannot see floor-coloured or overhanging obstacles.
- Battery drain is disabled because stock LeKiwi does not report battery state.
- RMF schedules mobile-base patrol and delivery. It does not schedule arm trajectories in this package.
