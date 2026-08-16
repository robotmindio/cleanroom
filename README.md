# LeKiwi ROS 2 + visual SLAM + Open-RMF

A shared ROS 2 Jazzy stack for running a LeKiwi mobile manipulator in Gazebo or on real hardware. Nav2 and Open-RMF see the same robot interface in both modes.

```text
Open-RMF -> Free Fleet -> Nav2 -> /cmd_vel
                         ^           |-- Gazebo Harmonic
                         |           `-- LeRobot ZMQ host -> LeKiwi
                         |
                RTAB-Map visual SLAM
                front RGB + wheel odometry
```

The package includes:

- a Gazebo cleanroom and LeKiwi model;
- a metric Nav2 occupancy map and matching RMF navigation graph;
- a LeRobot-to-ROS driver for velocity, odometry, arm joints, and the front camera;
- RTAB-Map monocular place recognition and loop closure using metric wheel odometry;
- a Free Fleet adapter connecting Nav2 to Open-RMF;
- optional rosbridge WebSocket access for browsers and external applications;
- one installer for the supported development/runtime stack.

## Supported platform

- Ubuntu 24.04, `amd64` or `arm64`
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12

The installer intentionally rejects other platforms. Jazzy/Harmonic is the official pairing documented by [Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/), and Open-RMF recommends its Jazzy binary packages via `ros-jazzy-rmf-dev`.

## Install everything

The installer uses `sudo` for apt packages, downloads pinned Free Fleet/RMF task sources, installs the Zenoh bridge in `~/.local/bin`, creates a Python virtual environment, installs LeRobot, and builds the workspace. It does not modify `.bashrc` or `.zshrc`.

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

Rerunning the installer is safe. It stops if a managed source checkout contains local changes.

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

Launch Gazebo, visual SLAM, Nav2, Zenoh, and RMF:

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim \
  rtabmap_database:=$HOME/.ros/lekiwi_sim.db
```

Test Nav2 directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: -2.0}, orientation: {w: 1.0}}}}'
```

Dispatch an RMF patrol from another sourced terminal:

```bash
ROS_DOMAIN_ID=55 ros2 run rmf_demos_tasks dispatch_patrol \
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
| `localization` | `visual_slam`, `amcl` | `visual_slam` | Select the sole `map -> odom` provider |
| `slam_mode` | `mapping`, `localization` | `mapping` | Extend or reuse the RTAB-Map database |
| `remote_ip` | IPv4/hostname | `127.0.0.1` | Address of the LeKiwi ZMQ host |
| `rtabmap_database` | file path | `~/.ros/lekiwi_rtabmap.db` | Visual map database |
| `camera_info_url` | ROS camera URL | `file://${ROS_HOME}/camera_info/lekiwi_front.yaml` | Real front-camera calibration |
| `camera_source` | `local`, `remote` | `local` | Read the camera here, or decompress what the robot's Pi publishes |
| `camera_device` | V4L2 path | `/dev/video0` | Front camera when `camera_source:=local` |
| `xy_velocity_scale` | float | `1.0` | Correction for reported and commanded translation |
| `yaw_velocity_scale` | float | `0.90` | Correction for reported and commanded rotation |
| `start_rmf` | `true`, `false` | `true` | Start Zenoh, RMF schedule, dispatcher, and fleet adapter |
| `rmf_domain` | integer | `55` | DDS domain used by RMF processes |
| `start_rosbridge` | `true`, `false` | `true` | Start rosbridge WebSocket and ROS API nodes |
| `rosbridge_address` | bind address | `0.0.0.0` | Interface exposed by rosbridge |
| `rosbridge_port` | TCP port | `9090` | WebSocket listening port |
| `rosbridge_domain` | integer | `0` | ROS graph exposed through rosbridge |

Only one localization mode should run. `visual_slam` publishes `map -> odom` through RTAB-Map; `amcl` publishes it from the fixed occupancy map.

### Where the camera comes from

The cameras are read by `v4l2_camera` nodes on whichever machine they are plugged into, not relayed through the LeRobot host. The host aborts the whole robot — motor control included — when a camera frame arrives more than half a second late, and USB webcams do that regularly.

With a Pi on the robot, the Pi runs the camera nodes (see [HARDWARE.md](HARDWARE.md)) and the workstation launches with `camera_source:=remote`. Only the compressed image crosses the network; the workstation expands it back onto `/camera/front/image_raw`. Raw 640x480 at 30 Hz is 27 MB/s, which robot wifi will not carry.

On the wired variant everything is local, so `camera_source:=local` with `camera_device` pointing at a `/dev/v4l/by-id/...` path.

### Odometry scale

LeRobot's kinematics assume a wheel 12.5 cm from the centre of rotation. Measure your robot — wheel centre to wheel centre, divided by √3 — and set `yaw_velocity_scale` to `0.125 / that`. Wheels 24 cm apart give 0.90, the default here. The factor corrects both what the base reports and what it executes, so a rotation Nav2 asks for is the rotation it gets.

Check translation against a printed checkerboard, which needs no measuring tools beyond the board itself:

```bash
ros2 run lekiwi_rmf odom_scale.py --axis linear
```

It drives a short leg and compares the distance the calibrated camera sees against the distance odometry claims. Rotation is better derived from the wheel measurement above: estimating orientation from a flat target viewed head-on is unreliable at small angles.

## WebSocket access with rosbridge

Rosbridge is separate from Zenoh: Zenoh remains the transport required by Free Fleet, while rosbridge exposes one ROS graph as JSON over WebSocket. It starts by default and listens on every network interface for simple test access.

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim
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

This test configuration has no authentication, authorization, or TLS. Use it only on a disposable or trusted test network.

## Real robot

Before anything here, bring the robot up under plain LeRobot: motor IDs,
calibration, cameras, and keyboard teleoperation. See
[HARDWARE.md](HARDWARE.md). If teleoperation does not work there, nothing in
this section will.

### 1. Start the LeRobot host

The computer attached to the motors and cameras needs LeRobot with the `lekiwi` extra. If it is a separate Raspberry Pi, copy this repository there and install only the host side — `scripts/install.sh` is for the ROS workstation and must not run on the Pi:

```bash
./scripts/install-pi.sh
```

It needs a 64-bit image with Python 3.12+ (Raspberry Pi OS Trixie or Ubuntu 24.04; Bookworm's 3.11 is too old). Then, on the Pi:

```bash
scripts/lekiwi.sh host
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
  camera_info_url:='file://${ROS_HOME}/camera_info/lekiwi_front.yaml' \
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
  camera_info_url:='file://${ROS_HOME}/camera_info/lekiwi_front.yaml' \
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
| `/camera/front/image_raw` | Gazebo or `lekiwi_driver` | RTAB-Map |
| `/camera/front/camera_info` | Gazebo or calibrated driver | RTAB-Map |
| `map -> odom` | RTAB-Map or AMCL | Nav2 and Free Fleet |
| `/navigate_to_pose` | Nav2 action server | Free Fleet through Zenoh |
| `ws://ROBOT_IP:9090` | rosbridge | Browser/external clients |

The checked-in PGM provides Nav2's 2D collision map. Monocular RTAB-Map builds a visual place database and corrects pose drift; it cannot infer obstacle depth or generate a reliable occupancy map from one RGB image.

## Troubleshooting

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
ROS_DOMAIN_ID=55 ros2 node list
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
- Rosbridge listens on every interface without authentication or TLS for testing. Use only a disposable or trusted network.
- The LeRobot watchdog stops the base when commands cease; this is not a substitute for an E-stop.
- Gazebo uses native planar velocity control. Omniwheels are visual geometry, not roller-contact physics.
- Monocular vision has no direct depth, so the static Nav2 map is the only obstacle source unless lidar/depth sensing is added.
- Battery drain is disabled because stock LeKiwi does not report battery state.
- RMF schedules mobile-base patrol and delivery. It does not schedule arm trajectories in this package.
