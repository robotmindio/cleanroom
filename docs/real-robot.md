# Real robot

Part of the [LeKiwi README](../README.md). Network exposure and its
protection are in the README's [Network security](../README.md#network-security).

Before anything here, bring the robot up under plain LeRobot: motor IDs,
calibration, cameras, and keyboard teleoperation. See
[HARDWARE.md](../HARDWARE.md). If teleoperation does not work there, nothing in
this section will.

Once the calibration steps below have been done once, the wired robot — motors on
the machine that runs ROS — comes up with one command:

```bash
scripts/up.sh
```

It starts the LeRobot host, waits for the nine servos to answer, and brings up the
ROS navigation stack; logs land in `~/.ros/lekiwi`. Run `scripts/rviz.sh` to watch it. With
`LEKIWI_DISARM_ON_FAILURE=true` real mode stays motion-denied until the production
safety supervisor has current healthy inputs and `config/safety_acceptance.yaml`
has been replaced by a validated physical acceptance record; by default the
supervisor only reports them. A successful launch alone does not mean the robot
is safe to move. On the 4 GB robot computer, MoveIt and RViz are intentionally
opt-in so they cannot starve camera safety. In a split deployment, the compute
installer and `scripts/workstation-up.sh` enable MoveIt on the workstation by
default.

With a robot computer holding the devices, its half runs there instead:

```bash
scripts/pi-up.sh                 # on the robot computer
scripts/workstation-up.sh        # on the workstation
```

`pi-up.sh` starts the motor host, cameras, LD06, Astra, and zenoh sensor bridge.
`workstation-up.sh` starts the ROS stack (MoveIt, remote cameras and lidar) and
RViz against the robot named by `LEKIWI_ROBOT_HOST`; pass the address as the first
argument to override it, and any following `name:=value` arguments go to the ROS
launch file, e.g. `scripts/workstation-up.sh slam_mode:=localization`. The device
half has to be up first — the driver gives up and exits if no host answers on
`5555/tcp`. `scripts/ros-stop.sh` stops what `up.sh`, `pi-up.sh` and
`workstation-up.sh` started; it never touches systemd-owned units, only reports
them.

## Configuration

Copy `.env.example` to `.env` in the repository root. Scripts read exactly three
keys from it: `LEKIWI_ROBOT_HOST` (the robot computer's hostname or IPv4 address,
used by `workstation-up.sh`, `ros-start.sh`, `deploy-split.sh` and the compute
installer), `LEKIWI_DISARM_ON_FAILURE` (`true` or `false`) and
`LEKIWI_WIFI_COUNTRY` (the two-letter Wi-Fi regulatory country, default `ID`,
applied by `install.sh` and `install-device-services.sh`). A value already
present in the environment wins. Every other `LEKIWI_*` override (`LEKIWI_WS`,
`LEKIWI_LOGS`, `LEKIWI_FRONT`, `LEKIWI_WRIST`, `LEKIWI_PORT`, `LEKIWI_ID`,
`LEKIWI_STACK_ARGS`, ...) is honoured only as an environment variable; the same
name in `.env` is ignored. Under systemd the compute stack's extra launch
arguments live in `/etc/default/lekiwi-stack` (`LEKIWI_STACK_ARGS`), written by
the installer.

## Boot services

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

Both fail early if the selected workspace or LeRobot Python is missing. They
render, verify, reload, and enable the units.

For a split device/compute deployment, install the device units on the motor
machine, then configure the compute machine with:

```bash
# From the compute machine: -t lets the device's sudo ask for the one-time
# bootstrap password.
ssh -t USER@DEVICE_IP 'cd /path/to/cleanroom && sudo ./scripts/install-device-services.sh'

# On the compute machine.
sudo scripts/install-compute-services.sh --service-user "$USER" \
  --workspace "$HOME/lekiwi_ws" --remote DEVICE_IP
```

With `LEKIWI_ROBOT_HOST` set in `.env` the compute installer uses it as
`--remote`. After changing it, reinstall that compute service with
`scripts/reinstall-compute.sh`. The installers restart a running service only
when its unit file or installed configuration changed.

Units installed:

| Machine | Unit | Owns |
| --- | --- | --- |
| Device | `lekiwi-host.service` | The motor bus, camera-less: motion on `5555/tcp`, observations on `5556/tcp`, torque safety on `5557/tcp` |
| Device | `lekiwi-cameras.service` | Front and wrist cameras, published by `v4l2_camera` right where they are plugged in — never read by the motor host: one reader per device, so a stalled camera frame cannot abort the motor bus (installed when `v4l2_camera` is available) |
| Device | `lekiwi-astra.service` | The Astra Pro RGB-D publisher (installed when `astra_camera` is available) |
| Device | `lekiwi-lidar.service` | The LD06 serial port; publishes the private `/pi/lidar/scan` |
| Device | `lekiwi-zenoh.service` | Zenoh bridge exporting the sensor topics to the compute machine on `7447/tcp` |
| Compute | `lekiwi-stack.service` | The ROS bringup (`scripts/ros-start.sh`) |
| Both | `lekiwi-ros-logrotate.timer` | ROS log rotation |

The compute installer picks the topology:

- without arguments it assumes the device side is this same machine and
  orders itself after the host, starting only once its ZMQ port answers. If a
  `lekiwi-cameras.service` is installed here too, the stack takes that
  service's compressed frames over loopback (`camera_source:=remote`) — v4l2
  allows one reader per camera, and the service already holds them;
- with `--remote <device-address>` it reaches a host on another machine;
  rate-limited compressed camera previews, the compact Astra cloud, and the
  device LD06's `/pi/lidar/scan` arrive through the zenoh bridge; image relays
  in the bringup expand them into
  the same canonical topics (`/scan` has the body-masked LD06 as its sole publisher
  by default), so nothing downstream can tell the topologies apart.

Both installers are re-runnable when the split changes; keep the machines'
clocks roughly in sync (anything NTP-ish) since camera stamps originate on the
device machine and are paired approximately. The stack keeps retrying until the
device half appears, however long the other machine takes to boot; stop
services with `systemctl`, since `scripts/ros-stop.sh` leaves them alone and
the restart policy would undo a raw kill anyway. RViz is deliberately not a
service — it needs a desktop session — so run `scripts/rviz.sh` when you sit
down at it. Inspect the units with:

```bash
systemctl status lekiwi-host.service lekiwi-cameras.service lekiwi-astra.service \
  lekiwi-lidar.service lekiwi-zenoh.service lekiwi-stack.service
journalctl -u lekiwi-host.service -f
```

All units run as the selected non-root service account with no new
privileges, a private temporary directory, a protected system tree, kernel and
control-group protection, and restrictive file creation permissions. Their
HOME remains writable for repository-managed ROS databases, logs, calibration,
and key material; serial and V4L2 devices remain visible because their dynamic
device paths are required by the host, camera and lidar services.

The motor host runs continuously. A clean stop or restart of
`lekiwi-host.service` disconnects it and cuts servo torque; the arming policy is
described in [Arming and recovery](launch-options.md#arming-and-recovery).

## Coordinated split deployment

The normal device and compute service installers also seed a least-privilege
deployment sudo rule for their selected account. It permits only LeKiwi unit
start, stop, restart, and reset; it does not grant an arbitrary shell or root
commands. Run the service installers once after changing a unit template or
topology. Ordinary code/configuration deployments then run from the compute
checkout with one command and never prompt for a password:

```bash
scripts/deploy-split.sh
```

The initial service installation still needs an administrator authentication:
creating a passwordless privilege rule without one would be a privilege-escalation path.
Reinstalling a stale compute configuration also needs full sudo; the deployer
checks `sudo -n true` before it touches the robot and stops with the command to
run instead of waiting for a password.

The deployer first exits quickly when the requested revision, both built
workspaces, and all services are already current. Otherwise it fast-forwards
both clean checkouts to the same pushed commit, disarms and confirms
torque-off, stops the compute stack, reinstalls a stale or misconfigured compute
service configuration without starting it, stops the device services, rebuilds both service workspaces, and starts the host,
Astra, cameras, LD06 and zenoh bridge before the compute stack. It verifies
revision, motor health, cameras, the Astra cloud and the LD06 scan with the
driver disarmed, then re-arms it unless `LEKIWI_DISARM_ON_FAILURE=true`, in
which case the robot stays disarmed for an operator. Any failure does not roll
back or resume a partially deployed robot. Inspect the failure and rerun the
deploy.

The two halves can also mix ownership: keep the device services running, stop
`lekiwi-stack.service`, and run `scripts/workstation-up.sh` on the compute
machine by hand. `scripts/ros-stop.sh` stops that manual stack and leaves the
active `lekiwi-*` boot services running, reporting each, so stopping a manual
stack never takes them down with it.

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

## Choosing what publishes /scan

Nav2's obstacle layer and the SLAM cloud both read `/scan`, and `laser_source` decides who
produces it on the real robot. LD06 scans pass through `scan_self_filter` first, which drops
the returns from the robot's own body (`config/lidar_self_mask.yaml`); without it those
points sit inside the collision monitor's stop zone and hold the base at zero. Re-measure the
sector with a stationary, stowed robot after changing anything mounted within 20 cm of the lidar. In simulation Gazebo always provides it and this argument
does nothing.

`laser_source:=camera` forces the camera fallback. `laser_source:=auto` (the default) uses
the LD06 only when its known CP2102 by-id path is present; otherwise it uses the camera. The
camera fallback needs no extra hardware: the floor is flat, so every
floor pixel is at a known distance, and the first pixel that stops looking like floor is an
obstacle.

It only means anything once the camera's geometry is measured. Lay the printed 8x6
checkerboard flat on the floor in view of the camera and, on a wired robot,
run `scripts/calibrate.sh height`, or with the stack running run:

```bash
ros2 run lekiwi_rmf free_space.py --ros-args -p calibrate:=true \
  -r image:=/camera/front/image_raw -r camera_info:=/camera/front/camera_info
```

It prints and saves the camera height and pitch in `~/.ros/lekiwi_launch_calibration.conf`.
Every launch through `scripts/ros-start.sh` (`scripts/up.sh`, `scripts/workstation-up.sh`,
`lekiwi-stack.service`) uses them automatically; an explicit launch argument overrides
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
and the standard compute installer publishes its private scan, with the robot's own
body masked out, as the sole `/scan` publisher. No LD06-specific installation or launch flag is needed.

Its 12 m range makes the camera trick redundant, which is why the two are
mutually exclusive. `laser_source:=none` is rejected in real mode: production
navigation must retain either camera or LD06 obstacle sensing. It is useful
only for non-motion diagnosis where the real driver is not launched.

The rest of this section walks through bringing up the real robot and the
calibration each piece depends on.

## 1. Start the LeRobot host

The computer attached to the motors and cameras needs LeRobot with the `lekiwi` extra. If it is a separate Raspberry Pi, copy this repository there and install the Pi side:

```bash
./scripts/install-pi.sh
```

It needs a 64-bit image with Python 3.12+. Use Ubuntu 24.04 when the Pi publishes cameras to this Jazzy workstation; Raspberry Pi OS Trixie can run the LeRobot host only. Then, on the Pi:

```bash
scripts/pi-up.sh
```

Keep the host watchdog enabled. Ports `5555/tcp`, `5556/tcp`, and `5557/tcp`
bind all interfaces by default (see [Network exposure](../README.md#network-security)) and
must be reachable from the ROS computer only; do not expose them to an untrusted
network. Check the repository health handshake on the Pi before starting ROS:

```bash
"$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/host-health-check.py \
  --host 127.0.0.1
```

## 2. Calibrate the front camera

Calibration is required for the camera-scan fallback and for any calibrated use of
the front camera. Use the exact camera resolution, lens focus, and mounting that will be used in operation. The example target is an 8-by-6 inner-corner checkerboard with 25 mm squares.

Run the calibration on the machine the camera is plugged into (the robot computer
in a split deployment; it needs that machine's camera publisher stopped first). It
runs the checkerboard calibrator and saves the result:

```bash
scripts/calibrate.sh camera
```

The saved file is:

```text
~/.ros/camera_info/lekiwi_front.yaml
```

Verify it before moving on:

```bash
ros2 topic hz /camera/front/image_raw
ros2 topic echo /camera/front/camera_info --once
```

The camera matrix `k` must not be all zeroes.

## 3. Calibrate wheel odometry

Place LeKiwi at the RMF charger pose `[-4.0, -2.5, 0.0]`. Measure commanded versus actual straight-line travel and rotation, then tune these driver parameters if needed (`scripts/calibrate.sh wheels` shows the measurement procedure and saves the results):

```text
xy_velocity_scale
yaw_velocity_scale
```

Accurate wheel scale matters: RTAB-Map's ICP starts from wheel odometry, and Nav2 tracks it.

## 4. Build the map

Use a new database filename for the first mapping run. Start the workstation side
(the robot address comes from `LEKIWI_ROBOT_HOST`; on a wired robot use
`scripts/up.sh` with the same arguments):

```bash
scripts/workstation-up.sh localization:=visual_slam slam_mode:=mapping \
  rtabmap_database:=$HOME/.ros/lekiwi_cleanroom.db
```

For an authenticated ZMQ link add `curve_client_secret_key_file:=/secure/path/driver.key_secret`
and `curve_server_public_key_file:=/secure/path/server.key`; the boot service takes
them from `install-compute-services.sh --curve-dir`.

This is the mapping procedure, not a safety bypass: with
`LEKIWI_DISARM_ON_FAILURE=true` the production profile still denies motion until
its required health inputs and physical acceptance record are installed. Do not
weaken the supervisor just to map; use a reviewed mapping configuration and retain
the hardwired E-stop.

Drive slowly around the complete route and return to previously visited areas so RTAB-Map can close loops. RTAB-Map maps from the merged LD06 scan and Astra cloud alone, using ICP scan matching and proximity loop closure, so a single sensor dropout does not stop mapping and lighting does not matter. Long featureless corridors and repetitive cleanroom walls are the weak case for ICP.

Stop with `scripts/ros-stop.sh`; RTAB-Map persists the database at the configured path.

## 5. Operate from the saved map

Start from the charger pose and switch RTAB-Map to localization mode:

```bash
scripts/workstation-up.sh localization:=visual_slam slam_mode:=localization \
  rtabmap_database:=$HOME/.ros/lekiwi_cleanroom.db
```

Then dispatch Nav2 goals. RMF tasks additionally need `localization:=amcl` and an approved map bundle (see [Immutable map bundles](../README.md#immutable-map-bundles)).
