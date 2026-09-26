# Launch options

Part of the [LeKiwi README](../README.md).

Arguments of `launch/bringup.launch.py`. The repository scripts pass extra
`name:=value` arguments straight through (`scripts/sim-up.sh`, `scripts/up.sh`,
`scripts/workstation-up.sh`, `scripts/ros-start.sh`).

| Argument | Values | Default | Purpose |
| --- | --- | --- | --- |
| `mode` | `sim`, `real` | `sim` | Select Gazebo or the LeRobot hardware bridge |
| `headless` | `true`, `false` | `true` | Run Gazebo server-only with offscreen rendering; set false to open its GUI |
| `localization` | `visual_slam`, `amcl` | `visual_slam` | Select the sole `map -> odom` provider; `visual_slam` is RTAB-Map running lidar-only |
| `slam_mode` | `mapping`, `localization` | `mapping` | Extend or reuse the RTAB-Map database; the session quota switches mapping to localization |
| `remote_ip` | IPv4/hostname | `127.0.0.1` | Address of the LeKiwi ZMQ host and of the device zenoh bridge (`scripts/ros-start.sh` takes it from `LEKIWI_ROBOT_HOST`) |
| `curve_client_secret_key_file`, `curve_server_public_key_file` | file paths | empty | Optional CURVE identity for the ZMQ link; set both or neither |
| `disarm_on_failure` | `true`, `false` | `false` | Strict failure policy for larger robots: every failure disarms, cuts torque and waits for `/safety/arm`; also makes the safety supervisor deny motion. See [Arming and recovery](#arming-and-recovery). Scripts set it from `LEKIWI_DISARM_ON_FAILURE` |
| `rtabmap_database` | file path | sim: `~/.ros/lekiwi_rtabmap_sim.db`; real: `~/.ros/lekiwi_rtabmap.db` | RTAB-Map map database |
| `rtabmap_wm_nodes` | integer | `300` | Nodes kept in RTAB-Map working memory before older ones move to the database |
| `rtabmap_mapping_max_bytes`, `rtabmap_mapping_max_seconds` | integers | `536870912`, `14400` | Mapping-session quota; reaching either switches RTAB-Map to localization |
| `static_map` | `true`, `false` | `false` | Serve the checked-in floor plan on `/map` and move RTAB-Map's own grid to `/rtabmap/map` |
| `map_bundle` | YAML path | `maps/bundles/cleanroom-development.yaml` | Immutable map bundle used for the map, graph and fleet config when `amcl`, `static_map` or RMF is selected |
| `publish_camera` | `true`, `false` | `true` | Enable the front-camera pipeline (local V4L2 or the remote relay) |
| `publish_astra` | `true`, `false` | `true` | Start the Astra Pro driver on this machine (real mode, `camera_source:=local`); with `camera_source:=remote` the device's `lekiwi-astra.service` supplies it |
| `hardware_config` | YAML path | `config/hardware.yaml` | Tracked hardware identities, including the required Astra serial when the Astra is launched |
| `camera_info_url` | ROS camera URL | `file://~/.ros/camera_info/lekiwi_front.yaml` | V4L2 front-camera calibration (not used by Astra Pro) |
| `wrist_camera_info_url` | ROS camera URL | `file://~/.ros/camera_info/lekiwi_wrist.yaml` | Optional wrist-camera calibration |
| `camera_source` | `local`, `remote` | `local` | Read the camera here, or decompress what the robot's Pi publishes |
| `camera_device` | V4L2 path | `/dev/video0` | Existing front V4L2 camera |
| `wrist_camera_device` | V4L2 path, `none` | `none` | Wrist camera; `scripts/ros-start.sh` passes the detected JYU2C, or `none` when `LEKIWI_WRIST=none` |
| `laser_source` | `auto`, `camera`, `ld06`, `none` | `auto` | Select camera fallback or LD06 on real hardware; Gazebo supplies `/scan` in sim |
| `lidar_source` | `local`, `remote` | `local` | Machine that opens the LD06 serial port; remote reads `/pi/lidar/scan` |
| `lidar_port` | serial path | CP2102 `/dev/serial/by-id/...` | LD06 device when `laser_source:=ld06` |
| `urdf/lekiwi.urdf.xacro` sensor-calibration properties | metres, radians | Astra `0 0 0.0155`, `-8°`; others `0.0` | Astra's CAD compact-mount contact pose plus measured wrist and LD06 corrections, shared by RViz, MoveIt, and robot_state_publisher |
| `camera_height`, `camera_offset_x`, `camera_offset_y` | metres | `0.093`, `0.03`, `0.0` | Front-camera pose used by the camera scan |
| `camera_pitch`, `camera_yaw`, `camera_roll` | radians | `0.031`, `0.0`, `0.0` | Front-camera orientation used by the camera scan |
| `xy_velocity_scale` | float | `1.0` | Correction for reported and commanded translation |
| `yaw_velocity_scale` | float | `0.90` | Correction for reported and commanded rotation |
| `start_rmf` | `true`, `false` | `false` | Start Zenoh, RMF schedule, dispatcher, and fleet adapter; requires `localization:=amcl`, `slam_mode:=localization` and an approved `map_bundle` |
| `rmf_domain` | integer | `0` | DDS domain used by RMF processes; validation currently requires `0` because no tracked cross-domain bridge is configured |
| `start_foxglove` | `true`, `false` | `true` | Start the read-only Foxglove WebSocket bridge |
| `foxglove_address` | bind address | `127.0.0.1` | Interface exposed by Foxglove; loopback by default |
| `foxglove_port` | TCP port | `8765` | Foxglove WebSocket listening port |
| `start_rosbridge` | `true`, `false` | `false` | Start rosbridge WebSocket and ROS API nodes |
| `start_moveit` | `true`, `false` | `false` | Start MoveIt arm planning and execution against the real or simulated action server |
| `rosbridge_address` | bind address | `127.0.0.1` | Interface exposed by rosbridge; keep loopback unless protected separately |
| `rosbridge_port` | TCP port | `9090` | WebSocket listening port |
| `rosbridge_domain` | integer | `0` | ROS graph exposed through rosbridge |

Only one localization mode should run. `visual_slam` (the value name is kept) publishes `map -> odom` through RTAB-Map; `amcl` publishes it from the fixed occupancy map.

## Arm planning

Start MoveIt with either robot mode. `bringup.launch.py` does not start a
desktop session; `scripts/workstation-up.sh` starts RViz after the stack, and
`scripts/rviz.sh` opens it on any running stack. The following
starts the real arm action server on a wired robot, but does not move it
(`scripts/workstation-up.sh` and the compute service already enable MoveIt):

```bash
scripts/up.sh start_moveit:=true
```

MoveIt executes through `/arm_controller/follow_joint_trajectory` in both
modes. In simulation, Gazebo's six-joint physics controller supplies actual
joint feedback and the adapter enforces the same trajectory limits and
tolerances as the hardware boundary. To exercise it, run
`scripts/sim-up.sh start_moveit:=true`; the delayed depth cloud feeds MoveIt's octomap.
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


## Arm pose calibration

With the updated stack running, run `scripts/calibrate.sh pose` on the compute
machine that owns the ROS driver. It captures fresh `/arm/raw_joint_states`,
independent of the driver's existing offsets, and backs up the previous calibration.
Support the disarmed arm in the **SO-101 new-calibration zero pose** shown in
[the model reference](../urdf/README.md). Folded-pose instructions from other SO-101 guides do not
apply to this model. Capture saves `~/.ros/lekiwi_arm_calibration.json` without
changing torque or restarting services. Restart the driver through the same
repository launch/deploy workflow to apply it, then verify individual joint
directions and several poses before executing a trajectory. If a joint moves
opposite in RViz, correct that joint's `directions` value and recapture the zero
mapping as needed. Redo motor calibration first if encoder readings wrap or
disagree with the measured travel range.

## Arming and recovery

Arming requires complete, fresh telemetry and current permission from the
continuous safety supervisor. The motor host runs continuously; a clean service
stop or restart disconnects it and cuts servo torque, and a restarted driver
arms itself again once telemetry and permission are healthy.

By default (`disarm_on_failure` off, the domestic robot) the robot stays armed:
a host session change, stale or failed telemetry, or withdrawn permission
cancels the interrupted trajectory, stops the base and freezes the arm at its
present position with servo torque on, and the driver re-arms itself every 2 s
until telemetry and permission are healthy again. Ordinary failures do not latch
`TORQUE_FAULT`. An operator's `/safety/disarm` cuts torque; if any servo does not
confirm torque-off, the service fails and `TORQUE_FAULT` latches even in this
mode. A confirmed disarm clears that fault. The robot stays disarmed across
later link losses until an operator calls `/safety/arm`. The ZMQ command,
observation and torque sockets use heartbeat and TCP keepalive, so a half-open
link is detected and reconnected instead of hanging.

For larger robots, set `LEKIWI_DISARM_ON_FAILURE=true` in `.env` on both the
workstation and the robot computer (launch argument `disarm_on_failure:=true`, host
option `--safety.disarm_on_failure=true`). This strict mode disarms on every
failure, cuts all servo torque, latches `TORQUE_FAULT` if the cut is unconfirmed,
never re-arms by itself, and stays disarmed until you inspect the robot and:

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
```

The driver publishes its `DISARMED`, `ARMED`, or `LINK_LOST` state on
`safety/driver_state` (the driver's own `safety/state`, remapped by bringup).
The supervisor publishes `safety/supervisor_state`,
`safety/base_motion_permitted`, and `safety/arm_motion_permitted`. By default (real
mode, domestic robot) it reports missing or unhealthy inputs in `/diagnostics` but
does not withhold motion. With `LEKIWI_DISARM_ON_FAILURE=true`, and always in
simulation, missing or unhealthy inputs deny motion and runtime faults latch until
`/safety/reset_fault` is called while the hardware driver is disarmed and all
required inputs are healthy. Simulation has no hardware driver; it accepts the
same explicit reset only after its required inputs recover. An e-stop always
latches in that mode. `/safety/disarm` stops ROS commands
and waits for the motor host to confirm that it cut torque on all nine servos.
Arming, manual or automatic, holds each arm joint at its measured position and
sends zero wheel velocity. The physical E-stop remains mandatory for any
electrical, mechanical, or process failure.

## Production safety prerequisites

Real mode loads `config/safety_production.yaml`. With `LEKIWI_DISARM_ON_FAILURE=true`
(larger robots) it denies base and arm motion until the required inputs below are
current, stamped and healthy. By default the supervisor only reports them. The
shipped robot has no IMU, bumper or battery monitor, and its E-stop cuts motor
power outside the electronics, so those four inputs are not required
(`require_imu`, `require_bumper`, `require_battery`, `require_estop` are false)
until hardware publishes them:

| Input | Topic | Purpose |
| --- | --- | --- |
| Driver state | `safety/driver_state` | Motor-link and torque state |
| Full scan | `/scan` | Obstacle coverage and freshness |
| Depth | `/camera/depth/points` | Arm-workspace obstacles |
| Odometry | `/odom` | Base state |
| IMU (not required) | `/imu/data` | Base dynamics |
| Joint state | `/joint_states` | Arm feedback and stow interlock |
| Bumper (not required) | `safety/bumper_active` | Contact stop |
| E-stop (not required) | `safety/estop_active` | Independent emergency stop state |
| Battery (not required) | `/battery_state` | Voltage and charge limits |
| Motor health | `/hardware/diagnostics` | Servo/bus faults |
| Arm collision gate | `/safety/arm_workspace_clear` | Live MoveIt scene/state validity |

The repository ships `config/safety_acceptance.yaml` with `validated: false`.
It is an acceptance template, not proof of safety. A qualified hardware
procedure must record all-direction stopping trials, fault responses, software
revision, sensor configuration, and the measured stow pose before setting it
true. No physical stopping, E-stop, depth, or full production sensor acceptance
is implied by a passing software test.

## Where the camera comes from

The local real robot has three cameras: the existing front V4L2 camera, the
existing wrist V4L2 camera, and an ORBBEC Astra Pro. The Astra's pinned
OpenNI/UVC driver publishes synchronized, depth-registered RGB-D under
`/camera/astra/...`; its filtered cloud is published at `/camera/depth/points`
for MoveIt's octomap updater and for the SLAM cloud (`/slam/cloud`), which merges
each LD06 scan with the Astra points between 3 cm and 1 m above the floor.
RTAB-Map consumes only that cloud plus wheel odometry, never the RGB or depth
images. Set the physical serial in the tracked
`config/hardware.yaml` before enabling Astra: an empty serial is rejected
instead of allowing the driver to claim an arbitrary compatible USB camera.
The full hardware installer also installs the camera's udev rule, so this
works from the managed service without an interactive permission fix.

The driver uses `astra_camera_optical_frame`; keep its physical mount
transform/calibration in `urdf/lekiwi.urdf.xacro` when the Astra mount is
measured, rather than adding a runtime TF. The compact bracket uses the existing
diagonal pair nearest each fingernail in the operator's photo: CAD
(-100, -20) and (-80, -60) mm, 44.721 mm apart. This places the bracket
26.565 degrees counterclockwise from the previous left-facing mount.
The Astra faces left/rear, pitched 8 degrees down; forward (+X) remains
the arm/fixed-camera side. Its optical-centre correction still needs measurement.
Verify the depth cloud overlay in RViz before enabling arm motion.

The repository host is started camera-less for ROS, so a delayed camera frame
cannot take the motor bus down. Direct LeRobot dataset/teleoperation mode may
still be camera-sensitive and should not be used as the ROS motor service.

The front/wrist V4L2 cameras are also the supported remote-camera topology:
frames are read by `v4l2_camera` on the machine where they are plugged in, then
relayed as below. The Astra is read by `ros-astra.sh` (`lekiwi-astra.service`)
on whichever machine holds its USB connection; on a split robot the bridge
carries a decimated depth cloud, LD06 scan, front preview at up to 3 Hz, wrist
preview at up to 2 Hz, and a JPEG Astra colour preview at up to 0.25 Hz. Raw
Astra colour frames stay on the device.

With a Pi on the robot, `ros-cameras.sh` reads each USB camera there and publishes a
compressed `/pi/camera/...` stream. The device zenoh bridge carries it to the
workstation, which expands it onto the canonical `/camera/...` topics when
`camera_source:=remote`; camera frames never pass through the LeRobot motor host.

On a wired robot, use the default `camera_source:=local`; the existing
`camera_device` continues to identify the front V4L2 camera.

The known JYU2C wrist camera is auto-detected unless `LEKIWI_WRIST=none`. It publishes on `/camera/wrist/image_raw` for watching the gripper. Run `scripts/calibrate.sh wrist` **on the machine the wrist camera is plugged into** before using its `camera_info` for calibrated perception; the camera-scan fallback uses only the front camera. In remote mode the camera node publishes its compressed wrist stream to the workstation. Both cameras share one USB 2.0 hub, so the wrist feed stays small.

## Odometry scale

LeRobot's kinematics assume a wheel 12.5 cm from the centre of rotation. Measure your robot — wheel centre to wheel centre, divided by √3 — and set `yaw_velocity_scale` to `0.125 / that`. Wheels 24 cm apart give 0.90, the default here. The factor corrects both what the base reports and what it executes, so a rotation Nav2 asks for is the rotation it gets.

Check translation against a printed checkerboard, which needs no measuring tools beyond the board itself:

```bash
ros2 run lekiwi_rmf odom_scale.py --axis linear
```

It drives a short leg and compares the distance the calibrated camera sees against the distance odometry claims. Rotation is better derived from the wheel measurement above: estimating orientation from a flat target viewed head-on is unreliable at small angles.
