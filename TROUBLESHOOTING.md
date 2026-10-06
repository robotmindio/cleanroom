# Troubleshooting

## Restarting the stack

Restart through the workflow that started it; a raw `kill` is undone by systemd or
leaves orphaned nodes (see the first symptom below).

| How it runs | Restart |
| --- | --- |
| Boot services (split or all-in-one robot) | `sudo systemctl restart lekiwi-stack.service` for the ROS stack; `lekiwi-host.service`, `lekiwi-cameras.service`, `lekiwi-astra.service`, `lekiwi-lidar.service` and `lekiwi-zenoh.service` restart the device side |
| Split robot by hand | `scripts/ros-stop.sh`, then `scripts/workstation-up.sh` on the workstation and `scripts/pi-up.sh` on the robot computer |
| Wired robot | `scripts/ros-stop.sh`, then `scripts/up.sh` |
| New revision on an installed split robot | `scripts/deploy-split.sh` |

`scripts/ros-stop.sh` never stops a systemd-owned unit; it reports each one it left
running.

## Arm model moves opposite to the robot

The arm's zero-pose capture and its joint directions are separate calibrations. Do
not change directions merely because the model looks wrong while the arm is at the
captured zero pose: a direction sign has no effect at zero. In that case, recapture
the SO-101 new-calibration zero pose (see [urdf/README.md](urdf/README.md)) with
`scripts/calibrate.sh pose` instead.

To validate a direction after a successful zero-pose capture:

1. Keep the area around the arm clear and support it if needed.
2. In RViz's **MotionPlanning** panel, select planning group **arm**, open the
   **Joints** tab, and set **Start State** to **Current**.
3. Preview exactly one joint by `+0.10` rad (about 5.7 degrees) from its displayed
   current value. Do not change any other joint.
4. Click **Plan** and confirm the green preview makes the intended small movement.
   Then execute it, or use `scripts/arm-jog.sh shoulder_pan +0.1` (substitute the
   tested joint). The jog tool requires confirmation and moves only that one joint.
5. If that one physical link moves opposite to the preview, set only that joint's
   value in `directions` to `-1` in `~/.ros/lekiwi_arm_calibration.json`; leave its
   `zero_positions` value unchanged. Restart the driver (see
   [Restarting the stack](#restarting-the-stack)) and repeat the same one-joint test.

Test joints one at a time. A `-0.10` rad target is the corresponding opposite
direction test.

## Navigation goal does not move the base

Do not send a goal until the robot is on a clear, level floor and an operator is
present. Navigation needs all three live inputs: `/scan`, `/map`, and Nav2.

```bash
ros2 topic info /scan
ros2 topic info /map
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

`/scan` must have one publisher and ranges beyond `0.11 m`; a scan in which every
beam is `0.11` is the camera-scan fail-safe and Nav2 will deliberately not move.
All three lifecycle nodes must report `active [3]`. Use a goal inside the visible,
mapped free space—not beyond an unknown map boundary. A fresh RTAB-Map map grows
only after the robot has observed the area, so its first goal should be nearby.

If RTAB-Map reports `Did not receive data since 5 seconds`, `/slam/cloud` is
silent. It is published while either the LD06 scan or the Astra cloud is alive, so
check `ros2 topic hz /scan /camera/depth/points`, fix the sensor service that
stopped (`systemctl status lekiwi-lidar.service lekiwi-astra.service`), and restart
the stack. Startup waits for `/slam/cloud` before it starts RTAB-Map. Old RTAB
databases are archived for 14 days and then automatically pruned at startup.

## Wrist or front camera disconnects while the arm moves

`Error dequeueing buffer: No such device (19)` means the USB camera reset or
lost electrical contact. The stack and `scripts/calibrate.sh camera|wrist`
supervise each V4L2 camera: they stop the dead node, wait for its stable
`/dev/v4l/by-id/...` path to return, and reopen it automatically. Keep the
calibration window open; it resumes receiving frames after reconnection.

If the by-id device does not reappear, software cannot restore power or a loose
cable. Stop moving the arm, reseat the wrist-camera cable and strain relief, then
wait for the device to return. Repeated resets at one arm position are a hardware
fault (cable flex, connector, hub, or power), not a ROS calibration problem.

The wrist camera also drops off the bus with the arm stationary: the kernel logs a
USB disconnect and re-enumeration, capture reports ENODEV, and the supervisor
restarts it about five seconds later. This is an open hardware fault (see
[DEFERRED.md](DEFERRED.md#arm-calibration-collision-model-and-physical-execution));
tightening the connector did not fix it. Check `journalctl -k` on the device for
the disconnect and `vcgencmd get_throttled` for power events, then substitute the
cable on the same port before changing the port or camera.

## `/dev/ttyACM0` does not appear

```bash
dmesg | tail -20
udevadm info -q property -n /dev/ttyACM0 | grep ID_VENDOR
```

Expect `1a86` (QinHeng). If `brltty` grabs the port — a known Ubuntu conflict
with CH34x adapters — remove it: `sudo apt-get remove brltty`.

## Permission denied on the motor port

`id -nG` must list `dialout`. Group changes need a fresh login, not just a new
terminal.

## `Incorrect status packet!` during motor reads

Feetech buses corrupt status packets when several joints move at once.
`LeKiwiConfig.num_read_retries` defaults to 2; raise it with
`--robot.num_read_retries=5`. Persistent failures usually mean a daisy-chain
cable or under-supplied bus voltage.

## LeRobot client connects but no images arrive

Host and client must agree on camera keys. The host publishes whatever is in its
`cameras` dict; a client expecting `wrist` when the host only serves `front` sees
no wrist frames. Pass the same `--robot.cameras` override to both.

## Host reachable but nothing moves

Check all three host endpoints, not just 5555:

```bash
ss -ltn | grep -E '5555|5556|5557'
"$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/host-health-check.py --host 127.0.0.1
```

The health check performs a read-only TCP/5555 connection and a `state`
request on the torque endpoint. It is a better service check than matching a
listening port owned by an unrelated process.

## Host reports every motor as model `777`

`777` is LeRobot's no-response sentinel. If every ID from 1 through 9 is
missing while the expected `/dev/serial/by-id/...USB_Single_Serial...` port still
exists, ROS is not the cause: the serial adapter is visible but the shared servo
bus returned no data. Do not rerun motor calibration.

With the robot powered off, check the battery/power switch, servo-bus power lead,
and the USB-to-servo-controller data cable. Restore power and reseat those two
connections, then restart the host (`sudo systemctl restart lekiwi-host.service`,
or `scripts/pi-up.sh` / `scripts/up.sh` for a manual run). A successful manual host
startup prints `host: up`; the host must come up before any ROS navigation or arm
issue can be diagnosed.

## Pi sensors never reach the compute machine

The device's sensor topics cross the network over one zenoh link (port 7447) that
requires mutual TLS. If `journalctl -u lekiwi-zenoh` on the device says
`missing /etc/lekiwi/zenoh-tls/...`, or the compute bridge logs `received fatal
alert` or `Unable to connect to tls/...`, the two machines do not share a
certificate authority. Run `scripts/reinstall-compute.sh` on the compute
machine, which provisions the identities; for a lost or expired one (certificates
last five years) run `scripts/setup-zenoh-tls.sh --renew DEVICE` instead. Then
restart both bridges with `scripts/deploy-split.sh`. The bridge never
falls back to plaintext.

## Topics go silent after a Wi-Fi change or a USB re-enumeration

Cyclone DDS binds its network interface once, and a serial node keeps a dead handle
after its USB device re-enumerates. Neither makes the process exit, so the service
looked healthy while publishing nothing. Under systemd, the wrappers watch for
this (`scripts/lib/self-heal.sh`): a removed or added IPv4 address on a real
interface, or a replaced lidar serial node, kills the service's main process and
systemd restarts it within about 15 seconds. The journal line to look for is
`self-heal: ... letting systemd restart the service`. Manual runs (`scripts/up.sh`,
`scripts/pi-up.sh`, `scripts/workstation-up.sh`) are not supervised; restart them
after changing networks.

## `ros2 topic list` hangs and never returns

Orphaned nodes from an earlier bringup are still holding DDS participants. `ros2 launch`
shuts its nodes down on SIGINT to the whole process group; killing the launcher alone
leaves around twenty nodes running. Those orphans keep talking to each other, so a new
stack still works while introspection dies silently. Stop a run with:

```bash
scripts/ros-stop.sh
```

## RTAB-Map floods the log with `Not found word N (dict size=M)`

The database holds a visual dictionary that does not match the running configuration,
for example one created while visual loop closure was enabled. Every loop closure is
then rejected with `Not enough features in images (old=0)`. Delete the database or
point `rtabmap_database:=` at a fresh path.

## The machine runs out of memory during a long mapping run

RTAB-Map's working memory lives in RAM. `rtabmap_wm_nodes` (default 300) caps how many
nodes stay resident; the rest move to the database and return when the robot comes back
near them. Lower it on a small machine, raise it where memory allows — a larger working
memory recognises places sooner.

## RTAB-Map database or old crash archives consume disk

The active `~/.ros/lekiwi_rtabmap.db` is never rotated or deleted during startup. Mapping
switches to localization when the configured database-size or session-duration quota is reached;
the saved map stays available, and later launches reopen the same database. Increase the tracked
`rtabmap_mapping_max_bytes` or `rtabmap_mapping_max_seconds` launch defaults if the robot needs a
longer mapping session and the device has enough storage. Old automatic `stale-*` and `corrupt-*`
archives are retained for at most 14 days, three sessions, and 1.5 GiB combined (including SQLite
sidecars). The same archive cleanup runs from repository-managed startup paths. If the stack is
already running, `scripts/rtabmap-db-maintenance.py --prune-only` applies only that archive
retention policy; it never opens or moves the active database.

## ROS logs consume disk

The service installers enable `lekiwi-ros-logrotate.timer`. Every five minutes
`lekiwi-ros-logrotate.service` rotates ROS logs above 100 MiB with `copytruncate` and keeps
twelve compressed archives. The same run deletes log files untouched for four days (unless a process still
holds them open) and launch directories left empty, so finished launches do not accumulate under `~/.ros/log`.

## Installer reports a ParaView/VTK conflict

Ubuntu's `python3-paraview` conflicts with the `python3-vtk9` package required by RTAB-Map through PCL. If you do not need the existing ParaView installation, remove it and rerun the installer:

```bash
sudo apt-get remove paraview python3-paraview
./scripts/install.sh
```

## Startup refuses a missing camera calibration

With a local camera, `scripts/ros-start.sh` (and so `scripts/up.sh`) stops when the
front-camera calibration is missing or empty, and the device's `scripts/ros-cameras.sh`
waits for one. Run `scripts/calibrate.sh camera` on the machine the camera is plugged
into, or pass the correct `camera_info_url`.

## RTAB-Map receives no data

RTAB-Map consumes only `/slam/cloud` (merged LD06 scan and Astra points) and `/odom`.
Check all three inputs:

```bash
ros2 topic hz /slam/cloud
ros2 topic hz /scan
ros2 topic hz /odom
```

Also verify the LD06 transform:

```bash
ros2 run tf2_ros tf2_echo base_footprint laser
```

## pytest fails with `PluginValidationError`

A newer `pytest` in `~/.local` shadows the one ROS's `launch_testing` plugins
expect. Disable plugin autoloading for the unit tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test -q
```

## Free Fleet does not discover `lekiwi_1`

Confirm that the bridge is running and Nav2 provides the action server:

```bash
pgrep -af zenoh-bridge-ros2dds
ros2 action list | grep navigate_to_pose
ROS_DOMAIN_ID=0 ros2 node list
```

## A rosbridge client cannot connect

Confirm that rosbridge is listening and uses the intended ROS domain:

```bash
ros2 node list | grep -E 'rosapi|rosbridge'
ss -ltn | grep ':9090'
```

Remote clients require an explicitly permitted firewall rule for TCP 9090 and
an authenticated proxy; a raw unauthenticated WebSocket is not supported for
robot control.

## The base stops intermittently with nothing nearby

Chassis self-returns in the LD06 scan that fall outside the body mask reach
CollisionMonitor's StopZone and stop the base. Record stationary raw scans with
the arm in `travel_stow`, find returns that lie inside the footprint, and widen
the tracked mask sector only within the footprint; the masked sectors must stay
inside it so exterior obstacles are still seen. See
[Obstacle coverage](docs/safety.md#obstacle-coverage).

## Localization does not recover in a dark room

Visual relocalization needs image features. In a dark room the front and Astra
images have mean brightness near zero and no ORB features, so RTAB-Map cannot
accept a global closure and Nav2 checks wait for localization, although depth
and lidar are still live. Restore lighting; do not lower the matching
thresholds. A small orientation search toward a previously mapped view then
recovers the saved map.

## Localization jumps or closes false loops

Recalibrate wheel scale and check the LD06 mount offsets first, then remap with slower
motion. Long uniform walls leave ICP unconstrained along the wall, and repetitive
cleanroom geometry can produce false proximity or ICP matches.
