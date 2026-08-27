# LeKiwi hardware bring-up (LeRobot only, no ROS)

Get the robot moving under plain LeRobot before any of the ROS stack is involved.
If teleoperation works here, every later failure is a ROS problem, not a wiring,
motor-ID, or calibration problem.

The main [README](README.md) picks up from the end of this document.

## Which machine runs what

| Machine | Runs | Installer |
| --- | --- | --- |
| Robot's Raspberry Pi | LeRobot host (Feetech bus, ZMQ server); optional ROS camera service | `scripts/install-pi.sh` |
| Workstation | ROS 2, Nav2, RTAB-Map, Open-RMF, and the LeRobot *client* | `scripts/install.sh` |

Motor commands travel over LeRobot ZMQ `5555/tcp`; observations and joint state
travel over `5556/tcp`. The repository-owned torque safety endpoint is
`5557/tcp`. These are control-plane interfaces: keep them loopback-bound unless
the authenticated ZMQ deployment and firewall policy are configured. CURVE
protects only the ZMQ sockets; it does not secure ROS 2 DDS, which remains a
separate exposure unless the deployment isolates it or enables DDS security.

In ROS operation the motor host is started camera-less (`--no-cameras`). A
separate `v4l2_camera` service owns each USB camera and publishes the front and
wrist streams; camera frames do not pass through the motor host. This isolates
camera stalls from actuator control.

The Pi normally runs the LeRobot host. The full stack belongs on the workstation. A Pi 5
can also run `scripts/install.sh` for occasional self-contained debugging, but Nav2 and
especially RTAB-Map are memory-constrained there.

For the **wired** LeKiwi variant there is no Pi: run both installers on the
workstation, use `remote_ip:=127.0.0.1`, and leave `camera_source:=local`.

## Choosing the Pi image

Use a **64-bit** image. LeRobot 0.6.1 declares `requires-python >=3.12`, which
decides this:

| Image | Python | Verdict |
| --- | --- | --- |
| Ubuntu Server 24.04 LTS (arm64) | 3.12 | **Required for a Jazzy workstation.** Has ROS 2 Jazzy packages |
| Raspberry Pi OS **Trixie** (64-bit) | 3.13 | LeRobot host works; no ROS packages exist, so no cameras in ROS |
| Raspberry Pi OS **Bookworm** | 3.11 | **Will not work** — below LeRobot's floor |
| Any 32-bit image | — | **Will not work** — no aarch64 PyTorch wheels |

Match the workstation: a Jazzy workstation needs a noble (24.04) Pi — ROS 2 does not
guarantee cross-distro wire compatibility. ROS 2 publishes binaries for noble/Jazzy, and the
Pi needs ROS to publish its cameras. On any other image `scripts/install-pi.sh` installs the LeRobot
host and says it skipped ROS.

Raspberry Pi OS Lite is enough; the host needs no desktop. Trixie is the newer
Raspberry Pi OS series, rebased on Debian 13, and moved system Python from
Bookworm's 3.11 to 3.13 — if you are upgrading an existing card rather than
flashing fresh, rebuild any virtualenv instead of copying it, since the old one
points at the 3.11 interpreter.

PyTorch comes in as a LeRobot base dependency, and its aarch64 wheel is the
memory-hungry step. Use a Pi 4 or Pi 5 with **4 GB or more**; on a 2 GB board the
`pip install` is what runs out of memory.

## Install on the Pi

Flash the image, enable SSH, then from the Pi:

```bash
git clone <this-repo> ~/cleanroom
~/cleanroom/scripts/install-pi.sh
```

It verifies the architecture and Python version up front, installs the system
prerequisites, removes `brltty` if present (it claims CH34x adapters and steals
the motor bus), adds you to `dialout` and `video`, builds the venv, installs
`lerobot[lekiwi,hardware]==0.6.1`, and clones the `v0.6.1` examples. It prints the
Pi's IP address at the end — that is the `remote_ip` the workstation needs.

Group membership only takes effect after a fresh login, so log out and back in
before running the motor commands below.

Then continue from [§1 Find the motor bus port](#1-find-the-motor-bus-port),
running those commands **on the Pi**. Camera calibration also runs on the machine
that physically owns each camera (normally the Pi); mapping runs on the workstation.

## What the installer already gives you

Both installers install the same `lerobot[lekiwi,hardware]==0.6.1`, into different
places:

| Machine | Venv | Activate with |
| --- | --- | --- |
| Pi (`install-pi.sh`) | `~/lerobot-venv` | `source ~/lerobot-venv/bin/activate` |
| Workstation (`install.sh`) | `$LEKIWI_WS/.venv-lerobot` | `source $LEKIWI_WS/.venv-lerobot/bin/activate` |

What the extras buy you:

| Extra | Brings | Needed for |
| --- | --- | --- |
| `lekiwi` | `feetech-servo-sdk`, `pyserial`, `pyzmq`, `deepdiff` | Motor bus, ZMQ host/client |
| `hardware` | `pynput` | Keyboard driving of the base |
| base | `opencv-python-headless`, `torch` | Cameras, observation tensors |

### Two environments on the workstation, on purpose

This split only exists on the workstation; the Pi has no ROS, so `~/lerobot-venv`
is the only environment there.

LeRobot requires `numpy>=2`; ROS 2's compiled extensions are built against the
system's numpy — 1.26 on Jazzy/Ubuntu 24.04. That mismatch does not merely warn —
`rmf_adapter` segfaults mid-run — so the workstation keeps two virtualenvs:

| Venv | numpy | Holds | Used by |
| --- | --- | --- | --- |
| `.venv` | 1.26 | zenoh, pycdr2, nudged, rosbags, transforms3d | Everything ROS; `scripts/setup.bash` activates it |
| `.venv-lerobot` | 2.2 | lerobot + feetech/pyzmq | LeRobot CLIs and the hardware driver only |

**For every command in this document, activate the LeRobot venv — never
`scripts/setup.bash`, which activates the ROS one and has no `lerobot` in it.**

`bringup.launch.py` handles this itself for `mode:=real`: it puts
`.venv-lerobot/bin` on the driver node's PATH, so the driver runs against numpy 2
while every other node keeps numpy 1.26.

Two extras are deliberately **not** installed, because they cost hundreds of
megabytes and only matter for dataset work:

```bash
pip install 'lerobot[lekiwi,core-scripts]==0.6.1'   # with the LeRobot venv activated
```

That adds `datasets`, `pandas`, `pyarrow`, `torchcodec` (recording) and
`rerun-sdk`, `foxglove-sdk` (`--display_data=true` visualisation).

## Get the example scripts

LeKiwi teleoperation and recording are **not** console entry points. The v0.6.1
docs run `examples/lekiwi/teleoperate.py`, but LeRobot's `pyproject.toml` packages
only `src/`, so `pip install lerobot` gives you no `examples/` directory. Clone the
matching tag:

```bash
git clone -b v0.6.1 --depth 1 --filter=blob:none \
  https://github.com/huggingface/lerobot ~/lerobot-src
```

`scripts/install-pi.sh` already does this clone for you. It is deliberately not
part of `scripts/install.sh` — the ROS stack never uses these scripts.

## Device access on Linux

The Feetech bus board is a QinHeng CH343 (`1a86:55d3`) and appears as
`/dev/ttyACM0`, owned `root:dialout`. Add yourself to `dialout` once and log out
and back in:

```bash
sudo usermod -aG dialout $USER
```

Prefer that over the `sudo chmod 666 /dev/ttyACM0` in the upstream docs, which
does not survive a replug. Cameras (`/dev/video*`) are `root:video` but carry a
systemd-logind ACL for the active desktop user, so a desktop session needs no
group change. A headless Pi does:

```bash
sudo usermod -aG video $USER
```

## 1. Find the motor bus port

```bash
lerobot-find-port
```

Unplug the board when prompted so the script can identify which port disappeared.

## 2. Set the motor IDs

Every servo ships as ID 1, so they must be assigned one at a time, in the order
the tool asks for: arm IDs 6→1, then wheels 9, 8, 7. Connect **one motor at a
time** when prompted.

```bash
lerobot-setup-motors --robot.type=lekiwi --robot.port=/dev/ttyACM0
```

LeKiwi uses a single motor control board for both the arm and the three wheels.
Wheel positions map to IDs 7, 8, 9 — see the
[LeKiwi assembly guide](https://github.com/SIGRobotics-UIUC/LeKiwi/blob/main/Assembly.md)
for which wheel is which.

## 3. Calibrate

Only the arms need calibration; the wheels do not.

Run `scripts/robot-host.sh` with no arguments. On its first run for an ID it
automatically starts calibration, then starts the complete host when calibration
finishes. To force calibration again, run:

```bash
scripts/robot-host.sh calibrate
```

Calibration wraps `lerobot-calibrate` with `--robot.cameras='{}'`. Calibration
only talks to the motor bus, but `LeKiwiConfig` still opens both cameras on
connect, so a missing or misnumbered camera aborts it before the first prompt.

Move every joint to the middle of its range, press Enter, then sweep each joint
through its full range. Use `lekiwi_1` as the ID — that is the default the ROS
driver and the Free Fleet adapter expect.

If you have a leader arm for teleoperation, calibrate it separately on the
machine it is plugged into:

```bash
lerobot-calibrate --teleop.type=so100_leader \
  --teleop.port=/dev/ttyACM1 --teleop.id=leader_1
```

## 4. Check the cameras

```bash
lerobot-find-cameras
```

`LeKiwiConfig` defaults to `front=/dev/video0` and `wrist=/dev/video2`. If you
have only one camera, or different indices, the host fails to open the missing
device — override the whole dict rather than fighting the default:

```bash
--robot.cameras="{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}"
```

Note that `/dev/video1` is usually the metadata node of the same UVC device as
`/dev/video0`, not a second camera.

On a laptop, `/dev/video0` is almost always the built-in webcam, so the stock
defaults point `front` at your screen and shift both robot cameras up by one.
Identify each device by model before trusting the numbering:

```bash
udevadm info -q property -n /dev/video2 | grep ID_MODEL=
```

`scripts/robot-host.sh` carries the resulting paths; set `LEKIWI_FRONT` and
`LEKIWI_WRIST` if yours differ from `/dev/video2` and `/dev/video4`.

## 5. Run the host

On the machine physically wired to the motors — the Pi on the robot, or your
laptop for the wired LeKiwi variant:

```bash
scripts/robot-host.sh
```

Defaults: command socket `5555/tcp`, observations `5556/tcp`, torque safety
`5557/tcp`, watchdog 500 ms, loop 30 Hz. The watchdog stops the base when
commands stop arriving. It is not an E-stop. The repository host starts
torque-off and only changes servo torque through the separate safety endpoint.

By default the direct host auto-detects both known cameras; set `LEKIWI_WRIST=none`
to leave the wrist feed out when USB bandwidth is tight. Its ZMQ clients can use
images for teleoperation and dataset recording. Under ROS this mode is an
exclusive alternative: ROS reads cameras through its own nodes on whichever
machine they are plugged into, one reader per device, and a stalled frame must
not be able to abort the motor host. So when ROS runs against this machine,
start the host without cameras (`scripts/robot-host.sh --no-cameras`, or just
use the boot services) and let ROS own the USB devices. The ROS camera
publisher on a remote device machine:

```bash
source scripts/setup-pi.bash
ros2 launch launch/pi_cameras.launch.py \
  front_device:=/dev/v4l/by-id/usb-YOUR_CAMERA-video-index0
```

Find that path with `ls /dev/v4l/by-id/`. Use it rather than `/dev/video0`, which is
reassigned whenever USB re-enumerates.

For the normal two-computer setup, use the device launcher instead. It starts
the camera-less motor host and the ROS camera publisher — v4l2_camera reads
the sensors here, and only compressed frames cross the network to the
workstation:

```bash
scripts/pi-up.sh
```

### Optional boot services

For unattended startup, install the device services on the machine that owns
the serial adapter and USB cameras. Install the compute service where the ROS
workspace runs:

```bash
sudo scripts/install-device-services.sh \
  --service-user "$USER" --workspace "$HOME/lekiwi_ws" \
  --lerobot-venv "$HOME/lekiwi_ws/.venv-lerobot" \
  --bind-address 127.0.0.1
sudo scripts/install-compute-services.sh \
  --service-user "$USER" --workspace "$HOME/lekiwi_ws"
```

For a separate ROS workstation, use `--remote DEVICE_IP` on the compute
installer. A direct root invocation must include `--service-user USER`; when
run through `sudo`, the invoking non-root account is selected. Both installers
fail early if the selected workspace or LeRobot Python is missing. They render,
verify, reload, and enable the units; inspect them with:

```bash
systemctl status lekiwi-host.service lekiwi-cameras.service lekiwi-stack.service
journalctl -u lekiwi-host.service -f
```

The host service runs camera-less and always starts torque-off. A host or ROS
restart is never permission to energize the servos. Motion requires fresh
telemetry, healthy safety inputs, and an explicit `/safety/arm` request.

At the default `jpeg_quality:=50` a 640x480 frame measures about 14 KB, so
30 Hz costs roughly 3 Mbit/s; the same frame at the library default of 95
costs 70–90 KB, or 18 Mbit/s. Raise it if RTAB-Map starts losing loop
closures, lower it if the link is saturated.

## 6. Teleoperate

Set `remote_ip` and `port` at the top of the script, then on the driving machine:

```bash
python ~/lerobot-src/examples/lekiwi/teleoperate.py
```

| Key | Action |
| --- | --- |
| W / S | Forward / backward |
| A / D | Left / right |
| Z / X | Rotate left / right |
| R / F | Speed up / down |
| Q | Quit |

Speed modes are 0.4 / 0.25 / 0.1 m/s with 90 / 60 / 30 deg/s rotation.

This script drives the **arm from a leader arm** and the base from the keyboard.
Without a leader arm built it will fail at connect; drive the base only by
the ROS teleoperation path after the production safety prerequisites are met.
Do not inject `x.vel`, `y.vel`, or `theta.vel` directly into the repository
motor host: it starts torque-off and its guarded ROS control path is the
supported interface for physical motion.

## 7. Record a dataset (optional)

Needs the `core-scripts` extra and a Hugging Face write token:

```bash
hf auth login --token $HUGGINGFACE_TOKEN --add-to-git-credential
python ~/lerobot-src/examples/lekiwi/record.py
```

Adapt `remote_ip`, `repo_id`, `port`, and `task` inside the script. Datasets land
in `~/.cache/huggingface/lerobot/{repo-id}`.

## 8. Mount the LD06 lidar (optional)

The LDROBOT LD06 replaces the camera-as-laser obstacle scan (`laser_source:=ld06`
instead of the default `auto` selection). It mounts partway up the mast: the URDF puts
its `laser` frame 8 cm above the mast's origin -- about 30 cm off the base
plate, level and facing forward -- so any bracket holding it there needs no TF
work, only screws.

It is a USB serial device (typically a CP2102 bridge), so it appears as
`/dev/ttyUSB0`, owned `root:dialout` -- the same group the motor bus needs, and
the installer adds you to it. Prefer its stable name over `/dev/ttyUSB0`, for
the same reason as with cameras:

```bash
ls /dev/serial/by-id/    # e.g. usb-Silicon_Labs_CP2102N_...-if00-port0
```

Nothing else is configurable: the LD06 speaks 230400 baud and the driver is
launched with that fixed. The repository's `laser_source:=auto` already selects
the known CP2102 by-id device when it is attached; force it explicitly while
commissioning so a wrong USB adapter cannot silently select the camera fallback:

```bash
scripts/up.sh laser_source:=ld06 lidar_port:=/dev/serial/by-id/usb-...-if00-port0
```

and check the scan against reality in RViz before trusting it: spin the robot by
hand and watch a nearby wall stay put in the LaserScan display.

## Troubleshooting

### `/dev/ttyACM0` does not appear

```bash
dmesg | tail -20
udevadm info -q property -n /dev/ttyACM0 | grep ID_VENDOR
```

Expect `1a86` (QinHeng). If `brltty` grabs the port — a known Ubuntu conflict
with CH34x adapters — remove it: `sudo apt-get remove brltty`.

### Permission denied on the port

`id -nG` must list `dialout`. Group changes need a fresh login, not just a new
terminal.

### `Incorrect status packet!` during motor reads

Feetech buses corrupt status packets when several joints move at once.
`LeKiwiConfig.num_read_retries` defaults to 2; raise it with
`--robot.num_read_retries=5`. Persistent failures usually mean a daisy-chain
cable or under-supplied bus voltage.

### Client connects but no images arrive

Host and client must agree on camera keys. The host publishes whatever is in its
`cameras` dict; a client expecting `wrist` when the host only serves `front` sees
no wrist frames. Pass the same `--robot.cameras` override to both.

### Host reachable but nothing moves

Check all three host endpoints, not just 5555:

```bash
ss -ltn | grep -E '5555|5556|5557'
"$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/host-health-check.py --host 127.0.0.1
```

The health check performs a read-only TCP/5555 connection and a `state`
request on the torque endpoint. It is a better service check than matching a
listening port owned by an unrelated process.

## Moving on to ROS

Once teleoperation works, leave the host running and start the ROS side against
it. The ROS driver (`lekiwi_rmf/driver.py`) is a pure ZMQ `LeKiwiClient` — it
never touches USB, so the ROS machine needs no serial or camera permissions at
all. A non-loopback host supports unauthenticated ZMQ on a trusted robot LAN.
Use the service installer above without `--curve-dir`, or opt into CURVE by
supplying the client secret and pinned server public key explicitly:

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=real remote_ip:=192.168.1.50 \
  curve_client_secret_key_file:=/secure/path/driver.key_secret \
  curve_server_public_key_file:=/secure/path/server.key
```

Calibrate each camera on the machine it is plugged into with
`scripts/calibrate.sh camera` or `scripts/calibrate.sh wrist`. The workstation
relays both feeds; navigation and RTAB-Map use only the front camera.

## Safety

The ROS production profile is default-deny. It requires current, stamped
feedback for the motor host, full scan, depth point cloud, odometry, IMU, joint
states, battery, motor diagnostics, bumper, and E-stop inputs before granting
base or arm permission. The arm must also be inside the configured stow pose
for base motion. These interfaces are:

```text
safety/driver_state             motor-link and torque state
/scan                           obstacle coverage
/camera/depth/points            arm-workspace obstacles
/odom, /imu/data                base state
/joint_states                   arm feedback and stow interlock
/battery_state                  voltage and charge limits
/hardware/diagnostics           servo and bus health
safety/arm_workspace_clear      live MoveIt scene/state validity
safety/bumper_active            contact stop
safety/estop_active             E-stop state
```

`config/safety_acceptance.yaml` is a schema-version-2 template shipped with
`validated: false`; it is not a claim that the robot has passed physical
stopping tests. A qualified hardware procedure must record reviewed acceptance
limits, at least 30 trials in every translation/rotation direction, independent
E-stop and fault-response tests, the software revision, sensor configuration,
payload, surface, stop latency, and measured worst-case stopping distances plus
measurement uncertainty before enabling it. Required fault tests include
unauthorised ZMQ rejection and DDS/rosbridge isolation or authentication; do
not mark them true merely because a local software launch succeeded. The
accepted footprint and padding must match both tracked Nav2 costmaps, the
collision-monitor obstacle-stop trial must pass, and its StopZone must leave at
least the measured worst stopping distance plus uncertainty around that exact
footprint; the supervisor checks those relationships at every startup.

The driver never auto-arms, and host or ROS restart always leaves the servos
torque-off. An operator must inspect the robot and call `/safety/arm`; after a
fault, call `/safety/reset_fault` only once the driver is disarmed and every
required input is healthy. Keep a hardwired physical E-stop reachable: ROS
topics and software torque control cannot remove energy after a process,
electrical, or mechanical failure.
