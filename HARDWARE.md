# LeKiwi hardware bring-up (LeRobot only, no ROS)

Get the robot moving under plain LeRobot before any of the ROS stack is involved.
If teleoperation works here, every later failure is a ROS problem, not a wiring,
motor-ID, or calibration problem.

The main [README](README.md) picks up from the end of this document.

## Which machine runs what

| Machine | Runs | Installer |
| --- | --- | --- |
| Robot's Raspberry Pi | LeRobot host (Feetech bus, ZMQ server); ROS camera, Astra, LD06 and zenoh bridge services | `scripts/install-pi.sh` |
| Workstation | ROS 2, Nav2, RTAB-Map, Open-RMF, and the LeRobot *client* | `scripts/install.sh` |

Motor commands travel over LeRobot ZMQ `5555/tcp`; observations and joint state
travel over `5556/tcp`. The repository-owned torque safety endpoint is
`5557/tcp`. The device sensor bridge listens on `7447/tcp` (zenoh, export-only).
The motor host binds all interfaces by default so the workstation can reach it,
and the ZMQ listeners accept any client unless CURVE is configured; the zenoh
bridge accepts only mutually authenticated TLS peers. Keep the ZMQ ports on a
trusted robot network or behind a firewall; see the README's
[Network security](README.md#network-security). ROS 2 DDS is not secured by
either and remains a separate exposure unless the deployment isolates it.

The motor host serves no cameras. A separate `v4l2_camera` service owns each
USB camera and publishes the front and wrist streams; camera frames never pass
through the motor host. This isolates camera stalls from actuator control.

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

On a Raspberry Pi 5, installation and every `scripts/deploy-split.sh` deployment
persist `usb_max_current_enable=1` in the firmware config, allowing up to 1.6 A
across USB ports. This requires a **5 V / 5 A supply**. Use a powered USB hub
with a lower-rated supply. Reboot once after the setting is written; verify with
`vcgencmd get_config usb_max_current_enable`.

It verifies the architecture and Python version up front, installs the system
prerequisites, removes `brltty` if present (it claims CH34x adapters and steals
the motor bus), adds you to `dialout` and `video`, builds the venv, installs
`lerobot[lekiwi,hardware]==0.6.1`, and clones the `v0.6.1` examples. It prints the
Pi's IP address at the end. Save that address as `LEKIWI_ROBOT_HOST` in the
workstation checkout's `.env`; startup and deployment use it as `remote_ip`
unless an explicit host is passed.

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
| `.venv-lerobot` | 2.2 | lerobot + feetech/pyzmq | LeRobot CLIs and the motor host only |

**For every command in this document, activate the LeRobot venv — never
`scripts/setup.bash`, which activates the ROS one and has no `lerobot` in it.**

The ROS driver runs in the ROS environment and communicates over ZMQ.
`robot-host.sh` runs the motor host in the LeRobot environment; LeRobot never
imports into the ROS process.

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

The LeRobot robot type remains `lekiwi` for an SO-101 arm: it already maps the
six follower servos to IDs 1 through 6 with the stable `arm_*` names. There is
no separate `lekiwi_so101` type.

## 3. Calibrate

Only the arms need calibration; the wheels do not.

Calibrate before the first host start; the host refuses to start without a
calibration file for its ID. To calibrate, or to calibrate again, run:

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
lerobot-calibrate --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 --teleop.id=leader_1
```

## 4. Check the cameras

The ROS camera nodes, not the motor host, read the front and wrist cameras.
`/dev/videoN` is renumbered by every USB re-enumeration, and on a laptop
`/dev/video0` is almost always the built-in webcam, so the camera scripts
select each camera by its stable `/dev/v4l/by-id/` name. List those names and
identify each device by model:

```bash
ls /dev/v4l/by-id/
udevadm info -q property -n /dev/video2 | grep ID_MODEL=
```

Note that `/dev/video1` is usually the metadata node of the same UVC device as
`/dev/video0`, not a second camera. `scripts/ros-cameras.sh` and
`scripts/ros-start.sh` find the known front and wrist cameras automatically;
set `LEKIWI_FRONT` and `LEKIWI_WRIST` to `/dev/v4l/by-id/...-video-index0`
paths for other hardware, or `LEKIWI_WRIST=none` to run without the wrist
camera.

## 5. Run the host

On the machine physically wired to the motors — the Pi on the robot, or your
laptop for the wired LeKiwi variant:

```bash
scripts/robot-host.sh
```

Defaults: command socket `5555/tcp`, observations `5556/tcp`, torque safety
`5557/tcp`, watchdog 500 ms, loop 30 Hz, bound to all interfaces (set
`LEKIWI_BIND_ADDRESS` to pin one). The watchdog stops the base when
commands stop arriving and, by default, leaves servo torque on. It is not an E-stop.
The repository host starts torque-off and changes servo torque through the separate
safety endpoint; only in the strict mode (`LEKIWI_DISARM_ON_FAILURE=true`) does the
watchdog also cut torque.

The host serves no camera images: ROS reads the cameras through its own nodes
on whichever machine they are plugged into, one reader per device, so a stalled
frame cannot abort the motor host. The ROS camera publisher on a remote device
machine:

```bash
scripts/ros-cameras.sh
```

It finds the cameras by name, as described in
[Check the cameras](#4-check-the-cameras).

For the normal two-computer setup, use the device launcher instead. It starts
the camera-less motor host, ROS camera publisher, LD06 publisher, Astra
publisher and zenoh sensor bridge — v4l2_camera reads the cameras here, and
rate-limited compressed previews and sensor clouds cross to the workstation:

```bash
scripts/pi-up.sh
```

At the default `jpeg_quality:=50` a 640x480 frame measures about 14 KB, so
30 Hz costs roughly 3 Mbit/s; the same frame at the library default of 95
costs 70–90 KB, or 18 Mbit/s. Raise it if RTAB-Map starts losing loop
closures, lower it if the link is saturated.

### Optional boot services

For unattended startup, install the device and compute systemd services; see
[Boot services](docs/real-robot.md#boot-services) for the units, their options,
CURVE keys and the split-deployment workflow.

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
LeRobot 0.6.1 also saturates wheel goals at 3000 ticks/s, which limits forward
translation to about 0.266 m/s with its 5 cm wheel radius. The repository motor
host derives its wheel ceiling from the highest validated tracked linear profile
(3912 ticks/s for the 0.30 m/s attended stage); body command limits still apply.
These command ceilings do not establish physical speed or braking acceptance.

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
in `~/.cache/huggingface/lerobot/{repo-id}`. The repository motor host sends
no camera images, so datasets recorded against it contain motor state only;
image recording is outside this repository's supported setup.

## 8. Mount the LD06 lidar

The LDROBOT LD06 replaces the camera-as-laser obstacle scan (`laser_source:=ld06`
instead of the default `auto` selection). It mounts on the RobotSkin base at the
rear of the upper plate, reusing the removed Pi case's screw pair and the
adjacent grid row. Its scan centre is 13.5 cm behind the origin, 0.5 cm left,
and 8.85 cm above `base_link`; the tracked CAD model supplies that TF.
Forward (+X) is the arm/fixed-camera side. Keep the unit level and verify scan
bearings against known objects in RViz after any bracket change.

It is a USB serial device (typically a CP2102 bridge), so it appears as
`/dev/ttyUSB0`, owned `root:dialout` -- the same group the motor bus needs, and
the installer adds you to it. Prefer its stable name over `/dev/ttyUSB0`, for
the same reason as with cameras:

```bash
ls /dev/serial/by-id/    # e.g. usb-Silicon_Labs_CP2102N_...-if00-port0
```

Nothing else is configurable: the LD06 speaks 230400 baud and the default
startup detects its known CP2102 by-id device automatically (`scripts/up.sh` on
a wired robot, `scripts/pi-up.sh` or `lekiwi-lidar.service` on the device side of
a split one).

The normal `scripts/install-device-services.sh` installation on the robot host
starts `lekiwi-lidar.service`; the normal compute installation relays its
private `/pi/lidar/scan` to the canonical `/scan`.

```bash
sudo scripts/install-compute-services.sh --service-user "$USER" \
  --workspace "$HOME/lekiwi_ws" --remote DEVICE_IP
```

and check the scan against reality in RViz before trusting it: spin the robot by
hand and watch a nearby wall stay put in the LaserScan display.

If it is offset, keep the CAD nominal pose intact and record the measured scan
correction in the `lidar_offset_xyz` and `lidar_offset_yaw` properties in
`urdf/lekiwi.urdf.xacro` (metres, metres, metres, radians). Rebuild before
restarting the stack so RViz, MoveIt, and robot_state_publisher use it together.

## Troubleshooting

Motor-port, status-packet and host-endpoint symptoms are in
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## Moving on to ROS

Once teleoperation works, leave the host running and start the ROS side against
it. The ROS driver (`lekiwi_rmf/driver.py`) is a pure ZMQ `LeKiwiClient` — it
never touches USB, so the ROS machine needs no serial or camera permissions at
all. A non-loopback host supports unauthenticated ZMQ on a trusted robot LAN.
Use the service installer above without `--curve-dir`, or opt into CURVE by
installing both halves with `--curve-dir`. Then start the workstation side with
the repository script, which reads the robot address from `LEKIWI_ROBOT_HOST`
in `.env` (or takes it as its first argument):

```bash
scripts/workstation-up.sh
```

Calibrate each camera on the machine it is plugged into with
`scripts/calibrate.sh camera` or `scripts/calibrate.sh wrist`. The workstation
relays both feeds; navigation and RTAB-Map use only the front camera.

## Safety

The production safety profile, its required inputs, the physical acceptance
record and the arming policy are described in
[Safety inputs and motor health](docs/safety.md). Keep a hardwired physical
E-stop reachable: ROS topics and software torque control cannot remove energy
after a process, electrical or mechanical failure.
