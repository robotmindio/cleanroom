# LeKiwi hardware bring-up (LeRobot only, no ROS)

Get the robot moving under plain LeRobot before any of the ROS stack is involved.
If teleoperation works here, every later failure is a ROS problem, not a wiring,
motor-ID, or calibration problem.

The main [README](README.md) picks up from the end of this document.

## Which machine runs what

| Machine | Runs | Installer |
| --- | --- | --- |
| Robot's Raspberry Pi | LeRobot host only: Feetech bus + cameras, ZMQ server | `scripts/install-pi.sh` |
| Workstation | ROS 2, Nav2, RTAB-Map, Open-RMF, and the LeRobot *client* | `scripts/install.sh` |

The two talk over ZMQ `5555/tcp` (commands) and `5556/tcp` (observations). The Pi
never runs ROS, so `scripts/install.sh` must not be run on it — it requires Ubuntu
24.04 amd64/arm64 with ROS Jazzy and would pull in the entire ROS stack the Pi has
no use for.

For the **wired** LeKiwi variant there is no Pi: run both installers on the
workstation and use `remote_ip:=127.0.0.1`.

## Choosing the Pi image

Use a **64-bit** image. LeRobot 0.6.1 declares `requires-python >=3.12`, which
decides this:

| Image | Python | Verdict |
| --- | --- | --- |
| Raspberry Pi OS **Trixie** (64-bit) | 3.13 | **Recommended.** Native Pi support, smallest install |
| Ubuntu Server 24.04 LTS (arm64) | 3.12 | Fine. Pick this if you may later run ROS 2 Jazzy on the Pi |
| Raspberry Pi OS **Bookworm** | 3.11 | **Will not work** — below LeRobot's floor |
| Any 32-bit image | — | **Will not work** — no aarch64 PyTorch wheels |

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
running those commands **on the Pi**. The camera calibration and mapping steps in
the main README stay on the workstation.

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

LeRobot requires `numpy>=2`; ROS 2 Jazzy's compiled extensions are built against
the system's numpy 1.26. In one interpreter they do not merely warn — `rmf_adapter`
segfaults mid-run. So the workstation has two virtualenvs:

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

```bash
lerobot-calibrate --robot.type=lekiwi --robot.id=lekiwi_1
```

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

## 5. Run the host

On the machine physically wired to the motors — the Pi on the robot, or your
laptop for the wired LeKiwi variant:

```bash
python -m lerobot.robots.lekiwi.lekiwi_host \
  --robot.id=lekiwi_1 \
  --host.connection_time_s=86400
```

Defaults: command socket `5555/tcp`, observations `5556/tcp`, watchdog 500 ms,
loop 30 Hz. The watchdog stops the base when commands stop arriving. It is not
an E-stop.

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
constructing a `LeKiwiClient` and sending `x.vel`, `y.vel`, `theta.vel` directly.

## 7. Record a dataset (optional)

Needs the `core-scripts` extra and a Hugging Face write token:

```bash
hf auth login --token $HUGGINGFACE_TOKEN --add-to-git-credential
python ~/lerobot-src/examples/lekiwi/record.py
```

Adapt `remote_ip`, `repo_id`, `port`, and `task` inside the script. Datasets land
in `~/.cache/huggingface/lerobot/{repo-id}`.

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

Check both ports, not just 5555:

```bash
ss -ltn | grep -E '5555|5556'
```

## Moving on to ROS

Once teleoperation works, leave the host running and start the ROS side against
it. The ROS driver (`lekiwi_rmf/driver.py`) is a pure ZMQ `LeKiwiClient` — it
never touches USB, so the ROS machine needs no serial or camera permissions at
all.

```bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=real remote_ip:=192.168.1.50
```

Continue with camera calibration in [README](README.md) §Real robot, step 2.
The driver reads only the `front` camera and ignores `wrist`.

## Safety

Use a physical emergency stop. Supervise every first run. The arm can reach
across the base footprint, so keep the workspace clear before the first
teleoperation attempt.
