# Scripts

Start with one of these:

| Goal | Command |
| --- | --- |
| Run a wired robot | `scripts/up.sh` |
| Run the device side of a split robot | `scripts/pi-up.sh` |
| Run the workstation side of a split robot | `scripts/workstation-up.sh [DEVICE]` |
| Run simulation | `scripts/sim-up.sh` |
| Stop a manually launched stack | `scripts/ros-stop.sh` |
| Calibrate the robot | `scripts/calibrate.sh` |
| Drive with the keyboard | `scripts/teleop.sh` |
| Open RViz or camera views | `scripts/rviz.sh`, `scripts/cameras.sh` |

Configuration comes from `.env` (only `LEKIWI_ROBOT_HOST`,
`LEKIWI_DISARM_ON_FAILURE` and `LEKIWI_WIFI_COUNTRY`) and from `LEKIWI_*`
environment variables; see [Configuration](../docs/real-robot.md#configuration).
Scripts that systemd, ROS or another script runs are kept here because installed
units refer to them directly.

## Installation and deployment

- `install.sh [--simulation]` installs a workstation, or with `--simulation` a simulation-only machine.
- `install-pi.sh` installs the LeRobot host on the robot's Raspberry Pi.
- `install-device-services.sh` installs the device boot services (motor host, cameras, Astra, LD06, zenoh bridge).
- `install-compute-services.sh [--remote DEVICE] [--no-start]` installs `lekiwi-stack.service`; it restarts a running stack only when its configuration changed.
- `install-device-network.sh --user USER [--country CC]` sets the Wi-Fi country, turns off Wi-Fi power saving and grants the deploy user NetworkManager control.
- `install-wifi-regdom.sh [CC]` sets the Wi-Fi regulatory country (default `LEKIWI_WIFI_COUNTRY`, else `ID`) now and at every boot.
- `install-deploy-sudoers.sh` grants the deploy user passwordless start/stop of the LeKiwi units only.
- `setup-zenoh-tls.sh DEVICE` creates the private CA and mutual-TLS identities of the zenoh bridge on both machines.
- `deploy-split.sh [DEVICE]` deploys one pushed revision to the compute machine and its device host.
- `reinstall-compute.sh` reinstalls the compute service from `.env` (`LEKIWI_ROBOT_HOST`) with MoveIt enabled.
- `build-lekiwi.sh` rebuilds this package in the workspace used by the managed services.
- `rebuild-all.sh` verifies and vendors the LeKiwi model, renders it, rebuilds and runs the CTest suite.
- `thirdparty-common.sh` (sourced) holds the pinned third-party sources and download helpers.

## Launchers

- `up.sh` starts the LeRobot host and ROS stack on a wired robot.
- `pi-up.sh` starts the device half of a split robot: motor host, cameras, LD06, Astra and zenoh bridge.
- `workstation-up.sh [DEVICE] [launch args]` starts the ROS stack (remote sensors, MoveIt) and RViz.
- `sim-up.sh [launch args]` checks the renderer and starts a managed headless simulation.
- `ros-start.sh [launch args]` runs `bringup.launch.py` against the real robot; `up.sh`, `workstation-up.sh` and `lekiwi-stack.service` execute it.
- `ros-stop.sh` stops the process groups the launchers recorded and leaves systemd units alone.
- `robot-host.sh [calibrate|--no-cameras]` runs the LeRobot motor host or its motor calibration; `lekiwi-host.service` executes it.
- `torque-host.py` is the LeRobot motor host with the torque-safety endpoint that `robot-host.sh` runs.
- `ros-astra.sh`, `ros-cameras.sh`, `ros-lidar.sh`, `ros-zenoh.sh` publish the Astra, the V4L2 cameras, the LD06 and the zenoh bridge on the device.
- `camera-supervisor.sh` keeps one `v4l2_camera` node alive across USB resets.
- `setup.bash` (sourced, bash or zsh) loads ROS, the workspace and its virtual environment.
- `setup-pi.bash` (sourced) loads the minimal environment for the Pi's camera publishers.
- `rviz.sh` opens RViz on a running stack; `cameras.sh` opens one viewer per published camera.
- `foxglove.sh` opens Foxglove Desktop on the local read-only bridge.
- `teleop.sh` and `teleop.py` drive the robot from the keyboard.

## Calibration and hardware tools

- `calibrate.sh` runs every missing motor, pose, camera, height and wheel calibration.
- `calibrate-camera.sh` calibrates one local camera; `checkerboard.py` prints its checkerboard PDF.
- `sync-calibration.sh` copies this robot's calibration files from the machine that produced them.
- `arm-jog.sh` and `arm_jog.py` send one bounded physical arm-joint jog.
- `arm_calibration.py` captures raw joint values for the SO-101 zero pose.
- `capture_stow.py` records the held arm's stow pose into both `safety_production.yaml` and `safety_acceptance.yaml`.
- `gripper-calibrate.py` records and applies a gripper-only calibration; `gripper-diagnose.py` reads its registers.
- `odom_scale.py` measures the odometry scale against a checkerboard.
- `lidar-self-mask.py` measures the LD06 returns from the stationary robot's own body and proposes `config/lidar_self_mask.yaml`.
- `rearm-robot.sh` explicitly re-arms a running real-robot stack.
- `generate-zmq-keys.py` generates the CURVE server identity and authorized client keys.
- `host-health-check.py` checks that the motor host serves both protocols (the host unit's start gate).

## Maps, simulation and qualification

- `free_space.py` turns the front camera into a floor-edge `LaserScan`.
- `validate-map-bundle.py` validates an immutable map/RMF bundle.
- `rtabmap-db-maintenance.py` bounds the RTAB-Map working database at startup.
- `rtabmap-session-guard.py` stops a mapping session at its database quota.
- `sim-qualification.py` collects the fail-closed simulation qualification evidence.
- `sim-renderer-check.py` fails fast without headless OpenGL 3.3.
- `sim-scan-check.py` waits for a usable simulated scan.
- `moveit-shutdown-probe.py` qualifies orderly `move_group` shutdown.
- `render-model.py` renders the zero-pose calibration reference image.
- `vendor-lekiwi-model.py` vendors the generated LeKiwi Xacro as the physical model.
- `prune-ros-logs.sh` deletes stale ROS logs (run by `lekiwi-ros-logrotate.service`).

## lib/

Source-only shell helpers, not commands: `runtime-common.sh` (`.env`, waits, port
probes), `self-heal.sh` (service self-heal watcher), `service-install-common.sh`
(unit rendering, restart tracking) and `service-install-revision.sh` (service
configuration fingerprints).
