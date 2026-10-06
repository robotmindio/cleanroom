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
- `install-pi.sh` installs the Pi host and enables the Pi 5 USB current setting by default; this requires a 5 V / 5 A supply.
- `install-device-services.sh` installs the device boot services (motor host, cameras, Astra, LD06, zenoh bridge).
- `install-compute-services.sh [--remote DEVICE] [--no-start]` installs `lekiwi-stack.service`; it restarts a running stack only when its configuration changed.
- `install-device-network.sh --user USER [--country CC]` sets the Wi-Fi country, turns off Wi-Fi power saving, persists 8 MiB DDS socket-buffer limits and grants the deploy user NetworkManager control.
- `switch-device-wifi.sh SSID` creates or reuses the matching 5 GHz profile with priority above the active fallback and a native timed rollback; it retains the original profile and does not expose credentials.
- `install-wifi-powersave.sh` disables Wi-Fi power saving on the active interface and on future NetworkManager connections; both service installers run it.
- `install-wifi-regdom.sh [CC]` sets the Wi-Fi regulatory country (default `LEKIWI_WIFI_COUNTRY`, else `ID`) now and at every boot.
- `install-deploy-sudoers.sh` grants the deploy user passwordless start/stop of the LeKiwi units only.
- `setup-zenoh-tls.sh DEVICE` creates the private CA and mutual-TLS identities of the zenoh bridge on both machines.
- `deploy-split.sh [DEVICE]` deploys one pushed revision to the compute machine and its device host.
- `stage-release.sh compute|device WORKSPACE REVISION` builds and tests a detached release before cutover, retaining the active workspace.
- `check-release.py seal|verify RELEASE REVISION compute|device` checks source, installed artifacts, local settings, and complete passing CTest evidence.
- `reinstall-compute.sh` reinstalls the compute service from `.env` (`LEKIWI_ROBOT_HOST`) with MoveIt enabled.
- `build-lekiwi.sh` rebuilds this package in the workspace used by the managed services.
- `reload-moveit.py` reloads installed collision rules through the managed planner's respawn while the driver is already DISARMED. It preserves live poses and calibration and leaves motor services and torque alone. Source `scripts/setup.bash` first.
- `build-native.sh` builds pinned class_loader 2.7.1, rclcpp 28.1.22 and Nav2 1.3.13 reliability patches into that workspace. Installation runs it on compute; staged split deployments run it on both hosts. `/opt/ros` remains a dependency underlay.
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
- `robot-host.sh --telemetry-fault-test` runs the same host with a qualification-only observation wrapper: SIGUSR1 injects motor diagnostic ERROR and SIGUSR2 repeats the last valid frame; each expires after eight seconds to cover the camera stop observation. Stop the normal host first, use a finite systemd test unit, and restore the normal service afterward. It never writes diagnostic faults to servos.
- `test-acceptance-gates.py obstacle|workspace` checks a real lidar obstacle or a temporary MoveIt collision object and rejects arm movement during the collision; it restores production and does not approve acceptance.
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

- `test-onboard-navigation.py` runs four short Nav2 goals and rotations with independent native Astra RGB-D odometry and snapshots from all robot cameras. It uses a temporary map, stops at an early 16 cm visual boundary, and restores production without changing its map or physical acceptance. Source `scripts/setup.bash` first; the arm must be in `travel_stow` and the attended 30 cm area clear.
- `test-onboard-braking.py --payload-g 200` records finite loaded stopping and fault checks using the robot cameras and independent raw LiDAR. It preserves production services and torque; the specification is `config/onboard_braking.yaml`. `--resume <run-directory>` retains qualified evidence, and `--center X Y YAW` selects a clear bounded test center. It never grants acceptance automatically.
- `test-physical-acceptance.py` runs finite live lidar, depth and telemetry-loss checks within the attended 30 cm envelope; `--restart-tests` tests compute-driver suspension and the Pi motor-host restart. It restores services and records fault evidence without approving physical braking distances.
- `test-braking.py` uses the fixed USB camera and the configured 35 mm chassis markers for finite speed/stopping characterization. `--inspect` checks stationary tracking; `--explore-only` omits the configured repetitions (five per direction by default); `--directions left right rotation_cw rotation_ccw` resumes only those directions, leaving earlier evidence in its original run. The tracked specification is `config/physical_test.yaml`; raw frames and measurements go under `.benchmarks/physical-braking/`. It restores production and never automatically grants physical acceptance.
  For a load revalidation, `--production --payload-g 200` records the reported load and tests the running stack without restarting services. Fresh chassis/floor marker tracking must pass before movement. Results still require review before changing the acceptance record.
- `free_space.py` turns the front camera into a floor-edge `LaserScan`.
- `validate-map-bundle.py` validates an immutable map/RMF bundle.
- `rtabmap-db-maintenance.py` prunes old RTAB-Map archives without replacing the active map.
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
