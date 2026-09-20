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

## Installation and deployment

- `install.sh` installs a workstation; `install-pi.sh` installs a device host.
- `install-sim-host.sh` installs a simulation-only machine.
- `install-device-services.sh` and `install-compute-services.sh` install boot services.
  The device installer also turns off Wi-Fi power saving (it adds hundreds of
  milliseconds of latency), sets the Wi-Fi regulatory country (`ID`; the default world
  domain hides 5 GHz networks) and lets the service user run NetworkManager commands
  over SSH without a password, through `install-device-network.sh`.
- `setup-zenoh-tls.sh DEVICE` creates a private CA and installs the mutual-TLS
  identities of the zenoh sensor bridge on both machines. `install-compute-services.sh`
  runs it (idempotently), so it only needs running by hand with `--renew`;
  `deploy-split.sh` refuses to run without the identities.
- `deploy-split.sh` updates an installed split robot.
- `reinstall-compute.sh` reinstalls and restarts `lekiwi-stack.service` from the
  `LEKIWI_ROBOT_HOST` in `.env`, with MoveIt enabled.
- `build-lekiwi.sh` rebuilds this package in an existing workspace.

## Calibration and maintenance

- `calibrate.sh` coordinates motor, pose, camera, height, and wheel calibration.
- `calibrate-camera.sh`, `checkerboard.py`, `arm-jog.sh`, and
  `sync-calibration.sh` support individual calibration tasks.
- `rearm-robot.sh` explicitly re-arms a running real-robot stack.
- `gripper-calibrate.py`, `gripper-diagnose.py`, and `odom_scale.py` are
  focused hardware tools.
- `sim-qualification.py` runs the documented simulation qualification.
- `validate-map-bundle.py` and `rtabmap-db-maintenance.py` validate or maintain
  stored navigation data.

## Launchers

- `up.sh` starts the LeRobot host and ROS stack on a wired robot (RViz: `rviz.sh`).
- `pi-up.sh` starts the device half of a split robot: motor host, cameras, LD06,
  Astra and the zenoh sensor bridge.
- `workstation-up.sh [DEVICE] [launch args]` starts the ROS stack (remote cameras
  and lidar, MoveIt) and RViz against the device named by `LEKIWI_ROBOT_HOST` or
  its first argument.
- `ros-start.sh [launch args]` runs `bringup.launch.py` against the real robot in
  the foreground: it applies `.env`, saved calibration and camera detection. It
  is what `up.sh`, `workstation-up.sh` and `lekiwi-stack.service` execute.
- `robot-host.sh [calibrate|--no-cameras]` runs the LeRobot motor host (or its
  motor calibration) with this machine's device paths; `lekiwi-host.service`
  executes it.
- `ros-stop.sh` stops the process groups the launchers above recorded; it leaves
  active systemd units alone and reports them.
- `foxglove.sh` opens Foxglove Desktop on the local read-only bridge.

Configuration comes from `.env` (only `LEKIWI_ROBOT_HOST` and
`LEKIWI_DISARM_ON_FAILURE`) and from `LEKIWI_*` environment variables; see the
README's [Configuration](../README.md#configuration).

## Internals

The remaining launchers and Python programs are called by the commands above,
ROS 2, or systemd. They are kept in this directory because installed services
refer to them directly. `lib/` contains source-only shell helpers and is not a
command directory.
