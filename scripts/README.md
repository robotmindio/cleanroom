# Scripts

Start with one of these:

| Goal | Command |
| --- | --- |
| Run a wired robot | `scripts/up.sh` |
| Run the device side of a split robot | `scripts/pi-up.sh` |
| Run the workstation side of a split robot | `scripts/workstation-up.sh DEVICE` |
| Run simulation | `scripts/sim-up.sh` |
| Stop a manually launched stack | `scripts/ros-stop.sh` |
| Calibrate the robot | `scripts/calibrate.sh` |
| Drive with the keyboard | `scripts/teleop.sh` |
| Open RViz or camera views | `scripts/rviz.sh`, `scripts/cameras.sh` |

## Installation and deployment

- `install.sh` installs a workstation; `install-pi.sh` installs a device host.
- `install-sim-host.sh` installs a simulation-only machine.
- `install-device-services.sh` and `install-compute-services.sh` install boot services.
- `deploy-split.sh` updates an installed split robot.
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

## Internals

The remaining launchers and Python programs are called by the commands above,
ROS 2, or systemd. They are kept in this directory because installed services
refer to them directly. `lib/` contains source-only shell helpers and is not a
command directory.
