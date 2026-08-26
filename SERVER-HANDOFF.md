# Simulation server acceptance handoff

Use this procedure only after the server has been provisioned from this exact
repository revision. This is a validation guide; installation belongs to the
repository's `scripts/install-sim-host.sh`.

## Scope and pass criteria

The server must run Ubuntu 24.04 and expose its GPU to the account that starts
the stack. For a VM/container, that means driver-compatible GPU passthrough,
not only additional vCPUs and RAM. The required result is a headless EGL
OpenGL **3.3 or newer** context and a simulated `/scan` with ranges beyond its
0.08 m blind-zone minimum.

Do not test physical motors, cameras, or arms from this procedure. It runs
`mode:=sim` only.

## 1. Automated regression checks

From the repository root, source the environment and run the installed build's
test suite:

```bash
source scripts/setup.bash
ctest --test-dir "${LEKIWI_WS:-$HOME/lekiwi_ws}/build/lekiwi_rmf" --output-on-failure
git diff --check
```

Pass: every CTest passes and `git diff --check` is silent.

## 2. Renderer qualification

Run the repository's EGL probe as the same user that will run Gazebo:

```bash
scripts/sim-renderer-check.py
```

Pass: it prints the EGL version, GPU renderer, an OpenGL version of at least
3.3, and `renderer preflight passed`.

Fail: do not launch Gazebo or attempt motion. Correct the server's GPU driver,
GPU passthrough, or render-device permissions, then repeat this check. A
desktop display is not required.

## 3. Start the managed simulation

Start the stack with rosbridge disabled for this local acceptance test:

```bash
scripts/sim-up.sh start_rosbridge:=false
```

This command runs the renderer check again, starts Gazebo/RTAB-Map/Nav2 in a
repository-recorded process group, and logs to `~/.ros/lekiwi/sim-stack.log`.
Keep it running while completing the remaining checks.

## 4. Sensor and safety topology

In a second sourced terminal, run:

```bash
source scripts/setup.bash
scripts/sim-scan-check.py --timeout 30
ros2 lifecycle get /collision_monitor
ros2 topic info -v /cmd_vel_safe
```

Pass:

- `sim-scan-check.py` reports a usable scan with at least 180 ranges and at
  least one range beyond `range_min`; it must never report all 360 values at
  0.08 m.
- `/collision_monitor` is `active [3]`.
- `/cmd_vel_safe` has collision monitoring as its final publisher and the
  Gazebo bridge as its subscriber.

If `/scan` is absent, all minimum-range, stale, or invalid, stop here. The
collision monitor holding the robot is correct; repair the renderer/sensor
path rather than bypassing safety.

## 5. Command-path test in the clear simulated room

After the scan test passes, verify mux priority and guarded motion. In one
terminal publish a short Nav2-style command; then publish a manual command:

```bash
ros2 topic pub -r 5 --times 10 /cmd_vel_smoothed geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}}'
ros2 topic pub -r 5 --times 10 /cmd_vel_manual geometry_msgs/msg/Twist \
  '{linear: {y: 0.07}}'
```

While each is running, inspect `/cmd_vel_muxed` and `/cmd_vel_safe` in another
terminal:

```bash
ros2 topic echo /cmd_vel_muxed
ros2 topic echo /cmd_vel_safe
```

Pass: the manual command preempts the Nav2-style command, the guarded output
matches only a clear path, and it returns to an all-zero `Twist` after both
publishers stop. Do not perform this test near a simulated wall or workbench.

## 6. Managed shutdown

Stop only the repository-owned process group:

```bash
scripts/ros-stop.sh
```

Pass: it reports the recorded stack stopped and removes its PID record. If an
external Gazebo or ROS binary ignores SIGINT, this command escalates inside the
recorded group only; do not use a process-name sweep or kill an unrelated ROS
stack.

Record the renderer output, scan-check output, collision-monitor state, and
the final 30 lines of `~/.ros/lekiwi/sim-stack.log` with the test result.
