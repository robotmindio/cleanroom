# Test handoff

This document is for the reviewer validating the current LeKiwi ROS 2 stack.
It describes the intended behaviour, test order, and the deliberate operational
trade-offs in this revision.

## Scope and current defaults

- Real-hardware startup arms automatically **once** after the driver receives a
  complete, finite, fresh LeRobot observation. It sends a zero base command at
  that point; arming alone must not cause motion.
- A stale, invalid, or failed telemetry link transitions the driver to
  `LINK_LOST`, clears its last velocity, and aborts an in-flight arm trajectory.
  After telemetry returns, it stays `DISARMED`; it must never automatically
  replay a base command or arm trajectory.
- The base command path is:

  ```text
  teleop /cmd_vel_manual ---+-> /cmd_vel_muxed -> collision monitor -> /cmd_vel_safe -> driver
  Nav2 /cmd_vel_smoothed --+
  ```

  The driver subscribes only to `/cmd_vel_safe`. The Gazebo bridge receives the
  guarded topic only.
- `start_rmf:=false` and `start_moveit:=false` by default. Rosbridge starts by
  default at `0.0.0.0:9090`, exactly as requested for trusted-LAN use.
- Rosbridge and unconfigured ROS 2 DDS have no authentication or authorization.
  A machine on the reachable network can potentially control the ROS graph.
  This is intentional for the requested deployment, not a security boundary.

## Safety prerequisites for physical testing

The robot will be armed after a successful startup. Before starting it:

1. Clear the base and arm sweep area; keep people outside it.
2. Verify the physical emergency stop is reachable and works.
3. Start with no pending Nav2 or MoveIt goal and no teleop publisher.
4. Do not run the CPU-heavy automated test suite while commanding hardware.
   On the 4 GB robot computer it can delay camera/scan frames. The collision
   monitor should stop the base in that case, but this is not a normal operating
   condition.

`/safety/disarm` disables ROS commands but does **not** remove servo holding
torque; use the physical emergency stop for that.

## Build and automated checks

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /home/nex/lekiwi_ws/log build \
  --base-paths /home/nex/cleanroom \
  --packages-select lekiwi_rmf \
  --build-base /home/nex/lekiwi_ws/build \
  --install-base /home/nex/lekiwi_ws/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

ctest --test-dir /home/nex/lekiwi_ws/build/lekiwi_rmf --output-on-failure
git diff --check
```

Expected: all ten CTest suites pass. They cover odometry, calibration-motion
guards, free-space scan production, camera relaying, trajectory validation,
driver telemetry/link handling, command muxing, collision models, readiness
gates, and teleop parsing.

## Simulation review

Use a new terminal and source the repository environment:

```bash
source scripts/setup.bash
ros2 launch lekiwi_rmf bringup.launch.py mode:=sim
```

Check the following without issuing motion first:

```bash
ros2 node list
ros2 topic echo --once /free_space/health
ros2 lifecycle get /collision_monitor
ros2 topic info -v /cmd_vel_safe
```

Expected:

- `/free_space/health` reports `state=OK` once camera and scan data are live.
- `/collision_monitor` is `active [3]`.
- The driver command input in real mode is `/cmd_vel_safe`; no direct manual or
  Nav2 topic is accepted by the driver.
- Nav2 starts only after the camera, odometry, and map readiness gates complete.

For command-path testing, use only a clear simulated scene. Confirm that a
fresh `/cmd_vel_manual` command preempts a fresh Nav2 command at the mux, that
stale inputs result in a zero output, and that collision monitor is the final
publisher of `/cmd_vel_safe`.

## Real-hardware startup and status review

Start the managed stack with:

```bash
scripts/up.sh
```

Then, from a sourced terminal, verify:

```bash
ros2 topic echo --once /safety/state
ros2 topic echo --once /free_space/health
ros2 lifecycle get /collision_monitor
ss -ltnp 'sport = :9090'
```

Expected after a healthy startup:

- `/safety/state`: `ARMED`
- `/free_space/health`: `state=OK`
- collision monitor: `active [3]`
- rosbridge: `0.0.0.0:9090`

Review `/home/nex/.ros/lekiwi/stack.log` for `Connected to LeKiwi host`,
`Armed after initial healthy LeKiwi telemetry`, and Nav2 lifecycle activation.
No non-zero command should be sent as part of this startup check.

## Link-loss recovery test

Perform this only in the cleared physical test area with the emergency stop in
reach. Interrupt the motor host/telemetry path briefly, then restore it. Do
not use an arm trajectory for this test.

Expected sequence:

```text
ARMED -> LINK_LOST -> DISARMED
```

After recovery, verify that the base remains still and no interrupted arm
trajectory resumes. A human must explicitly re-arm before a new command:

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
```

Do not change the driver to retain or replay raw velocity commands through a
link loss. The collision monitor filters current base velocity against current
scan data; it cannot prove that the robot, arm, map, or surroundings stayed
safe while the telemetry/command link was unavailable.

## Camera and collision fail-safe test

With the robot stationary, temporarily interrupt the front camera. Verify that
the camera supervisor restarts it, `free_space` reports a blocked scan while
images are unavailable, and collision monitor stops output on stale/invalid
scan data. Restore the camera and verify health returns to `OK` before any
motion test.

The monocular floor-segmentation scan is conservative but is not a substitute
for lidar/depth sensing. It cannot reliably detect floor-coloured or overhanging
obstacles; do not use that limitation as justification for automatic motion
resumption.

## Rosbridge and network review

Rosbridge is intentionally enabled and unauthenticated on every IPv4
interface. A reviewer should verify it is reachable from the intended trusted
LAN using a read-only WebSocket/roslib client, then confirm it exposes the
expected ROS domain. Do not publish a non-zero velocity as a connectivity test.

ROS 2 DDS also uses UDP multicast discovery (typically ports 7400/7401 plus
dynamic unicast ports). It is not a web service, but it lets compatible ROS 2
participants on the same reachable LAN discover topics, services, and actions.
With DDS security disabled, this is another control-plane exposure.

An older `free_fleet_adapter` process may be present outside the managed stack.
It comes from the Free Fleet source package built by `scripts/install.sh` and
bridges RMF tasks to Nav2 goals for `lekiwi_1`. The current observed process was
started from a prior GNOME Terminal session, not by this `start_rmf:=false`
stack. It must be inventoried before testing RMF to avoid two adapters managing
the same robot:

```bash
pgrep -af free_fleet_adapter
ss -ltnp | rg free_fleet_adapter
```

Stop or retain that process only after deciding which RMF deployment owns the
robot; do not run a second adapter against the same Nav2 instance.

## Review targets

Pay particular attention to:

- fresh/finite telemetry gating and one-shot startup arming in
  `lekiwi_rmf/driver.py`;
- safe command topology in `lekiwi_rmf/cmd_vel_mux.py`, collision-monitor
  parameters, and launch remappings;
- camera supervisor restart and calibration scaling in
  `scripts/camera-supervisor.sh`;
- readiness ordering in `launch/bringup.launch.py` and
  `lekiwi_rmf/readiness_gate.py`;
- exact CAD geometry, collision envelopes, joint limits, SRDF exclusions, and
  the MoveIt/RViz configuration;
- runtime scripts and systemd units, including PID ownership and the rule that
  repository shutdown scripts must not kill unrelated robot processes.
