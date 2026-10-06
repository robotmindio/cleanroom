# Simulation qualification

This is the procedure for the qualified simulation-server acceptance tracked in
[DEFERRED.md](DEFERRED.md#qualified-simulation-server-acceptance). The runner
collects evidence and never reports qualification; an administrator reviews
the evidence. Physical acceptance is independent: the runner neither changes
`config/safety_acceptance.yaml` nor requires an existing approval to be revoked.

## Host requirements

- Ubuntu 24.04 with this exact repository revision.
- A GPU exposed to the service account, including render-device permissions
  for a VM/container.
- A headless EGL OpenGL context version 3.3 or newer. The Raspberry Pi exposes
  OpenGL 3.1 and cannot qualify Ogre2. The installed Ogre Vulkan path has an
  unresolved `glslang::InitializeProcess` symbol and is not an accepted
  substitute.

## Repository evidence

Provision the host, build the repository, then run the runner from the source
checkout (it resolves its repository root from its own path):

```bash
./scripts/install.sh --simulation
source scripts/setup.bash
scripts/sim-qualification.py \
  --build-dir build/lekiwi_rmf \
  --output-dir "$HOME/.ros/lekiwi/qualification-evidence/final-revision"
```

The runner is strict. It requires a clean exact revision; a CMake build and
installed package bound to this checkout; the guarded simulation profile;
ShellCheck; pyzmq in CMake's selected interpreter; the complete expected CTest
set, including the three pyzmq-dependent tests, with every test passing;
static source and simulation safety-profile checks; and an EGL/OpenGL renderer
of at least 3.3. It writes every command result, the exact revision/dirty
state and `summary.json` to the output directory, including when a check
fails. The output directory must be outside the source checkout so the
runner's own output cannot make the revision dirty.

Retain in particular the renderer-free physics test and the native
actuator-failsafe fault-injection test. The native failsafe must zero stale
wheel targets and interrupt an arm trajectory on heartbeat loss. Those tests
launch Gazebo as a launch-managed process, use a fresh transport partition per
invocation, and hold a shared CTest resource lock so a stale or parallel server
cannot supply their evidence.

The runner directly invokes `scripts/moveit-shutdown-probe.py` and keeps its
revision-bound JSON. It puts the selected build's install prefix ahead of any
older sourced overlay and requires schema 1, that exact package prefix, the
current Git SHA, package-version metadata, `clean_shutdown: true`, and
`move_group_exit_code: 0`. The ordinary E2E CTest does not assert
`move_group`'s shutdown exit code, so a passing test alone cannot qualify this
behaviour.

## Runtime evidence and manual observations

Start the managed simulation with no external bridge:

```bash
scripts/sim-up.sh start_rosbridge:=false
```

In another sourced terminal, before issuing motion:

```bash
scripts/sim-scan-check.py --timeout 30
ros2 lifecycle get /collision_monitor
ros2 topic info -v /cmd_vel_safe
ros2 topic hz /camera/depth/points
```

Pass only if `/scan` has at least 180 ranges with usable values beyond
`range_min`, the collision monitor is active, `/cmd_vel_safe` has the intended
publisher/subscriber topology, and the delayed/noisy depth cloud remains live.
An absent or all-minimum scan is a correct fail-closed stop, not permission to
bypass collision monitoring. Then collect bounded runtime evidence without
altering the stack:

```bash
scripts/sim-qualification.py \
  --build-dir build/lekiwi_rmf \
  --output-dir "$HOME/.ros/lekiwi/qualification-evidence/runtime-final" \
  --collect-runtime
```

It retains scan, lifecycle, topology, depth and MoveIt-parameter observations
plus tails from the managed simulation and newest ROS launch logs, and writes
`runtime-checklist.md`. Command success alone is not administrator review;
complete the checklist with these observations.

With a clear simulated room, check mux priority and stale-command stopping:

```bash
ros2 topic pub -r 5 --times 10 /cmd_vel_smoothed geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}}'
ros2 topic pub -r 5 --times 10 /cmd_vel_manual geometry_msgs/msg/Twist \
  '{linear: {y: 0.07}}'
ros2 topic echo /cmd_vel_muxed
ros2 topic echo /cmd_vel_safe
```

Manual input must preempt Nav2, guarded output must remain zero for an occupied
path, and both outputs must return to zero after publishers stop. Then launch
with `start_moveit:=true`, place an obstacle in the arm workspace, and verify
it appears in both `move_group` and a newly launched RViz MotionPlanning panel.
Confirm the arm-workspace gate withdraws permission and interrupts execution.

Fault-inject loss of the ROS omni controller/bridge and the arm adapter
heartbeat. The Gazebo-native 250 ms failsafe must stop wheel demand and replace
an active arm trajectory with a measured-position hold. This supplements, but
does not represent, physical E-stop or braking acceptance.

Stop only the recorded process group:

```bash
scripts/ros-stop.sh
```

Store the renderer output, test results, scan/depth evidence, collision-monitor
state, fault-injection result, RViz/move_group values and the final 30 lines of
`~/.ros/lekiwi/sim-stack.log` beside the runner's `summary.json`. The checklist
remains incomplete until the manual mux, obstacle, RViz and heartbeat-loss
observations are attached and reviewed.
