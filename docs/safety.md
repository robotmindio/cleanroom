# Safety inputs and motor health

This document records which requested safety functions can be provided by the
current robot hardware and which require an additional physical signal source.
The repository safety supervisor denies the affected capability when a required input is
absent, stale, or unhealthy. Normal hold mode restores permission when inputs
recover, while strict mode (`LEKIWI_DISARM_ON_FAILURE=true`, which launches
`safety_policy:=strict`, and always in simulation) also latches faults. A ROS topic alone is not evidence of a real
safety function.

| Function | Software support exists? | What is required for a real implementation |
| --- | --- | --- |
| Bumper | Yes. The supervisor monitors `safety/bumper_active` when `require_bumper` is true; the production profile leaves it false because no bumper is fitted. | Install physical contact switches or an equivalent contact sensor, then add a hardware bridge that publishes its actual state. |
| E-stop | Yes. The supervisor monitors `safety/estop_active` when `require_estop` is true; the production profile leaves it false because the fitted E-stop cuts motor power outside the electronics and reports nothing. The driver can request servo torque-off. | Fitted and tested (`estop_independent_of_ros: true` in `config/safety_acceptance.yaml`). Software torque-off is not an E-stop. |
| Battery | Yes. The supervisor validates `/battery_state` when `require_battery` is true; the production profile leaves it false because no battery monitor is fitted. | Add a BMS or voltage/current monitor that publishes real, stamped battery data. Servo voltage alone is not a qualified state-of-charge source. |
| Motor health | Yes, for bus/servo telemetry. | Bench-validate the electrical and thermal thresholds before they are allowed to stop motion. |
| IMU | Yes. The safety supervisor can consume `/imu/data`; it is not required, and the EKF does not fuse it, until an IMU exists. | Add, mount, and calibrate a physical IMU and its ROS driver. |

Motor-health telemetry is the only item that can be materially extended using
the installed robot hardware plus repository software. The other items need a
physical sensor or safety component before they can provide a meaningful safety
measurement. Dummy publishers may be useful for testing, but must never be
treated as safety functionality or used to validate the production profile.

## Production safety profile

The production profile (`config/safety_production.yaml`) requires current,
stamped feedback on these inputs before it grants base or arm permission. Base
motion also requires the arm inside the accepted stow pose.

| Input | Topic | Purpose |
| --- | --- | --- |
| Driver state | `safety/driver_state` | Motor-link and torque state |
| Full scan | `/scan` | Obstacle coverage and freshness |
| Depth | `/camera/depth/points` | Arm-workspace obstacles |
| Odometry | `/odom` | Base state |
| Joint state | `/joint_states` | Arm feedback and stow interlock |
| Motor health | `/hardware/diagnostics` | Servo and bus faults |
| Arm collision gate | `/safety/arm_workspace_clear` | Live MoveIt scene/state validity |

`/imu/data`, `/battery_state`, `safety/bumper_active` and `safety/estop_active`
are not required, because the robot has no source for them (see the table
above). Production MoveIt's execution-time workspace gate needs fresh complete
joint state, a fresh `/moveit/filtered_cloud` and repeated successful
`/check_state_validity` responses; collision, timeout or perception failure
withdraws the arm lease. The camera floor-scan fallback is supplemental and
cannot see all side, rear, floor-coloured, low or overhanging obstacles.

Keep the hardwired motor-power stop reachable and supervise every hardware
run. The ROS E-stop topic, `/safety/disarm` and software torque cut are
status/control interfaces; they cannot remove actuator energy after a process,
electrical or mechanical failure. Arming and recovery behaviour is described in
[Arming and recovery](launch-options.md#arming-and-recovery) and
[Current motor-health behavior](#current-motor-health-behavior).

## Physical acceptance

`config/safety_acceptance.yaml` is a schema-version-4 record with
`validated: true`, validated on 2026-10-06. Its scope is **attended autonomous
base** operation:

| Condition | Accepted value |
| --- | --- |
| Operation | Attended, operator continuously at the physical motor-power stop |
| Surface | Dry concrete |
| Payload | `payload_kg: 0.2`, the installed operator-reported 200 g load |
| Arm posture | Folded `travel_stow`, as recorded in `accepted_stow_joint_positions` |
| Maximum speeds | 0.03 m/s linear, 0.06 rad/s angular (the production command limits) |
| Stopping trials | 5 per direction (forward, reverse, left, right, both rotations) |
| Worst stop + 20 mm measurement uncertainty | 49.752 mm, against the 50 mm budget |
| Worst command-stop latency | 1.198 s, against 1.5 s |
| Absent hardware | Bumper, IMU and battery monitor; their fault tests are `null` |

The evidence is
[physical-acceptance-evidence-200g-20261006.json](physical-acceptance-evidence-200g-20261006.json):
loaded stopping windows, moving-fault results, exclusions and raw-artifact
hashes. Hardware and authentication fault tests whose mechanisms did not change
with the load (independent E-stop, unauthorized ZMQ, DDS and rosbridge policy,
restarts, replay, collision-monitor and arm-workspace stops) are carried over
explicitly from the unloaded
[physical-acceptance-evidence-20261004.json](physical-acceptance-evidence-20261004.json);
they are not relabelled as loaded trials. Functional loaded navigation
observations are in
[base-payload-evidence-20261006.json](base-payload-evidence-20261006.json).

The record grants no unattended scope, other load placement, larger payload or
higher speed. `payload_kg` identifies the load in the stopping measurements:
the supervisor validates its format but has no payload input and does not
compare the current load with it. The holder mass and attachment location are
unconfirmed, so 200 g is not a maximum payload rating. The arm completed a
controlled HOME cycle with this load at 0.10 velocity/acceleration scaling;
higher arm speeds, other masses, repeated cycles and thermal endurance are
untested. The operating conditions are a site procedure; software cannot
verify that the operator is present.

The record is invalid until it has reviewed limits, at least
`minimum_trials_per_direction` trials in every direction, worst-case distances
plus uncertainty, stop latency, traceable software/sensor/payload/surface
details, and every applicable fault test marked true. Its hardware record must
agree with the production profile's `require_bumper`, `require_imu` and
`require_battery` settings. At startup the supervisor also requires the
accepted footprint and padding to match both tracked Nav2 costmaps and proves
the enabled StopZone leaves at least the worst stopping distance plus
uncertainty around that footprint. A newly captured arm stow invalidates the
record. Tested speeds must cover the tracked MPPI limits; the manual driver
uses the same limits, so slow trials cannot approve a faster command.

### Revalidating

A change to the accepted conditions (payload, surface, speed, stow, sensors or
hardware) needs new physical evidence; software tests cannot establish stopping
performance or obstacle coverage. Before base trials, physically verify a
compact arm stow, record it with `scripts/capture_stow.py`, and check that the
full arm and cable envelope fits the accepted footprint. Predeclare stopping
limits, measure at least five stops per direction on the recorded surface and
payload, and update the tracked Nav2 StopZone to cover the worst distance plus
measurement uncertainty. Reuse qualified evidence only for unchanged
conditions, and repeat the fault cases affected by the change. Host and ROS
restart tests must show an immediate command stop, then re-arm only once
telemetry and arm permission are healthy.

Each runner below is finite, preserves production services and the SLAM
database, and records observations without granting acceptance. Run them only
with the arm in `travel_stow`, the 30 cm test area clear, and an operator at
the motor-power stop:

```bash
source scripts/setup.bash
# Loaded stopping trials and affected moving faults.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 \
  /usr/bin/python3 scripts/test-onboard-braking.py --payload-g 200
# Short production Nav2 goals and rotations observed by the onboard cameras.
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/test-onboard-navigation.py
```

For a bounded base test inside a 30 cm radius, `python3
scripts/test-navigation.py` temporarily replaces the production compute stack
with `bounded_base_test:=true`, performs a finite route, and restores the
service on exit. The driver caps speed from the tracked production MPPI limits
and stops at a 20 cm wheel-odometry radius; the runner cancels at 18 cm. Wheel
slip still requires independent observation of physical position. All
measured arm/base inputs and a test-client lease renewed within 250 ms are
required. Production startup keeps this mode disabled. Starting a base goal
does not move the arm into `travel_stow`.

### Obstacle coverage

The LD06 produces a 360-degree scan. Its measured body mask covers 100 degrees,
leaving 260 degrees (4.54 rad) of effective coverage. The production
supervisor requires 4.3 rad (about 246 degrees), leaving roughly 14 degrees
for scan-span variation. Masked directions provide no obstacle sensing, which
is why the operator must remain at the motor-power stop. Remeasure the mask
with the arm stowed and before acceptance trials: chassis self-returns that
reach the StopZone cause intermittent false stops, and nearby hardware changes
them. Full surrounding coverage needs a moved lidar or a second sensor.

The Astra depth cloud is not a collision-monitor source: it faces
sideways/rearward and its minimum depth (~0.55 m) lies beyond both zones, so it
cannot see an obstacle inside them. `require_depth` gates base motion on its
freshness only; obstacles below the LD06 plane are not detected.

## Motor-host motion limits

The motor host validates every streamed or locally sampled action before writing
it to LeRobot. Base commands are checked against the tracked Nav2 limits after
applying the saved wheel scales. Arm commands use the tracked joint limits and
the host's saved `~/.ros/lekiwi_arm_calibration.json`; trajectory uploads must
match that calibration. The gripper remains within its normalized 0–100 range.
An exact previously measured hold is retained even if torque-off sag put it
outside a joint boundary; new out-of-bounds targets are rejected. Rejected
streamed actions do not refresh the existing command watchdog or change torque.
This envelope does not establish obstacle clearance or extend physical acceptance.

## Current motor-health behavior

The motor host is the only serial-bus owner. At 10 Hz it reads each STS3215's
torque state, position, raw status register, load, voltage, temperature,
current, and programmed minimum/maximum voltage and maximum-temperature limits.
It sends this snapshot in its observation telemetry (authenticated when CURVE
is configured); the ROS driver
validates it and publishes `/hardware/diagnostics`.

By default the driver arms itself at startup and re-arms every 2 s after a
failure, but it cannot energize the servos until it has fresh host telemetry and
current safety-supervisor permission; a failure stops the base and freezes the
arm with torque on. In strict mode a failed or missing safety input disarms the
driver, cuts torque, and leaves it disarmed until an operator calls
`/safety/arm`. An operator's `/safety/disarm` holds in either mode. See
[Arming and recovery](launch-options.md#arming-and-recovery).

The diagnostics use these units:

| Value | Reported representation |
| --- | --- |
| Voltage | raw register × 0.1 V |
| Current | raw register × 6.5 mA |
| Load | raw value and signed duty-cycle estimate (`raw / 1000`), not physical torque |
| Temperature | internal servo temperature in °C |

The following conditions are `ERROR` and revoke base and arm permission in both modes;
strict mode additionally latches the fault:

- communication failure, incomplete readback, or invalid value;
- torque readback that differs from the host's safety latch; or
- nonzero raw servo `Status` register.

Voltage or temperature at/beyond a servo-programmed limit is currently a
`WARN`: it remains visible but does not revoke permission. Current and load are
telemetry only. This is deliberate—those thresholds must be qualified for this
robot before they become automatic stop conditions.

## Qualifying temperature, current, load, and voltage

1. **Confirm the installed hardware.** With power off, record each servo's
   exact model/revision, firmware, and programmed protective registers.
2. **Collect a baseline.** With the arm stowed and the base safely supported,
   record all values at 10 Hz over representative low, nominal, and peak work.
   Repeat for approved payloads, surfaces, ambient temperatures, and battery
   states.
3. **Set reviewed thresholds.** Use measured normal ranges plus margin. Use
   separate policies for arm and drive servos where their duties differ.
   - Voltage: detect persistent low/high voltage across the system.
   - Temperature: warn well before the configured torque-cut limit.
   - Current: use a duration-based threshold; acceleration spikes are normal.
   - Load: use only alongside current and velocity, since it is a duty-cycle
     estimate rather than physical torque.
4. **Track the policy.** Add thresholds, persistence intervals, and hysteresis
   to repository-owned YAML configuration. Do not rely on ROS parameters,
   RViz, or shell changes after startup.
5. **Validate fault response.** In controlled, recoverable tests, induce low
   supply voltage, sustained drive load on a safe stand, protected arm
   resistance using a fixture, thermal soak, and bus/servo faults. Never
   restrain the robot by hand.
6. **Promote proven conditions.** Start with telemetry, then warning/alerts,
   and only then promote sustained, verified conditions to `ERROR` and motion
   denial. Record physical evidence in `config/safety_acceptance.yaml` before
   treating it as production safety behavior.

## What is an IMU?

An IMU (Inertial Measurement Unit) is a compact sensor that measures
acceleration and rotational motion. It normally contains accelerometers and
gyroscopes, and may also contain a magnetometer. On this robot it would help
estimate tilt, vibration, and especially turn rate. Together with wheel
odometry, it makes the navigation state estimate more reliable.
