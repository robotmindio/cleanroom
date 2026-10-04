# Safety inputs and motor health

This document records which requested safety functions can be provided by the
current robot hardware and which require an additional physical signal source.
The repository safety supervisor denies the affected capability when a required input is
absent, stale, or unhealthy. Normal hold mode restores permission when inputs
recover, while strict mode (`LEKIWI_DISARM_ON_FAILURE=true`, and always in
simulation) also latches faults. A ROS topic alone is not evidence of a real
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

## Reduced hardware base acceptance

For an explicitly authorized, attended test inside a 30 cm radius, source
`scripts/setup.bash` and run `python3 scripts/test-navigation.py`. The script
temporarily replaces the production compute stack with `bounded_base_test:=true`,
performs a finite route, and restores the service on exit. The driver caps speed
from the tracked production MPPI limits and stops at a 20 cm wheel-odometry radius; the runner
cancels at 18 cm. This leaves stopping margin, but wheel slip still requires
independent observation of physical position. All measured arm/base inputs and
a test-client lease renewed within 250 ms are required. Production startup keeps
this mode disabled; its validated acceptance record governs base permission.
The folded `travel_stow` in SRDF and `safety_production.yaml` is the required
navigation posture; starting a base goal does not automatically move the arm.

`config/safety_acceptance.yaml` records an **attended autonomous base** scope:
0 kg added payload, dry concrete, and an operator continuously next to the physical
motor-power stop. The installed hardware record marks the bumper, IMU, and
battery monitor absent. Their fault tests are `null` (not applicable), and the
validator checks that this record agrees with the production profile's
`require_bumper`, `require_imu`, and `require_battery` settings. All other fault
tests and six-direction stopping trials remain mandatory. The operating
condition is a site procedure; software cannot verify that the operator is
present. The 2026-10-04 physical evidence is reviewed and the record is
`validated: true`, at 0.03 m/s and 0.06 rad/s with 20 mm measurement uncertainty
inside the unchanged 50 mm stopping budget. See
[the verification report](physical-verification-20261003.md). It grants no
unattended scope or additional payload.

The configured hold mode automatically re-arms after a fault or restart only
when telemetry and arm permission recover. The acceptance tests for host and
ROS restarts must show immediate command stop, then re-arm only after those
inputs are healthy. A new captured arm stow invalidates the old acceptance.
Tested linear and angular speeds must cover the tracked MPPI limits. The manual
driver uses those same limits, so slow commissioning trials cannot approve a
faster production or manual command.

Before base trials, physically verify a compact arm stow, record its measured
joints with `scripts/capture_stow.py`, and check that the full arm and cable
envelope fits the accepted footprint. Predeclare stopping limits, measure at
least 30 trials per direction on the recorded surface and payload, and update the tracked Nav2
StopZone to cover the worst distance plus measurement uncertainty. A software
pass alone cannot establish obstacle coverage or stopping performance.

The current LD06 produces a 360-degree scan. Its measured body mask covers
100 degrees, leaving 260 degrees (4.54 rad) of effective coverage. The
production supervisor requires 4.3 rad (about 246 degrees), leaving roughly
14 degrees for scan-span variation. This is a reduced, attended operating
scope: masked directions provide no obstacle sensing, and an operator must
remain at the physical motor-power stop. Remeasure the mask after the arm is
stowed and before acceptance trials because nearby hardware can change the
self-returns. Full surrounding coverage requires moving the lidar or adding a
second sensor.

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
