# Safety inputs and motor health

This document records which requested safety functions can be provided by the
current robot hardware and which require an additional physical signal source.
The repository safety supervisor consumes the topics below and default-denies
motion when a required input is absent, stale, or unhealthy. A ROS topic alone
is not evidence of a real safety function.

| Function | Software support exists? | What is required for a real implementation |
| --- | --- | --- |
| Bumper | Yes. The supervisor monitors `safety/bumper_active`. | Install physical contact switches or an equivalent contact sensor, then add a hardware bridge that publishes its actual state. |
| E-stop | Yes. The supervisor monitors `safety/estop_active`, and the driver can request servo torque-off. | Install a hardwired, independently wired emergency stop that removes actuator energy without ROS, the motor host, DDS, or the compute OS. Software torque-off is not an E-stop. |
| Battery | Yes. The supervisor validates `/battery_state`. | Add a BMS or voltage/current monitor that publishes real, stamped battery data. Servo voltage alone is not a qualified state-of-charge source. |
| Motor health | Yes, for bus/servo telemetry. | Bench-validate the electrical and thermal thresholds before they are allowed to stop motion. |
| IMU | Yes. The EKF and safety supervisor consume `/imu/data`. | Add, mount, and calibrate a physical IMU and its ROS driver. |

Motor-health telemetry is the only item that can be materially extended using
the installed robot hardware plus repository software. The other items need a
physical sensor or safety component before they can provide a meaningful safety
measurement. Dummy publishers may be useful for testing, but must never be
treated as safety functionality or used to validate the production profile.

## Current motor-health behavior

The motor host is the only serial-bus owner. At 10 Hz it reads each STS3215's
torque state, position, raw status register, load, voltage, temperature,
current, and programmed minimum/maximum voltage and maximum-temperature limits.
It sends this snapshot in authenticated observation telemetry; the ROS driver
validates it and publishes `/hardware/diagnostics`.

The driver requests a guarded arm automatically at startup. It cannot energize
the servos until it has fresh host telemetry and current safety-supervisor
permission; a failed or missing safety input still leaves it disarmed.

The diagnostics use these units:

| Value | Reported representation |
| --- | --- |
| Voltage | raw register × 0.1 V |
| Current | raw register × 6.5 mA |
| Load | raw value and signed duty-cycle estimate (`raw / 1000`), not physical torque |
| Temperature | internal servo temperature in °C |

The following conditions are `ERROR` and therefore revoke safety permission:

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
