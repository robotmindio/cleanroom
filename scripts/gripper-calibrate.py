#!/usr/bin/env python3
"""Record and apply a LeKiwi gripper-only calibration.

This utility changes only servo ID 6's two hardware position limits and its
matching LeRobot calibration cache.  It deliberately requires a person to
place the free gripper at the open and closed endpoints: driving an unknown
gripper into its mechanical end stops is neither a safe nor a reliable way to
calibrate it.

Before using it, support the arm, clear the jaws, and stop
``lekiwi-host.service``.  The tool refuses a serial port already owned by a
process.  It disables torque only on the gripper while endpoints are recorded,
then restores the prior torque state before it closes the port.

Run with the LeRobot virtual environment:

    "$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/gripper-calibrate.py --apply
"""

import argparse
import glob
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


GRIPPER = "arm_gripper"
SERVO_ID = 6
MINIMUM_SPAN_TICKS = 100


def default_port():
    matches = sorted(glob.glob("/dev/serial/by-id/*USB_Single_Serial*"))
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one motor serial device; pass --port explicitly "
            f"(found: {', '.join(matches) or 'none'})"
        )
    return matches[0]


def default_calibration_file():
    cache_root = os.environ.get("HF_LEROBOT_HOME")
    if not cache_root:
        cache_root = os.path.join(os.environ.get("HF_HOME", os.path.expanduser("~/.cache/huggingface")), "lerobot")
    return Path(cache_root) / "calibration" / "robots" / "lekiwi" / "lekiwi_1.json"


def require_unowned(port):
    result = subprocess.run(["fuser", "-s", port], check=False)
    if result.returncode == 0:
        raise RuntimeError(f"refusing to calibrate {port}: another process owns the motor bus")
    if result.returncode != 1:
        raise RuntimeError(f"could not determine whether {port} is in use (fuser={result.returncode})")


def load_calibration(path):
    try:
        calibration = json.loads(path.read_text())
        gripper = calibration[GRIPPER]
        if int(gripper["id"]) != SERVO_ID:
            raise ValueError(f"{GRIPPER} must use ID {SERVO_ID}")
        for name in ("homing_offset", "range_min", "range_max", "drive_mode"):
            int(gripper[name])
        return calibration
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(f"invalid LeRobot calibration {path}: {error}") from error


def confirm(prompt):
    try:
        input(prompt)
    except EOFError as error:
        raise RuntimeError("interactive input is required; run this from a terminal") from error


def read_position(bus):
    return int(bus.read("Present_Position", GRIPPER, normalize=False, num_retry=2))


def atomic_write_calibration(path, calibration):
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(calibration, indent=2, sort_keys=True) + "\n"
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as handle:
        handle.write(payload)
        temporary = Path(handle.name)
    try:
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def calibrate(port, calibration_path):
    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech.feetech import FeetechMotorsBus
    except ImportError as error:
        raise RuntimeError(
            "LeRobot is unavailable; run this script with the LeRobot virtual-environment Python"
        ) from error

    calibration = load_calibration(calibration_path)
    cached = calibration[GRIPPER]
    bus = FeetechMotorsBus(
        port=port,
        motors={GRIPPER: Motor(SERVO_ID, "sts3215", MotorNormMode.RANGE_0_100)},
    )
    original_torque = None
    original_lock = None
    initial_goal = None
    try:
        bus.connect(handshake=True)
        status = int(bus.read("Status", GRIPPER, normalize=False, num_retry=2))
        operating_mode = int(bus.read("Operating_Mode", GRIPPER, normalize=False, num_retry=2))
        if status != 0:
            raise RuntimeError(f"servo reports Status={status}; correct the fault before calibration")
        if operating_mode != 0:
            raise RuntimeError(
                f"servo is in Operating_Mode={operating_mode}, not position mode; do not calibrate it"
            )
        original_torque = int(bus.read("Torque_Enable", GRIPPER, normalize=False, num_retry=2))
        original_lock = int(bus.read("Lock", GRIPPER, normalize=False, num_retry=2))
        initial_goal = int(bus.read("Goal_Position", GRIPPER, normalize=False, num_retry=2))
        homing_offset = int(bus.read("Homing_Offset", GRIPPER, normalize=False, num_retry=2))

        print(
            "Only the gripper torque will now be released. Do not force the arm, "
            "and keep fingers and objects clear of the jaws."
        )
        bus.disable_torque(GRIPPER, num_retry=2)
        confirm("Place the jaws at their fully OPEN mechanical endpoint, without forcing them, then press Enter: ")
        open_position = read_position(bus)
        confirm("Place the jaws at their fully CLOSED mechanical endpoint, without forcing them, then press Enter: ")
        closed_position = read_position(bus)
        lower, upper = sorted((open_position, closed_position))
        span = upper - lower
        if span < MINIMUM_SPAN_TICKS:
            raise RuntimeError(
                f"recorded gripper span is only {span} ticks; expected at least "
                f"{MINIMUM_SPAN_TICKS}. Nothing was written."
            )

        # Range_0_100 maps a zero command to range_min.  Preserve the physical
        # endpoint ordering in the servo limits and flip just the LeRobot mapping
        # when the mechanically open endpoint is numerically higher.
        cached["homing_offset"] = homing_offset
        cached["range_min"] = lower
        cached["range_max"] = upper
        cached["drive_mode"] = 0 if open_position == lower else 1

        backup = calibration_path.with_name(
            f"{calibration_path.name}.before-gripper-{time.strftime('%Y%m%d-%H%M%S')}.bak"
        )
        backup.write_text(calibration_path.read_text())
        # These non-volatile limit registers may only be changed while torque
        # is disabled.  Both values were manually demonstrated reachable above.
        bus.write("Min_Position_Limit", GRIPPER, lower, normalize=False, num_retry=2)
        bus.write("Max_Position_Limit", GRIPPER, upper, normalize=False, num_retry=2)
        atomic_write_calibration(calibration_path, calibration)
        final_position = read_position(bus)
        return {
            "backup": str(backup),
            "calibration": str(calibration_path),
            "closed_position": closed_position,
            "drive_mode": cached["drive_mode"],
            "final_position": final_position,
            "homing_offset": homing_offset,
            "open_position": open_position,
            "range_max": upper,
            "range_min": lower,
            "span_ticks": span,
        }
    finally:
        if bus.is_connected:
            try:
                # Restoring the current measured target avoids a jump when the
                # gripper is re-energized. The managed host configures it again
                # after this tool exits.
                if original_torque:
                    position = read_position(bus)
                    bus.write("Goal_Position", GRIPPER, position, normalize=False, num_retry=2)
                    bus.write("Torque_Enable", GRIPPER, original_torque, normalize=False, num_retry=2)
                    bus.write("Lock", GRIPPER, original_lock, normalize=False, num_retry=2)
                elif initial_goal is not None:
                    bus.write("Goal_Position", GRIPPER, initial_goal, normalize=False, num_retry=2)
            finally:
                bus.disconnect(disable_torque=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="perform the guided calibration")
    parser.add_argument("--port", help="motor serial path (default: detected /dev/serial/by-id device)")
    parser.add_argument(
        "--calibration-file", type=Path, default=default_calibration_file(),
        help="LeRobot calibration JSON to update",
    )
    args = parser.parse_args()
    if not args.apply:
        raise RuntimeError("refusing to change hardware without --apply")
    if not sys.stdin.isatty():
        raise RuntimeError("interactive input is required; run this from a terminal")
    port = args.port or default_port()
    if not os.path.exists(port):
        raise RuntimeError(f"motor serial device does not exist: {port}")
    require_unowned(os.path.realpath(port))
    result = calibrate(os.path.realpath(port), args.calibration_file.expanduser())
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        print(f"gripper calibration failed: {error}", file=sys.stderr)
        raise SystemExit(1)
