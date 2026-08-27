#!/usr/bin/env python3
"""Read the LeKiwi gripper servo registers without commanding it.

The motor host must be stopped first. This program refuses a serial port held
by another process, opens only servo ID 6, reads diagnostic registers, and
closes the port with ``disable_torque=False``. It contains no write operation.

Run it with the LeRobot virtual environment, for example:

    "$HOME/lekiwi_ws/.venv-lerobot/bin/python" scripts/gripper-diagnose.py
"""

import argparse
import glob
import json
import os
import subprocess
import sys


REGISTERS = (
    "Torque_Enable",
    "Lock",
    "Operating_Mode",
    "Goal_Position",
    "Present_Position",
    "Present_Velocity",
    "Present_Load",
    "Present_Current",
    "Present_Voltage",
    "Present_Temperature",
    "Status",
    "Moving",
)


def default_port():
    matches = sorted(glob.glob("/dev/serial/by-id/*USB_Single_Serial*"))
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one motor serial device; pass --port explicitly "
            f"(found: {', '.join(matches) or 'none'})"
        )
    return matches[0]


def require_unowned(port):
    """Reject a shared bus rather than competing with a motor host."""
    result = subprocess.run(["fuser", "-s", port], check=False)
    if result.returncode == 0:
        raise RuntimeError(f"refusing to read {port}: another process owns the motor bus")
    if result.returncode != 1:
        raise RuntimeError(f"could not determine whether {port} is in use (fuser={result.returncode})")


def read_registers(port):
    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech.feetech import FeetechMotorsBus
    except ImportError as error:
        raise RuntimeError(
            "LeRobot is unavailable; run this script with the LeRobot virtual-environment Python"
        ) from error

    bus = FeetechMotorsBus(
        port=port,
        motors={"arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)},
    )
    try:
        # The handshake only pings the configured servo and reads its firmware.
        bus.connect(handshake=True)
        values = {}
        errors = {}
        for register in REGISTERS:
            try:
                values[register] = bus.read(register, "arm_gripper", normalize=False, num_retry=2)
            except Exception as error:  # report every readable register, not only the first failure
                errors[register] = str(error)
        return {"port": os.path.realpath(port), "servo_id": 6, "values": values, "errors": errors}
    finally:
        if bus.is_connected:
            # This diagnostic must not change torque state. The stopped host may
            # deliberately have released it; the host service owns restoration.
            bus.disconnect(disable_torque=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="motor serial path (default: detected /dev/serial/by-id device)")
    args = parser.parse_args()
    port = args.port or default_port()
    if not os.path.exists(port):
        raise RuntimeError(f"motor serial device does not exist: {port}")
    require_unowned(os.path.realpath(port))
    print(json.dumps(read_registers(port), indent=2, sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        print(f"gripper diagnosis failed: {error}", file=sys.stderr)
        raise SystemExit(1)
