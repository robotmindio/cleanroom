import math
import os
from pathlib import Path
import sys


KEYS = ("camera_height", "camera_pitch", "xy_velocity_scale", "yaw_velocity_scale")


def calibration_path(path=None) -> Path:
    return Path(path if path is not None else os.environ.get(
        "LEKIWI_LAUNCH_CALIBRATION", "~/.ros/lekiwi_launch_calibration.conf"
    )).expanduser()


def _validated_value(key, value):
    if (key not in KEYS or isinstance(value, bool) or not math.isfinite(value)
            or (key != "camera_pitch" and value <= 0)):
        raise ValueError(f"invalid launch calibration {key}={value!r}")
    return value


def load_launch_calibration(path=None) -> dict[str, float]:
    path = calibration_path(path)
    try:
        lines = path.read_text().splitlines()
    except FileNotFoundError:
        return {}
    values = {}
    for number, line in enumerate(lines, 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        key = key.strip()
        try:
            if not separator or key in values:
                raise ValueError("expected a unique KEY=VALUE")
            values[key] = _validated_value(key, float(value))
        except ValueError as error:
            raise ValueError(f"{path}:{number}: {error}") from error
    return values


def save_launch_calibration(**values: float) -> Path:
    values = {key: _validated_value(key, value) for key, value in values.items()}
    path = calibration_path()
    saved = load_launch_calibration(path)
    saved.update(values)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_text("".join(f"{key}={saved[key]:.6f}\n" for key in KEYS if key in saved))
    os.replace(temporary, path)
    return path


if __name__ == "__main__":
    # Explicit launch arguments take precedence over saved measurements.
    overrides = {argument.partition(":=")[0] for argument in sys.argv[1:]}
    try:
        for key, value in load_launch_calibration().items():
            if key not in overrides:
                print(f"{key}:={value:.6f}")
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
