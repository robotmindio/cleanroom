import math
import os
from pathlib import Path


KEYS = ("camera_height", "camera_pitch", "xy_velocity_scale", "yaw_velocity_scale")


def save_launch_calibration(**values: float) -> Path:
    if not values.keys() <= set(KEYS) or any(not math.isfinite(value) for value in values.values()):
        raise ValueError("launch calibration values must be known and finite")
    path = Path(os.environ.get(
        "LEKIWI_LAUNCH_CALIBRATION", "~/.ros/lekiwi_launch_calibration.conf"
    )).expanduser()
    saved = {}
    try:
        for line in path.read_text().splitlines():
            key, separator, value = line.strip().partition("=")
            if separator and key in KEYS:
                saved[key] = value
    except FileNotFoundError:
        pass
    saved.update({key: f"{value:.6f}" for key, value in values.items()})
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_text("".join(f"{key}={saved[key]}\n" for key in KEYS if key in saved))
    os.replace(temporary, path)
    return path
