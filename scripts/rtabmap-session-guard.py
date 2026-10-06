#!/usr/bin/env python3
"""Stop a mapping session from growing its RTAB-Map database past its quota.

This standalone utility reports ``QUOTA_EXIT`` when a limit is reached. Normal
bringup uses robot_explorer's persistent ROS monitor, which also sees mode
changes after localization startup and switches RTAB-Map to localization.
"""

from __future__ import annotations

import argparse
import math
import signal
import time
from pathlib import Path

from lekiwi_rmf.exploration import database_size


QUOTA_EXIT = 75
_stop = False


def _request_stop(_signum, _frame) -> None:
    global _stop
    _stop = True


def monitor_database(
    database: Path,
    maximum_bytes: int,
    maximum_seconds: float,
    poll_seconds: float,
    *,
    clock=time.monotonic,
    sleep=time.sleep,
    should_stop=lambda: _stop,
) -> int:
    """Monitor one database, isolated from CLI/signal state for testing."""
    started = clock()
    while not should_stop():
        size = database_size(database)
        elapsed = clock() - started
        if size >= maximum_bytes:
            print(
                f"RTAB-Map mapping quota reached: {size} bytes >= {maximum_bytes}; "
                "freezing the map",
                flush=True,
            )
            return QUOTA_EXIT
        if elapsed >= maximum_seconds:
            print(
                f"RTAB-Map mapping duration reached: {elapsed:.1f}s >= {maximum_seconds:.1f}s; "
                "freezing the map",
                flush=True,
            )
            return QUOTA_EXIT
        sleep(poll_seconds)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("database", type=Path)
    parser.add_argument("--maximum-bytes", type=int, default=512 * 1024 * 1024)
    parser.add_argument("--maximum-seconds", type=float, default=4 * 60 * 60)
    parser.add_argument("--poll-seconds", type=float, default=2.0)
    args = parser.parse_args()
    if (
        args.maximum_bytes <= 0
        or not math.isfinite(args.maximum_seconds)
        or args.maximum_seconds <= 0.0
        or not math.isfinite(args.poll_seconds)
        or args.poll_seconds <= 0.0
    ):
        parser.error("all quota values must be finite and positive")
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    database = args.database.expanduser()
    return monitor_database(
        database, args.maximum_bytes, args.maximum_seconds, args.poll_seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())
