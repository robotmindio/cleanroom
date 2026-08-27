#!/usr/bin/env python3
"""End a mapping launch before its live RTAB-Map database exceeds its quota.

SQLite databases must not be renamed while open. This companion exits with
``QUOTA_EXIT`` when the database plus sidecars reaches the configured size or
the mapping session reaches its duration. The launch file translates that exit
into an orderly stack shutdown, after which startup maintenance may archive the
closed database safely.
"""

from __future__ import annotations

import argparse
import math
import signal
import time
from pathlib import Path


QUOTA_EXIT = 75
SIDECARS = ("-wal", "-shm", "-journal")
_stop = False


def database_size(database: Path) -> int:
    total = 0
    for candidate in (database, *(Path(f"{database}{suffix}") for suffix in SIDECARS)):
        try:
            if candidate.is_file():
                total += candidate.stat().st_size
        except OSError:
            # SQLite sidecars can be created/deleted between is_file() and
            # stat() while RTAB-Map checkpoints.  Re-sample next poll rather
            # than turning a transient race into an unsafe launcher failure.
            continue
    return total


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
                "requesting orderly launch shutdown",
                flush=True,
            )
            return QUOTA_EXIT
        if elapsed >= maximum_seconds:
            print(
                f"RTAB-Map mapping duration reached: {elapsed:.1f}s >= {maximum_seconds:.1f}s; "
                "requesting orderly launch shutdown",
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
