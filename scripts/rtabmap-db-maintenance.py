#!/usr/bin/env python3
"""Bound the repository-managed RTAB-Map working database at startup.

The default working database is deliberately disposable: mapping sessions can
grow it without bound, while a deliberate ``rtabmap_database:=...`` argument
is normally a map an operator wants to retain.  This helper is therefore run
only by the repository launchers and never rotates an explicit database.
"""

from __future__ import annotations

import fcntl
import os
import re
import sys
import time
from pathlib import Path
from typing import Iterable


DEFAULT_DATABASE_NAME = "lekiwi_rtabmap.db"
MAX_DATABASE_BYTES = 512 * 1024 * 1024
MAX_ARCHIVES = 3
MAX_ARCHIVED_BYTES = 1536 * 1024 * 1024
ARCHIVE_RETENTION_SECONDS = 14 * 24 * 60 * 60
SIDECARS = ("-wal", "-shm", "-journal")


def launch_database(arguments: Iterable[str], home: Path) -> tuple[Path, bool]:
    """Return the selected RTAB-Map database and whether it was explicit."""
    database = home / ".ros" / DEFAULT_DATABASE_NAME
    explicit = False
    for argument in arguments:
        if argument.startswith("rtabmap_database:="):
            database = Path(argument.split(":=", 1)[1]).expanduser()
            explicit = True
    return database, explicit


def archive_pattern(database: Path) -> re.Pattern[str]:
    # Older launchers used a date-only suffix, so retain it in the bounded
    # cleanup policy too. Sidecars deliberately do not match this expression.
    return re.compile(
        rf"^{re.escape(database.name)}\.(?:stale|corrupt)-\d{{8}}(?:-\d{{6}})?$"
    )


def archives(database: Path) -> list[Path]:
    matcher = archive_pattern(database)
    return sorted(
        (path for path in database.parent.iterdir() if path.is_file() and matcher.match(path.name)),
        key=lambda path: (path.stat().st_mtime, path.name),
    )


def remove_archive(database: Path) -> None:
    """Remove one automatic archive and only the SQLite sidecars paired with it."""
    for candidate in (database, *(Path(f"{database}{suffix}") for suffix in SIDECARS)):
        try:
            candidate.unlink()
        except FileNotFoundError:
            pass


def archive_size(database: Path) -> int:
    """Include SQLite write-ahead sidecars in the archive capacity budget."""
    return sum(
        candidate.stat().st_size
        for candidate in (database, *(Path(f"{database}{suffix}") for suffix in SIDECARS))
        if candidate.is_file()
    )


def prune_archives(database: Path, now: float | None = None) -> list[Path]:
    """Remove expired or excess automatic archives, returning their main paths."""
    now = time.time() if now is None else now
    removed: list[Path] = []
    for archive in archives(database):
        if archive.stat().st_mtime < now - ARCHIVE_RETENTION_SECONDS:
            remove_archive(archive)
            removed.append(archive)

    # Keep the newest bounded diagnostic window. A single large, newest archive
    # is retained so an operator still has the last failed session to inspect.
    kept = 0
    kept_bytes = 0
    for archive in reversed(archives(database)):
        archive_bytes = archive_size(archive)
        if kept == 0 or (kept < MAX_ARCHIVES and kept_bytes + archive_bytes <= MAX_ARCHIVED_BYTES):
            kept += 1
            kept_bytes += archive_bytes
            continue
        remove_archive(archive)
        removed.append(archive)
    return sorted(removed)


def rotate_if_oversized(database: Path, explicit: bool, now: float | None = None) -> Path | None:
    """Archive an oversized default database and return its new archive path."""
    if explicit or not database.is_file() or database.stat().st_size <= MAX_DATABASE_BYTES:
        return None
    stamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime(time.time() if now is None else now))
    archive = Path(f"{database}.stale-{stamp}")
    # The timestamp collision is only possible for parallel launchers. The
    # process-wide maintenance lock prevents it, but do not overwrite evidence
    # if an interrupted manual invocation left the same name behind.
    if archive.exists():
        raise RuntimeError(f"refusing to overwrite RTAB-Map archive {archive}")
    os.replace(database, archive)
    for suffix in SIDECARS:
        sidecar = Path(f"{database}{suffix}")
        if sidecar.exists():
            os.replace(sidecar, Path(f"{archive}{suffix}"))
    return archive


def maintain(
    arguments: Iterable[str], home: Path | None = None, now: float | None = None, rotate: bool = True
) -> list[str]:
    """Rotate/prune the default database and return human-readable actions."""
    home = Path.home() if home is None else home
    database, explicit = launch_database(arguments, home)
    database.parent.mkdir(parents=True, exist_ok=True)
    lock_path = database.parent / f".{DEFAULT_DATABASE_NAME}.maintenance.lock"
    messages: list[str] = []
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        rotated = rotate_if_oversized(database, explicit, now) if rotate else None
        if rotated:
            messages.append(f"RTAB-Map: archived oversized working database to {rotated}")
        for archive in prune_archives(database, now):
            messages.append(f"RTAB-Map: removed expired/excess automatic archive {archive}")
    return messages


def main(arguments: list[str]) -> int:
    try:
        prune_only = arguments == ["--prune-only"]
        if "--prune-only" in arguments and not prune_only:
            print("--prune-only cannot be combined with ROS launch arguments", file=sys.stderr)
            return 2
        for message in maintain([] if prune_only else arguments, rotate=not prune_only):
            print(message)
    except OSError as error:
        print(f"RTAB-Map database maintenance failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
