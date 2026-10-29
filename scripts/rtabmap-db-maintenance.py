#!/usr/bin/env python3
"""Prune old RTAB-Map archives without replacing the active map database."""

from __future__ import annotations

from contextlib import closing
import fcntl
import hashlib
import json
import re
import sqlite3
import sys
import time
from pathlib import Path
from typing import Iterable


DEFAULT_DATABASE_NAME = "lekiwi_rtabmap_floor_v2.db"
MAX_ARCHIVES = 3
MAX_ARCHIVED_BYTES = 1536 * 1024 * 1024
ARCHIVE_RETENTION_SECONDS = 14 * 24 * 60 * 60
SIDECARS = ("-wal", "-shm", "-journal")


def backup_database(source: Path, destination: Path) -> dict:
    """Snapshot committed SQLite data, including WAL, without replacing either database."""
    source = source.expanduser().resolve(strict=True)
    if not source.is_file():
        raise ValueError(f"not a database file: {source}")
    destination = destination.expanduser()
    destination.mkdir(mode=0o700, parents=True, exist_ok=False)
    snapshot = destination / "rtabmap.db"
    with closing(sqlite3.connect(source.as_uri() + "?mode=ro", uri=True)) as reader:
        with closing(sqlite3.connect(snapshot)) as writer:
            reader.backup(writer)
            if writer.execute("PRAGMA integrity_check").fetchall() != [("ok",)]:
                raise ValueError("RTAB-Map snapshot failed its integrity check")
    snapshot.chmod(0o600)
    with snapshot.open("rb") as stream:
        sha256 = hashlib.file_digest(stream, "sha256").hexdigest()
    manifest = {"schema": "lekiwi.rtabmap_backup/1", "source": str(source),
                "file": snapshot.name, "size": snapshot.stat().st_size, "sha256": sha256,
                "integrity": "ok"}
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    manifest_path.chmod(0o600)
    return manifest


def launch_database(arguments: Iterable[str], home: Path) -> Path:
    """Return the selected RTAB-Map database path."""
    database = home / ".ros" / DEFAULT_DATABASE_NAME
    for argument in arguments:
        if argument.startswith("rtabmap_database:="):
            database = Path(argument.split(":=", 1)[1]).expanduser()
    return database


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


def maintain(arguments: Iterable[str], home: Path | None = None, now: float | None = None) -> list[str]:
    """Prune expired or excess old archives; leave the active database untouched."""
    home = Path.home() if home is None else home
    database = launch_database(arguments, home)
    database.parent.mkdir(parents=True, exist_ok=True)
    lock_path = database.parent / f".{DEFAULT_DATABASE_NAME}.maintenance.lock"
    messages: list[str] = []
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        for archive in prune_archives(database, now):
            messages.append(f"RTAB-Map: removed expired/excess automatic archive {archive}")
    return messages


def main(arguments: list[str]) -> int:
    try:
        if "--backup" in arguments:
            if len(arguments) != 3 or arguments[0] != "--backup":
                print("usage: rtabmap-db-maintenance.py --backup SOURCE DESTINATION", file=sys.stderr)
                return 2
            print(json.dumps(backup_database(Path(arguments[1]), Path(arguments[2])), indent=2))
            return 0
        prune_only = arguments == ["--prune-only"]
        if "--prune-only" in arguments and not prune_only:
            print("--prune-only cannot be combined with ROS launch arguments", file=sys.stderr)
            return 2
        for message in maintain([] if prune_only else arguments):
            print(message)
    except (OSError, sqlite3.Error, ValueError) as error:
        print(f"RTAB-Map database maintenance failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
