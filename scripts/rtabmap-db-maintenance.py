#!/usr/bin/env python3
"""Snapshot an RTAB-Map database without replacing it."""

from __future__ import annotations

from contextlib import closing
import hashlib
import json
import sqlite3
import sys
from pathlib import Path


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


def main(arguments: list[str]) -> int:
    if len(arguments) != 3 or arguments[0] != "--backup":
        print("usage: rtabmap-db-maintenance.py --backup SOURCE DESTINATION", file=sys.stderr)
        return 2
    try:
        print(json.dumps(backup_database(Path(arguments[1]), Path(arguments[2])), indent=2))
    except (OSError, sqlite3.Error, ValueError) as error:
        print(f"RTAB-Map database backup failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
