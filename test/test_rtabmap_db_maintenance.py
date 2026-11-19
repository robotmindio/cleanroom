"""Checks for the non-destructive RTAB-Map database snapshot."""

import importlib.util
import hashlib
import json
from pathlib import Path
import sqlite3

import pytest


_PATH = Path(__file__).parents[1] / "scripts" / "rtabmap-db-maintenance.py"
_SPEC = importlib.util.spec_from_file_location("rtabmap_db_maintenance", _PATH)
maintenance = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(maintenance)


def test_backup_includes_committed_wal_and_preserves_the_source(tmp_path):
    source = tmp_path / "map.db"
    with sqlite3.connect(source) as writer:
        writer.execute("PRAGMA journal_mode=WAL")
        writer.execute("PRAGMA wal_autocheckpoint=0")
        writer.execute("CREATE TABLE Node (id INTEGER)")
        writer.execute("INSERT INTO Node VALUES (1)")
        writer.commit()
        destination = tmp_path / "backup"
        result = maintenance.backup_database(source, destination)
        writer.execute("INSERT INTO Node VALUES (2)")
        writer.commit()
        assert writer.execute("SELECT id FROM Node").fetchall() == [(1,), (2,)]
    snapshot = destination / result["file"]
    with sqlite3.connect(snapshot) as reader:
        assert reader.execute("SELECT id FROM Node").fetchall() == [(1,)]
    assert json.loads((destination / "manifest.json").read_text()) == result
    assert result["sha256"] == hashlib.sha256(snapshot.read_bytes()).hexdigest()
    assert result["size"] == snapshot.stat().st_size
    assert snapshot.stat().st_mode & 0o777 == 0o600
    assert destination.stat().st_mode & 0o777 == 0o700
    with pytest.raises(FileExistsError):
        maintenance.backup_database(source, destination)
    assert snapshot.read_bytes() and result["sha256"] == hashlib.sha256(snapshot.read_bytes()).hexdigest()


def test_backup_rejects_invalid_database_without_modifying_it(tmp_path):
    source = tmp_path / "invalid.db"
    source.write_bytes(b"not SQLite")
    destination = tmp_path / "incomplete"
    with pytest.raises(sqlite3.DatabaseError):
        maintenance.backup_database(source, destination)
    assert source.read_bytes() == b"not SQLite"
    assert not (destination / "manifest.json").exists()
    assert maintenance.main(["--backup", str(source)]) == 2
