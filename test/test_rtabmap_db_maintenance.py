"""Checks for the bounded, non-destructive RTAB-Map startup policy."""

import importlib.util
import os
from pathlib import Path


_PATH = Path(__file__).parents[1] / "scripts" / "rtabmap-db-maintenance.py"
_SPEC = importlib.util.spec_from_file_location("rtabmap_db_maintenance", _PATH)
maintenance = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(maintenance)


def database(home):
    path = home / ".ros" / maintenance.DEFAULT_DATABASE_NAME
    path.parent.mkdir(parents=True)
    return path


def test_oversized_active_database_and_sidecars_are_preserved(tmp_path):
    path = database(tmp_path)
    with path.open("wb") as handle:
        handle.truncate(512 * 1024 * 1024 + 1)
    wal = Path(f"{path}-wal")
    shm = Path(f"{path}-shm")
    wal.write_bytes(b"wal")
    shm.write_bytes(b"shm")

    actions = maintenance.maintain([], home=tmp_path, now=1_700_000_000)

    assert path.exists()
    assert path.stat().st_size == 512 * 1024 * 1024 + 1
    assert wal.read_bytes() == b"wal"
    assert shm.read_bytes() == b"shm"
    assert actions == []


def test_explicit_oversized_database_is_preserved(tmp_path):
    path = database(tmp_path)
    with path.open("wb") as handle:
        handle.truncate(512 * 1024 * 1024 + 1)

    actions = maintenance.maintain([f"rtabmap_database:={path}"], home=tmp_path, now=1_700_000_000)

    assert path.exists()
    assert actions == []


def test_pruning_keeps_a_small_newest_window_and_paired_sidecars(tmp_path):
    path = database(tmp_path)
    now = 2_000_000_000
    created = []
    for index in range(5):
        archive = Path(f"{path}.stale-20330518-0{index}0000")
        archive.write_bytes(b"x" * 10)
        Path(f"{archive}-wal").write_bytes(b"wal")
        mtime = now - (5 - index) * 60
        os.utime(archive, (mtime, mtime))
        created.append(archive)

    removed = maintenance.prune_archives(path, now=now)

    assert removed == created[:2]
    assert all(not item.exists() and not Path(f"{item}-wal").exists() for item in created[:2])
    assert all(item.exists() for item in created[2:])


def test_pruning_removes_old_archives_even_before_the_count_limit(tmp_path):
    path = database(tmp_path)
    archive = Path(f"{path}.corrupt-20300101-000000")
    archive.write_bytes(b"evidence")
    old = 2_000_000_000 - maintenance.ARCHIVE_RETENTION_SECONDS - 1
    os.utime(archive, (old, old))

    removed = maintenance.prune_archives(path, now=2_000_000_000)

    assert removed == [archive]
    assert not archive.exists()


def test_capacity_budget_counts_sqlite_sidecars(tmp_path, monkeypatch):
    path = database(tmp_path)
    monkeypatch.setattr(maintenance, "MAX_ARCHIVES", 3)
    monkeypatch.setattr(maintenance, "MAX_ARCHIVED_BYTES", 12)
    older = Path(f"{path}.stale-20330518-000000")
    newer = Path(f"{path}.stale-20330518-010000")
    for archive in (older, newer):
        archive.write_bytes(b"x" * 8)
    Path(f"{newer}-wal").write_bytes(b"walwal")
    os.utime(older, (2_000_000_000 - 1, 2_000_000_000 - 1))
    os.utime(newer, (2_000_000_000, 2_000_000_000))

    removed = maintenance.prune_archives(path, now=2_000_000_000)

    assert removed == [older]
    assert newer.exists() and Path(f"{newer}-wal").exists()
