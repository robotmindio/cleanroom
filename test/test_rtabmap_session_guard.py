import importlib.util
from pathlib import Path


ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location(
    "rtabmap_session_guard", ROOT / "scripts" / "rtabmap-session-guard.py"
)
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


def test_database_size_includes_sqlite_sidecars(tmp_path):
    database = tmp_path / "map.db"
    database.write_bytes(b"a" * 10)
    Path(f"{database}-wal").write_bytes(b"b" * 7)
    Path(f"{database}-shm").write_bytes(b"c" * 3)
    assert guard.database_size(database) == 20


def test_missing_database_has_zero_size(tmp_path):
    assert guard.database_size(tmp_path / "missing.db") == 0


def test_database_size_tolerates_sidecar_disappearing_during_stat(tmp_path, monkeypatch):
    database = tmp_path / "map.db"
    database.write_bytes(b"a" * 10)
    sidecar = Path(f"{database}-wal")
    sidecar.write_bytes(b"b" * 7)
    original_stat = Path.stat

    def flaky_stat(path, *args, **kwargs):
        if path == sidecar:
            raise FileNotFoundError(path)
        return original_stat(path, *args, **kwargs)

    monkeypatch.setattr(Path, "stat", flaky_stat)
    assert guard.database_size(database) == 10


def test_monitor_stops_on_database_quota_without_waiting_for_duration(tmp_path):
    database = tmp_path / "map.db"
    database.write_bytes(b"x" * 10)
    now = iter((0.0, 0.1))
    assert guard.monitor_database(
        database, maximum_bytes=10, maximum_seconds=100, poll_seconds=1,
        clock=lambda: next(now), sleep=lambda _seconds: None,
    ) == guard.QUOTA_EXIT


def test_monitor_stops_on_duration_even_before_database_exists(tmp_path):
    now = iter((0.0, 2.0))
    assert guard.monitor_database(
        tmp_path / "not-created.db", maximum_bytes=100, maximum_seconds=1, poll_seconds=1,
        clock=lambda: next(now), sleep=lambda _seconds: None,
    ) == guard.QUOTA_EXIT


def test_monitor_can_be_stopped_cleanly(tmp_path):
    assert guard.monitor_database(
        tmp_path / "not-created.db", maximum_bytes=100, maximum_seconds=100, poll_seconds=1,
        clock=lambda: 0.0, sleep=lambda _seconds: None, should_stop=lambda: True,
    ) == 0
