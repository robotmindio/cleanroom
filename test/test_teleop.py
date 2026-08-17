import importlib.util
import os
import pathlib

_spec = importlib.util.spec_from_file_location(
    "teleop", pathlib.Path(__file__).parents[1] / "scripts" / "teleop.py"
)
teleop = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(teleop)


class _Pipe:
    """Stands in for sys.stdin: select and os.read both want only a file descriptor."""

    def __init__(self, data=b""):
        # The write end stays open, or an empty pipe would read as EOF instead of
        # as the quiet keyboard it is standing in for.
        self.fd, self.write_fd = os.pipe()
        os.write(self.write_fd, data)

    def fileno(self):
        return self.fd


def _read(data, monkeypatch):
    monkeypatch.setattr(teleop.sys, "stdin", _Pipe(data))
    return teleop.read_key(0.5)


def test_arrow_key_arrives_as_one_token(monkeypatch):
    # The whole escape sequence, not just the escape byte, or up would never drive.
    assert _read(b"\x1b[A", monkeypatch) == "\x1b[A"
    assert teleop.KEYS["\x1b[A"] == (1.0, 0.0, 0.0)


def test_application_cursor_mode_arrows_still_drive(monkeypatch):
    assert _read(b"\x1bOD", monkeypatch) in teleop.KEYS


def test_plain_key_is_unchanged(monkeypatch):
    assert _read(b"1", monkeypatch) == "1"


def test_nothing_to_read_is_none(monkeypatch):
    monkeypatch.setattr(teleop.sys, "stdin", _Pipe())
    assert teleop.read_key(0.01) is None
