import importlib.util
import pathlib

_spec = importlib.util.spec_from_file_location(
    "teleop", pathlib.Path(__file__).parents[1] / "scripts" / "teleop.py"
)
teleop = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(teleop)


def test_qwerty_keys_are_the_wasd_block(monkeypatch):
    monkeypatch.setenv("LEKIWI_LAYOUT", "qwerty")
    keys, slower, faster, text = teleop.layout()
    assert keys["w"] == (1.0, 0.0, 0.0)
    assert keys["d"] == (0.0, -1.0, 0.0)
    assert "-" in slower and "+" in faster
    assert "QWERTY" in text


def test_dvorak_keys_are_the_same_physical_block(monkeypatch):
    monkeypatch.setenv("LEKIWI_LAYOUT", "dvorak")
    keys, slower, faster, text = teleop.layout()
    # ,aoe sits where wasd does, and '. where qe does.
    assert keys[","] == (1.0, 0.0, 0.0)
    assert keys["o"] == (-1.0, 0.0, 0.0)
    assert keys["a"] == (0.0, 1.0, 0.0)
    assert keys["e"] == (0.0, -1.0, 0.0)
    assert keys["'"] == (0.0, 0.0, 1.0)
    assert keys["."] == (0.0, 0.0, -1.0)
    assert keys[" "] == (0.0, 0.0, 0.0)
    assert "[" in slower and "]" in faster
    assert "," in text and "Dvorak" in text
