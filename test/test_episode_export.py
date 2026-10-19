"""Episode export alignment: fixed-rate ticks, no interpolation, explicit drops."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

source = Path(__file__).parents[1] / 'scripts/export-lerobot.py'
spec = importlib.util.spec_from_file_location('export_lerobot', source)
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)


def joints(value):
    return SimpleNamespace(name=list(export.ARM_JOINTS), position=[value] * 6)


def action(value, names=('arm_gripper.pos', 'x.vel')):
    return SimpleNamespace(name=list(names), position=[value] * len(names))


def image(value, encoding='bgr8'):
    pixels = np.zeros((2, 2, 3), np.uint8)
    pixels[..., 0] = value
    return SimpleNamespace(height=2, width=2, step=6, encoding=encoding, data=pixels.tobytes())


def streams(gap=False):
    times = [0.0, 0.1, 0.2, 0.3, 0.4]
    return {
        'state': [(t, joints(t)) for t in times],
        'action': [(t, action(t)) for t in times],
        'image': [(t, image(int(t * 100))) for t in times if not (gap and 0.15 < t < 0.35)],
        'intervention': [(0.25, None)] if gap else [],
    }


def test_each_tick_uses_the_latest_sample_without_interpolation():
    frames, names, dropped = export.frames(streams(), fps=10, tolerance=.05)
    assert names == ['arm_gripper.pos', 'x.vel'] and dropped == {'state': 0, 'action': 0, 'image': 0}
    assert len(frames) == 5
    assert frames[2]['observation.state'][0] == pytest.approx(.2) and frames[2]['action'][1] == pytest.approx(.2)
    # bgr8 input becomes RGB, so the blue channel value lands last.
    assert frames[1]['image'][0, 0].tolist() == [0, 0, 10]


def test_stale_streams_drop_the_tick_and_are_counted():
    frames, _, dropped = export.frames(streams(gap=True), fps=10, tolerance=.05)
    assert len(frames) == 3 and dropped == {'state': 0, 'action': 0, 'image': 2}
    assert [frame['intervened'][0] for frame in frames] == [0.0, 0.0, 1.0]


def test_changed_action_keys_and_missing_streams_are_rejected():
    data = streams()
    data['action'][3] = (0.3, action(.3, names=('x.vel',)))
    with pytest.raises(ValueError, match='keys changed'):
        export.frames(data, fps=10, tolerance=.05)
    data = streams()
    data['action'] = []
    with pytest.raises(ValueError, match='no action'):
        export.frames(data, fps=10, tolerance=.05)
