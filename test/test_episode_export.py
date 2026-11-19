"""Episode recording labels and export: outcome filter, fixed-rate ticks, explicit drops, declared action space."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPTS = Path(__file__).parents[1] / 'scripts'
spec = importlib.util.spec_from_file_location('export_lerobot', SCRIPTS / 'export-lerobot.py')
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)

COMMAND = [*export.ARM_JOINTS, 'base_x_vel', 'base_y_vel', 'base_yaw_rate']
FRONT, WRIST = 'observation.images.front', 'observation.images.wrist'


def joints(value):
    return SimpleNamespace(name=list(export.ARM_JOINTS), position=[value] * 6)


def action(value, base=0.0, names=COMMAND):
    return SimpleNamespace(name=list(names), position=[value] * 6 + [base] * (len(names) - 6))


def image(value, encoding='bgr8'):
    pixels = np.zeros((2, 2, 3), np.uint8)
    pixels[..., 0] = value
    return SimpleNamespace(height=2, width=2, step=6, encoding=encoding, data=pixels.tobytes())


def mark(when, data):
    return when, SimpleNamespace(data=data)


def streams(gap=False, base=0.0, marks=()):
    times = [0.0, 0.1, 0.2, 0.3, 0.4]
    return {
        'state': [(t, joints(t)) for t in times],
        'action': [(t, action(t, base)) for t in times],
        FRONT: [(t, image(int(t * 100))) for t in times if not (gap and 0.15 < t < 0.35)],
        WRIST: [(t, image(int(t * 100) + 1)) for t in times],
        'intervention': list(marks),
    }


def test_each_tick_uses_the_latest_sample_of_every_camera_without_interpolation():
    frames, names, dropped = export.frames(streams(), fps=10, tolerance=.05)
    assert dropped == {'state': 0, 'action': 0, FRONT: 0, WRIST: 0} and len(frames) == 5
    assert frames[2]['observation.state'][0] == pytest.approx(.2) and frames[2]['action'][5] == pytest.approx(.2)
    # bgr8 input becomes RGB, so the blue channel value lands last.
    assert frames[1][FRONT][0, 0].tolist() == [0, 0, 10] and frames[1][WRIST][0, 0].tolist() == [0, 0, 11]


def test_the_action_is_the_arm_command_and_a_moving_base_is_refused_unless_exported():
    _, names, _ = export.frames(streams(), fps=10, tolerance=.05)
    assert names == list(export.ARM_JOINTS)
    with pytest.raises(ValueError, match='commands the base'):
        export.frames(streams(base=.1), fps=10, tolerance=.05)
    frames, names, _ = export.frames(streams(base=.1), fps=10, tolerance=.05, base_actions=True)
    assert names == COMMAND and frames[0]['action'].tolist() == pytest.approx([0.0] * 6 + [.1] * 3)


def test_stale_streams_drop_the_tick_and_are_counted():
    frames, _, dropped = export.frames(streams(gap=True), fps=10, tolerance=.05)
    assert len(frames) == 3 and dropped == {'state': 0, 'action': 0, FRONT: 2, WRIST: 0}


def test_intervention_is_the_span_between_a_start_and_an_end_mark():
    marked = streams(marks=[mark(.05, 'start'), mark(.25, 'end')])
    assert [frame['intervened'][0] for frame in export.frames(marked, fps=10, tolerance=.05)[0]] == [0, 1, 1, 0, 0]
    # A span still open when the recording stops lasts to its end.
    still_open = streams(marks=[mark(.05, 'start'), mark(.15, 'end'), mark(.35, 'start')])
    assert [frame['intervened'][0] for frame in export.frames(still_open, fps=10, tolerance=.05)[0]] == [0, 1, 0, 0, 1]
    for bad in ([mark(.1, 'end')], [mark(.1, 'start'), mark(.2, 'start')], [mark(.1, 'takeover')]):
        with pytest.raises(ValueError, match='alternating start/end'):
            export.frames(streams(marks=bad), fps=10, tolerance=.05)


def test_changed_action_keys_and_missing_streams_are_rejected():
    data = streams()
    data['action'][3] = (0.3, action(.3, names=COMMAND[:6]))
    with pytest.raises(ValueError, match='keys changed'):
        export.frames(data, fps=10, tolerance=.05)
    data = streams()
    data['action'] = []
    with pytest.raises(ValueError, match='no action'):
        export.frames(data, fps=10, tolerance=.05)


FAKE_ROS2 = '''#!/usr/bin/env python3
import signal, sys, time
from pathlib import Path
bag = Path(sys.argv[sys.argv.index('-o') + 1])
bag.mkdir()
(bag / 'bag_0.mcap').write_bytes(b'recorded')
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
time.sleep(30)
'''


def record(tmp_path, *arguments):
    """scripts/record-episode.py with a stand-in bag recorder and no terminal."""
    (tmp_path / 'bin').mkdir(exist_ok=True)
    (tmp_path / 'bin/ros2').write_text(FAKE_ROS2)
    (tmp_path / 'bin/ros2').chmod(0o755)
    completed = subprocess.run(
        [sys.executable, str(SCRIPTS / 'record-episode.py'), *map(str, arguments)], stdin=subprocess.DEVNULL,
        capture_output=True, text=True, timeout=60, env={**os.environ, 'PATH': f'{tmp_path / "bin"}:{os.environ["PATH"]}'})
    return completed


def test_the_recorder_labels_the_outcome_and_only_successes_are_exported_by_default(tmp_path):
    episode = tmp_path / 'episode'
    assert record(tmp_path, episode, '--task', 'push the block', '--cameras', 'front,astra_color', '--duration', '.3',
                  '--outcome', 'failure', '--parameters', '{"side": "left"}', '--operator', 'ana').returncode == 0
    manifest = json.loads((episode / 'manifest.json').read_text())
    assert manifest['status'] == 'complete' and manifest['outcome'] == 'failure' and manifest['outcome_by'] == 'ana'
    assert manifest['parameters'] == {'side': 'left'} and '/camera/astra/depth/image_raw' in manifest['topics']
    assert export.outcome(manifest) == 'failure' and export.verified_episode(episode)['outcome'] == 'failure'
    # The label can be corrected after the recording; the bag is untouched.
    assert record(tmp_path, episode, '--label', 'success', '--operator', 'ana').returncode == 0
    assert export.outcome(export.verified_episode(episode)) == 'success'
    unattended = tmp_path / 'unattended'
    assert record(tmp_path, unattended, '--task', 'push the block', '--cameras', 'front', '--duration', '.3').returncode == 0
    assert export.outcome(json.loads((unattended / 'manifest.json').read_text())) == 'unlabeled'
    assert record(tmp_path, tmp_path / 'bad', '--task', 'x', '--cameras', 'astra', '--duration', '.3').returncode == 2
    assert record(tmp_path, tmp_path / 'missing', '--label', 'success').returncode == 2
