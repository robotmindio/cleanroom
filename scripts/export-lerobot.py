#!/usr/bin/env python3
"""Export recorded episodes (scripts/record-episode.py) to a local LeRobot dataset.

Runs in the isolated LeRobot environment (scripts/install-lerobot-export.sh); it
reads MCAP directly and needs no ROS installation. Each fixed-rate tick takes
the latest message of every stream; a tick where any stream is older than the
tolerance is dropped and counted, never interpolated.

Only episodes the recorder labelled `success` are exported unless `--outcomes` says
otherwise. The action is the arm command; an episode that also commanded the base is
refused unless `--base-actions` exports the base commands too, which the skill runner
refuses to execute. Every frame's `task` is lekiwi_rmf.skills.conditioning of the
episode's task and parameters, the same text the runner gives the policy.
"""

import argparse
import bisect
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.arm_trajectory import ARM_JOINTS  # noqa: E402
from lekiwi_rmf.skills import BASE_COMMANDS, camera_sources, conditioning  # noqa: E402

# Base commands in ROS units (/hardware/applied_command) and in the host packet (/hardware/applied_action).
BASE_ACTIONS = frozenset({*BASE_COMMANDS, 'x.vel', 'y.vel', 'theta.vel'})
OUTCOMES = ('success', 'failure', 'unlabeled')


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def verified_episode(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    if manifest.get('kind') != 'lekiwi_episode' or manifest.get('status') != 'complete':
        raise ValueError(f'{directory}: episode is not a complete LeKiwi recording')
    for name, expected in manifest['files'].items():
        with (directory / 'bag' / name).open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != expected['sha256']:
                raise ValueError(f'{directory}: {name} changed after recording')
    return manifest


def read_streams(directory, cameras, action_topic):
    """Header-stamped samples per stream, sorted; publish order is not trusted."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    wanted = {'/joint_states': 'state', action_topic: 'action', '/episode/intervention': 'intervention',
              **{topic: f'observation.images.{name}' for name, topic in cameras.items()}}
    streams = {name: [] for name in wanted.values()}
    for path in sorted((directory / 'bag').glob('*.mcap')):
        with path.open('rb') as stream:
            for _schema, channel, record, message in make_reader(
                    stream, decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=list(wanted)):
                kind = wanted[channel.topic]
                when = record.log_time * 1e-9 if kind == 'intervention' else stamp(message)
                streams[kind].append((when, message))
    for samples in streams.values():
        samples.sort(key=lambda item: item[0])
    return streams


def intervention_intervals(marks):
    """[start, end] spans from the supervisor's `start` and `end` marks; one left open lasts to the end."""
    intervals, opened = [], None
    for when, message in marks:
        mark = message.data.strip()
        if mark not in ('start', 'end') or (mark == 'start') != (opened is None):
            raise ValueError(f'intervention mark {mark!r} at {when:.3f} s is not an alternating start/end')
        if mark == 'start':
            opened = when
        else:
            intervals.append((opened, when))
            opened = None
    if opened is not None:
        intervals.append((opened, float('inf')))
    return intervals


def latest(samples, times, tick, tolerance):
    index = bisect.bisect_right(times, tick) - 1
    if index < 0 or tick - samples[index][0] > tolerance:
        return None
    return samples[index][1]


def pixels(message):
    image = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    image = image[:, :message.width * 3].reshape(message.height, message.width, 3)
    if message.encoding == 'bgr8':
        return image[..., ::-1].copy()
    if message.encoding != 'rgb8':
        raise ValueError(f'unsupported image encoding {message.encoding}')
    return image.copy()


def frames(streams, fps, tolerance, base_actions=False):
    """Aligned frames, the exported action names and the count of dropped ticks by missing stream."""
    required = [kind for kind in streams if kind != 'intervention']
    for kind in required:
        if not streams[kind]:
            raise ValueError(f'episode has no {kind} samples')
    times = {kind: [item[0] for item in samples] for kind, samples in streams.items()}
    start = max(times[kind][0] for kind in required)
    end = min(times[kind][-1] for kind in required)
    recorded = list(streams['action'][0][1].name)
    kept = [index for index, name in enumerate(recorded) if base_actions or name not in BASE_ACTIONS]
    base = [index for index, name in enumerate(recorded) if name in BASE_ACTIONS and not base_actions]
    intervals = intervention_intervals(streams['intervention'])
    dropped = dict.fromkeys(required, 0)
    output = []
    for index in range(int((end - start) * fps) + 1):
        tick = start + index / fps
        sample = {kind: latest(streams[kind], times[kind], tick, tolerance) for kind in required}
        missing = [kind for kind, value in sample.items() if value is None]
        if missing:
            for kind in missing:
                dropped[kind] += 1
            continue
        joints = dict(zip(sample['state'].name, sample['state'].position))
        if list(sample['action'].name) != recorded:
            raise ValueError('applied action keys changed within the episode')
        moving = [recorded[item] for item in base if abs(sample['action'].position[item]) > 1e-6]
        if moving:
            # An arm-only policy trained on this episode would imitate half of what was demonstrated.
            raise ValueError(f'the episode commands the base ({", ".join(moving)}); record it with the base still, '
                             'or pass --base-actions for a skill the runner will refuse to execute')
        output.append({
            'observation.state': np.array([joints[name] for name in ARM_JOINTS], np.float32),
            'action': np.array([sample['action'].position[item] for item in kept], np.float32),
            **{kind: pixels(sample[kind]) for kind in required if kind.startswith('observation.images.')},
            'intervened': np.array([any(opened <= tick <= closed for opened, closed in intervals)], np.float32),
        })
    return output, [recorded[item] for item in kept], dropped


def outcome(manifest):
    """The recorder's label; an episode recorded without one is unlabeled, never assumed successful."""
    return manifest.get('outcome') or 'unlabeled'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+')
    parser.add_argument('--root', required=True, help='new local dataset directory')
    parser.add_argument('--repo-id', default='local/lekiwi')
    parser.add_argument('--cameras', default='front', help='comma-separated camera identities '
                        '(config/object_cameras.yaml); each becomes observation.images.<identity>')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--action-topic', default='/hardware/applied_command',
                        help='applied_command (ROS units, replayable) or applied_action (raw host packet)')
    parser.add_argument('--base-actions', action='store_true',
                        help='export the base commands as actions too; the skill runner refuses such a skill')
    parser.add_argument('--outcomes', default='success',
                        help=f'comma-separated episode outcomes to export, from {OUTCOMES}')
    parser.add_argument('--tolerance', type=float, default=.15, help='maximum stream age per tick, seconds')
    args = parser.parse_args()
    if not 1 <= args.fps <= 60 or not 0 < args.tolerance <= 1:
        parser.error('fps must be 1..60 and tolerance (0, 1] s')
    sources, names, outcomes = camera_sources(), args.cameras.split(','), set(args.outcomes.split(','))
    if not names or len(set(names)) != len(names) or set(names) - set(sources):
        parser.error(f'cameras must be distinct names from {sorted(sources)}')
    if not outcomes <= set(OUTCOMES):
        parser.error(f'outcomes must be from {OUTCOMES}')
    cameras = {name: sources[name]['image_topic'] for name in names}
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset, report = None, {'schema': 2, 'fps': args.fps, 'tolerance_sec': args.tolerance,
                             'cameras': names, 'action_topic': args.action_topic,
                             'action_space': 'arm_and_base' if args.base_actions else 'arm',
                             'outcomes': sorted(outcomes), 'interpolation': 'none', 'episodes': [], 'skipped': []}
    for directory in map(Path, args.episodes):
        manifest = verified_episode(directory)
        if outcome(manifest) not in outcomes:
            report['skipped'].append({'source': str(directory), 'outcome': outcome(manifest)})
            continue
        try:
            exported, action_names, dropped = frames(read_streams(directory, cameras, args.action_topic), args.fps,
                                                     args.tolerance, args.base_actions)
        except ValueError as error:
            raise ValueError(f'{directory}: {error}') from error
        if not exported:
            raise ValueError(f'{directory}: no tick had every stream within tolerance')
        if dataset is None:
            dataset = LeRobotDataset.create(
                repo_id=args.repo_id, fps=args.fps, root=args.root, robot_type='lekiwi', use_videos=True,
                features={
                    'observation.state': {'dtype': 'float32', 'shape': (6,), 'names': list(ARM_JOINTS)},
                    'action': {'dtype': 'float32', 'shape': (len(action_names),), 'names': action_names},
                    **{f'observation.images.{name}': {
                        'dtype': 'video', 'shape': exported[0][f'observation.images.{name}'].shape,
                        'names': ['height', 'width', 'channels']} for name in names},
                    'intervened': {'dtype': 'float32', 'shape': (1,), 'names': None},
                })
            report['action_names'] = action_names
        elif action_names != report['action_names']:
            raise ValueError(f'{directory}: applied action keys differ from earlier episodes')
        parameters = manifest.get('parameters') or {}
        task = conditioning(manifest['task'], parameters)
        for frame in exported:
            dataset.add_frame({**frame, 'task': task})
        dataset.save_episode()
        # Episodes recorded before the domain field existed are physical recordings.
        report['episodes'].append({'source': str(directory), 'frames': len(exported), 'dropped_ticks': dropped,
                                   'task': manifest['task'], 'parameters': parameters, 'outcome': outcome(manifest),
                                   'intervened_frames': int(sum(frame['intervened'][0] for frame in exported)),
                                   'provenance': manifest['provenance'],
                                   'domain': manifest.get('domain', 'physical'),
                                   'manifest_sha256': hashlib.sha256((directory / 'manifest.json').read_bytes()).hexdigest()})
    if dataset is None:
        raise ValueError(f'no episode has an outcome in {sorted(outcomes)}; nothing was exported')
    dataset.finalize()
    # Simulated and physical demonstrations stay distinguishable in everything trained from this export.
    report['domains'] = sorted({item['domain'] for item in report['episodes']})
    Path(args.root, 'export-report.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'root': args.root, 'episodes': len(report['episodes']), 'skipped': len(report['skipped']),
                      'frames': sum(item['frames'] for item in report['episodes'])}))


if __name__ == '__main__':
    main()
