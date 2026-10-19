#!/usr/bin/env python3
"""Export recorded episodes (scripts/record-episode.py) to a local LeRobot dataset.

Runs in the isolated LeRobot environment (scripts/install-lerobot-export.sh); it
reads MCAP directly and needs no ROS installation. Each fixed-rate tick takes
the latest message of every stream; a tick where any stream is older than the
tolerance is dropped and counted, never interpolated.
"""

import argparse
import bisect
import hashlib
import json
from pathlib import Path

import numpy as np

ARM_JOINTS = ('arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
              'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper')


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


def read_streams(directory, camera, action_topic):
    """Header-stamped samples per stream, sorted; publish order is not trusted."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    wanted = {'/joint_states': 'state', action_topic: 'action',
              f'/camera/{camera}/image_raw': 'image', '/episode/intervention': 'intervention'}
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


def frames(streams, fps, tolerance):
    """Aligned frames plus the count of dropped ticks by missing stream."""
    for required in ('state', 'action', 'image'):
        if not streams[required]:
            raise ValueError(f'episode has no {required} samples')
    times = {kind: [item[0] for item in samples] for kind, samples in streams.items()}
    start = max(times[kind][0] for kind in ('state', 'action', 'image'))
    end = min(times[kind][-1] for kind in ('state', 'action', 'image'))
    action_names = list(streams['action'][0][1].name)
    marks = times['intervention']
    dropped = {'state': 0, 'action': 0, 'image': 0}
    output = []
    for index in range(int((end - start) * fps) + 1):
        tick = start + index / fps
        sample = {kind: latest(streams[kind], times[kind], tick, tolerance) for kind in dropped}
        missing = [kind for kind, value in sample.items() if value is None]
        if missing:
            for kind in missing:
                dropped[kind] += 1
            continue
        joints = dict(zip(sample['state'].name, sample['state'].position))
        if list(sample['action'].name) != action_names:
            raise ValueError('applied action keys changed within the episode')
        output.append({
            'observation.state': np.array([joints[name] for name in ARM_JOINTS], np.float32),
            'action': np.array(sample['action'].position, np.float32),
            'image': pixels(sample['image']),
            'intervened': np.array([bisect.bisect_right(marks, tick) > 0], np.float32),
        })
    return output, action_names, dropped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+')
    parser.add_argument('--root', required=True, help='new local dataset directory')
    parser.add_argument('--repo-id', default='local/lekiwi')
    parser.add_argument('--camera', default='front')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--action-topic', default='/hardware/applied_command',
                        help='applied_command (ROS units, replayable) or applied_action (raw host packet)')
    parser.add_argument('--tolerance', type=float, default=.15, help='maximum stream age per tick, seconds')
    args = parser.parse_args()
    if not 1 <= args.fps <= 60 or not 0 < args.tolerance <= 1:
        parser.error('fps must be 1..60 and tolerance (0, 1] s')
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset, report = None, {'schema': 1, 'fps': args.fps, 'tolerance_sec': args.tolerance,
                             'camera': args.camera, 'action_topic': args.action_topic,
                             'interpolation': 'none', 'episodes': []}
    for directory in map(Path, args.episodes):
        manifest = verified_episode(directory)
        exported, action_names, dropped = frames(read_streams(directory, args.camera, args.action_topic), args.fps, args.tolerance)
        if not exported:
            raise ValueError(f'{directory}: no tick had every stream within tolerance')
        if dataset is None:
            height, width, _ = exported[0]['image'].shape
            dataset = LeRobotDataset.create(
                repo_id=args.repo_id, fps=args.fps, root=args.root, robot_type='lekiwi', use_videos=True,
                features={
                    'observation.state': {'dtype': 'float32', 'shape': (6,), 'names': list(ARM_JOINTS)},
                    'action': {'dtype': 'float32', 'shape': (len(action_names),), 'names': action_names},
                    f'observation.images.{args.camera}': {'dtype': 'video', 'shape': (height, width, 3),
                                                          'names': ['height', 'width', 'channels']},
                    'intervened': {'dtype': 'float32', 'shape': (1,), 'names': None},
                })
            report['action_names'] = action_names
        elif action_names != report['action_names']:
            raise ValueError(f'{directory}: applied action keys differ from earlier episodes')
        for frame in exported:
            image = frame.pop('image')
            dataset.add_frame({**frame, f'observation.images.{args.camera}': image, 'task': manifest['task']})
        dataset.save_episode()
        report['episodes'].append({'source': str(directory), 'frames': len(exported), 'dropped_ticks': dropped,
                                   'task': manifest['task'], 'provenance': manifest['provenance'],
                                   'manifest_sha256': hashlib.sha256((directory / 'manifest.json').read_bytes()).hexdigest()})
    dataset.finalize()
    Path(args.root, 'export-report.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'root': args.root, 'episodes': len(report['episodes']),
                      'frames': sum(item['frames'] for item in report['episodes'])}))


if __name__ == '__main__':
    main()
