#!/usr/bin/env python3
"""Record one multimodal episode as an MCAP bag plus a manifest; never commands the robot.

The bag is the canonical source: exports (for example LeRobot) are regenerated
from it. Requested commands, the exact applied host action and measured state
are separate topics, and nothing is interpolated or dropped silently.
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

# Requested motion, applied host action and measured state stay distinguishable.
STATE_TOPICS = [
    '/joint_states', '/arm/raw_joint_states', '/hardware/applied_action',
    '/cmd_vel_safe', '/wheel/odometry', '/odom', '/tf', '/tf_static',
    '/safety/state', '/safety/arm_motion_permitted', '/safety/base_motion_permitted',
    '/safety/arm_stowed', '/episode/intervention',
]
CAMERA_TOPICS = {
    'front': ['/camera/front/image_raw', '/camera/front/camera_info'],
    'wrist': ['/camera/wrist/image_raw', '/camera/wrist/camera_info'],
    'astra': ['/camera/astra/color/image_raw', '/camera/astra/color/camera_info',
              '/camera/astra/depth/image_raw'],
}


def digest(path):
    if not path.is_file():
        return None
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, document):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    os.replace(temporary, path)


def provenance(repository):
    revision = subprocess.run(['git', '-C', str(repository), 'rev-parse', 'HEAD'],
                              capture_output=True, text=True, check=False).stdout.strip()
    home = Path.home()
    return {
        'source_revision': revision or None,
        'deployed_revision': (home / '.ros/lekiwi/deployed-revision').read_text().strip()
        if (home / '.ros/lekiwi/deployed-revision').is_file() else None,
        'arm_calibration_sha256': digest(home / '.ros/lekiwi_arm_calibration.json'),
        'urdf_sha256': digest(repository / 'urdf/lekiwi.urdf.xacro'),
        'srdf_sha256': digest(repository / 'config/lekiwi.srdf'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='new episode directory')
    parser.add_argument('--task', required=True, help='natural-language task the episode attempts')
    parser.add_argument('--cameras', default='front,wrist,astra')
    parser.add_argument('--duration', type=float, help='stop after this many seconds')
    parser.add_argument('--operator', default='', help='who supervised or intervened, if anyone')
    args = parser.parse_args()
    cameras = [name for name in args.cameras.split(',') if name]
    if any(name not in CAMERA_TOPICS for name in cameras) or len(set(cameras)) != len(cameras):
        parser.error(f'cameras must be distinct names from {sorted(CAMERA_TOPICS)}')
    if args.duration is not None and not 0 < args.duration <= 3600:
        parser.error('duration must be in (0, 3600] seconds')
    if not args.task.strip() or len(args.task) > 1000:
        parser.error('task must be 1..1000 characters')
    output = Path(args.output).expanduser()
    output.mkdir(parents=True, exist_ok=False)
    topics = STATE_TOPICS + [topic for name in cameras for topic in CAMERA_TOPICS[name]]
    manifest = {
        'schema': 1, 'kind': 'lekiwi_episode', 'status': 'recording', 'task': args.task.strip(),
        'operator': args.operator, 'cameras': cameras, 'topics': topics,
        'started_at': datetime.datetime.now(datetime.UTC).isoformat(),
        'provenance': provenance(Path(__file__).resolve().parents[1]),
        'action_semantics': {
            '/hardware/applied_action': 'exact host action after permission, limit and boundary gating; host units',
            '/cmd_vel_safe': 'requested base twist after the collision monitor; not what the wheels received',
            '/joint_states': 'measured arm joints in URDF radians',
            '/episode/intervention': 'std_msgs/String marks published by a supervisor during the episode',
        },
        'interpolation': 'none; consumers must align by header stamps and report gaps',
    }
    write(output / 'manifest.json', manifest)
    bag = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(output / 'bag'),
                            '--topics', *topics], start_new_session=True)
    stop_reason = 'duration'

    def interrupt(_signal, _frame):
        nonlocal stop_reason
        stop_reason = 'operator_stop'
        os.killpg(bag.pid, signal.SIGINT)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    deadline = None if args.duration is None else time.monotonic() + args.duration
    while bag.poll() is None:
        if deadline is not None and time.monotonic() >= deadline:
            os.killpg(bag.pid, signal.SIGINT)
            break
        time.sleep(.1)
    returncode = bag.wait()
    files = sorted((output / 'bag').glob('*')) if (output / 'bag').is_dir() else []
    complete = returncode == 0 and any(path.suffix == '.mcap' for path in files)
    manifest.update({
        'status': 'complete' if complete else 'failed',
        'stopped_at': datetime.datetime.now(datetime.UTC).isoformat(),
        'stop_reason': stop_reason if complete else f'recorder_exit_{returncode}',
        'files': {path.name: {'sha256': digest(path), 'bytes': path.stat().st_size} for path in files},
    })
    write(output / 'manifest.json', manifest)
    print(json.dumps({'status': manifest['status'], 'episode': str(output)}))
    sys.exit(0 if complete else 1)


if __name__ == '__main__':
    main()
