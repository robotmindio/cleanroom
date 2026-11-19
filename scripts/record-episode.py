#!/usr/bin/env python3
"""Record one multimodal episode as an MCAP bag plus a manifest; never commands the robot.

The bag is the canonical source: exports (for example LeRobot) are regenerated
from it. Requested commands, the exact applied host action and measured state
are separate topics, and nothing is interpolated or dropped silently.

On a terminal, type `i` and Enter to mark where a supervisor intervention starts and
again where it ends (published on /episode/intervention and recorded in the bag), and
answer the outcome question when the recording stops. `--outcome` answers it in advance;
`--label OUTCOME` sets the outcome of an episode that is already recorded. Only episodes
labelled `success` are exported for training by default.
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.skills import camera_sources  # noqa: E402

# Requested motion, applied host action and measured state stay distinguishable.
STATE_TOPICS = [
    '/joint_states', '/arm/raw_joint_states', '/hardware/applied_action', '/hardware/applied_command',
    '/cmd_vel_safe', '/wheel/odometry', '/odom', '/tf', '/tf_static',
    '/safety/state', '/safety/arm_motion_permitted', '/safety/base_motion_permitted',
    '/safety/arm_stowed', '/episode/intervention',
]
OUTCOMES = ('success', 'failure')


def camera_topics():
    """Image, calibration and depth topics of every tracked camera identity."""
    return {name: [source[key] for key in ('image_topic', 'camera_info_topic', 'depth_topic') if source.get(key)]
            for name, source in camera_sources().items()}


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


def label(directory, outcome, operator):
    manifest = json.loads((directory / 'manifest.json').read_text())
    if manifest.get('kind') != 'lekiwi_episode' or manifest.get('status') != 'complete':
        raise ValueError(f'{directory}: only a complete LeKiwi recording can be labelled')
    manifest.update({'outcome': outcome, 'outcome_by': operator})
    write(directory / 'manifest.json', manifest)


def intervention_marker():
    """Publishes the supervisor's alternating start/end marks; the bag records them with the episode."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    # The recorder's own handlers stop the bag; rclpy must not take the signals.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    publisher = rclpy.create_node('episode_recorder').create_publisher(String, '/episode/intervention', 10)
    marks = []

    def mark():
        marks.append('end' if len(marks) % 2 else 'start')
        publisher.publish(String(data=marks[-1]))
        print(f'intervention {marks[-1]}', file=sys.stderr)
    return mark


def asked_outcome():
    try:
        while True:
            answer = input(f'outcome [{"/".join(OUTCOMES)}] (empty leaves the episode unlabeled): ').strip()
            if answer in OUTCOMES or not answer:
                return answer or None
    except (EOFError, KeyboardInterrupt):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output', help='new episode directory')
    parser.add_argument('--task', help='natural-language task the episode attempts')
    parser.add_argument('--parameters', default='{}', help='JSON object of the skill parameter values this '
                        'demonstration was performed with; exported into the policy task text')
    parser.add_argument('--outcome', choices=OUTCOMES, help='whether the episode achieved the task; asked when '
                        'the recording stops on a terminal, otherwise the episode stays unlabeled')
    parser.add_argument('--label', choices=OUTCOMES, help='set the outcome of an already recorded episode and exit')
    parser.add_argument('--cameras', default=','.join(camera_topics()))
    parser.add_argument('--duration', type=float, help='stop after this many seconds')
    parser.add_argument('--operator', default='', help='who supervised or intervened, if anyone')
    parser.add_argument('--domain', choices=['physical', 'sim'], default='physical',
                        help='sim marks an episode recorded in simulation; exports and reports keep the domain')
    parser.add_argument('--context', type=Path, help='JSON describing how the episode was set up (for example a '
                        'scripts/sim-episodes.py scene reset), stored in the manifest')
    args = parser.parse_args()
    output = Path(args.output).expanduser()
    if args.label is not None:
        try:
            label(output, args.label, args.operator)
        except (OSError, ValueError) as error:
            parser.error(str(error))
        print(json.dumps({'episode': str(output), 'outcome': args.label}))
        return
    if args.task is None:
        parser.error('recording requires --task')
    try:
        parameters = json.loads(args.parameters)
    except ValueError:
        parameters = None
    if not isinstance(parameters, dict) or any(type(value) not in (bool, int, float, str) for value in parameters.values()):
        parser.error('parameters must be a JSON object of numbers, booleans and strings')
    sources = camera_topics()
    context = None
    if args.context is not None:
        if args.context.stat().st_size > 256 * 1024:
            parser.error('context exceeds 256 KiB')
        context = json.loads(args.context.read_text())
    cameras = [name for name in args.cameras.split(',') if name]
    if any(name not in sources for name in cameras) or len(set(cameras)) != len(cameras):
        parser.error(f'cameras must be distinct names from {sorted(sources)}')
    if args.duration is not None and not 0 < args.duration <= 3600:
        parser.error('duration must be in (0, 3600] seconds')
    if not args.task.strip() or len(args.task) > 1000:
        parser.error('task must be 1..1000 characters')
    # Marks need a keyboard; an unattended recording has no supervisor to intervene.
    interactive = sys.stdin.isatty()
    mark = intervention_marker() if interactive else None
    output.mkdir(parents=True, exist_ok=False)
    topics = STATE_TOPICS + [topic for name in cameras for topic in sources[name]]
    manifest = {
        'schema': 1, 'kind': 'lekiwi_episode', 'status': 'recording', 'task': args.task.strip(),
        'parameters': parameters, 'outcome': None,
        'operator': args.operator, 'domain': args.domain, 'context': context, 'cameras': cameras, 'topics': topics,
        'started_at': datetime.datetime.now(datetime.UTC).isoformat(),
        'provenance': provenance(Path(__file__).resolve().parents[1]),
        'action_semantics': {
            '/hardware/applied_action': 'exact host packet after gating, host units; during a host trajectory its arm keys are the superseded hold',
            '/hardware/applied_command': 'applied command in ROS units: arm URDF radians (trajectory sample when one runs) and base m/s, rad/s',
            '/cmd_vel_safe': 'requested base twist after the collision monitor; not what the wheels received',
            '/joint_states': 'measured arm joints in URDF radians',
            '/episode/intervention': 'std_msgs/String `start` and `end` marks around each span a supervisor intervened',
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
    if interactive:
        print('recording: `i` + Enter marks an intervention start, then its end; Ctrl-C stops', file=sys.stderr)
    while bag.poll() is None:
        if deadline is not None and time.monotonic() >= deadline:
            os.killpg(bag.pid, signal.SIGINT)
            break
        if not interactive:
            time.sleep(.1)
        elif select.select([sys.stdin], [], [], .1)[0]:
            line = sys.stdin.readline()
            interactive = bool(line)  # end of input: stop reading marks
            if line.strip() == 'i':
                mark()
    returncode = bag.wait()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    files = sorted((output / 'bag').glob('*')) if (output / 'bag').is_dir() else []
    complete = returncode == 0 and any(path.suffix == '.mcap' for path in files)
    outcome = args.outcome
    if outcome is None and complete and sys.stdin.isatty():
        outcome = asked_outcome()
    manifest.update({
        'outcome': outcome, 'outcome_by': args.operator if outcome else None,
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
