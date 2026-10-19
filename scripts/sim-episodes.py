#!/usr/bin/env python3
"""Generate scripted demonstration episodes in a Gazebo task scene, labeled as simulation data.

Needs the simulation started for skill acquisition (`scripts/sim-up.sh skill_acquisition:=true`),
which starts the scene bridges. For each episode: reset the scene for its trial index
(lekiwi_rmf/sim_scene.py: arm to its start state, objects to the trial's seeded poses, then
confirm the whole initial state from ground truth), record with scripts/record-episode.py
`--domain sim`, and execute the joint-waypoint program as one trajectory through
/arm_controller/follow_joint_trajectory, the guarded path every arm motion uses. A reset that
is not confirmed skips that trial; it never becomes a demonstration from an unknown start.

The program is data, not a skill: {"waypoints": [{"positions": {"arm_shoulder_pan": 0.1, ...},
"time_from_start_sec": 2.0}, ...]}. Each episode manifest stores the scene, the trial's reset
record, the program digest and the ground truth after execution; scripts/export-lerobot.py
keeps the domain in its report.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))
from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS  # noqa: E402
from lekiwi_rmf.sim_scene import load_scene  # noqa: E402


def load_program(path):
    document = json.loads(Path(path).read_text())
    waypoints = document.get('waypoints') if isinstance(document, dict) and set(document) == {'waypoints'} else None
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 100:
        raise ValueError('a program is {"waypoints": [1..100 waypoints]}')
    previous = 0.0
    for waypoint in waypoints:
        if not isinstance(waypoint, dict) or set(waypoint) != {'positions', 'time_from_start_sec'}:
            raise ValueError('each waypoint is {positions, time_from_start_sec}')
        moment = waypoint['time_from_start_sec']
        if type(moment) not in (int, float) or not previous < moment <= 120:
            raise ValueError('waypoint times must increase within 120 s')
        previous = moment
        positions = waypoint['positions']
        if not isinstance(positions, dict) or set(positions) != set(waypoints[0]['positions']) \
                or not set(positions) <= set(ARM_JOINTS) or not positions:
            raise ValueError('every waypoint sets the same arm joints')
        for name, value in positions.items():
            lower, upper = JOINT_LIMITS[name]
            if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'{name} target is outside its joint limits')
    return document, hashlib.sha256(Path(path).read_bytes()).hexdigest()


def execute(node, client, program, timeout):
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    names = list(program['waypoints'][0]['positions'])
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    for waypoint in program['waypoints']:
        point = JointTrajectoryPoint(positions=[float(waypoint['positions'][name]) for name in names])
        seconds = waypoint['time_from_start_sec']
        point.time_from_start.sec, point.time_from_start.nanosec = int(seconds), int(seconds % 1 * 1e9)
        goal.trajectory.points.append(point)

    def wait(future, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{what} did not answer within {timeout:.0f} s')
            time.sleep(.02)
        return future.result()
    handle = wait(client.send_goal_async(goal), 'the arm controller')
    if not handle.accepted:
        return {'accepted': False, 'error_code': None, 'detail': 'the arm controller rejected the program'}
    result = wait(handle.get_result_async(), 'the program trajectory').result
    return {'accepted': True, 'error_code': result.error_code, 'detail': result.error_string}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scene', required=True, help='config/sim_scenes.yaml scene')
    parser.add_argument('--program', type=Path, required=True, help='joint-waypoint program JSON')
    parser.add_argument('--task', required=True, help='natural-language task the episodes demonstrate')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--first-trial', type=int, default=1, help='trial index of the first episode (its reset seed)')
    parser.add_argument('--output', type=Path, required=True, help='new directory for the episodes')
    parser.add_argument('--cameras', default='front')
    parser.add_argument('--margin-sec', type=float, default=2.0, help='recording before and after the program')
    args = parser.parse_args()
    if not 1 <= args.episodes <= 1000 or args.first_trial < 1 or not 0 <= args.margin_sec <= 30:
        parser.error('episodes 1..1000, first trial from 1, margin 0..30 s')
    try:
        scene, scene_sha256 = load_scene(args.scene)
        program, program_sha256 = load_program(args.program)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.output.mkdir(parents=True, exist_ok=False)

    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from lekiwi_rmf.sim_scene import SceneController

    rclpy.init()
    node = Node('sim_episodes', parameter_overrides=[Parameter('use_sim_time', value=True)])
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    summary = {'scene': args.scene, 'scene_sha256': scene_sha256, 'program_sha256': program_sha256, 'episodes': []}
    try:
        controller = SceneController(node, scene)
        controller.ready()
        arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        duration = program['waypoints'][-1]['time_from_start_sec']
        for trial in range(args.first_trial, args.first_trial + args.episodes):
            reset = controller.reset(trial)
            if not reset['confirmed']:
                summary['episodes'].append({'trial': trial, 'recorded': False, 'reason': reset['reasons']})
                print(json.dumps({'trial': trial, 'skipped': reset['reasons']}), file=sys.stderr)
                continue
            directory = args.output / f'trial-{trial:04d}'
            context = {'kind': 'sim_scripted_episode', 'scene': args.scene, 'scene_sha256': scene_sha256,
                       'trial': trial, 'reset': reset, 'program_sha256': program_sha256, 'method': 'scripted',
                       'intervened': False}
            with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as stream:
                json.dump(context, stream)
            recorder = subprocess.Popen([sys.executable, str(REPOSITORY / 'scripts/record-episode.py'), str(directory),
                                         '--task', args.task, '--cameras', args.cameras, '--domain', 'sim',
                                         '--context', stream.name, '--operator', 'scripted:sim-episodes',
                                         '--duration', str(duration + 2 * args.margin_sec)])
            time.sleep(args.margin_sec)
            executed = execute(node, arm, program, duration + 30)
            code = recorder.wait(timeout=duration + 2 * args.margin_sec + 60)
            os.unlink(stream.name)
            after, robot, joints, _ = controller.ground_truth()
            summary['episodes'].append({'trial': trial, 'recorded': code == 0, 'episode': str(directory),
                                        'execution': executed, 'ground_truth_after': after})
            print(json.dumps({'trial': trial, 'recorded': code == 0, 'execution': executed}), file=sys.stderr)
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    finally:
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    recorded = sum(item['recorded'] for item in summary['episodes'])
    print(json.dumps({'output': str(args.output), 'recorded': recorded, 'skipped': len(summary['episodes']) - recorded}))
    return 0 if recorded else 1


if __name__ == '__main__':
    sys.exit(main())
