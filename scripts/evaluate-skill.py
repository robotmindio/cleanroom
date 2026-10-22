#!/usr/bin/env python3
"""Evaluate a registered skill over N trials, judged by observation evidence rather than the runner.

Each trial: optionally run a reset command, observe the scene, run the skill once
through /robot/skills/run, wait for the scene to settle, observe again. The runner's
outcome is recorded but never decides success: a trial succeeds only when the
object observation store (/robot/observations/query) shows the success predicate
held in every observation of the after-window and did not already hold before the
run. The evidence of each trial is pinned in the store, and the report carries the
success rate with a Wilson score interval; it passes when the interval's lower
bound reaches the required rate. scripts/register-skill.py promotes only on a pass.

The skill runner must admit the skill: an experimental one needs the runner started
for skill acquisition (see docs/skill-acquisition.md).
"""

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import secrets
import shlex
import statistics
import subprocess
import sys
import threading
import time

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))

PREDICATE_FIELDS = {'label', 'present', 'min_score', 'camera_id', 'region', 'min_observations'}


def wilson(successes, total, confidence=.95):
    """Wilson score interval for a binomial proportion."""
    if not total:
        return None
    z = statistics.NormalDist().inv_cdf(1 - (1 - confidence) / 2)
    rate = successes / total
    denominator = 1 + z * z / total
    centre = (rate + z * z / (2 * total)) / denominator
    margin = z * math.sqrt(rate * (1 - rate) / total + z * z / (4 * total * total)) / denominator
    return [max(0.0, centre - margin), min(1.0, centre + margin)]


def success_predicate(text):
    """What must be observed after a successful run, stated by the evaluator, never by the skill."""
    value = json.loads(text)
    if not isinstance(value, dict) or set(value) - PREDICATE_FIELDS or not isinstance(value.get('label'), str) \
            or not value['label']:
        raise ValueError(f'success predicate needs a label and only {sorted(PREDICATE_FIELDS)}')
    predicate = {'label': value['label'], 'present': value.get('present', True), 'min_score': value.get('min_score', .5),
                 'camera_id': value.get('camera_id'), 'region': value.get('region'),
                 'min_observations': value.get('min_observations', 3)}
    if type(predicate['present']) is not bool or type(predicate['min_score']) not in (int, float) \
            or not 0 <= predicate['min_score'] <= 1:
        raise ValueError('present must be boolean and min_score in [0, 1]')
    if type(predicate['min_observations']) is not int or not 1 <= predicate['min_observations'] <= 20:
        raise ValueError('min_observations must be an integer in 1..20')
    region = predicate['region']
    if region is not None and (not isinstance(region, list) or len(region) != 4
                               or any(type(item) not in (int, float) for item in region)
                               or not 0 <= region[0] < region[2] <= 1 or not 0 <= region[1] < region[3] <= 1):
        raise ValueError('region is [x0, y0, x1, y1] in normalized image coordinates')
    return predicate


def matching(observation, predicate):
    """Detections that satisfy the predicate's label, score and optional image region."""
    found = []
    for detection in observation.get('detections', []):
        if detection.get('label') != predicate['label'] or detection.get('score', 0) < predicate['min_score']:
            continue
        if predicate['region'] is not None:
            x, y, w, h = detection['bbox']
            cx, cy = (x + w / 2) / observation['width'], (y + h / 2) / observation['height']
            x0, y0, x1, y1 = predicate['region']
            if not (x0 <= cx <= x1 and y0 <= cy <= y1):
                continue
        found.append(detection)
    return found


def judge(observations, predicate):
    """Holds only when enough observations exist and every one of them satisfies the predicate."""
    evidence = []
    for observation in observations:
        found = matching(observation, predicate)
        evidence.append({'observation_id': observation['id'], 'camera_id': observation['camera_id'],
                         'stamp': observation['stamp'], 'evidence_sha256': observation['evidence']['sha256'],
                         'matching_detections': found, 'satisfied': bool(found) == predicate['present']})
    holds = len(evidence) >= predicate['min_observations'] and all(item['satisfied'] for item in evidence)
    return holds, evidence


def stamp_seconds(observation):
    return observation['stamp']['sec'] + observation['stamp']['nanosec'] * 1e-9


class Evaluator:
    """Drives one skill through the runner and reads the observation store with one ROS node."""

    def __init__(self, node, skill, predicate, *, observe_sec, settle_sec, timeout_sec):
        from rclpy.action import ActionClient
        from lekiwi_rmf.action import RunSkill
        from lekiwi_rmf.srv import PinObservationEvidence, QueryObservations

        self.node, self.skill, self.predicate = node, skill, predicate
        self.observe_sec, self.settle_sec, self.timeout_sec = observe_sec, settle_sec, timeout_sec
        self.RunSkill = RunSkill
        self.run_client = ActionClient(node, RunSkill, '/robot/skills/run')
        self.query_client = node.create_client(QueryObservations, '/robot/observations/query')
        self.pin_client = node.create_client(PinObservationEvidence, '/robot/observations/pin')
        self.Query, self.Pin = QueryObservations, PinObservationEvidence
        self.epoch, self.cursor = None, 0

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _wait(self, future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{what} did not answer within {timeout:.0f} s')
            time.sleep(.02)
        return future.result()

    def ready(self, timeout=10.0):
        if not self.run_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('/robot/skills/run is not available; is the skill runner running?')
        if not self.query_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('/robot/observations/query is not available; the verifier needs object perception')
        self._query({'limit': 1})  # establishes the store epoch and skips history

    def _query(self, request):
        if self.epoch is not None and 'after_sequence' in request:
            request['epoch'] = self.epoch
        response = self._wait(self.query_client.call_async(self.Query.Request(request_json=json.dumps(request))),
                              10.0, 'observation query')
        if not response.success:
            raise RuntimeError(f'observation query failed: {response.detail}')
        result = json.loads(response.response_json)
        if self.epoch is None:
            self.epoch, self.cursor = result['epoch'], result['latest_sequence']
        elif result['epoch'] != self.epoch:
            raise RuntimeError('the observation store restarted during the evaluation')
        return result

    def observe(self, start, end):
        """Observations captured in [start, end] on the node clock, reading the store as it grows."""
        observations = []
        while True:
            finished = self.now() >= end + .5  # captures stamped inside the window may be stored late
            while True:
                result = self._query({'after_sequence': self.cursor, 'limit': 100})
                if result.get('cursor_gap'):
                    raise RuntimeError('observations were evicted before the verifier read them')
                self.cursor = result['next_sequence']
                observations.extend(item for item in result['observations'] if start <= stamp_seconds(item) <= end and (
                    self.predicate['camera_id'] is None or item['camera_id'] == self.predicate['camera_id']))
                if not result['observations']:
                    break
            if finished:
                return observations[-20:]
            time.sleep(.1)

    def run_skill(self, parameters):
        goal = self.RunSkill.Goal(skill_id=self.skill.skill_id, parameters_json=json.dumps(parameters, sort_keys=True))
        handle = self._wait(self.run_client.send_goal_async(goal), 10.0, 'skill runner')
        if not handle.accepted:
            return {'outcome': 'rejected', 'detail': 'the runner rejected the goal (unknown, experimental, busy or '
                                                     'invalid parameters)', 'result': None}
        result = self._wait(handle.get_result_async(), self.skill.max_duration_sec + self.timeout_sec, 'skill run')
        try:
            report = json.loads(result.result.result_json)
        except ValueError:
            report = None
        return {'outcome': result.result.outcome, 'detail': result.result.detail, 'result': report}

    def pin(self, request_id, identifiers):
        if not identifiers:
            return False
        if not self.pin_client.wait_for_service(timeout_sec=2.0):
            return False
        response = self._wait(self.pin_client.call_async(self.Pin.Request(request_json=json.dumps(
            {'request_id': request_id, 'observation_ids': identifiers}))), 10.0, 'evidence pin')
        return bool(response.success)

    def trial(self, index, parameters, evaluation_id):
        started = self.now()
        before_holds, before = judge(self.observe(started, started + self.observe_sec), self.predicate)
        trial = {'trial': index, 'started_sec': started, 'before': before}
        if len(before) < self.predicate['min_observations']:
            return {**trial, 'verdict': 'invalid', 'reason': 'too few observations before the run; the verifier is blind'}
        if before_holds:
            return {**trial, 'verdict': 'invalid', 'reason': 'the success predicate already held before the run'}
        run = self.run_skill(parameters)
        trial['runner'] = run
        if run['outcome'] == 'rejected':
            return {**trial, 'verdict': 'invalid', 'reason': run['detail']}
        if run['result'] is not None and run['result'].get('artifact_sha256') != self.skill.artifact_sha256:
            return {**trial, 'verdict': 'invalid', 'reason': 'the runner executed a different checkpoint than registered'}
        ended = self.now()
        after_holds, after = judge(self.observe(ended + self.settle_sec, ended + self.settle_sec + self.observe_sec),
                                   self.predicate)
        trial['after'] = after
        identifiers = list(dict.fromkeys([item['observation_id'] for item in before[-10:] + after[-10:]]))
        trial['evidence_pinned'] = self.pin(f'{evaluation_id}-{index}', identifiers)
        if len(after) < self.predicate['min_observations']:
            return {**trial, 'verdict': 'failure', 'reason': 'too few observations after the run'}
        return {**trial, 'verdict': 'success' if after_holds else 'failure',
                'reason': 'predicate held in every observation after the run' if after_holds
                else 'predicate did not hold in every observation after the run'}


def evaluate(evaluator, args, evaluation_id, reset=None, log=print):
    trials = []
    for index in range(1, args.trials + 1):
        if reset is not None:
            reset(index)
        trial = evaluator.trial(index, args.parameters, evaluation_id)
        trials.append(trial)
        log(json.dumps({'trial': index, 'verdict': trial['verdict'], 'reason': trial['reason']}))
    successes = sum(trial['verdict'] == 'success' for trial in trials)
    interval = wilson(successes, len(trials), args.confidence)
    skill = evaluator.skill
    return {
        'schema': 1, 'kind': 'lekiwi_skill_evaluation', 'evaluation_id': evaluation_id,
        'created_utc': datetime.datetime.now(datetime.UTC).isoformat(timespec='seconds'),
        'skill_id': skill.skill_id, 'version': skill.version, 'artifact_sha256': skill.artifact_sha256,
        'status_at_evaluation': skill.status, 'parameters': args.parameters, 'success_predicate': evaluator.predicate,
        'verifier': {'source': '/robot/observations/query', 'independent_of_runner': True,
                     'observe_sec': evaluator.observe_sec, 'settle_sec': evaluator.settle_sec},
        'reset_command': args.reset_command, 'trials': trials, 'n': len(trials), 'successes': successes,
        # Invalid trials (blind verifier, goal already met, rejected) count against the skill, never for it.
        'invalid': sum(trial['verdict'] == 'invalid' for trial in trials),
        'success_rate': successes / len(trials), 'confidence': args.confidence, 'wilson_interval': interval,
        'required_lower_bound': args.required_lower_bound, 'min_trials': args.min_trials,
        'passed': len(trials) >= args.min_trials and interval[0] >= args.required_lower_bound,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('skill_id')
    parser.add_argument('--success', required=True, type=success_predicate,
                        help='JSON: {"label": ..., "present": true, "min_score": .5, "camera_id": null, '
                             '"region": [x0, y0, x1, y1], "min_observations": 3}')
    parser.add_argument('--output', type=Path, required=True, help='report path')
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--parameters', default='{}', type=json.loads, help='skill parameters JSON')
    parser.add_argument('--observe-sec', type=float, default=3.0, help='observation window before and after a run')
    parser.add_argument('--settle-sec', type=float, default=1.0, help='wait after a run before observing')
    parser.add_argument('--timeout-sec', type=float, default=30.0, help='runner slack beyond max_duration_sec')
    parser.add_argument('--reset-command', default=None, help='command that resets the scene before each trial')
    parser.add_argument('--confidence', type=float, default=.95)
    parser.add_argument('--required-lower-bound', type=float, default=.8)
    parser.add_argument('--min-trials', type=int, default=20)
    parser.add_argument('--registry', default=None, help='registry file (default: the installed config)')
    parser.add_argument('--artifacts', default=None)
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.trials <= 1000 or not 1 <= args.min_trials or not 0 < args.confidence < 1 \
            or not 0 <= args.required_lower_bound <= 1 or args.observe_sec <= 0 or args.settle_sec < 0:
        parser.error('invalid trial count, confidence, bound or window')
    if not isinstance(args.parameters, dict):
        parser.error('parameters must be a JSON object')
    if args.output.exists():
        parser.error(f'{args.output} exists; reports are never overwritten')

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from lekiwi_rmf.skills import load_registry

    try:
        skill = load_registry(args.registry, args.artifacts)[args.skill_id]
    except KeyError:
        parser.error(f'{args.skill_id} is not registered')
    reset = None
    if args.reset_command:
        command = shlex.split(args.reset_command)

        def reset(index):
            subprocess.run(command, check=True, timeout=300, env={**os.environ, 'LEKIWI_EVALUATION_TRIAL': str(index)})
    rclpy.init()
    node = Node('skill_evaluator', parameter_overrides=[Parameter('use_sim_time', value=args.use_sim_time)])
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    evaluation_id = f'skill-eval-{datetime.datetime.now(datetime.UTC):%Y%m%d%H%M%S}-{secrets.token_hex(3)}'
    try:
        evaluator = Evaluator(node, skill, args.success, observe_sec=args.observe_sec, settle_sec=args.settle_sec,
                              timeout_sec=args.timeout_sec)
        evaluator.ready()
        report = evaluate(evaluator, args, evaluation_id, reset, log=lambda line: print(line, file=sys.stderr))
    except (RuntimeError, subprocess.SubprocessError) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    temporary = args.output.with_name(f'.{args.output.name}.tmp')
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    os.replace(temporary, args.output)
    print(json.dumps({key: report[key] for key in ('skill_id', 'version', 'n', 'successes', 'success_rate',
                                                     'wilson_interval', 'passed')}))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
