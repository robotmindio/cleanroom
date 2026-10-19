#!/usr/bin/env python3
"""Evaluate a registered skill over N trials, judged by an independent verifier rather than the runner.

Each trial runs the skill once through /robot/skills/run. The runner's outcome is recorded
but never decides success; the verifier does, and the trial is invalid (counted against the
skill, never for it) when the verifier is blind, the goal already held, or the start is not
the declared one. The report carries the success rate with a Wilson score interval and
passes when its lower bound reaches the required rate.

Domains:
- physical (default): the verifier is the object observation store (/robot/observations/query).
  Optionally run a reset command, observe, run, settle, observe again; the success predicate
  must hold in every observation after the run and not before it. Evidence is pinned.
- sim (`--sim-scene ID`): the simulator's ground truth (lekiwi_rmf/sim_scene.py). Each trial
  resets the scene deterministically for its index and confirms the initial state; an
  unconfirmed reset makes the trial invalid. A sim report says `domain: sim` and
  scripts/register-skill.py never promotes on it: physical evaluation is still required.

The protocol (domain, predicate, parameters, trial counts, thresholds, windows, reset) is
recorded with its digest. `--suite NAME` evaluates exactly the protocol of a stored
qualification suite (config/qualification_suites.yaml), which a new version must pass before
it replaces a qualified one.

The skill runner must admit the skill: an experimental one needs the stack started for
skill acquisition (see docs/skill-acquisition.md).
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
from lekiwi_rmf.qualification import canonical_sha256  # noqa: E402

PREDICATE_FIELDS = {'label', 'present', 'min_score', 'camera_id', 'region', 'min_observations'}
PROTOCOL_FIELDS = ('domain', 'success', 'parameters', 'trials', 'min_trials', 'confidence', 'required_lower_bound',
                   'observe_sec', 'settle_sec', 'timeout_sec', 'reset_command', 'sim_scene')


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
    value = json.loads(text) if isinstance(text, str) else text
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


class SkillTrials:
    """Runs one skill through the runner; subclasses add the verifier of their domain."""

    def __init__(self, node, skill, predicate, *, settle_sec, timeout_sec):
        from rclpy.action import ActionClient
        from lekiwi_rmf.action import RunSkill
        self.node, self.skill, self.predicate = node, skill, predicate
        self.settle_sec, self.timeout_sec = settle_sec, timeout_sec
        self.RunSkill = RunSkill
        self.run_client = ActionClient(node, RunSkill, '/robot/skills/run')

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

    def checked_run(self, trial, parameters):
        """The run, or the trial as invalid when the runner refused or ran other weights."""
        run = self.run_skill(parameters)
        trial['runner'] = run
        if run['outcome'] == 'rejected':
            return {**trial, 'verdict': 'invalid', 'reason': run['detail']}
        if run['result'] is not None and run['result'].get('weights_sha256') != self.skill.weights_sha256:
            return {**trial, 'verdict': 'invalid', 'reason': 'the runner executed different weights than registered'}
        return None

    def sleep_sim(self, seconds):
        """Wait `seconds` on the node clock (simulation time in sim), bounded in wall time."""
        end, deadline = self.now() + seconds, time.monotonic() + 20 * seconds + 10
        while self.now() < end:
            if time.monotonic() > deadline:
                raise RuntimeError('the node clock is not advancing')
            time.sleep(.02)


class Evaluator(SkillTrials):
    """Physical domain: the observation store is the verifier."""

    domain = 'physical'

    def __init__(self, node, skill, predicate, *, observe_sec, settle_sec, timeout_sec):
        from lekiwi_rmf.srv import PinObservationEvidence, QueryObservations
        super().__init__(node, skill, predicate, settle_sec=settle_sec, timeout_sec=timeout_sec)
        self.observe_sec = observe_sec
        self.query_client = node.create_client(QueryObservations, '/robot/observations/query')
        self.pin_client = node.create_client(PinObservationEvidence, '/robot/observations/pin')
        self.Query, self.Pin = QueryObservations, PinObservationEvidence
        self.epoch, self.cursor = None, 0

    def verifier(self):
        return {'source': '/robot/observations/query', 'independent_of_runner': True,
                'observe_sec': self.observe_sec, 'settle_sec': self.settle_sec}

    def ready(self, timeout=10.0):
        super().ready(timeout)
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
        refused = self.checked_run(trial, parameters)
        if refused:
            return refused
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


class SimEvaluator(SkillTrials):
    """Sim domain: each trial resets the scene and the simulator's ground truth is the verifier."""

    domain = 'sim'

    def __init__(self, node, skill, predicate, controller, *, settle_sec, timeout_sec):
        super().__init__(node, skill, predicate, settle_sec=settle_sec, timeout_sec=timeout_sec)
        self.controller = controller

    def verifier(self):
        scene = self.controller.scene
        return {'source': 'sim_ground_truth', 'independent_of_runner': True, 'scene': scene['id'],
                'modeled': scene['modeled'], 'reads': self.predicate['reads'], 'settle_sec': self.settle_sec}

    def ready(self, timeout=10.0):
        super().ready(timeout)
        self.controller.ready()

    def trial(self, index, parameters, evaluation_id):
        from lekiwi_rmf.sim_scene import judge as judge_pose
        reset = self.controller.reset(index)
        trial = {'trial': index, 'started_sec': self.now(), 'reset': reset}
        if not reset['confirmed']:
            # Not another trial from the same origin: the start state is unknown.
            return {**trial, 'verdict': 'invalid', 'reason': 'reset not confirmed: ' + '; '.join(reset['reasons'])}
        start = self.controller.ground_truth()[0].get(self.predicate['object'])
        before_holds, before = judge_pose(self.predicate, start, start)
        trial['before'] = before
        if before_holds:
            return {**trial, 'verdict': 'invalid', 'reason': 'the success predicate already held before the run'}
        refused = self.checked_run(trial, parameters)
        if refused:
            return refused
        self.sleep_sim(self.settle_sec)
        after_holds, after = judge_pose(self.predicate, self.controller.ground_truth()[0].get(self.predicate['object']),
                                        start)
        trial['after'] = after
        return {**trial, 'verdict': 'success' if after_holds else 'failure',
                'reason': 'ground truth satisfies the predicate after the run' if after_holds
                else 'ground truth does not satisfy the predicate after the run'}


def protocol_of(args, predicate, scene=None):
    """Everything that defines what passing means; its digest names a qualification suite."""
    return {'domain': 'sim' if scene else 'physical', 'success': predicate, 'parameters': args.parameters,
            'trials': args.trials, 'min_trials': args.min_trials, 'confidence': args.confidence,
            'required_lower_bound': args.required_lower_bound, 'observe_sec': args.observe_sec,
            'settle_sec': args.settle_sec, 'timeout_sec': args.timeout_sec, 'reset_command': args.reset_command,
            'sim_scene': {'id': scene[0]['id'], 'sha256': scene[1]} if scene else None}


def evaluate(evaluator, protocol, evaluation_id, reset=None, log=print, suite=None):
    trials = []
    for index in range(1, protocol['trials'] + 1):
        if reset is not None:
            reset(index)
        trial = evaluator.trial(index, protocol['parameters'], evaluation_id)
        trials.append(trial)
        log(json.dumps({'trial': index, 'verdict': trial['verdict'], 'reason': trial['reason']}))
    successes = sum(trial['verdict'] == 'success' for trial in trials)
    interval = wilson(successes, len(trials), protocol['confidence'])
    skill = evaluator.skill
    return {
        'schema': 2, 'kind': 'lekiwi_skill_evaluation', 'evaluation_id': evaluation_id,
        'created_utc': datetime.datetime.now(datetime.UTC).isoformat(timespec='seconds'),
        'skill_id': skill.skill_id, 'version': skill.version, 'weights_sha256': skill.weights_sha256,
        'status_at_evaluation': skill.status, 'domain': evaluator.domain,
        # Only a physical evaluation can qualify a skill; a sim result is evidence of another domain.
        'promotable': evaluator.domain == 'physical',
        'protocol': protocol, 'protocol_sha256': canonical_sha256(protocol), 'suite': suite,
        'parameters': protocol['parameters'], 'success_predicate': evaluator.predicate,
        'verifier': evaluator.verifier(), 'reset_command': protocol['reset_command'], 'trials': trials,
        'n': len(trials), 'successes': successes,
        # Invalid trials (blind verifier, goal already met, rejected, unconfirmed reset) count against the skill.
        'invalid': sum(trial['verdict'] == 'invalid' for trial in trials),
        'success_rate': successes / len(trials), 'confidence': protocol['confidence'], 'wilson_interval': interval,
        'required_lower_bound': protocol['required_lower_bound'], 'min_trials': protocol['min_trials'],
        'passed': len(trials) >= protocol['min_trials'] and interval[0] >= protocol['required_lower_bound'],
    }


def apply_suite(args, parser, suites):
    """Replace the protocol options with exactly the stored suite's protocol."""
    stored = {suite['name']: suite for suite in suites['skills'].get(args.skill_id, [])}
    if args.suite not in stored:
        parser.error(f'{args.skill_id} has no stored suite {args.suite!r}; known: {sorted(stored)}')
    protocol = stored[args.suite]['protocol']
    if protocol['domain'] != 'physical' or args.sim_scene:
        parser.error('qualification suites are physical protocols')
    for name in PROTOCOL_FIELDS:
        if name not in ('domain', 'success', 'sim_scene'):
            setattr(args, name, protocol[name])
    args.success = json.dumps(protocol['success'])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('skill_id')
    parser.add_argument('--success', help='physical: {"label": ..., "present": true, "min_score": .5, "camera_id": null, '
                        '"region": [x0, y0, x1, y1], "min_observations": 3}; sim: {"object": ..., "region": '
                        '[x0, y0, x1, y1] (world), "min_z": .., "max_z": .., "min_displacement_m": ..}')
    parser.add_argument('--sim-scene', help='evaluate in this config/sim_scenes.yaml scene, judged by ground truth')
    parser.add_argument('--suite', help='evaluate exactly this stored qualification suite of the skill')
    parser.add_argument('--suites', type=Path, default=None, help='qualification suites file (default: the tracked one)')
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
    if args.suite:
        if args.success:
            parser.error('--suite fixes the success predicate; do not pass --success')
        from lekiwi_rmf.qualification import load_suites
        apply_suite(args, parser, load_suites(args.suites))
    if not args.success:
        parser.error('pass --success or --suite')
    if not 1 <= args.trials <= 1000 or not 1 <= args.min_trials or not 0 < args.confidence < 1 \
            or not 0 <= args.required_lower_bound <= 1 or args.observe_sec <= 0 or args.settle_sec < 0:
        parser.error('invalid trial count, confidence, bound or window')
    if not isinstance(args.parameters, dict):
        parser.error('parameters must be a JSON object')
    if args.output.exists():
        parser.error(f'{args.output} exists; reports are never overwritten')
    scene = None
    try:
        if args.sim_scene:
            from lekiwi_rmf.sim_scene import load_scene, sim_predicate
            if args.reset_command:
                parser.error('a sim scene resets itself; --reset-command is for physical evaluation')
            scene = load_scene(args.sim_scene)
            predicate = sim_predicate(args.success, scene[0])
        else:
            predicate = success_predicate(args.success)
    except ValueError as error:
        parser.error(str(error))
    protocol = protocol_of(args, predicate, scene)

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
    node = Node('skill_evaluator', parameter_overrides=[
        Parameter('use_sim_time', value=args.use_sim_time or scene is not None)])
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    evaluation_id = f'skill-eval-{datetime.datetime.now(datetime.UTC):%Y%m%d%H%M%S}-{secrets.token_hex(3)}'
    try:
        if scene is not None:
            from lekiwi_rmf.sim_scene import SceneController
            evaluator = SimEvaluator(node, skill, predicate, SceneController(node, scene[0]),
                                     settle_sec=max(args.settle_sec, scene[0]['settle_sec']), timeout_sec=args.timeout_sec)
        else:
            evaluator = Evaluator(node, skill, predicate, observe_sec=args.observe_sec, settle_sec=args.settle_sec,
                                  timeout_sec=args.timeout_sec)
        evaluator.ready()
        report = evaluate(evaluator, protocol, evaluation_id, reset, log=lambda line: print(line, file=sys.stderr),
                          suite=args.suite)
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
    print(json.dumps({key: report[key] for key in ('skill_id', 'version', 'domain', 'n', 'successes', 'success_rate',
                                                     'wilson_interval', 'passed')}))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
