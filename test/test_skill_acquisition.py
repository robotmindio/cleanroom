"""Skill acquisition tooling: frozen training manifests, independent evaluation, gated registration."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from lekiwi_rmf.action import RunSkill
from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.skills import artifact_digest, load_registry
from lekiwi_rmf.srv import PinObservationEvidence, QueryObservations
from ros_test_utils import release_graph

SCRIPTS = Path(__file__).parents[1] / 'scripts'
ACTIONS = [*ARM_JOINTS, 'base_x_vel', 'base_y_vel', 'base_yaw_rate']
HEADER = '# tracked registry header\n# second line\n'


def script(name):
    spec = importlib.util.spec_from_file_location(name.replace('-', '_'), SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


train = script('train-skill')
evaluate = script('evaluate-skill')
register = script('register-skill')


def dataset(root, action_names=ACTIONS):
    (root / 'meta').mkdir(parents=True)
    (root / 'data').mkdir()
    (root / 'meta/info.json').write_text(json.dumps({'fps': 10, 'total_episodes': 1, 'total_frames': 3, 'features': {
        'observation.state': {'names': list(ARM_JOINTS)}, 'action': {'names': action_names},
        'observation.images.front': {'dtype': 'video'}}}))
    (root / 'data/file-000.parquet').write_bytes(b'frames')
    (root / 'export-report.json').write_text(json.dumps({'schema': 1, 'fps': 10, 'action_topic': '/hardware/applied_command'}))
    return root


FAKE_LEROBOT = '''#!/usr/bin/env python3
import json, sys
from pathlib import Path
arguments = dict(item[2:].split('=', 1) for item in sys.argv[4:])
if arguments.get('steps') == '13':
    print('simulated training failure'); sys.exit(3)
model = Path(arguments['output_dir']) / 'checkpoints/last/pretrained_model'
model.mkdir(parents=True)
(model / 'model.safetensors').write_bytes(b'weights')
(model / 'config.json').write_text('{}')
Path(sys.argv[3]).write_text(json.dumps({'elapsed_sec': 1.0, 'device_name': 'cpu'}))
'''


@pytest.fixture
def fake_python(tmp_path):
    path = tmp_path / 'fake-python'
    path.write_text(FAKE_LEROBOT)
    path.chmod(0o755)
    return path


def run_train(monkeypatch, *arguments):
    monkeypatch.setattr('sys.argv', ['train-skill.py', *map(str, arguments)])
    return train.main()


def test_local_training_freezes_the_dataset_and_pins_the_weights(tmp_path, fake_python, monkeypatch):
    root = dataset(tmp_path / 'export')
    output = tmp_path / 'run'
    assert run_train(monkeypatch, root, '--output', output, '--steps', 5, '--python', fake_python, '--skip-load-check') == 0
    manifest = json.loads((output / 'manifest.json').read_text())
    files = manifest['dataset']['files']
    assert set(files) == {'meta/info.json', 'data/file-000.parquet', 'export-report.json'}
    assert files['data/file-000.parquet']['sha256'] == hashlib.sha256(b'frames').hexdigest()
    assert manifest['dataset']['action_names'] == ACTIONS and manifest['dataset']['cameras'] == ['front']
    assert manifest['dataset']['export']['action_topic'] == '/hardware/applied_command'
    assert manifest['checkpoint']['weights_sha256'] == hashlib.sha256(b'weights').hexdigest()
    assert manifest['checkpoint']['artifact_sha256'] == artifact_digest(output / 'checkpoint')
    assert '--steps=5' in manifest['training']['lerobot_arguments']
    assert manifest['compute']['backend'] == 'local' and manifest['compute']['declared']['time_limit_min'] == 120.0
    assert manifest['compute']['within_declared'] and 'source_revision' in manifest['provenance']
    assert (output / 'checkpoint/model.safetensors').is_file() and not (output / 'lerobot').exists()


def test_training_failures_are_reported_and_never_produce_a_manifest(tmp_path, fake_python, monkeypatch, capsys):
    root = dataset(tmp_path / 'export')
    assert run_train(monkeypatch, root, '--output', tmp_path / 'run', '--steps', 13, '--python', fake_python) == 1
    assert 'training failed (3)' in capsys.readouterr().err and not (tmp_path / 'run/manifest.json').exists()
    assert run_train(monkeypatch, tmp_path / 'missing', '--output', tmp_path / 'other') == 1
    assert 'not a LeRobot dataset' in capsys.readouterr().err
    (root / 'link').symlink_to(root / 'export-report.json')
    assert run_train(monkeypatch, root, '--output', tmp_path / 'third', '--python', fake_python) == 1
    assert 'symbolic links' in capsys.readouterr().err


def test_cluster_job_targets_the_working_gpu_and_bounds_its_lifetime():
    args = SimpleNamespace(memory_gib=8, time_limit_min=10, slot_wait_min=5, node='robotmind-web-001',
                           context='morelos', namespace='robotmind', cluster_device='cuda')
    cluster = train.Cluster(args)
    job = cluster.job()
    pod = job['spec']['template']['spec']
    container = pod['containers'][0]
    pin = {'nodeAffinity': {'requiredDuringSchedulingIgnoredDuringExecution': {'nodeSelectorTerms': [
        {'matchExpressions': [{'key': 'kubernetes.io/hostname', 'operator': 'In', 'values': ['robotmind-web-001']}]}]}}}
    assert pod['affinity'] == pin and pod['runtimeClassName'] == 'nvidia'
    assert container['resources']['limits']['nvidia.com/gpu'] == 1 and '@sha256:' in container['image']
    assert pod['volumes'][0]['hostPath'] == {'path': cluster.directory, 'type': 'Directory'}
    assert job['spec']['backoffLimit'] == 0 and job['spec']['activeDeadlineSeconds'] == 900 + 1800
    assert 'sha256sum --check' in container['command'][2] and 'mv "$result/status.tmp"' in container['command'][2]
    # The CPU fallback runs the same image on the same node without claiming the GPU.
    pod = train.Cluster(SimpleNamespace(**{**vars(args), 'cluster_device': 'cpu'})).job()['spec']['template']['spec']
    assert 'nvidia.com/gpu' not in pod['containers'][0]['resources']['limits'] and 'runtimeClassName' not in pod
    assert pod['containers'][0]['image'] == container['image'] and pod['affinity'] == pin


class FakeCluster(train.Cluster):
    """The cluster as kubectl and ssh would report it, without either."""

    def __init__(self, args, pods, status=None):
        super().__init__(args)
        self.pods, self.status_text, self.directory = pods, status, '/var/tmp/skill-train/x'

    def ssh(self, command, check=True):
        found = command.startswith('cat') and self.status_text is not None
        return SimpleNamespace(returncode=0 if found else 1, stdout=self.status_text or '', stderr='')

    def pod_state(self):
        return self.pods.pop(0) if len(self.pods) > 1 else self.pods[0]

    def events(self):
        return 'events'


def pending(message):
    return {'metadata': {'name': 'p'}, 'status': {'phase': 'Pending', 'conditions': [
        {'type': 'PodScheduled', 'status': 'False', 'reason': 'Unschedulable', 'message': message}]}}


def test_a_job_waiting_for_a_gpu_slot_reports_why_and_gives_up_in_bounded_time(monkeypatch, capsys):
    monkeypatch.setattr(train.time, 'sleep', lambda _seconds: None)
    clock = iter(range(0, 10**6, 30))
    monkeypatch.setattr(train.time, 'monotonic', lambda: next(clock))
    args = SimpleNamespace(cluster_device='cuda', node='robotmind-web-001', slot_wait_min=5, time_limit_min=10)
    cluster = FakeCluster(args, [pending('0/2 nodes are available: 1 Insufficient nvidia.com/gpu')])
    with pytest.raises(train.TrainingError, match='no slot on robotmind-web-001 within 5 min'):
        cluster.wait(10**6)
    assert capsys.readouterr().err.count('waiting for a GPU slot on robotmind-web-001: 0/2 nodes') == 1
    # Once scheduled and finished, the published status is returned.
    running = {'metadata': {'name': 'p'}, 'status': {'phase': 'Running'}}
    assert FakeCluster(args, [running], status='0\n').wait(10**6) == 0
    failed = {'metadata': {'name': 'p'}, 'status': {'phase': 'Failed'}}
    with pytest.raises(train.TrainingError, match='ended without a result'):
        FakeCluster(args, [failed]).wait(10**6)


def training_dir(tmp_path, weights=b'weights', action_names=ACTIONS, name='run'):
    output = tmp_path / name
    (output / 'checkpoint').mkdir(parents=True)
    (output / 'checkpoint/model.safetensors').write_bytes(weights)
    (output / 'checkpoint/config.json').write_text('{"type": "act"}')
    (output / 'checkpoint/policy_preprocessor.json').write_text(
        '{"steps": [{"registry_name": "normalizer_processor", "state_file": "normalizer.safetensors"}]}')
    (output / 'checkpoint/normalizer.safetensors').write_bytes(b'stats')
    (output / 'manifest.json').write_text(json.dumps({
        'kind': 'lekiwi_skill_training', 'checkpoint': {'weights_sha256': hashlib.sha256(weights).hexdigest(),
                                                        'artifact_sha256': artifact_digest(output / 'checkpoint')},
        'dataset': {'action_names': action_names, 'cameras': ['front'], 'fps': 10}}))
    return output


@pytest.fixture
def registry_file(tmp_path):
    path = tmp_path / 'robot_skills.yaml'
    path.write_text(HEADER + 'skills: {}\n')
    return path


def run_register(monkeypatch, registry_file, artifacts, *arguments):
    monkeypatch.setattr('sys.argv', ['register-skill.py', *map(str, arguments), '--registry', str(registry_file),
                                     '--artifacts', str(artifacts)])
    return register.main()


def test_registration_installs_the_checkpoint_as_an_experimental_entry_that_loads(tmp_path, registry_file, monkeypatch):
    artifacts = tmp_path / 'skills'
    training = training_dir(tmp_path)
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', training, '--version', 1,
                        '--description', 'learned from demonstrations') == 0
    skill = load_registry(registry_file, artifacts)['learned']
    assert skill.status == 'experimental' and skill.action_names == tuple(ACTIONS) and skill.cameras == ('front',)
    assert skill.checkpoint == artifacts / 'learned/1' and (skill.checkpoint / 'training-manifest.json').is_file()
    assert registry_file.read_text().startswith(HEADER)
    # Same version, same weights: idempotent. Different weights need a new version.
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', training, '--version', 1,
                        '--description', 'learned from demonstrations') == 0
    other = training_dir(tmp_path, b'other', name='other')
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', other, '--version', 1,
                        '--description', 'x') == 1
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', other, '--version', 2,
                        '--description', 'x') == 1  # superseding requires --replace
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', other, '--version', 2,
                        '--description', 'x', '--replace') == 0
    assert load_registry(registry_file, artifacts)['learned'].version == 2


def test_a_training_manifest_without_the_artifact_digest_or_a_changed_checkpoint_is_refused(
        tmp_path, registry_file, monkeypatch):
    artifacts = tmp_path / 'skills'
    training = training_dir(tmp_path)
    manifest = json.loads((training / 'manifest.json').read_text())
    del manifest['checkpoint']['artifact_sha256']
    (training / 'manifest.json').write_text(json.dumps(manifest))
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', training, '--version', 1,
                        '--description', 'x') == 1
    changed = training_dir(tmp_path, name='changed')
    (changed / 'checkpoint/policy_preprocessor.json').write_text('{"steps": []}')
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', changed, '--version', 1,
                        '--description', 'x') == 1
    assert not artifacts.exists() and registry_file.read_text() == HEADER + 'skills: {}\n'


def test_host_unit_actions_are_refused_before_anything_is_installed(tmp_path, registry_file, monkeypatch):
    artifacts = tmp_path / 'skills'
    training = training_dir(tmp_path, action_names=['arm_gripper.pos', 'x.vel'])
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', training, '--version', 1,
                        '--description', 'x') == 1
    assert not artifacts.exists() and registry_file.read_text() == HEADER + 'skills: {}\n'


def report(skill, **changes):
    value = {'kind': 'lekiwi_skill_evaluation', 'passed': True, 'skill_id': 'learned', 'version': skill.version,
             'artifact_sha256': skill.artifact_sha256, 'success_rate': 1.0, 'wilson_interval': [.84, 1.0]}
    value.update(changes)
    return value


def test_promotion_requires_a_passing_report_for_exactly_these_weights(tmp_path, registry_file, monkeypatch):
    artifacts = tmp_path / 'skills'
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--training', training_dir(tmp_path),
                        '--version', 1, '--description', 'x') == 0
    skill = load_registry(registry_file, artifacts)['learned']
    path = tmp_path / 'report.json'
    for bad in ({'passed': False}, {'artifact_sha256': 'b' * 64}, {'version': 2}, {'skill_id': 'other'}, {'kind': 'x'}):
        path.write_text(json.dumps(report(skill, **bad)))
        assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--promote', path) == 1
        assert load_registry(registry_file, artifacts)['learned'].status == 'experimental'
    path.write_text(json.dumps(report(skill)))
    # Weights, configuration and normalization are all part of what was evaluated.
    for name, original in (('model.safetensors', b'weights'), ('config.json', b'{"type": "act"}'),
                           ('normalizer.safetensors', b'stats')):
        (skill.checkpoint / name).write_bytes(b'{"tampered": 1}')
        assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--promote', path) == 1
        (skill.checkpoint / name).write_bytes(original)
    (skill.checkpoint / 'extra.safetensors').write_bytes(b'added')
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--promote', path) == 1
    (skill.checkpoint / 'extra.safetensors').unlink()
    assert run_register(monkeypatch, registry_file, artifacts, 'learned', '--promote', path) == 0
    assert load_registry(registry_file, artifacts)['learned'].status == 'qualified'
    assert (skill.checkpoint / 'evaluation-report.json').is_file() and registry_file.read_text().startswith(HEADER)


def test_wilson_interval_and_predicate_contract():
    low, high = evaluate.wilson(20, 20)
    assert low == pytest.approx(.8389, abs=1e-3) and high == 1.0
    low, high = evaluate.wilson(5, 10)
    assert low == pytest.approx(.2366, abs=1e-3) and high == pytest.approx(.7634, abs=1e-3)
    assert evaluate.wilson(0, 0) is None
    for bad in ['{}', '{"label": "cup", "colour": "blue"}', '{"label": "cup", "region": [0.5, 0, 0.2, 1]}',
                '{"label": "cup", "present": 1}', '{"label": "cup", "min_observations": 0}']:
        with pytest.raises(ValueError):
            evaluate.success_predicate(bad)
    predicate = evaluate.success_predicate('{"label": "cup", "region": [0, 0, 0.5, 1], "min_observations": 2}')
    inside = {'id': 'a', 'camera_id': 'front', 'stamp': {}, 'evidence': {'sha256': 'e'}, 'width': 100, 'height': 100,
              'detections': [{'label': 'cup', 'score': .9, 'bbox': [10, 10, 10, 10]}]}
    outside = {**inside, 'id': 'b', 'detections': [{'label': 'cup', 'score': .9, 'bbox': [80, 10, 10, 10]}]}
    assert evaluate.judge([inside, inside], predicate)[0]
    assert not evaluate.judge([inside], predicate)[0]  # too few observations
    holds, evidence = evaluate.judge([inside, outside], predicate)
    assert not holds and [item['satisfied'] for item in evidence] == [True, False]


class FakeWorld(Node):
    """A skill runner and an observation store that see one shared scene, independently."""

    def __init__(self, context, weights):
        super().__init__('fake_world', context=context)
        self.cup_present, self.runner_changes_world, self.weights = False, True, weights
        self.observations, self.pins, self.lock = [], {}, threading.Lock()
        self.server = ActionServer(self, RunSkill, '/robot/skills/run', self.execute,
                                   goal_callback=lambda _goal: GoalResponse.ACCEPT)
        self.create_service(QueryObservations, '/robot/observations/query', self.query)
        self.create_service(PinObservationEvidence, '/robot/observations/pin', self.pin)
        self.create_timer(.05, self.capture)

    def execute(self, handle):
        time.sleep(.1)
        if self.runner_changes_world:
            self.cup_present = True
        handle.succeed()
        return RunSkill.Result(outcome='finished', result_json=json.dumps({'artifact_sha256': self.weights}))

    def capture(self):
        stamp = self.get_clock().now().nanoseconds
        with self.lock:
            sequence = len(self.observations) + 1
            self.observations.append({
                'id': f'{sequence:064x}', 'sequence': sequence, 'camera_id': 'front', 'width': 100, 'height': 100,
                'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}, 'evidence': {'sha256': f'{sequence:064x}'},
                'detections': [{'label': 'cup', 'score': .9, 'bbox': [10, 10, 10, 10]}] if self.cup_present else []})

    def query(self, request, response):
        values = json.loads(request.request_json)
        with self.lock:
            records = [item for item in self.observations if item['sequence'] > values.get('after_sequence', 0)]
            latest = len(self.observations)
        records = records[:values.get('limit', 20)] if 'after_sequence' in values else []
        response.response_json = json.dumps({'epoch': 'e' * 32, 'observations': records, 'latest_sequence': latest,
                                             'next_sequence': records[-1]['sequence'] if records else values.get('after_sequence', latest),
                                             'cursor_gap': False})
        response.success = True
        return response

    def pin(self, request, response):
        values = json.loads(request.request_json)
        self.pins[values['request_id']] = values['observation_ids']
        response.success = True
        return response


@pytest.fixture
def world():
    context = Context()
    rclpy.init(context=context)
    weights = 'a' * 64
    fake, client = FakeWorld(context, weights), Node('skill_evaluator', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(fake)
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    skill = SimpleNamespace(skill_id='learned', version=1, artifact_sha256=weights, status='experimental',
                            max_duration_sec=5.0)
    predicate = evaluate.success_predicate('{"label": "cup", "min_observations": 2}')
    evaluator = evaluate.Evaluator(client, skill, predicate, observe_sec=.4, settle_sec=.1, timeout_sec=5.0)
    try:
        evaluator.ready()
        yield fake, evaluator
    finally:
        # Node.destroy_node leaves action waitables alive; a leaked server answers the next suite's goals.
        release_graph(executor=executor, thread=thread, waitables=(fake.server, evaluator.run_client),
                      nodes=(fake, client), context=context)


def arguments(trials, **changes):
    value = {'trials': trials, 'parameters': {}, 'confidence': .95, 'required_lower_bound': .1, 'min_trials': 1,
             'reset_command': None}
    value.update(changes)
    return SimpleNamespace(**value)


def test_evaluation_judges_the_scene_not_the_runner_outcome(world):
    fake, evaluator = world

    def reset(_index):
        fake.cup_present = False
    result = evaluate.evaluate(evaluator, arguments(2), 'skill-eval-test', reset, log=lambda _line: None)
    assert [trial['verdict'] for trial in result['trials']] == ['success', 'success'] and result['passed']
    assert result['wilson_interval'] == pytest.approx(evaluate.wilson(2, 2))
    trial = result['trials'][0]
    assert trial['runner']['outcome'] == 'finished' and trial['evidence_pinned']
    assert set(fake.pins['skill-eval-test-1']) == {item['observation_id'] for item in trial['before'][-10:] + trial['after'][-10:]}
    # The runner still reports `finished`, but the scene did not change: the verifier says failure.
    fake.cup_present, fake.runner_changes_world = False, False
    result = evaluate.evaluate(evaluator, arguments(1, required_lower_bound=.5), 'skill-eval-test2', log=lambda _line: None)
    assert result['trials'][0]['runner']['outcome'] == 'finished'
    assert result['trials'][0]['verdict'] == 'failure' and not result['passed']


def test_trials_that_cannot_be_judged_count_against_the_skill(world):
    fake, evaluator = world
    fake.cup_present = True  # goal already satisfied before the run
    result = evaluate.evaluate(evaluator, arguments(1), 'skill-eval-pre', log=lambda _line: None)
    assert result['trials'][0]['verdict'] == 'invalid' and 'already held' in result['trials'][0]['reason']
    assert result['successes'] == 0 and result['invalid'] == 1 and not result['passed']
    fake.cup_present, fake.weights = False, 'b' * 64
    result = evaluate.evaluate(evaluator, arguments(1), 'skill-eval-weights', log=lambda _line: None)
    assert 'different checkpoint' in result['trials'][0]['reason']


@pytest.mark.parametrize('acquisition', ['false', 'true'])
def test_bringup_admits_experimental_skills_only_for_acquisition(acquisition):
    from launch_snapshot import find_nodes, resolve_bringup
    for profile in ('sim', 'split'):
        runners = find_nodes(resolve_bringup(profile=profile, skill_acquisition=acquisition), name='skill_runner')
        assert len(runners) == 1 and runners[0]['respawn']
        assert runners[0]['parameters']['allow_experimental'] is (acquisition == 'true')


def test_runner_uses_the_policy_environment_without_importing_torch_at_startup(monkeypatch, tmp_path):
    from lekiwi_rmf import skill_runner
    assert 'import torch' not in Path(skill_runner.__file__).read_text().split('class LeRobotPolicy')[0]
    calls = []
    monkeypatch.setattr(skill_runner, 'POLICY_RUNTIME', tmp_path / 'venv/bin/python')
    monkeypatch.setattr(os, 'execve', lambda *call: calls.append(call))
    skill_runner.use_policy_runtime()
    assert calls == []  # absent environment: stay on the current interpreter
    (tmp_path / 'venv/bin').mkdir(parents=True)
    (tmp_path / 'venv/bin/python').write_text('')
    skill_runner.use_policy_runtime()
    assert calls and calls[0][0] == tmp_path / 'venv/bin/python' and calls[0][2]['PYTHONNOUSERSITE'] == '1'
