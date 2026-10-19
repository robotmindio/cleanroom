"""Perception trainer: frozen datasets, the trainer contract, cluster staging, registry and promotion gates."""

import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sqlite3
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from lekiwi_rmf import object_recognition, training
from lekiwi_rmf.object_observations import capture_artifact
from lekiwi_rmf.qualification import load_suites, store_report

REPOSITORY = Path(__file__).parents[1]
PERCEPTION_PYTHON = Path.home() / '.ros/lekiwi/perception-venv/bin/python'
BASE_MODEL = Path.home() / '.ros/lekiwi/models/rtdetr_v2_r18'


def script(name):
    spec = importlib.util.spec_from_file_location(name.replace('-', '_'), REPOSITORY / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


build = script('build-perception-dataset')
trainer = script('train-recognizer')
register = script('register-recognizer')


def frame(seed, cards=1, size=(240, 320)):
    """A distinct noisy frame with checker cards at known boxes."""
    generator = np.random.default_rng(seed)
    pixels = generator.integers(0, 255, (*size, 3), dtype=np.uint8)
    pixels = cv2.GaussianBlur(pixels, (0, 0), 6)
    boxes = []
    for index in range(cards):
        side = 40 + 10 * index
        x, y = 20 + 120 * index, 30 + 60 * (seed % 3)
        tile = ((np.indices((side, side)) // 8).sum(0) % 2).astype(np.uint8)
        pixels[y:y + side, x:x + side] = np.where(tile[..., None] == 1, (40, 40, 220), (235, 235, 235))
        boxes.append([x, y, side, side])
    return pixels, boxes


def annotation(method='human_correction', author='user:ana'):
    return {'author': author, 'method': method, 'confidence': 1.0, 'revision': 'r1'}


def views(tmp_path, count=8, heldout_method='human_review', label='checker card'):
    result = []
    for index in range(count):
        pixels, boxes = frame(index * 7 + 1, cards=1 + index % 2)
        path = tmp_path / f'frame-{index}.png'
        cv2.imwrite(str(path), pixels)
        heldout = index >= count - 2
        method = heldout_method if heldout else 'automatic'
        result.append({'view_id': f'v{index}', 'scene_group': f'g{index}', 'split': 'heldout' if heldout else 'development',
                       'source': {'image': str(path)}, 'labeled_by': {'author': 'user:ana', 'method': method},
                       'objects': [{'label': label, 'bbox': box, 'annotation': annotation(method)} for box in boxes]})
    return result


def write_annotations(tmp_path, items, name='annotations.json'):
    path = tmp_path / name
    path.write_text(json.dumps({'dataset_id': 'cards', 'views': items}))
    return path


def test_held_out_views_must_carry_human_labels_unless_declared_unreviewed(tmp_path):
    source = write_annotations(tmp_path, views(tmp_path, heldout_method='automatic'))
    with pytest.raises(build.DatasetError, match='held-out views carry only human annotations'):
        build.build(source, tmp_path / 'strict')
    assert not (tmp_path / 'strict').exists() and not (tmp_path / '.strict.partial').exists()
    manifest = build.build(source, tmp_path / 'measurement', unreviewed_heldout=True)
    assert manifest['heldout_authority'] == 'unreviewed' and manifest['annotation_methods'] == {'automatic': 12}
    human = build.build(write_annotations(tmp_path, views(tmp_path), 'human.json'), tmp_path / 'human')
    assert human['heldout_authority'] == 'human' and human['labels'] == ['checker card']
    assert human['splits'] == {'development': {'scenes': 6, 'scene_groups': 6}, 'heldout': {'scenes': 2, 'scene_groups': 2}}
    # Every annotation keeps who made it, how, with what confidence and revision.
    first = human['views'][0]['annotations'][0]
    assert first['method'] == 'automatic' and first['author'] == 'user:ana' and first['revision'] == 'r1'
    # The stored images are the recognizer's model input: the frame's evidence normalization.
    pixels, _ = frame(1, cards=1)
    assert human['views'][0]['image_sha256'] == hashlib.sha256(capture_artifact(pixels)).hexdigest()
    files = {name: item for name, item in training.frozen_files(tmp_path / 'human').items() if name != 'dataset.json'}
    assert files == human['files']
    with pytest.raises(build.DatasetError, match='never rewritten'):
        build.build(source, tmp_path / 'human')


def test_near_duplicates_never_span_development_and_held_out(tmp_path):
    items = views(tmp_path)
    pixels = cv2.imread(items[0]['source']['image'])
    near = pixels.copy()
    near[0, 0] = (near[0, 0].astype(int) + 3) % 256  # different bytes, same view
    cv2.imwrite(str(tmp_path / 'near.png'), near)
    items[-1] = {**items[0], 'view_id': 'copy', 'scene_group': 'other', 'split': 'heldout',
                 'source': {'image': str(tmp_path / 'near.png')}, 'labeled_by': {'author': 'user:ana', 'method': 'human_review'},
                 'objects': [{**item, 'annotation': annotation()} for item in items[0]['objects']]}
    with pytest.raises(build.DatasetError, match='nearly duplicates'):
        build.build(write_annotations(tmp_path, items), tmp_path / 'dataset')


def test_observation_sources_are_read_byte_exact_from_the_store(tmp_path):
    store = tmp_path / 'observations.sqlite3'
    connection = sqlite3.connect(store)
    connection.execute('CREATE TABLE observations (sequence INTEGER PRIMARY KEY, id TEXT, stored_at REAL, document TEXT, '
                       'evidence BLOB, bytes INTEGER)')
    items = views(tmp_path)
    for item in items[:2]:
        data = capture_artifact(cv2.imread(item['source']['image']))
        identifier = hashlib.sha256(item['view_id'].encode()).hexdigest()
        evidence = data if item['view_id'] == 'v0' else data + b'tampered'
        connection.execute('INSERT INTO observations (id, stored_at, document, evidence, bytes) VALUES (?, 0, ?, ?, 0)', (
            identifier, json.dumps({'camera_id': 'front', 'stamp': {'sec': 1, 'nanosec': 0}, 'model_sha256': 'm' * 64,
                                    'evidence': {'sha256': hashlib.sha256(data).hexdigest()}}), evidence))
        item['source'] = {'observation_id': identifier}
    connection.commit()
    connection.close()
    with pytest.raises(build.DatasetError, match='does not match its record'):
        build.build(write_annotations(tmp_path, items), tmp_path / 'tampered', store)
    manifest = build.build(write_annotations(tmp_path, items[:1] + items[2:], 'ok.json'), tmp_path / 'ok', store)
    assert manifest['views'][0]['source']['observation_id'] == items[0]['source']['observation_id']
    assert manifest['views'][0]['source']['camera_id'] == 'front'


def test_the_trainer_contract_rejects_undeclared_or_out_of_range_configuration(tmp_path):
    declaration = training.trainer_declaration('rtdetr_detector')
    assert declaration['checkpoints']['resumable'] and 'model/' in declaration['outputs']
    assert training.trainer_declaration('lerobot_policy')['script'] == 'scripts/train-skill.py'
    config = {'steps': 3, 'batch_size': 2, 'learning_rate': 1e-4, 'backbone_learning_rate': 1e-5, 'weight_decay': 0.0,
              'seed': 1, 'checkpoint_every': 1, 'horizontal_flip': False}
    assert training.validate_config(declaration, config) == config
    for bad in ({**config, 'steps': 0}, {**config, 'extra': 1}, {**config, 'horizontal_flip': 1},
                {key: value for key, value in config.items() if key != 'seed'}, {**config, 'learning_rate': float('nan')}):
        with pytest.raises(training.TrainingError):
            training.validate_config(declaration, bad)
    with pytest.raises(ValueError, match='no trainer declaration'):
        training.trainer_declaration('unknown')
    # A resume is the same run: the recorded dataset and configuration, nothing else.
    record = training.RunRecord(tmp_path, declaration, config, 'd' * 64, 'local')
    record.start(device='cpu')
    record.finish('failed', 'pod lost')
    for dataset, changed in (('e' * 64, config), ('d' * 64, {**config, 'seed': 2})):
        with pytest.raises(training.TrainingError, match='differs from the run being resumed'):
            training.RunRecord.resume(tmp_path, declaration, changed, dataset, 'local')
    resumed = training.RunRecord.resume(tmp_path, declaration, config, 'd' * 64, 'cluster')
    resumed.start(device='cuda')
    resumed.finish('completed')
    run = json.loads((tmp_path / 'run.json').read_text())
    assert [attempt['termination'] for attempt in run['attempts']] == ['failed', 'completed']
    with pytest.raises(training.TrainingError, match='already completed'):
        training.RunRecord.resume(tmp_path, declaration, config, 'd' * 64, 'local')


class StagedCluster(trainer.Cluster):
    """The Job mechanism with kubectl, ssh and rsync recorded instead of run."""

    def __init__(self, args, code=0, fail_wait=False):
        super().__init__(args)
        self.calls, self.code, self.fail_wait = [], code, fail_wait

    def kubectl(self, *command, check=True, **kwargs):
        self.calls.append(('kubectl', command, kwargs.get('input')))
        return SimpleNamespace(returncode=0, stdout='10.0.0.1', stderr='')

    def ssh(self, command, check=True):
        self.calls.append(('ssh', command))
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    def rsync(self, source, target):
        self.calls.append(('rsync', source, target))
        if source.endswith('/job/'):
            self.staged = {path.name: path.read_text() for path in Path(source).iterdir()}

    def wait(self, deadline):
        if self.fail_wait:
            raise training.TrainingError('pod lost')
        return self.code


def cluster_args():
    return SimpleNamespace(memory_gib=8, time_limit_min=10, slot_wait_min=5, node='robotmind-web-001', context='morelos',
                           namespace='robotmind', cluster_device='cuda')


def test_the_recognizer_job_reuses_the_shared_job_and_returns_results_even_on_failure(tmp_path):
    inputs = tmp_path / 'dataset'
    inputs.mkdir()
    (inputs / 'scenes.json').write_text('[]')
    files = training.frozen_files(inputs)
    cluster = StagedCluster(cluster_args(), code=13)
    assert cluster.run({'run.sh': 'true', 'install.sh': 'true'}, {'dataset': (inputs, files)}, tmp_path / 'result') == 13
    assert cluster.staged['inputs.sha256'] == f'{files["scenes.json"]["sha256"]}  dataset/scenes.json\n'
    pulls = [call for call in cluster.calls if call[0] == 'rsync' and call[1].endswith('/result/')]
    assert len(pulls) == 1  # the result (logs, checkpoints) comes back although training failed
    job = json.loads(next(call[2] for call in cluster.calls if call[0] == 'kubectl' and call[1][0] == 'create'))
    container = job['spec']['template']['spec']['containers'][0]
    assert container['image'] == trainer.IMAGE and {'name': 'PACKAGES', 'value': trainer.PACKAGES} in container['env']
    assert job['metadata']['labels'] == {'app.kubernetes.io/name': 'recognizer-train'}
    assert 'sha256sum --check' in container['command'][2]
    lost = StagedCluster(cluster_args(), fail_wait=True)
    with pytest.raises(training.TrainingError, match='pod lost'):
        lost.run({}, {'dataset': (inputs, files)}, tmp_path / 'lost')
    assert any(call[0] == 'rsync' and call[1].endswith('/result/') for call in lost.calls)
    error = training.cluster_failure(10, {}, tmp_path)
    assert error.termination == 'invalid_input' and 'digests did not verify' in str(error)


def registry_copy(tmp_path):
    path = tmp_path / 'recognition_models.yaml'
    shutil.copy(REPOSITORY / 'config/recognition_models.yaml', path)
    suites = tmp_path / 'qualification_suites.yaml'
    suites.write_text('# suites\nrecognizers: {}\nskills: {}\n')
    return path, suites


def fake_training(tmp_path, dataset_root, heldout_authority='human', moved_label=False):
    """A train-recognizer run directory with stand-in model files and the manifest the trainer writes."""
    run = tmp_path / 'run'
    (run / 'model').mkdir(parents=True)
    base = yaml.safe_load((REPOSITORY / 'config/recognition_models.yaml').read_text())['rtdetr_v2_r18']
    labels = [f'label{index}' for index in range(80)]
    if moved_label:
        labels[3] = 'moved'
    vocabulary = labels + ['checker card']
    (run / 'model/config.json').write_text(json.dumps({'id2label': {str(i): label for i, label in enumerate(vocabulary)}}))
    (run / 'model/model.safetensors').write_bytes(b'weights')
    (run / 'model/preprocessor_config.json').write_text('{}')
    files = {name: {'sha256': training.digest(run / 'model' / name), 'size': (run / 'model' / name).stat().st_size}
             for name in ('model.safetensors', 'config.json', 'preprocessor_config.json')}
    manifest = {'kind': 'lekiwi_recognizer_training',
                'artifact': {'path': 'model', 'spec': {'backend': 'rtdetr_v2', 'input_size': 384, 'prompt': '', 'files': files}},
                'base': {'name': 'rtdetr_v2_r18', 'files': base['files']},
                'vocabulary': {'labels': [f'label{index}' for index in range(80)] + ['checker card'], 'base_labels': 80,
                               'new_labels': ['checker card']},
                'dataset': {'root': str(dataset_root), 'dataset_id': 'cards', 'sha256': 'd' * 64, 'scenes_sha256': '5' * 64,
                            'heldout_authority': heldout_authority}}
    (run / 'manifest.json').write_text(json.dumps(manifest))
    return run


def run_register(monkeypatch, *arguments):
    monkeypatch.setattr('sys.argv', ['register-recognizer.py', *map(str, arguments)])
    return register.main()


def test_registration_appends_an_experimental_entry_and_keeps_the_base_catalogue(tmp_path, monkeypatch):
    registry, suites = registry_copy(tmp_path)
    before = registry.read_text()
    models = tmp_path / 'models'
    common = ('--registry', registry, '--suites', suites, '--models', models)
    moved = fake_training(tmp_path / 'moved', tmp_path, moved_label=True)
    assert run_register(monkeypatch, 'cards_v1', '--training', moved, *common) == 1
    assert registry.read_text() == before and not models.exists()
    run = fake_training(tmp_path, tmp_path)
    assert run_register(monkeypatch, 'cards_v1', '--training', run, *common) == 0
    assert registry.read_text().startswith(before)  # every existing entry stays byte-identical
    entry = yaml.safe_load(registry.read_text())['cards_v1']
    assert entry['status'] == 'experimental' and entry['replaces'] == 'rtdetr_v2_r18'
    assert entry['training']['heldout_manifest_sha256'] == '5' * 64 and (models / 'cards_v1/model.safetensors').is_file()
    # Promotion changes governance fields only, which stay out of the recognizer identity.
    monkeypatch.setattr(object_recognition, 'model_spec', lambda name: entry)
    identity = object_recognition.recognition_identity('cards_v1')
    monkeypatch.setattr(object_recognition, 'model_spec', lambda name: {**entry, 'status': 'qualified'})
    assert object_recognition.recognition_identity('cards_v1') == identity
    # The detector refuses an experimental recognizer; only the evaluator admits one.
    monkeypatch.setattr(object_recognition, 'model_spec', lambda name: entry)
    with pytest.raises(ValueError, match='experimental recognizer'):
        object_recognition.Recognizer('cards_v1', models / 'cards_v1')
    # Re-registering the same files is idempotent; other files under the same name are refused.
    assert run_register(monkeypatch, 'cards_v1', '--training', run, *common) == 0
    other = fake_training(tmp_path / 'other', tmp_path)
    (other / 'model/model.safetensors').write_bytes(b'other')
    manifest = json.loads((other / 'manifest.json').read_text())
    manifest['artifact']['spec']['files']['model.safetensors']['sha256'] = training.digest(other / 'model/model.safetensors')
    (other / 'manifest.json').write_text(json.dumps(manifest))
    assert run_register(monkeypatch, 'cards_v1', '--training', other, *common) == 1


def scene_report(model, manifest, f1_drop=False, passed=True, labels=('checker card',)):
    goals = {label: [5, 0, 0] for label in labels}
    if f1_drop:
        goals['person'] = [1, 3, 4]
    return {'schema': 3, 'passed': passed, 'model_sha256': model, 'manifest_sha256': manifest, 'split': 'heldout',
            'labels': list(labels), 'scenes': [{'scene_group': f'g{index}', 'scene_id': f's{index}', 'conditions': [],
                                                'goals': goals} for index in range(3)]}


def promote_setup(tmp_path, monkeypatch, heldout_authority='human'):
    registry, suites = registry_copy(tmp_path)
    models = tmp_path / 'models'
    run = fake_training(tmp_path, tmp_path, heldout_authority)
    assert run_register(monkeypatch, 'cards_v1', '--training', run, '--registry', registry, '--suites', suites,
                        '--models', models) == 0
    document = yaml.safe_load(registry.read_text())
    monkeypatch.setattr(object_recognition, 'model_spec', lambda name: document[name])
    monkeypatch.setattr(object_recognition, 'recognition_identity', lambda name: ('c' * 64, {}))
    monkeypatch.setattr(object_recognition, 'use_model_runtime', lambda *names: None)
    return registry, suites, models


def promote(monkeypatch, tmp_path, registry, suites, models, report, *regressions):
    path = tmp_path / f'report-{len(list(tmp_path.glob("report-*")))}.json'
    path.write_text(json.dumps(report))
    extra = []
    for item in regressions:
        regression = tmp_path / f'regression-{len(list(tmp_path.glob("regression-*")))}.json'
        regression.write_text(json.dumps(item))
        extra += ['--regression', regression]
    return run_register(monkeypatch, 'cards_v1', '--promote', path, *extra, '--registry', registry, '--suites', suites,
                        '--models', models, '--store', tmp_path / 'store')


def test_unreviewed_held_out_labels_never_qualify(tmp_path, monkeypatch, capsys):
    registry, suites, models = promote_setup(tmp_path, monkeypatch, 'unreviewed')
    assert promote(monkeypatch, tmp_path, registry, suites, models, scene_report('c' * 64, '5' * 64)) == 1
    assert 'never reviewed by a person' in capsys.readouterr().err
    assert yaml.safe_load(registry.read_text())['cards_v1']['status'] == 'experimental'


def test_promotion_requires_its_own_held_out_pass_and_no_regression_on_qualified_labels(tmp_path, monkeypatch, capsys):
    registry, suites, models = promote_setup(tmp_path, monkeypatch)
    for bad in (scene_report('c' * 64, '5' * 64, passed=False), scene_report('9' * 64, '5' * 64),
                scene_report('c' * 64, '0' * 64)):
        assert promote(monkeypatch, tmp_path, registry, suites, models, bad) == 1
    # The recognizer it replaces was qualified on a person suite; that suite must be re-run without regression.
    baseline = tmp_path / 'baseline.json'
    baseline.write_text(json.dumps(scene_report('b' * 64, '7' * 64, labels=('person',))))
    document = load_suites(suites)
    document['recognizers']['rtdetr_v2_r18'] = [{
        'name': 'people-heldout', 'manifest': '/data/people/scenes.json', 'manifest_sha256': '7' * 64, 'split': 'heldout',
        'labels': ['person'], 'qualified_model': 'rtdetr_v2_r18', 'report_sha256': store_report(baseline, tmp_path / 'store')}]
    suites.write_text(yaml.safe_dump(document))
    own = scene_report('c' * 64, '5' * 64)
    assert promote(monkeypatch, tmp_path, registry, suites, models, own) == 1
    assert 'no --regression report' in capsys.readouterr().err
    regressed = scene_report('c' * 64, '7' * 64, f1_drop=True, labels=('person',))
    assert promote(monkeypatch, tmp_path, registry, suites, models, own, regressed) == 1
    assert 'label person' in capsys.readouterr().err
    assert yaml.safe_load(registry.read_text())['cards_v1']['status'] == 'experimental'
    held = scene_report('c' * 64, '7' * 64, labels=('person',))
    assert promote(monkeypatch, tmp_path, registry, suites, models, own, held) == 0
    assert yaml.safe_load(registry.read_text())['cards_v1']['status'] == 'qualified'
    recorded = load_suites(suites)['recognizers']['cards_v1']
    assert [suite['name'] for suite in recorded] == ['people-heldout', 'cards-heldout']
    assert all(suite['qualified_model'] == 'cards_v1' for suite in recorded)


def real_runtime():
    if not PERCEPTION_PYTHON.is_file() or not (BASE_MODEL / 'model.safetensors').is_file():
        pytest.skip('needs the perception environment and rtdetr_v2_r18 (scripts/install-object-model.py --name rtdetr_v2_r18)')


def run_trainer(monkeypatch, dataset, output, *extra):
    monkeypatch.setattr('sys.argv', ['train-recognizer.py', str(dataset), '--output', str(output), '--steps', '3',
                                     '--batch-size', '2', '--checkpoint-every', '1', '--seed', '7', *map(str, extra)])
    return trainer.main()


def test_real_training_resumes_exactly_and_exports_a_runtime_loadable_artifact(tmp_path, monkeypatch):
    """T-54/T-55/T-56 with the real components: RT-DETR v2, transformers and the runtime Recognizer on CPU."""
    real_runtime()
    dataset = tmp_path / 'dataset'
    build.build(write_annotations(tmp_path, views(tmp_path)), dataset)
    # An interrupted run: the budget ends it after its first checkpoint.
    interrupted = tmp_path / 'interrupted'
    assert run_trainer(monkeypatch, dataset, interrupted, '--time-limit-min', '0.0001', '--skip-load-check') == 1
    run = json.loads((interrupted / 'run.json').read_text())
    assert run['termination'] == 'budget_exceeded' and (interrupted / 'checkpoints/latest').is_file()
    assert not (interrupted / 'manifest.json').exists()
    # A resume with another configuration is refused; the recorded one continues from the checkpoint.
    monkeypatch.setattr('sys.argv', ['train-recognizer.py', str(dataset), '--output', str(interrupted), '--resume',
                                     '--steps', '4', '--batch-size', '2', '--checkpoint-every', '1', '--seed', '7'])
    assert trainer.main() == 1
    assert run_trainer(monkeypatch, dataset, interrupted, '--resume') == 0
    manifest = json.loads((interrupted / 'manifest.json').read_text())
    metrics = [json.loads(line) for line in (interrupted / 'metrics.jsonl').read_text().splitlines()]
    assert [line['step'] for line in metrics] == [1, 2, 3] and manifest['training']['attempts'] == 2
    vocabulary = manifest['vocabulary']
    assert vocabulary['new_labels'] == ['checker card'] and vocabulary['labels'][-1] == 'checker card'
    base = json.loads((BASE_MODEL / 'config.json').read_text())['id2label']
    assert vocabulary['labels'][:80] == [base[str(index)] for index in range(80)]
    # The runtime class loads the exported files and reports the extended catalogue.
    loaded = manifest['artifact']['load_check']
    assert loaded['labels'] == vocabulary['labels'] and len(loaded['model_sha256']) == 64
    assert 'model.enc_score_head.weight' in manifest['compute']['measured']['extended_tensors']
    files = manifest['artifact']['spec']['files']
    assert files['model.safetensors']['sha256'] == training.digest(interrupted / 'model/model.safetensors')
    assert files['preprocessor_config.json']['sha256'] == training.digest(BASE_MODEL / 'preprocessor_config.json')
    # The same seed without an interruption reaches exactly the same weights.
    straight = tmp_path / 'straight'
    assert run_trainer(monkeypatch, dataset, straight, '--skip-load-check') == 0
    assert json.loads((straight / 'manifest.json').read_text())['artifact']['spec']['files']['model.safetensors'] == \
        files['model.safetensors']
