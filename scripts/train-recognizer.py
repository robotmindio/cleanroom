#!/usr/bin/env python3
"""Fine-tune a pinned RT-DETR v2 recognizer on a frozen perception dataset; never commands the robot.

The dataset is a scripts/build-perception-dataset.py directory; every file is verified against
its dataset.json before training. The trainer (config/trainers.yaml `rtdetr_detector`) trains
on the development split only and keeps the held-out split for scripts/evaluate-object-recognition.py.
The vocabulary keeps every base label at its base index and appends the dataset's new labels
(lekiwi_rmf/detector_finetune.py), so the catalogue and decoder of the base remain.

The output directory holds `run.json` (contract record and termination reason, whatever the
outcome), `train.log`, `metrics.jsonl`, `checkpoints/` (the latest resumable state) and, for a
completed run, `model/` (config.json, model.safetensors, the base's preprocessor_config.json)
and `manifest.json`: dataset identity, base artifact, vocabulary and ID mapping, configuration,
declared and measured compute, the SHA-256 and size of every model file, and a load check that
runs the exact runtime class (lekiwi_rmf.object_recognition.Recognizer) on a held-out image.

`--resume` continues an interrupted or failed run from its latest checkpoint with exactly the
recorded dataset and configuration. Backends: local (the perception environment from
scripts/install-object-model.py) or `--cluster`, the same one-pod Job mechanism as
scripts/train-skill.py (lekiwi_rmf/training.py) on the GPU node.
"""

import argparse
import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))
from lekiwi_rmf.training import (  # noqa: E402
    ClusterJob, RunRecord, TrainingError, add_cluster_arguments, check_cluster_arguments, cluster_failure, digest,
    files_digest, frozen_files, revision, tail, trainer_declaration, validate_config, write_json)

TRAINER = 'rtdetr_detector'
FINETUNE = REPOSITORY / 'lekiwi_rmf/detector_finetune.py'
# pytorch/pytorch:2.7.1-cuda12.6-cudnn9-runtime, the image scripts/train-skill.py pins; sm_60 kernels run on the GTX 1070.
IMAGE = 'docker.io/pytorch/pytorch@sha256:2b59b1b91885677814f78be1f8df48a25d5dc952eb6580eaecfefca510f9afd3'
# The runtime's own pins (config/requirements-perception.txt) plus the matcher's scipy.
PACKAGES = 'transformers==4.51.3 safetensors==0.5.3 Pillow==11.2.1 scipy==1.15.3'
LOCAL_PYTHON = Path.home() / '.ros/lekiwi/perception-venv/bin/python'
MODELS = Path.home() / '.ros/lekiwi/models'
INSTALL = r'''set -e
pip install --no-cache-dir --progress-bar off $PACKAGES
'''
RUN = r'''python job/detector_finetune.py job/job.json
'''
FAILURES = {13: 'the training budget ran out; the latest checkpoint was saved for --resume'}

# The exact runtime class on the exported artifact: what the robot would load, not a notebook checkpoint.
LOAD_CHECK = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
from lekiwi_rmf import object_recognition
from lekiwi_rmf.object_observations import artifact_pixels
spec = json.loads(sys.argv[3])
object_recognition.model_spec = lambda name: spec
model = object_recognition.Recognizer('candidate', sys.argv[2], allow_experimental=True)
data = open(sys.argv[4], 'rb').read()
rows = model.predict(artifact_pixels(data), artifact=data)
print(json.dumps({'labels': model.labels, 'detections': len(rows), 'model_sha256': model.sha256,
                  'image_sha256': sys.argv[4].rsplit('/', 1)[-1].split('.')[0]}))
'''


def verified_dataset(root):
    """The dataset manifest, after proving every file is exactly the one it lists."""
    path = root / 'dataset.json'
    if not path.is_file():
        raise TrainingError(f'{root}: not a perception dataset (dataset.json is missing)', 'invalid_input')
    manifest = json.loads(path.read_text())
    if manifest.get('kind') != 'lekiwi_perception_dataset' or manifest.get('input_normalization') != 'evidence_jpeg_q80':
        raise TrainingError(f'{root}: not a scripts/build-perception-dataset.py directory', 'invalid_input')
    files = frozen_files(root)
    listed = {name: item for name, item in files.items() if name != 'dataset.json'}
    if listed != manifest['files']:
        raise TrainingError(f'{root}: files differ from dataset.json; a frozen dataset never changes', 'invalid_input')
    return manifest, files


def base_artifact(name, directory):
    from lekiwi_rmf.object_recognition import model_spec, verify_model
    spec = model_spec(name)
    if spec.get('backend') != 'rtdetr_v2':
        raise TrainingError(f'{name} is not an rtdetr_v2 recognizer', 'invalid_input')
    directory = verify_model(directory or MODELS / name, spec)
    labels = json.loads((directory / 'config.json').read_text())['id2label']
    return spec, directory, [labels[str(index)] for index in range(len(labels))]


def job_document(args, paths, vocabulary, base_labels, input_size, config, budget_sec):
    return {**paths, 'vocabulary': vocabulary, 'base_labels': base_labels, 'input_size': input_size,
            'config': config, 'device': args.cluster_device if args.cluster else args.device, 'budget_sec': budget_sec}


def train_local(args, output, document):
    python = Path(args.python).expanduser()
    if not python.is_file():
        raise TrainingError(f'{python} is missing; run scripts/install-object-model.py --name rtdetr_v2_r18 first',
                            'invalid_input')
    write_json(output / 'job.json', document)
    with (output / 'train.log').open('a') as log:
        completed = subprocess.run([str(python), str(FINETUNE), str(output / 'job.json')], stdout=log,
                                   stderr=subprocess.STDOUT, check=False,
                                   env={**os.environ, 'PYTHONNOUSERSITE': '1', 'HF_HUB_OFFLINE': '1'})
    if completed.returncode:
        termination = 'budget_exceeded' if completed.returncode == 13 else 'failed'
        raise TrainingError(f'{FAILURES.get(completed.returncode, f"training failed ({completed.returncode})")}; '
                            f'see {output / "train.log"}:\n{tail(output / "train.log")}', termination)


class Cluster(ClusterJob):
    """The recognizer trainer's Job: the runtime's transformers pins over the pinned PyTorch image."""

    def __init__(self, args):
        super().__init__(args, prefix='recognizer-train', image=IMAGE, env={'PACKAGES': PACKAGES})

    def train(self, output, document, inputs):
        result = output / 'cluster-result'
        shutil.rmtree(result, ignore_errors=True)
        code = self.run({'detector_finetune.py': FINETUNE.read_text(), 'job.json': json.dumps(document),
                         'install.sh': INSTALL, 'run.sh': RUN}, inputs, result)
        # The pod's checkpoints replace the local ones, so a later --resume starts from the newest state.
        if (result / 'checkpoints/latest').is_file():
            shutil.rmtree(output / 'checkpoints', ignore_errors=True)
            shutil.move(result / 'checkpoints', output / 'checkpoints')
        for name in ('metrics.jsonl', 'compute.json', 'install.log'):
            if (result / name).is_file():
                shutil.copy(result / name, output / name)
        if (result / 'train.log').is_file():
            with (output / 'train.log').open('a') as log:
                log.write((result / 'train.log').read_text(errors='replace'))
        if code:
            error = cluster_failure(code, FAILURES, output)
            if code == 13:
                error.termination = 'budget_exceeded'
            raise error
        shutil.rmtree(output / 'model', ignore_errors=True)
        shutil.move(result / 'model', output / 'model')
        shutil.rmtree(result)


def candidate_spec(base_spec, files):
    """The recognition_models.yaml entry the artifact would have; scripts/register-recognizer.py writes it."""
    return {'backend': 'rtdetr_v2', 'input_size': base_spec['input_size'], 'prompt': '', 'files': files}


def load_check(python, model, spec, image):
    completed = subprocess.run([str(python), '-c', LOAD_CHECK, str(REPOSITORY), str(model), json.dumps(spec), str(image)],
                               capture_output=True, text=True, check=False,
                               env={**os.environ, 'PYTHONNOUSERSITE': '1', 'HF_HUB_OFFLINE': '1'})
    if completed.returncode:
        raise TrainingError(f'the exported model does not load in the runtime recognizer:\n{completed.stderr[-2000:]}')
    return json.loads(completed.stdout.strip().splitlines()[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset', type=Path, help='scripts/build-perception-dataset.py directory')
    parser.add_argument('--output', type=Path, required=True, help='new training directory (or the run to --resume)')
    parser.add_argument('--resume', action='store_true', help='continue the run in --output from its latest checkpoint')
    parser.add_argument('--base', default='rtdetr_v2_r18', help='pinned rtdetr_v2 recognizer to start from')
    parser.add_argument('--base-directory', type=Path, help='its verified files (default ~/.ros/lekiwi/models/BASE)')
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument('--device', choices=['cpu', 'cuda'], default='cpu', help='local training device')
    backend.add_argument('--cluster', action='store_true', help='train on the GPU cluster as a Kubernetes Job')
    add_cluster_arguments(parser)
    parser.add_argument('--steps', type=int, default=2000)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--learning-rate', type=float, default=1e-4)
    parser.add_argument('--backbone-learning-rate', type=float, default=1e-5)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--checkpoint-every', type=int, default=250)
    parser.add_argument('--no-flip', action='store_true', help='disable horizontal flip augmentation')
    parser.add_argument('--gpu-memory-gib', type=float, default=4.0, help='declared accelerator memory requirement')
    parser.add_argument('--time-limit-min', type=float, default=60.0, help='declared training time budget')
    parser.add_argument('--python', default=str(LOCAL_PYTHON), help='perception environment for local runs and the load check')
    parser.add_argument('--skip-load-check', action='store_true')
    args = parser.parse_args()
    if args.time_limit_min <= 0 or args.gpu_memory_gib <= 0:
        parser.error('budgets must be positive')
    check_cluster_arguments(parser, args)
    args.dataset = args.dataset.expanduser().resolve()
    output = args.output.expanduser().resolve()
    try:
        declaration = trainer_declaration(TRAINER)
        config = validate_config(declaration, {
            'steps': args.steps, 'batch_size': args.batch_size, 'learning_rate': args.learning_rate,
            'backbone_learning_rate': args.backbone_learning_rate, 'weight_decay': args.weight_decay,
            'seed': args.seed, 'checkpoint_every': args.checkpoint_every, 'horizontal_flip': not args.no_flip})
    except (TrainingError, ValueError) as error:
        parser.error(str(error))
    if args.resume != output.is_dir():
        parser.error('--resume needs the existing run directory' if args.resume else f'{output} exists; pass --resume')
    output.mkdir(parents=True, exist_ok=True)
    record = None
    try:
        dataset, files = verified_dataset(args.dataset)
        dataset_sha256 = files_digest(files)
        backend_name = 'cluster' if args.cluster else 'local'
        record = (RunRecord.resume if args.resume else RunRecord)(output, declaration, config, dataset_sha256, backend_name)
        record.start(device=args.cluster_device if args.cluster else args.device)
        started = datetime.datetime.now(datetime.UTC)
        base_spec, base_directory, base_labels = base_artifact(args.base, args.base_directory)
        new_labels = sorted(set(dataset['labels']) - set(base_labels))
        vocabulary = base_labels + new_labels
        budget = args.time_limit_min * 60
        if args.cluster:
            paths = {'base': '/work/base', 'dataset': '/work/dataset', 'output': '/work/result/model',
                     'checkpoints': '/work/result/checkpoints', 'metrics': '/work/result/metrics.jsonl',
                     'compute': '/work/result/compute.json', 'resume_from': None}
            inputs = {'dataset': (args.dataset, files), 'base': (base_directory, frozen_files(base_directory))}
            if (output / 'checkpoints/latest').is_file():
                paths['resume_from'] = '/work/resume'
                inputs['resume'] = (output / 'checkpoints', frozen_files(output / 'checkpoints'))
            document = job_document(args, paths, vocabulary, len(base_labels), base_spec['input_size'], config, budget)
            cluster = Cluster(args)
            try:
                cluster.train(output, document, inputs)
            finally:
                cluster.delete()
        else:
            paths = {'base': str(base_directory), 'dataset': str(args.dataset), 'output': str(output / 'model'),
                     'checkpoints': str(output / 'checkpoints'), 'metrics': str(output / 'metrics.jsonl'),
                     'compute': str(output / 'compute.json'), 'resume_from': None}
            document = job_document(args, paths, vocabulary, len(base_labels), base_spec['input_size'], config, budget)
            train_local(args, output, document)
        if frozen_files(args.dataset) != files:
            raise TrainingError('the dataset changed during training; the manifest would not describe it')
        model = output / 'model'
        exported = {name: {'sha256': digest(model / name), 'size': (model / name).stat().st_size}
                    for name in ('model.safetensors', 'config.json', 'preprocessor_config.json')}
        trained_labels = json.loads((model / 'config.json').read_text())['id2label']
        if [trained_labels[str(index)] for index in range(len(trained_labels))] != vocabulary:
            raise TrainingError('the exported vocabulary differs from the requested one')
        spec = candidate_spec(base_spec, exported)
        heldout = next(scene for scene in json.loads((args.dataset / 'scenes.json').read_text())
                       if scene['split'] == 'heldout')
        loaded = None if args.skip_load_check else load_check(Path(args.python).expanduser(), model, spec,
                                                              args.dataset / heldout['image'])
        if loaded is not None and loaded['labels'] != vocabulary:
            raise TrainingError('the runtime recognizer reports another catalogue than the trained vocabulary')
        measured = json.loads((output / 'compute.json').read_text())
        metrics = [json.loads(line) for line in (output / 'metrics.jsonl').read_text().splitlines() if line]
        steps = [line['step'] for line in metrics]
        if steps != list(range(1, args.steps + 1)):
            raise TrainingError('metrics do not cover every step exactly once')
        peak = measured.get('peak_gpu_memory_mib')
        device = args.cluster_device if args.cluster else args.device
        manifest = {
            'schema': 1, 'kind': 'lekiwi_recognizer_training', 'created_utc': started.isoformat(timespec='seconds'),
            'trainer': TRAINER, 'provenance': revision(REPOSITORY),
            'dataset': {'root': str(args.dataset), 'dataset_id': dataset['dataset_id'], 'sha256': dataset_sha256,
                        'scenes_sha256': dataset['scenes_sha256'], 'labels': dataset['labels'], 'splits': dataset['splits'],
                        'annotation_methods': dataset['annotation_methods'], 'trained_split': 'development',
                        'heldout_authority': dataset['heldout_authority']},
            'base': {'name': args.base, 'files': base_spec['files'], 'input_size': base_spec['input_size']},
            'vocabulary': {'labels': vocabulary, 'base_labels': len(base_labels), 'new_labels': new_labels,
                           'id2label': dict(enumerate(vocabulary))},
            'training': {**config, 'packages': PACKAGES, 'attempts': len(record.document['attempts'])},
            'compute': {'backend': backend_name, 'device': device, 'image': IMAGE if args.cluster else None,
                        'node': args.node if args.cluster else None,
                        'declared': {'gpu_memory_gib': args.gpu_memory_gib if device == 'cuda' else 0,
                                     'time_limit_min': args.time_limit_min},
                        'measured': measured,
                        'within_declared': peak is None or peak <= args.gpu_memory_gib * 1024},
            'metrics': {'steps': len(metrics), 'final_loss': metrics[-1]['loss'], 'first_loss': metrics[0]['loss']},
            'artifact': {'path': 'model', 'spec': spec, 'load_check': loaded},
        }
        write_json(output / 'manifest.json', manifest)
        record.finish('completed')
    except (TrainingError, OSError, ValueError, KeyError, StopIteration) as error:
        if record is not None:
            record.finish(getattr(error, 'termination', 'failed'), str(error))
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        if record is not None:
            record.finish('interrupted', 'interrupted; resume with --resume')
        print(f'{parser.prog}: interrupted; resume with --resume', file=sys.stderr)
        return 130
    print(json.dumps({'output': str(output), 'weights_sha256': exported['model.safetensors']['sha256'],
                      'new_labels': new_labels, 'final_loss': manifest['metrics']['final_loss'],
                      'compute': measured, 'load_check': loaded}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
