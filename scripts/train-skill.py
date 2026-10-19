#!/usr/bin/env python3
"""Train a skill policy from a LeRobot export (scripts/export-lerobot.py); never commands the robot.

The output directory holds `checkpoint/` (the LeRobot pretrained model), `train.log`,
`run.json` (the trainer contract record: declaration, configuration, dataset digest and
termination reason, written whatever the outcome) and, for a completed run only,
`manifest.json`: the frozen dataset (every file with its SHA-256), the export
parameters, the source revision, the exact training arguments, the declared and
measured compute, and the checkpoint's artifact digest (every file the runner loads) that
scripts/register-skill.py pins.

Backends: local (`--device cpu|cuda`, in the LeRobot environment from
scripts/install-lerobot-export.sh) or `--cluster`, a one-pod Kubernetes Job on the
GPU node (lekiwi_rmf/training.py ClusterJob; `--cluster-device cpu` runs the same Job
without claiming the GPU). The pod installs the pinned LeRobot release over a pinned
PyTorch image (CUDA 12.6 still ships sm_60 kernels, which the GTX 1070, sm_61, runs).
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
from lekiwi_rmf.skills import artifact_digest  # noqa: E402

TRAINER = 'lerobot_policy'
LEROBOT = 'lerobot==0.4.1'
# pytorch/pytorch:2.7.1-cuda12.6-cudnn9-runtime; its torch matches LeRobot 0.4.1's torch<2.8 pin.
IMAGE = 'docker.io/pytorch/pytorch@sha256:2b59b1b91885677814f78be1f8df48a25d5dc952eb6580eaecfefca510f9afd3'
LOCAL_PYTHON = Path.home() / '.ros/lekiwi/lerobot-venv/bin/python'

# Runs LeRobot's training entry point in-process so peak accelerator memory is measured, not guessed.
TRAIN = r'''
import importlib.metadata, json, sys, time
import torch
from lerobot.scripts.lerobot_train import main
compute_path = sys.argv[1]
arguments = sys.argv[2:]
if len(arguments) == 1 and arguments[0].startswith('@'):
    arguments = json.load(open(arguments[0][1:]))
sys.argv = ['lerobot_train', *arguments]
started = time.monotonic()
main()
compute = {'elapsed_sec': round(time.monotonic() - started, 1), 'torch': torch.__version__,
           'lerobot': importlib.metadata.version('lerobot'), 'cuda': torch.version.cuda, 'device_name': 'cpu'}
if torch.cuda.is_available():
    compute.update(device_name=torch.cuda.get_device_name(0), capability='sm_%d%d' % torch.cuda.get_device_capability(0),
                   peak_gpu_memory_mib=round(torch.cuda.max_memory_allocated() / 2**20))
open(compute_path, 'w').write(json.dumps(compute))
'''

# Proves the returned checkpoint loads the way lekiwi_rmf/skill_runner.py LeRobotPolicy loads it (on CPU)
# and produces a chunk; keep the two in step.
LOAD_CHECK = r'''
import json, sys
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
path = sys.argv[1]
config = PreTrainedConfig.from_pretrained(path)
config.device = 'cpu'
policy = get_policy_class(config.type).from_pretrained(path, config=config).eval()
# Same CPU overrides as LeRobotPolicy: a cuda-trained checkpoint must run on the CPU compute host.
cpu = {'device_processor': {'device': 'cpu'}}
pre, post = make_pre_post_processors(policy.config, pretrained_path=path, preprocessor_overrides=cpu,
                                     postprocessor_overrides=cpu)
observation = {name: torch.zeros(feature.shape) for name, feature in policy.config.input_features.items()}
with torch.inference_mode():
    chunk = post(policy.predict_action_chunk(pre({**observation, 'task': ''})))
print(json.dumps({'policy': config.type, 'chunk_shape': list(chunk.shape)}))
'''

# The trainer's steps inside the shared pod prelude, which has already verified the dataset digests.
# LeRobot's pynput dependency builds evdev from source; the runtime image has no compiler.
INSTALL = r'''set -e
apt-get update
apt-get install -y --no-install-recommends gcc libc6-dev
pip install --no-cache-dir --progress-bar off "$LEROBOT"
'''
RUN = r'''set -e
python job/train.py /work/result/compute.json @job/arguments.json
cp -rL output/checkpoints/last/pretrained_model /work/result/checkpoint || exit 12
'''


def dataset_summary(root):
    info_path = root / 'meta/info.json'
    if not info_path.is_file():
        raise TrainingError(f'{root}: not a LeRobot dataset (meta/info.json is missing)', 'invalid_input')
    info = json.loads(info_path.read_text())
    features = info.get('features', {})
    if 'action' not in features or 'observation.state' not in features:
        raise TrainingError(f'{root}: dataset lacks action or observation.state features', 'invalid_input')
    report = root / 'export-report.json'
    return {'fps': info.get('fps'), 'total_episodes': info.get('total_episodes'), 'total_frames': info.get('total_frames'),
            'action_names': features['action'].get('names'),
            'cameras': sorted(name.removeprefix('observation.images.') for name in features
                              if name.startswith('observation.images.')),
            'export': json.loads(report.read_text()) if report.is_file() else None}


def lerobot_arguments(args, dataset_root, output_dir, device):
    return [f'--dataset.repo_id={args.repo_id}', f'--dataset.root={dataset_root}', '--dataset.video_backend=pyav',
            f'--policy.type={args.policy}', f'--policy.device={device}', '--policy.push_to_hub=false',
            f'--output_dir={output_dir}', f'--steps={args.steps}', f'--save_freq={args.steps}',
            f'--batch_size={args.batch_size}', f'--seed={args.seed}', f'--num_workers={args.num_workers}',
            f'--log_freq={max(1, min(200, args.steps // 10))}', '--wandb.enable=false', *args.lerobot_args]


def train_local(args, output):
    python = Path(args.python).expanduser()
    if not python.is_file():
        raise TrainingError(f'{python} is missing; run scripts/install-lerobot-export.sh first', 'invalid_input')
    arguments = lerobot_arguments(args, args.dataset, output / 'lerobot', args.device)
    with (output / 'train.log').open('w') as log:
        completed = subprocess.run([str(python), '-c', TRAIN, str(output / 'compute.json'), *arguments],
                                   stdout=log, stderr=subprocess.STDOUT, check=False,
                                   env={**os.environ, 'PYTHONNOUSERSITE': '1', 'HF_HUB_OFFLINE': '1'})
    if completed.returncode:
        raise TrainingError(f'training failed ({completed.returncode}); see {output / "train.log"}:\n{tail(output / "train.log")}')
    shutil.copytree(output / 'lerobot/checkpoints/last/pretrained_model', output / 'checkpoint')
    shutil.rmtree(output / 'lerobot')
    return arguments, json.loads((output / 'compute.json').read_text())


class Cluster(ClusterJob):
    """The skill trainer's Job: the pinned LeRobot release over the pinned PyTorch image."""

    def __init__(self, args):
        super().__init__(args, prefix='skill-train', image=IMAGE, env={'LEROBOT': LEROBOT})

    def train(self, output, files):
        arguments = lerobot_arguments(self.args, '/work/dataset', '/work/output', self.args.cluster_device)
        result = output / 'cluster-result'
        code = self.run({'train.py': TRAIN, 'install.sh': INSTALL, 'run.sh': RUN,
                         'arguments.json': json.dumps(arguments)}, {'dataset': (self.args.dataset, files)}, result)
        for name in ('train.log', 'install.log', 'compute.json'):
            if (result / name).is_file():
                shutil.copy(result / name, output / name)
        if code:
            raise cluster_failure(code, {12: 'training produced no last checkpoint'}, output)
        shutil.copytree(result / 'checkpoint', output / 'checkpoint')
        shutil.rmtree(result)
        return arguments, json.loads((output / 'compute.json').read_text())


def load_check(python, checkpoint):
    completed = subprocess.run([str(python), '-c', LOAD_CHECK, str(checkpoint)], capture_output=True, text=True,
                               check=False, env={**os.environ, 'PYTHONNOUSERSITE': '1', 'HF_HUB_OFFLINE': '1'})
    if completed.returncode:
        raise TrainingError(f'the checkpoint does not load:\n{completed.stderr[-2000:]}')
    return json.loads(completed.stdout.strip().splitlines()[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset', type=Path, help='LeRobot dataset root from scripts/export-lerobot.py')
    parser.add_argument('--output', type=Path, required=True, help='new training directory')
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument('--device', choices=['cpu', 'cuda'], default='cpu', help='local training device')
    backend.add_argument('--cluster', action='store_true', help='train on the GPU cluster as a Kubernetes Job')
    add_cluster_arguments(parser)
    parser.add_argument('--policy', default='act')
    parser.add_argument('--steps', type=int, default=20000)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--num-workers', type=int, default=2)
    parser.add_argument('--repo-id', default='local/lekiwi')
    parser.add_argument('--gpu-memory-gib', type=float, default=4.0, help='declared accelerator memory requirement')
    parser.add_argument('--time-limit-min', type=float, default=120.0, help='declared training time budget')
    parser.add_argument('--python', default=str(LOCAL_PYTHON), help='LeRobot environment for local training and the load check')
    parser.add_argument('--skip-load-check', action='store_true')
    parser.add_argument('lerobot_args', nargs='*', help='extra lerobot_train arguments after --')
    args = parser.parse_args()
    if args.time_limit_min <= 0 or args.gpu_memory_gib <= 0:
        parser.error('budgets must be positive')
    check_cluster_arguments(parser, args)
    args.dataset = args.dataset.expanduser().resolve()
    output = args.output.expanduser()
    try:
        declaration = trainer_declaration(TRAINER)
        config = validate_config(declaration, {'policy': args.policy, 'steps': args.steps, 'batch_size': args.batch_size,
                                               'seed': args.seed, 'num_workers': args.num_workers})
    except (TrainingError, ValueError) as error:
        parser.error(str(error))
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    record = None
    try:
        summary = dataset_summary(args.dataset)
        files = frozen_files(args.dataset)
        record = RunRecord(output, declaration, config, files_digest(files), 'cluster' if args.cluster else 'local')
        record.start(device=args.cluster_device if args.cluster else args.device)
        started = datetime.datetime.now(datetime.UTC)
        if args.cluster:
            cluster = Cluster(args)
            try:
                arguments, measured = cluster.train(output, files)
            finally:
                cluster.delete()
        else:
            arguments, measured = train_local(args, output)
            if frozen_files(args.dataset) != files:
                raise TrainingError('the dataset changed during training; the manifest would not describe it')
        weights = output / 'checkpoint/model.safetensors'
        if not weights.is_file():
            raise TrainingError('the checkpoint has no model.safetensors')
        artifact = artifact_digest(output / 'checkpoint')
        loaded = None if args.skip_load_check else load_check(Path(args.python).expanduser(), output / 'checkpoint')
        peak = measured.get('peak_gpu_memory_mib')
        manifest = {
            'schema': 1, 'kind': 'lekiwi_skill_training', 'created_utc': started.isoformat(timespec='seconds'),
            'trainer': TRAINER, 'dataset': {'root': str(args.dataset), 'files': files, **summary},
            'provenance': revision(REPOSITORY),
            'training': {'policy': args.policy, 'steps': args.steps, 'batch_size': args.batch_size, 'seed': args.seed,
                         'lerobot_arguments': arguments, 'package': LEROBOT},
            'compute': {
                'backend': 'cluster' if args.cluster else 'local',
                'device': args.cluster_device if args.cluster else args.device,
                'image': IMAGE if args.cluster else None,
                'node': args.node if args.cluster else None,
                'declared': {'gpu_memory_gib': args.gpu_memory_gib if (
                    args.cluster_device if args.cluster else args.device) == 'cuda' else 0,
                             'time_limit_min': args.time_limit_min},
                'measured': measured,
                'within_declared': (measured['elapsed_sec'] <= args.time_limit_min * 60
                                    and (peak is None or peak <= args.gpu_memory_gib * 1024)),
            },
            'checkpoint': {'path': 'checkpoint', 'weights_sha256': digest(weights), 'artifact_sha256': artifact,
                           'load_check': loaded},
        }
        write_json(output / 'manifest.json', manifest)
        record.finish('completed')
    except (TrainingError, OSError, ValueError) as error:
        if record is not None:
            record.finish(getattr(error, 'termination', 'failed'), str(error))
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        if record is not None:
            record.finish('interrupted')
        print(f'{parser.prog}: interrupted; no manifest was written', file=sys.stderr)
        return 130
    print(json.dumps({'output': str(output), 'artifact_sha256': manifest['checkpoint']['artifact_sha256'],
                      'compute': manifest['compute']['measured'], 'load_check': loaded}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
