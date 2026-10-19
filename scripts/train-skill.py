#!/usr/bin/env python3
"""Train a skill policy from a LeRobot export (scripts/export-lerobot.py); never commands the robot.

The output directory holds `checkpoint/` (the LeRobot pretrained model), `train.log`
and `manifest.json`: the frozen dataset (every file with its SHA-256), the export
parameters, the source revision, the exact training arguments, the declared and
measured compute, and the checkpoint's artifact digest (every file the runner loads) that
scripts/register-skill.py pins.

Backends: local (`--device cpu|cuda`, in the LeRobot environment from
scripts/install-lerobot-export.sh) or `--cluster`, a one-pod Kubernetes Job on the
GPU node (`--cluster-device cpu` runs the same Job without claiming the GPU). The
dataset goes in and the checkpoint comes out with rsync over SSH through a
node-local directory the pod mounts; the pod verifies the dataset digests before
training, and the Job and the directory are removed afterwards, whatever the result.
The pod installs the pinned LeRobot release over a pinned PyTorch image (CUDA 12.6
still ships sm_60 kernels, which the GTX 1070, sm_61, runs).
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
import tempfile
import time

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))
from lekiwi_rmf.skills import artifact_digest  # noqa: E402

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

# The pod waits for its inputs, verifies them, trains, stages the result and waits to be collected.
POD = r'''set -u
cd /work
result=/work/result
mkdir -p "$result"
status=0
(cd dataset && sha256sum --check --quiet --strict ../job/dataset.sha256) > "$result/train.log" 2>&1 || status=10
if [ "$status" = 0 ]; then
  # LeRobot's pynput dependency builds evdev from source; the runtime image has no compiler.
  (apt-get update && apt-get install -y --no-install-recommends gcc libc6-dev) > "$result/pip.log" 2>&1 || status=11
fi
if [ "$status" = 0 ]; then
  pip install --no-cache-dir --progress-bar off "$LEROBOT" >> "$result/pip.log" 2>&1 || status=11
fi
if [ "$status" = 0 ]; then
  python -c "$(cat job/train.py)" "$result/compute.json" @job/arguments.json >> "$result/train.log" 2>&1 || status=$?
fi
if [ "$status" = 0 ]; then
  cp -rL output/checkpoints/last/pretrained_model "$result/checkpoint" || status=12
fi
# Hand everything back to the node user that created the directory, then publish the status last.
echo "$status" > "$result/status.tmp"
chown -R "$(stat -c %u:%g /work)" /work
mv "$result/status.tmp" "$result/status"
'''


class TrainingError(RuntimeError):
    pass


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dataset_files(root):
    """Every file of the export, so the manifest freezes exactly what was trained on."""
    files = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise TrainingError(f'{path}: symbolic links are not allowed in a frozen dataset')
        if path.is_file():
            files[path.relative_to(root).as_posix()] = {'sha256': digest(path), 'bytes': path.stat().st_size}
    return files


def dataset_summary(root):
    info_path = root / 'meta/info.json'
    if not info_path.is_file():
        raise TrainingError(f'{root}: not a LeRobot dataset (meta/info.json is missing)')
    info = json.loads(info_path.read_text())
    features = info.get('features', {})
    if 'action' not in features or 'observation.state' not in features:
        raise TrainingError(f'{root}: dataset lacks action or observation.state features')
    report = root / 'export-report.json'
    return {'fps': info.get('fps'), 'total_episodes': info.get('total_episodes'), 'total_frames': info.get('total_frames'),
            'action_names': features['action'].get('names'),
            'cameras': sorted(name.removeprefix('observation.images.') for name in features
                              if name.startswith('observation.images.')),
            'export': json.loads(report.read_text()) if report.is_file() else None}


def git(*command):
    return subprocess.run(['git', '-C', str(REPOSITORY), *command], capture_output=True, text=True,
                          check=False).stdout.strip()


def revision():
    return {'source_revision': git('rev-parse', 'HEAD') or None, 'dirty': bool(git('status', '--porcelain'))}


def lerobot_arguments(args, dataset_root, output_dir, device):
    return [f'--dataset.repo_id={args.repo_id}', f'--dataset.root={dataset_root}', '--dataset.video_backend=pyav',
            f'--policy.type={args.policy}', f'--policy.device={device}', '--policy.push_to_hub=false',
            f'--output_dir={output_dir}', f'--steps={args.steps}', f'--save_freq={args.steps}',
            f'--batch_size={args.batch_size}', f'--seed={args.seed}', f'--num_workers={args.num_workers}',
            f'--log_freq={max(1, min(200, args.steps // 10))}', '--wandb.enable=false', *args.lerobot_args]


def train_local(args, output):
    python = Path(args.python).expanduser()
    if not python.is_file():
        raise TrainingError(f'{python} is missing; run scripts/install-lerobot-export.sh first')
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


def tail(path, lines=20):
    return '\n'.join(path.read_text(errors='replace').splitlines()[-lines:]) if path.is_file() else '(no log)'


class Cluster:
    """One Job on one node. Data moves with rsync over SSH to a node-local directory the pod mounts:
    the workstation link to the cluster is slow, and kubectl exec/cp streams run far below the raw link."""

    def __init__(self, args):
        self.args = args
        self.name = f'skill-train-{datetime.datetime.now(datetime.UTC):%Y%m%d%H%M%S}-{secrets.token_hex(2)}'
        self.directory = f'/var/tmp/skill-train/{self.name}'
        self.pod = self.host = None

    def kubectl(self, *command, check=True, **kwargs):
        completed = subprocess.run(['kubectl', '--context', self.args.context, '-n', self.args.namespace, *command],
                                   capture_output=True, text=True, check=False, **kwargs)
        if check and completed.returncode:
            raise TrainingError(f'kubectl {" ".join(command[:2])} failed: {completed.stderr.strip() or completed.stdout.strip()}')
        return completed

    def ssh(self, command, check=True):
        completed = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=20', self.host, command],
                                   capture_output=True, text=True, check=False)
        if check and completed.returncode:
            raise TrainingError(f'ssh {self.host} failed: {completed.stderr.strip()}')
        return completed

    def rsync(self, source, target):
        completed = subprocess.run(['rsync', '-a', '--partial', '-e', 'ssh -o BatchMode=yes -o ConnectTimeout=20',
                                    source, target], capture_output=True, text=True, check=False)
        if completed.returncode:
            raise TrainingError(f'rsync {source} -> {target} failed: {completed.stderr.strip()}')

    def job(self):
        # The node's memory is mostly reserved by requests; ask for little and allow bursting to the limit.
        request, limit = f'{min(2, self.args.memory_gib)}Gi', f'{self.args.memory_gib}Gi'
        gpu = self.args.cluster_device == 'cuda'
        pod = {
            'restartPolicy': 'Never',
            # A hard pin: robotmind-morelos-1 also advertises a GPU, but its driver is broken.
            'affinity': {'nodeAffinity': {'requiredDuringSchedulingIgnoredDuringExecution': {'nodeSelectorTerms': [
                {'matchExpressions': [{'key': 'kubernetes.io/hostname', 'operator': 'In', 'values': [self.args.node]}]}]}}},
            'tolerations': [{'key': 'node.robotmind.io/secondary', 'operator': 'Exists', 'effect': 'PreferNoSchedule'}],
            'containers': [{
                'name': 'train', 'image': IMAGE, 'command': ['bash', '-c', POD],
                'env': [{'name': 'LEROBOT', 'value': LEROBOT}, {'name': 'HF_HUB_OFFLINE', 'value': '1'},
                        {'name': 'PIP_ROOT_USER_ACTION', 'value': 'ignore'}],
                'resources': {'requests': {'cpu': '2', 'memory': request, **({'nvidia.com/gpu': 1} if gpu else {})},
                              'limits': {'memory': limit, **({'nvidia.com/gpu': 1} if gpu else {})}},
                'volumeMounts': [{'name': 'work', 'mountPath': '/work'}, {'name': 'shm', 'mountPath': '/dev/shm'}],
            }],
            'volumes': [{'name': 'work', 'hostPath': {'path': self.directory, 'type': 'Directory'}},
                        {'name': 'shm', 'emptyDir': {'medium': 'Memory', 'sizeLimit': '2Gi'}}],
        }
        if gpu:
            pod['runtimeClassName'] = 'nvidia'
        return {
            'apiVersion': 'batch/v1', 'kind': 'Job',
            'metadata': {'name': self.name, 'labels': {'app.kubernetes.io/name': 'skill-train'}},
            'spec': {
                'backoffLimit': 0, 'ttlSecondsAfterFinished': 3600,
                # Slot wait and training budget, plus image pull and package install.
                # Pending time counts too, so the cluster removes a Job whose client vanished.
                'activeDeadlineSeconds': int((self.args.slot_wait_min + self.args.time_limit_min) * 60) + 1800,
                'template': {
                    'metadata': {'labels': {'app.kubernetes.io/name': 'skill-train'}},
                    'spec': pod}}}

    def pod_state(self):
        pods = json.loads(self.kubectl('get', 'pods', '-l', f'job-name={self.name}', '-o', 'json').stdout)['items']
        return pods[0] if pods else None

    def events(self):
        target = ('pod', self.pod) if self.pod else ('job', self.name)
        return self.kubectl('describe', *target, check=False).stdout[-2000:]

    def wait(self, deadline):
        """The pod's exit status. A pod that cannot be scheduled (both GPU slots taken) waits at most
        --slot-wait-min, reporting why; the caller deletes the Job on every exit."""
        started, reason = time.monotonic(), None
        while time.monotonic() < deadline:
            status = self.ssh(f'cat {self.directory}/result/status', check=False)
            if status.returncode == 0 and status.stdout.strip():
                return int(status.stdout.strip())
            pod = self.pod_state()
            if pod is not None:
                self.pod, phase = pod['metadata']['name'], pod['status'].get('phase')
                waiting = [item['state']['waiting'].get('reason') for item in pod['status'].get('containerStatuses', [])
                           if 'waiting' in item.get('state', {})]
                if set(waiting) & {'ErrImagePull', 'ImagePullBackOff', 'CreateContainerConfigError'}:
                    raise TrainingError(f'training pod did not start: {waiting}\n{self.events()}')
                blocked = [condition.get('message', '') for condition in pod['status'].get('conditions', [])
                           if condition.get('type') == 'PodScheduled' and condition.get('reason') == 'Unschedulable']
                if blocked:
                    if blocked[0] != reason:
                        reason = blocked[0]
                        print(f'cluster: waiting for a {"GPU slot" if self.args.cluster_device == "cuda" else "node slot"} '
                              f'on {self.args.node}: {reason}', file=sys.stderr)
                    if time.monotonic() - started > self.args.slot_wait_min * 60:
                        raise TrainingError(f'no slot on {self.args.node} within {self.args.slot_wait_min} min: {reason}')
                    time.sleep(10)
                    continue
                if phase in ('Failed', 'Succeeded'):
                    time.sleep(2)  # the status file is published just before the container exits
                    if not self.ssh(f'test -s {self.directory}/result/status', check=False).returncode:
                        continue
                    raise TrainingError(f'training pod ended without a result ({phase})\n{self.events()}')
            time.sleep(10)
        raise TrainingError(f'training exceeded its declared {self.args.time_limit_min} min budget')

    def run(self, output, files):
        address = self.kubectl('get', 'node', self.args.node, '-o',
                               'jsonpath={.status.addresses[?(@.type=="InternalIP")].address}').stdout.strip()
        if not address:
            raise TrainingError(f'node {self.args.node} has no internal address')
        self.host = address
        with tempfile.TemporaryDirectory(prefix='skill-train-', dir=output) as staging:
            job = Path(staging) / 'job'
            job.mkdir()
            (job / 'train.py').write_text(TRAIN)
            (job / 'dataset.sha256').write_text(''.join(f'{item["sha256"]}  {name}\n' for name, item in files.items()))
            arguments = lerobot_arguments(self.args, '/work/dataset', '/work/output', self.args.cluster_device)
            (job / 'arguments.json').write_text(json.dumps(arguments))
            self.ssh(f'mkdir -p {self.directory}')
            self.rsync(f'{job}/', f'{self.host}:{self.directory}/job/')
            self.rsync(f'{self.args.dataset}/', f'{self.host}:{self.directory}/dataset/')
            print(f'cluster: dataset staged on {self.args.node}:{self.directory}', file=sys.stderr)
            self.kubectl('create', '-f', '-', input=json.dumps(self.job()))
            print(f'cluster: job {self.name} created ({self.args.cluster_device})', file=sys.stderr)
            code = self.wait(time.monotonic() + (self.args.slot_wait_min + self.args.time_limit_min) * 60 + 1800)
            result = Path(staging) / 'result'
            self.rsync(f'{self.host}:{self.directory}/result/', f'{result}/')
            for name in ('train.log', 'pip.log', 'compute.json'):
                if (result / name).is_file():
                    shutil.copy(result / name, output / name)
            if code:
                reason = {10: 'dataset digests did not verify inside the pod', 11: f'installing {LEROBOT} failed',
                          12: 'training produced no last checkpoint'}.get(code, f'training exited {code}')
                log = output / ('pip.log' if code == 11 else 'train.log')
                raise TrainingError(f'{reason}; see {log}:\n{tail(log)}')
            shutil.copytree(result / 'checkpoint', output / 'checkpoint')
            return arguments, json.loads((result / 'compute.json').read_text())

    def delete(self):
        completed = self.kubectl('delete', 'job', self.name, '--ignore-not-found', '--wait=false', check=False)
        if completed.returncode:
            print(f'warning: could not delete job {self.name}: {completed.stderr.strip()}', file=sys.stderr)
        if self.host is not None and self.ssh(f'rm -rf {self.directory}', check=False).returncode:
            print(f'warning: could not remove {self.host}:{self.directory}', file=sys.stderr)


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
    parser.add_argument('--cluster-device', choices=['cuda', 'cpu'], default='cuda',
                        help='cpu runs the same Job image on the same node without claiming the GPU')
    parser.add_argument('--policy', default='act')
    parser.add_argument('--steps', type=int, default=20000)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--num-workers', type=int, default=2)
    parser.add_argument('--repo-id', default='local/lekiwi')
    parser.add_argument('--gpu-memory-gib', type=float, default=4.0, help='declared accelerator memory requirement')
    parser.add_argument('--time-limit-min', type=float, default=120.0, help='declared training time budget')
    parser.add_argument('--memory-gib', type=int, default=8, help='cluster pod memory')
    parser.add_argument('--slot-wait-min', type=float, default=30.0, help='how long a Job may wait for a GPU slot')
    parser.add_argument('--context', default='morelos')
    parser.add_argument('--namespace', default='robotmind')
    # robotmind-morelos-1 also advertises a GPU, but its driver is broken.
    parser.add_argument('--node', default='robotmind-web-001')
    parser.add_argument('--python', default=str(LOCAL_PYTHON), help='LeRobot environment for local training and the load check')
    parser.add_argument('--skip-load-check', action='store_true')
    parser.add_argument('lerobot_args', nargs='*', help='extra lerobot_train arguments after --')
    args = parser.parse_args()
    if args.steps < 1 or args.batch_size < 1 or args.num_workers < 0 or args.time_limit_min <= 0 \
            or args.gpu_memory_gib <= 0 or args.slot_wait_min < 0:
        parser.error('steps and batch size must be positive, workers nonnegative, budgets positive')
    if not 1 <= args.memory_gib <= 24:
        parser.error('pod memory must be 1..24 GiB')
    args.dataset = args.dataset.expanduser().resolve()
    try:
        summary = dataset_summary(args.dataset)
        files = dataset_files(args.dataset)
        output = args.output.expanduser()
        output.mkdir(parents=True, exist_ok=False)
        started = datetime.datetime.now(datetime.UTC)
        if args.cluster:
            missing = [tool for tool in ('kubectl', 'ssh', 'rsync') if shutil.which(tool) is None]
            if missing:
                raise TrainingError(f'cluster training needs {", ".join(missing)}')
            cluster = Cluster(args)
            try:
                arguments, measured = cluster.run(output, files)
            finally:
                cluster.delete()
        else:
            arguments, measured = train_local(args, output)
            if dataset_files(args.dataset) != files:
                raise TrainingError('the dataset changed during training; the manifest would not describe it')
        weights = output / 'checkpoint/model.safetensors'
        if not weights.is_file():
            raise TrainingError('the checkpoint has no model.safetensors')
        artifact = artifact_digest(output / 'checkpoint')
        loaded = None if args.skip_load_check else load_check(Path(args.python).expanduser(), output / 'checkpoint')
        peak = measured.get('peak_gpu_memory_mib')
        manifest = {
            'schema': 1, 'kind': 'lekiwi_skill_training', 'created_utc': started.isoformat(timespec='seconds'),
            'dataset': {'root': str(args.dataset), 'files': files, **summary},
            'provenance': revision(),
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
        temporary = output / 'manifest.json.tmp'
        temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        os.replace(temporary, output / 'manifest.json')
    except (TrainingError, OSError, ValueError) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(f'{parser.prog}: interrupted; no manifest was written', file=sys.stderr)
        return 130
    print(json.dumps({'output': str(output), 'artifact_sha256': manifest['checkpoint']['artifact_sha256'],
                      'compute': manifest['compute']['measured'], 'load_check': loaded}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
