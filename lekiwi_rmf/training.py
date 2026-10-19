"""The trainer contract and its cluster backend, shared by every trainer script; never commands the robot.

A trainer (config/trainers.yaml) declares its model types, modalities, data format, body
compatibility, configuration schema, resources, checkpoints and outputs. A run receives a
frozen dataset and a validated configuration and publishes `run.json` from the start: the
declaration, the configuration, the dataset digest, then the termination reason, whatever
the outcome. Only a completed run also writes the trainer's `manifest.json`.

`ClusterJob` runs one trainer as a one-pod Kubernetes Job on the GPU node. Inputs go in and
results come back with rsync over SSH through a node-local directory the pod mounts; the pod
verifies every input digest before the trainer's own install and run steps, results come back
even when the trainer fails (so checkpoints survive for a resume), and the Job and the
directory are removed afterwards.
"""

import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
import tempfile
import time

import yaml

from lekiwi_rmf.object_cameras import config_directory

TERMINATIONS = ('completed', 'failed', 'interrupted', 'budget_exceeded', 'no_slot', 'invalid_input')
CONFIG_TYPES = {'integer': (int,), 'number': (int, float), 'string': (str,), 'boolean': (bool,)}
# Exit codes the shared pod prelude reserves; a trainer's run step uses any other nonzero code.
INPUTS_UNVERIFIED, INSTALL_FAILED = 10, 11

# The pod verifies its inputs, runs the trainer's install and run steps, and publishes the status last.
POD = r'''set -u
cd /work
result=/work/result
mkdir -p "$result"
status=0
sha256sum --check --quiet --strict job/inputs.sha256 > "$result/train.log" 2>&1 || status=10
if [ "$status" = 0 ]; then
  bash job/install.sh > "$result/install.log" 2>&1 || status=11
fi
if [ "$status" = 0 ]; then
  bash job/run.sh >> "$result/train.log" 2>&1 || status=$?
fi
# Hand everything back to the node user that created the directory, then publish the status last.
echo "$status" > "$result/status.tmp"
chown -R "$(stat -c %u:%g /work)" /work
mv "$result/status.tmp" "$result/status"
'''


class TrainingError(RuntimeError):
    """A run that cannot complete; `termination` is the reason run.json records."""

    def __init__(self, message, termination='failed'):
        super().__init__(message)
        self.termination = termination


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def frozen_files(root):
    """Every file under `root` with its SHA-256, so a manifest freezes exactly what was used."""
    files = {}
    for path in sorted(Path(root).rglob('*')):
        if path.is_symlink():
            raise TrainingError(f'{path}: symbolic links are not allowed in a frozen dataset', 'invalid_input')
        if path.is_file():
            files[path.relative_to(root).as_posix()] = {'sha256': digest(path), 'bytes': path.stat().st_size}
    return files


def files_digest(files):
    """One digest over a frozen file listing: the dataset identity a resume must match."""
    return hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def revision(repository):
    def git(*command):
        return subprocess.run(['git', '-C', str(repository), *command], capture_output=True, text=True,
                              check=False).stdout.strip()
    return {'source_revision': git('rev-parse', 'HEAD') or None, 'dirty': bool(git('status', '--porcelain'))}


def tail(path, lines=20):
    path = Path(path)
    return '\n'.join(path.read_text(errors='replace').splitlines()[-lines:]) if path.is_file() else '(no log)'


def write_json(path, document):
    """Atomic replace, so a reader never sees a partial manifest or run record."""
    path = Path(path)
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + '\n')
    os.replace(temporary, path)


def trainer_declaration(name, path=None):
    """The tracked declaration of one trainer; a malformed file rejects every trainer."""
    document = yaml.safe_load(Path(path or config_directory() / 'trainers.yaml').read_text())
    if not isinstance(document, dict) or set(document) != {'trainers'} or not isinstance(document['trainers'], dict):
        raise ValueError('trainer declarations must be a mapping with only `trainers`')
    keys = {'script', 'model_types', 'modalities', 'data_format', 'body', 'config', 'resources', 'checkpoints',
            'outputs'}
    for trainer, value in document['trainers'].items():
        if not isinstance(value, dict) or set(value) != keys:
            raise ValueError(f'trainer {trainer}: fields must be exactly {sorted(keys)}')
        for field in ('model_types', 'modalities', 'outputs'):
            if not isinstance(value[field], list) or not value[field] or not all(isinstance(v, str) for v in value[field]):
                raise ValueError(f'trainer {trainer}: {field} must be a non-empty list of names')
        for setting, spec in value['config'].items():
            if not isinstance(spec, dict) or spec.get('type') not in CONFIG_TYPES or set(spec) - {'type', 'min', 'max', 'enum'}:
                raise ValueError(f'trainer {trainer}: config {setting} needs a type and optional min, max, enum')
        if not isinstance(value['checkpoints'], dict) or type(value['checkpoints'].get('resumable')) is not bool:
            raise ValueError(f'trainer {trainer}: checkpoints must state whether runs resume')
    if name not in document['trainers']:
        raise ValueError(f'no trainer declaration for {name}')
    return {'name': name, **document['trainers'][name]}


def validate_config(declaration, config):
    """The run configuration exactly as the declaration's schema admits it."""
    schema = declaration['config']
    if set(config) != set(schema):
        raise TrainingError(f'{declaration["name"]} configuration must set exactly {sorted(schema)}', 'invalid_input')
    for name, value in config.items():
        spec = schema[name]
        if type(value) not in CONFIG_TYPES[spec['type']] or (
                isinstance(value, float) and not math.isfinite(value)):
            raise TrainingError(f'{name} must be {spec["type"]}', 'invalid_input')
        if 'enum' in spec and value not in spec['enum']:
            raise TrainingError(f'{name} must be one of {spec["enum"]}', 'invalid_input')
        numeric = spec['type'] in ('integer', 'number')
        if numeric and not spec.get('min', -math.inf) <= value <= spec.get('max', math.inf):
            raise TrainingError(f'{name} is outside its declared range', 'invalid_input')
    return config


class RunRecord:
    """`run.json`: written when a run starts and again with its termination reason when it ends."""

    def __init__(self, output, declaration, config, dataset_sha256, backend):
        self.path = Path(output) / 'run.json'
        self.document = {'schema': 1, 'kind': 'lekiwi_training_run', 'trainer': declaration, 'config': config,
                         'dataset_sha256': dataset_sha256, 'backend': backend, 'attempts': []}

    @classmethod
    def resume(cls, output, declaration, config, dataset_sha256, backend):
        """The same run continued: refused unless the dataset and configuration are exactly the recorded ones."""
        path = Path(output) / 'run.json'
        if not path.is_file():
            raise TrainingError(f'{output} has no run.json to resume', 'invalid_input')
        previous = json.loads(path.read_text())
        if previous.get('dataset_sha256') != dataset_sha256:
            raise TrainingError('the dataset differs from the run being resumed; start a new run', 'invalid_input')
        if previous.get('config') != config or previous.get('trainer', {}).get('name') != declaration['name']:
            raise TrainingError('the configuration differs from the run being resumed; start a new run', 'invalid_input')
        if previous.get('termination') == 'completed':
            raise TrainingError('that run already completed', 'invalid_input')
        record = cls(output, declaration, config, dataset_sha256, backend)
        record.document['attempts'] = previous.get('attempts', [])
        return record

    def start(self, **details):
        self.document['attempts'].append({'started_utc': now(), **details})
        self.document.update(status='running', termination=None, detail=None)
        write_json(self.path, self.document)

    def finish(self, termination, detail=''):
        if termination not in TERMINATIONS:
            raise ValueError(f'unknown termination {termination}')
        self.document['attempts'][-1].update(ended_utc=now(), termination=termination, detail=detail[-2000:])
        self.document.update(status='ended', termination=termination, detail=detail[-2000:])
        write_json(self.path, self.document)


def now():
    return datetime.datetime.now(datetime.UTC).isoformat(timespec='seconds')


def add_cluster_arguments(parser):
    parser.add_argument('--cluster-device', choices=['cuda', 'cpu'], default='cuda',
                        help='cpu runs the same Job image on the same node without claiming the GPU')
    parser.add_argument('--memory-gib', type=int, default=8, help='cluster pod memory')
    parser.add_argument('--slot-wait-min', type=float, default=30.0, help='how long a Job may wait for a GPU slot')
    parser.add_argument('--context', default='morelos')
    parser.add_argument('--namespace', default='robotmind')
    # robotmind-morelos-1 also advertises a GPU, but its driver is broken.
    parser.add_argument('--node', default='robotmind-web-001')


def check_cluster_arguments(parser, args):
    if not 1 <= args.memory_gib <= 24:
        parser.error('pod memory must be 1..24 GiB')
    if args.slot_wait_min < 0:
        parser.error('slot wait must be nonnegative')
    missing = [tool for tool in ('kubectl', 'ssh', 'rsync') if shutil.which(tool) is None] if args.cluster else []
    if missing:
        parser.error(f'cluster training needs {", ".join(missing)}')


class ClusterJob:
    """One Job on one node. Data moves with rsync over SSH to a node-local directory the pod mounts:
    the workstation link to the cluster is slow, and kubectl exec/cp streams run far below the raw link."""

    def __init__(self, settings, *, prefix, image, env=None):
        self.args, self.prefix, self.image, self.env = settings, prefix, image, dict(env or {})
        self.name = f'{prefix}-{datetime.datetime.now(datetime.UTC):%Y%m%d%H%M%S}-{secrets.token_hex(2)}'
        self.directory = f'/var/tmp/{prefix}/{self.name}'
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
        labels = {'app.kubernetes.io/name': self.prefix}
        pod = {
            'restartPolicy': 'Never',
            # A hard pin: robotmind-morelos-1 also advertises a GPU, but its driver is broken.
            'affinity': {'nodeAffinity': {'requiredDuringSchedulingIgnoredDuringExecution': {'nodeSelectorTerms': [
                {'matchExpressions': [{'key': 'kubernetes.io/hostname', 'operator': 'In', 'values': [self.args.node]}]}]}}},
            'tolerations': [{'key': 'node.robotmind.io/secondary', 'operator': 'Exists', 'effect': 'PreferNoSchedule'}],
            'containers': [{
                'name': 'train', 'image': self.image, 'command': ['bash', '-c', POD],
                'env': [{'name': key, 'value': value} for key, value in
                        {'HF_HUB_OFFLINE': '1', 'PIP_ROOT_USER_ACTION': 'ignore', **self.env}.items()],
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
            'apiVersion': 'batch/v1', 'kind': 'Job', 'metadata': {'name': self.name, 'labels': labels},
            'spec': {
                'backoffLimit': 0, 'ttlSecondsAfterFinished': 3600,
                # Slot wait and training budget, plus image pull and package install.
                # Pending time counts too, so the cluster removes a Job whose client vanished.
                'activeDeadlineSeconds': int((self.args.slot_wait_min + self.args.time_limit_min) * 60) + 1800,
                'template': {'metadata': {'labels': labels}, 'spec': pod}}}

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
                        raise TrainingError(f'no slot on {self.args.node} within {self.args.slot_wait_min} min: {reason}',
                                            'no_slot')
                    time.sleep(10)
                    continue
                if phase in ('Failed', 'Succeeded'):
                    time.sleep(2)  # the status file is published just before the container exits
                    if not self.ssh(f'test -s {self.directory}/result/status', check=False).returncode:
                        continue
                    raise TrainingError(f'training pod ended without a result ({phase})\n{self.events()}')
            time.sleep(10)
        raise TrainingError(f'training exceeded its declared {self.args.time_limit_min} min budget', 'budget_exceeded')

    def run(self, job_files, inputs, result):
        """Stage `job_files` (name -> text, including install.sh and run.sh) and `inputs` (name -> (local
        directory, its frozen_files listing), mounted at /work/<name>), run the Job and copy
        /work/result back into `result`.
        Returns the pod's exit status; the result comes back whatever that status is."""
        address = self.kubectl('get', 'node', self.args.node, '-o',
                               'jsonpath={.status.addresses[?(@.type=="InternalIP")].address}').stdout.strip()
        if not address:
            raise TrainingError(f'node {self.args.node} has no internal address')
        self.host = address
        with tempfile.TemporaryDirectory(prefix=f'{self.prefix}-') as staging:
            job = Path(staging) / 'job'
            job.mkdir()
            for name, text in job_files.items():
                (job / name).write_text(text)
            # Every input file is verified inside the pod before the trainer sees it.
            (job / 'inputs.sha256').write_text(''.join(
                f'{item["sha256"]}  {name}/{path}\n' for name, (_, files) in inputs.items()
                for path, item in files.items()))
            self.ssh(f'mkdir -p {self.directory}')
            self.rsync(f'{job}/', f'{self.host}:{self.directory}/job/')
            for name, (directory, _) in inputs.items():
                self.rsync(f'{directory}/', f'{self.host}:{self.directory}/{name}/')
        print(f'cluster: inputs staged on {self.args.node}:{self.directory}', file=sys.stderr)
        self.kubectl('create', '-f', '-', input=json.dumps(self.job()))
        print(f'cluster: job {self.name} created ({self.args.cluster_device})', file=sys.stderr)
        try:
            code = self.wait(time.monotonic() + (self.args.slot_wait_min + self.args.time_limit_min) * 60 + 1800)
        finally:
            # Logs and checkpoints of a failed or interrupted pod are evidence and the resume point.
            Path(result).mkdir(parents=True, exist_ok=True)
            try:
                self.rsync(f'{self.host}:{self.directory}/result/', f'{result}/')
            except TrainingError as error:
                print(f'warning: {error}', file=sys.stderr)
        return code

    def delete(self):
        completed = self.kubectl('delete', 'job', self.name, '--ignore-not-found', '--wait=false', check=False)
        if completed.returncode:
            print(f'warning: could not delete job {self.name}: {completed.stderr.strip()}', file=sys.stderr)
        if self.host is not None and self.ssh(f'rm -rf {self.directory}', check=False).returncode:
            print(f'warning: could not remove {self.host}:{self.directory}', file=sys.stderr)


def cluster_failure(code, reasons, logs):
    """The error for a nonzero pod status: the shared prelude's codes, then the trainer's own."""
    reason = {INPUTS_UNVERIFIED: 'input digests did not verify inside the pod',
              INSTALL_FAILED: 'installing the trainer packages failed', **reasons}.get(code, f'training exited {code}')
    log = Path(logs) / ('install.log' if code == INSTALL_FAILED else 'train.log')
    return TrainingError(f'{reason}; see {log}:\n{tail(log)}',
                         'invalid_input' if code == INPUTS_UNVERIFIED else 'failed')
