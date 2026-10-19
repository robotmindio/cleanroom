# Skill acquisition

A skill is learned, never hand-written: record demonstrations, export, train,
register as experimental, evaluate against independent evidence, then promote.
`config/robot_skills.yaml` is the only place a skill is named; the runner
(`lekiwi_rmf/skill_runner.py`, `/robot/skills/run`) executes registered entries
through the guarded arm trajectory path.

| Step | Command | Result |
| --- | --- | --- |
| Record | `scripts/record-episode.py DIR --task TEXT` | MCAP episode + manifest |
| Export | `scripts/export-lerobot.py EPISODE... --root DATASET` | LeRobot dataset in ROS units |
| Train | `scripts/train-skill.py DATASET --output RUN [--cluster]` | `RUN/checkpoint`, `RUN/manifest.json` |
| Register | `scripts/register-skill.py ID --training RUN --version N --description TEXT` | experimental entry |
| Evaluate | `scripts/evaluate-skill.py ID --success JSON --output REPORT` | report with Wilson interval |
| Promote | `scripts/register-skill.py ID --promote REPORT` | `status: qualified` |

## Training

`manifest.json` freezes the dataset (every file and its SHA-256), the export
report, the source revision (and whether the tree was dirty), the exact
`lerobot_train` arguments, declared compute (`--gpu-memory-gib`,
`--time-limit-min`) and measured compute (elapsed time, peak GPU memory). After
training the checkpoint is loaded the way the runner loads it and asked for one
action chunk; `--skip-load-check` turns that off.

- Local: `--device cpu` (default) in `~/.ros/lekiwi/lerobot-venv`
  (`scripts/install-lerobot-export.sh`).
- Cluster: `--cluster` creates a one-pod Job (kubectl context `morelos`,
  namespace `robotmind`) on `robotmind-web-001`, whose GTX 1070 (sm_61) runs the
  sm_60 kernels of the pinned `pytorch/pytorch:2.7.1-cuda12.6` image
  (`robotmind-morelos-1` also advertises a GPU, but its driver is broken). The
  GPU is time-shared, so declare GPU memory conservatively. The Job is pinned to
  that node with a required node affinity. When both GPU slots are taken the pod
  stays Pending; the script prints "waiting for a GPU slot" with the scheduler's
  reason and gives up after `--slot-wait-min` (default 30), deleting the Job.
  `--cluster-device cpu` runs the same Job on the same node without claiming the GPU.
- The dataset is copied with rsync over SSH to `/var/tmp/skill-train/<job>` on
  the node, which the pod mounts; the pod verifies the dataset digests, and the
  checkpoint and logs come back the same way. `kubectl cp` is not used: from a
  remote workstation its stream ran at about 10 KB/s against about 190 KB/s for
  SSH, too slow for a 200 MB ACT checkpoint. The Job and the node directory are
  deleted on success, failure or interrupt. Each run installs `lerobot==0.4.1`
  in the pod (a few minutes) instead of maintaining a training image. This needs
  SSH access to the node as a user who may create that directory.

## Registration and promotion

Registration copies the checkpoint to `~/.ros/lekiwi/skills/ID/N` and writes the
entry with `weights_sha256` and `status: experimental`. Action names, cameras and
rate come from the training manifest, so a dataset exported in host units is
refused. An installed version is never overwritten; superseding a registered
version needs `--replace`. The registry is validated with the runner's loader
before the tracked file is replaced. Commit the registry change and deploy it with
the artifact present on the compute host.

Promotion requires a report that passed and names the same skill, version and
weights digest that are installed; the report is kept beside the artifact.

## Evaluation

The runner admits experimental skills only when the stack was started for
acquisition, which layers `config/skill_acquisition.yaml`:

```bash
scripts/sim-up.sh skill_acquisition:=true          # simulation
scripts/workstation-up.sh skill_acquisition:=true  # split robot
```

`evaluate-skill.py` runs N trials through `/robot/skills/run`. The runner's
outcome is recorded but never decides success. Success is a predicate over object
observations (`/robot/observations/query`, so object perception must run), for
example `--success '{"label": "cup", "region": [0.4, 0.5, 0.9, 1.0]}'`:

- before the run, at least `min_observations` must exist and the predicate must
  not already hold (otherwise the trial is invalid);
- after `--settle-sec`, every observation in the `--observe-sec` window must
  satisfy it, and there must be at least `min_observations`.

Invalid trials count against the skill. Each trial's observation IDs and evidence
hashes are in the report and pinned in the store. The report passes when at least
`--min-trials` ran and the Wilson lower bound (`--confidence`, default 0.95)
reaches `--required-lower-bound` (default 0.8; 20 of 20 gives 0.84).
`--reset-command` runs before each trial with `LEKIWI_EVALUATION_TRIAL` set.
