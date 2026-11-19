# Skill acquisition

A skill is learned, never hand-written: record demonstrations, export, train,
register as experimental, evaluate against independent evidence, then promote.
`config/robot_skills.yaml` is the only place a skill is named; the runner
(`lekiwi_rmf/skill_runner.py`, `/robot/skills/run`) executes registered entries
through the guarded arm trajectory path.

| Step | Command | Result |
| --- | --- | --- |
| Record | `scripts/record-episode.py DIR --task TEXT` | MCAP episode + manifest with an outcome label |
| Export | `scripts/export-lerobot.py EPISODE... --root DATASET` | LeRobot dataset of the successful episodes, ROS units |
| Train | `scripts/train-skill.py DATASET --output RUN [--cluster]` | `RUN/checkpoint`, `RUN/manifest.json` |
| Register | `scripts/register-skill.py ID --training RUN --version N --description TEXT` | experimental entry |
| Evaluate | `scripts/evaluate-skill.py ID --success JSON --output REPORT` | report with Wilson interval |
| Promote | `scripts/register-skill.py ID --promote REPORT...` | `status: qualified` |

Simulation is an experimental environment around that loop (scripted demonstrations
and sim trials, below); it never replaces the physical evaluation that promotes.

## Demonstrations

A demonstration is one recorded episode of the arm doing the task while the base
stands still. Operator procedure, per episode:

1. Set the scene and start the recorder in a terminal:
   `scripts/record-episode.py DIR --task "push the block" --operator NAME`
   (`--cameras` defaults to every identity in `config/object_cameras.yaml`; add
   `--parameters '{"side": "left"}'` when the demonstration is one value of a
   skill parameter).
2. Perform the task. When a supervisor takes over or corrects the motion, type
   `i` and Enter at the start of that span and again at its end. The recorder
   publishes `start` and `end` on `/episode/intervention` and the bag records them.
3. Stop with Ctrl-C (or `--duration SEC`) and answer the outcome question:
   `success` when the task was achieved, `failure` otherwise. An empty answer, or
   a recording without a terminal and without `--outcome`, leaves the episode
   unlabeled. `scripts/record-episode.py DIR --label success|failure` corrects the
   label of a recorded episode; the bag is not touched.

The manifest keeps `task`, `parameters`, `outcome` and `outcome_by`. The export
takes only `success` episodes (`--outcomes success,failure,unlabeled` widens it and
`export-report.json` lists what was skipped), writes `intervened` as 1 on the frames
between a `start` and its `end` mark (a span still open lasts to the end of the
episode; marks that do not alternate reject the episode) and counts them per
episode. An episode has one outcome for its whole length; it is not divided into
segments.

The repository has no way to move the physical arm by hand while recording:
`scripts/teleop.py` drives only the base, and `scripts/arm_jog.py` steps one joint
at a time. Physical demonstrations therefore need an arm teleoperation source that
commands `/arm_controller/follow_joint_trajectory` (the guarded path, so
`/hardware/applied_command` records what was applied). Until one exists,
demonstrations come from MoveIt motions, another skill, or scripted simulation
episodes (below).

## Action space, cameras and task text

- **Action**: the six arm joints of `/hardware/applied_command`, in URDF radians.
  The driver also reports the three base commands; an episode in which any of them
  is not zero is refused by the export. `--base-actions` exports all nine, the
  registry entry then lists the base commands (`resources` includes `base`), and
  the runner rejects every request for that skill: it has no guarded base path and
  never executes the arm half of an arm-and-base action.
- **Cameras**: `--cameras a,b` on the recorder and the export name identities of
  `config/object_cameras.yaml` (`front`, `wrist`, `astra_color`); each becomes
  `observation.images.<identity>`, and the runner subscribes to the same
  identity's image topic.
- **Task text**: `lekiwi_rmf.skills.conditioning(task, parameters)` is the
  episode's task followed by its parameter values as canonical JSON. The export
  writes it on every frame and the runner passes it to the policy, built from the
  registered `task` and the request's parameters, so training and inference use
  the same text.

## Trainer contract

`config/trainers.yaml` declares each trainer (`lerobot_policy` here,
`rtdetr_detector` in [perception training](perception-training.md)): model types,
modalities, data format, body compatibility, configuration schema, resources,
checkpoints and outputs. A run validates its configuration against that schema and
writes `RUN/run.json` from the start: the declaration, the configuration, the
dataset digest and, at the end, the termination reason (`completed`, `failed`,
`interrupted`, `budget_exceeded`, `no_slot`, `invalid_input`) with every attempt.
Only a completed run writes `manifest.json`, so nothing incomplete can be registered.
Both trainers use the same cluster Job (`lekiwi_rmf/training.py`).

`lerobot_policy` admits the policy type `act`, which does not read the task text,
and is not resumable: a run saves its checkpoint once, at the last step, so an
interrupted run is started again from the frozen dataset. LeRobot 0.4.1's
task-conditioned types (`smolvla`, `pi0`, `pi05`, `groot`) are not admitted; they
need extras, base-model weights and a tokenizer that the pinned environment does
not install and the artifact digest does not pin.

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
- The inputs are copied with rsync over SSH to `/var/tmp/<trainer>/<job>` on
  the node, which the pod mounts; the pod verifies every input digest before the
  trainer's install and run steps, and results and logs come back the same way,
  also when training fails. `kubectl cp` is not used: from a
  remote workstation its stream ran at about 10 KB/s against about 190 KB/s for
  SSH, too slow for a 200 MB ACT checkpoint. The Job and the node directory are
  deleted on success, failure or interrupt. Each run installs `lerobot==0.4.1`
  in the pod (a few minutes) instead of maintaining a training image. This needs
  SSH access to the node as a user who may create that directory.

## Registration and promotion

Registration copies the checkpoint to `~/.ros/lekiwi/skills/ID/N` and writes the
entry with `artifact_sha256` and `status: experimental`. The artifact digest is the
SHA-256 of the sorted `<sha256>  <name>` manifest of every file the runner loads:
weights, policy configuration, processor configurations and their normalization
tensors. Only the provenance records `training-manifest.json` and
`evaluation-report.json` are outside it; LeRobot never opens them. A registry entry
with the older `weights_sha256` field is refused. Action names, cameras,
rate and the demonstrated task text come from the training manifest, so a dataset
exported in host units is refused. The operator declares the description, the
limits (`--max-duration-sec`, `--max-joint-step-rad`), `--parameters` and
`--effects '{"reversibility": "reversible"|"compensable"|"irreversible",
"compensation": TEXT}'`; a skill without declared effects is treated as
irreversible. Parameters are refused unless the checkpoint's policy type reads the
task text, the demonstrations are of one task and they were recorded with those
parameters; the runner refuses a parameterised skill whose policy is not
task-conditioned. An installed version is never overwritten; superseding a
registered version needs `--replace`, and registering over an existing entry keeps
every declaration that is not passed again. The registry is validated with the
runner's loader before the tracked file is replaced. Commit the registry change and deploy it with
the artifact present on the compute host.

Promotion requires reports that passed and name the same skill, version and
artifact digest that are installed, from the physical domain: a sim report
(`domain: sim`, `promotable: false`) never qualifies a skill. The first report is
kept beside the artifact.

## Regression suites

`config/qualification_suites.yaml` keeps, per skill, the protocols that qualified
it (domain, predicate, parameters, trial counts, thresholds, windows, reset), each
named by its digest and pointing at the content-addressed report that last passed
it (`~/.ros/lekiwi/qualification-suites/<sha256>.json`). A first promotion stores its
report protocols as the suites. A new version registered with `--replace` is
experimental, and promoting it needs a passing report for every stored suite:

```bash
scripts/evaluate-skill.py ID --suite NAME --output REPORT   # exactly the stored protocol
scripts/register-skill.py ID --promote REPORT [REPORT...]
```

The new reports become the suites' baselines; a report with a new protocol adds a
suite. Suites are never dropped by a promotion.

## Evaluation

The runner admits experimental skills only when the stack was started for
acquisition, which layers `config/skill_acquisition.yaml`:

```bash
scripts/sim-up.sh skill_acquisition:=true          # simulation
scripts/workstation-up.sh skill_acquisition:=true  # split robot
```

The runner reports `finished`, `failed` or `canceled` only after the arm controller
reported the last chunk terminal. A cancel or a chunk that overruns cancels the
arm goal and waits for that status; when the controller does not give it within
5 s the run is `failed` with `arm_stopped: false` in `result_json`, meaning the
arm may still be moving. A chunk is cut to the steps that fit in what remains of
`max_duration_sec`.

`evaluate-skill.py` runs N trials through `/robot/skills/run`. The runner's
outcome is recorded but never decides success. Success is a predicate over object
observations (`/robot/observations/query`, so object perception must run): a
label that is present (or, with `"present": false`, absent) with at least
`min_score`, optionally from one `camera_id`, inside a normalized image `region`
and inside a metric `volume`, for example
`--success '{"label": "cup", "region": [0.4, 0.5, 0.9, 1.0]}'` or
`--success '{"label": "cup", "volume": {"frame": "map", "min": [0, 0, 0.2], "max": [2, 3, 1]}}'`.
A volume is a box in metres in the observing camera's optical frame (`camera`) or
the map frame (`map`), compared with the detection's measured position. Positions
exist only for a camera with registered depth and a metric calibration
(`calibration_file` in `config/object_cameras.yaml`); a detection without one
never satisfies a volume and never proves absence from it. The verifier expresses
nothing else: no contact, force, joint, electrical or material state. A trial
proceeds as follows:

- before the run, at least `min_observations` must exist and the predicate must
  not already hold (otherwise the trial is invalid);
- after `--settle-sec`, every observation in the `--observe-sec` window must
  satisfy it, and there must be at least `min_observations`.

Invalid trials count against the skill. Each trial's observation IDs and evidence
hashes are in the report and pinned in the store. The report passes when at least
`--min-trials` ran and the Wilson lower bound (`--confidence`, default 0.95)
reaches `--required-lower-bound` (default 0.8; 20 of 20 gives 0.84).
`--reset-command` runs before each trial with `LEKIWI_EVALUATION_TRIAL` set.

## Simulation

`config/sim_scenes.yaml` declares task scenes for the Gazebo world: what the
simulator models (`rigid_geometry`, `articulation`, `friction_contact`; never
deformables, heat or electricity), the robot's start pose and the arm's start
(an SRDF arm group state), and objects with shape, mass, friction, nominal pose and
seeded randomization. `lekiwi_rmf/sim_scene.py` derives each trial's object poses
from the scene seed and the trial index only, so trial N always starts from the
same state. A reset moves the arm to its start through
`/arm_controller/follow_joint_trajectory` (the guarded path), places the objects
(created once) with Gazebo's entity services, waits `settle_sec` of simulation time
and confirms objects, robot base and arm from ground truth (`gz model -p` by entity
name, a fresh `/joint_states`). A reset never moves the base; an unconfirmed reset
makes the trial invalid instead of a new origin. Ground truth and placement use
the `gz` command line from the caller's environment, which must reach the
simulation's transport (the same `GZ_PARTITION`, if one is set).

```bash
scripts/sim-up.sh skill_acquisition:=true
scripts/sim-episodes.py --scene floor_block --program PROGRAM.json --task TEXT --output EPISODES \
  --success '{"object": "block", "min_z": 0.05}'
scripts/evaluate-skill.py ID --sim-scene floor_block \
  --success '{"object": "block", "min_z": 0.05}' --output REPORT
```

`sim-episodes.py` records scripted demonstrations: per trial it resets the scene,
records with `record-episode.py --domain sim` (the reset is stored in the manifest
`context`) and executes a joint-waypoint program as one trajectory. With
`--success` it labels each episode `success` or `failure` from ground truth after
the program ran; without it the episodes stay unlabeled and the export skips them. The program is
data, open loop over the declared waypoints; it does not react to the randomized
object poses. The simulated arm publishes `/hardware/applied_command` like the
driver, so exports use the same action contract, and `export-lerobot.py` keeps each
episode's `domain` in `export-report.json`.

In sim, `evaluate-skill.py` runs the same trials through `/robot/skills/run` with
the simulator's ground truth as the verifier. A sim predicate names a scene object
and a world region, height bounds or a minimum displacement; it reads only rigid
body poses, and a scene that does not model `rigid_geometry` cannot verify it.
Physical evaluation is still required for promotion.
