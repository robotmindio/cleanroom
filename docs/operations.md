# Operations: budgets, health, data and compatibility

How the robot measures its latency budgets, reports health, isolates learned
code, classifies and backs up its local data, and keeps its versioned contracts
compatible with RobotMind. Cluster-side counterparts (perception-provider and
Knowledge dashboards and alerts, Knowledge retention classes and restore
reconciliation, Fiber's `robot_world_v1` migration) live in the robotmind
repository.

## Latency budgets

Every operation that turns sensor data into action records one sample per
stage per answer and publishes `lekiwi/budget/<operation>` on `/diagnostics`
once a second ([`latency_budget.py`](../lekiwi_rmf/latency_budget.py)). The
stage vocabulary is closed: `sensor_age`, `queue`, `inference`, `network`,
`planning` and `execution`. Operations are the keys of
[`config/latency_budgets.yaml`](../config/latency_budgets.yaml), so status names
and keys are bounded; task, goal and entity IDs stay in the journal and logs.

| Operation | Node | Stages | Age gate for control |
| --- | --- | --- | --- |
| `perception` | `object_detector` | sensor age at receipt, queue (robot wait plus the provider's decode and queue), inference (the provider's reported model time, or the local call), network (round trip the provider did not account for) | `max_source_age_sec` |
| `skill_control` | `skill_runner` | oldest input of the observation, policy chunk, chunk execution | `max_observation_age_sec` |
| `navigation` | `robot_explorer` | `ComputePathToPose`, `NavigateToPose` | the explorer's service and navigation timeouts |
| `task` | `robot_tasks` | admission to dispatch, dispatch to journaled terminal state | each capability's own limits |

Each status carries `<stage>_p50_sec`, `_p95_sec`, `_p99_sec`, `_count` and
`_budget_p95_sec` over the last 512 samples, plus `max_age_sec`, `fresh`,
`late`, `late_in_window` and `invalid` (negative or non-finite samples, which
never lower a percentile). It is WARN when a stage's p95 exceeds its budget or
an answer in the window missed the age gate.

An answer older than its operation's age gate never feeds control:

- The detector does not publish a detection that finished after
  `max_source_age_sec`; it is counted as late. Captures that were fresh remain
  in the observation store as history.
- The skill runner discards a chunk whose observation aged past
  `max_observation_age_sec` while the policy computed it, observes again, and
  reports `late_chunks` in the result. Three late chunks in a row, or a run in
  which every chunk was late, fail the run instead of idling the arm.

The tracked budgets derive from the existing limits and the measured GPU
provider p95; the stages of each gated operation sum to no more than its gate.
Replace them with measured p95 plus margin once the deployed diagnostics have
recorded normal load and controlled saturation (both cameras at the maximum
rate with the provider loaded). A budget never widens an age gate.

## Liveness, readiness and capabilities

- Liveness is the process: the systemd services and the nodes' diagnostics.
- Readiness is the minimal dependency set for an operation:
  [`stack_readiness.py`](../lekiwi_rmf/stack_readiness.py) for navigation and
  exploration, the detector's per-camera `lekiwi/perception/<camera>` status
  for recognition.
- Capabilities are `/robot/capabilities`: each capability's availability,
  parameters and defaults, the observation services and the perception query.
  `available` means enabled and discovered, not physically permitted; ready for
  image-only grounding (`grounding: image_only`) is not ready for 3D pose.

## Learned code and model loading

Recognizers load only files pinned by size and SHA-256 in
`config/recognition_models.yaml`, with `local_files_only` and
`use_safetensors`; remote providers must echo their pinned operating-point
hash. Skill checkpoints pass `skills.verify_artifact` before loading: only
regular safetensors and JSON files, no links or subdirectories, not
world-writable, no JSON key that names code to import (`class`, `auto_map`,
`_target_`, `trust_remote_code`, `custom_pipeline`; LeRobot would import and
instantiate a processor step's `class`), and the checkpoint matching the
registered `artifact_sha256`, a digest over every file the runner loads
(weights, configuration, processors and normalization tensors). The policy
loads with the Hugging Face hub offline and without torchvision's backbone
download.

## Data classes, backup and restore

[`config/data_retention.yaml`](../config/data_retention.yaml) defines the
robot-local classes; retention references the parameter that enforces it.

| Class | Where | Retention and expiry | Mode |
| --- | --- | --- | --- |
| Transient buffer | RAM queues; unpinned captures in the observation store | `retention_sec` (7 days), 100,000 records or 2 GiB; expired evidence answers `evidence_expired` | 0600 |
| Pinned evidence | Observation store pins | Pin expires `retention_sec` after pinning; at most half the quota | 0600 |
| Training samples | `~/.ros/lekiwi/episodes` | Never automatic; removed by the operator after the cluster export is acknowledged | 0700 |
| Model artifacts | `~/.ros/lekiwi/models`, `~/.ros/lekiwi/skills` | Kept while tracked configuration references them | 0775 |

`scripts/robot-data.py audit` reports anything more permissive than its class.
Backups snapshot the task journal, observation store (records and evidence),
spatial memory and arm calibration with SQLite's online backup, and list model
and skill files by size and digest; weights are reinstalled from their pinned
sources. RTAB-Map keeps its own archives.

Restore and verify:

1. Stop the robot services (`scripts/ros-stop.sh`); restore refuses a store
   whose node holds its lock, and spatial memory has no lock of its own.
2. `scripts/robot-data.py restore <backup>` checks every file against the
   backup manifest, keeps each replaced file as `<name>.pre-restore-<UTC>`,
   restores, and verifies.
3. Verification runs `integrity_check`, checks each schema version against what
   its owner opens, rebuilds the indexes (`REINDEX`) and recomputes every
   observation's evidence digest. `visual_evidence` is `verified` only when
   every record matched its image; records restored without matching images
   recover history but report `unverified`. Exit status 1 on any failure.

## Versioned contracts and compatibility

| Contract | Producer | Consumers | Compatibility |
| --- | --- | --- | --- |
| Task journal (`user_version` 4) | `robot_tasks` | itself; restored backups | Opens versions 0–4 and migrates them; never replays rows that may have been sent |
| `/robot/tasks/*`, `/robot/capabilities` (`schema` 1) | `robot_tasks` | Fiber robot connector | Additive fields only; request IDs, defaults and budgets of navigate/explore are unchanged |
| Observation store (`user_version` 1), documents and `/robot/observations/*` (`schema` 1) | `object_observations` | Fiber sighting importer, search evidence | IDs are content digests; a reused ID with other content is refused |
| `robotmind.spacetime/1` | `spatial_memory` | Knowledge spacetime memory, search | Shared conventions in [spatial memory](spatial-memory.md) |
| `/robot/perception/profile` (`schema` 1) | `object_detector` | search, Fiber | Every frame records the model and query revision it ran under |
| Perception provider `/v1/infer` (`schema` 1) | RobotMind perception-provider | `object_recognition` | A response outside the pinned operating point is refused; `timings_ms` is optional |
| Skill registry | `config/robot_skills.yaml` | `skill_runner`, `robot_tasks` | Exact field set; a malformed entry rejects the registry |
| Backup manifest (`lekiwi.robot_data_backup/1`) | `scripts/robot-data.py` | its restore | Unknown schemas and stores are refused |

`robot_world_v1` is Fiber's bounded site catalog; this repository neither
produces nor reads it. Its migration into Knowledge sightings and the
retirement condition of its compatibility window are in robotmind's
`fiber-core/docs/ROBOT-GOALS.md` and `retirement-gates.md`.

The task journal's v0–v3 migrations stay while any kept backup or deployed
robot can hold those versions; `scripts/robot-data.py verify` reports a
store's `schema_version`. Retire a migration step only when no retained backup
reports that version.
