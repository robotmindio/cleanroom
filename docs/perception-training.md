# Perception training

When teaching by exemplar cannot separate a new category
(`scripts/learn-perception-concept.py` reports that no match threshold separates
held-out positives from negatives), the category needs a reviewed dataset and a
trained detector. The loop mirrors [skill acquisition](skill-acquisition.md):

| Step | Command | Result |
| --- | --- | --- |
| Freeze a dataset | `scripts/build-perception-dataset.py ANNOTATIONS --output DATASET` | `images/`, `scenes.json`, `dataset.json` |
| Train | `scripts/train-recognizer.py DATASET --output RUN [--cluster] [--resume]` | `RUN/model/`, `RUN/manifest.json` |
| Register | `scripts/register-recognizer.py NAME --training RUN` | experimental `config/recognition_models.yaml` entry |
| Evaluate | `scripts/evaluate-object-recognition.py DATASET/scenes.json --model-name NAME --output REPORT` | schema-3 report |
| Regression | the same evaluator on every suite manifest of the recognizer it replaces | one report per suite |
| Promote | `scripts/register-recognizer.py NAME --promote REPORT --regression REPORT...` | `status: qualified` |

A learned concept can be queried alone: set `labels: []` and its immutable
revision in `concept_revisions` on `/robot/perception/set_query`. Adding an
unrelated label changes the detector prompt and can suppress a proposal. Empty
labels and empty concept revisions together restore the tracked default query.
The published operating point records the concept name and revision.

## Datasets

The annotation file lists views of taught or corrected observations (an
`observation_id` read byte-exact from the observation store, or an image normalized
like a robot frame), their scene group and split, and boxes whose annotations each
keep author, method (`human_teaching`, `human_correction`, `human_review`,
`automatic`), confidence and revision. `labeled_by` asserts that every instance of
every dataset label is boxed in the view, so absence is a true negative.

- Held-out views carry only human annotations: automatic labels and pseudo-labels
  never become qualification evidence. `--unreviewed-heldout` builds a
  measurement-only dataset (`heldout_authority: unreviewed`) that can never promote.
- A scene group stays in one split; exact and near-duplicate images (8x8 average
  hash within 4 bits) never span development and held-out.
- The output is frozen: every file's SHA-256 is in `dataset.json`, a dataset is never
  rewritten, and a failed build leaves nothing.

## Training

`train-recognizer.py` (trainer `rtdetr_detector` in `config/trainers.yaml`) fine-tunes
the pinned `rtdetr_v2_r18` on the development split. The vocabulary keeps every base
label at its base index and appends the dataset's new labels; every class-indexed
head row is copied from the base for the old indices
(`lekiwi_rmf/detector_finetune.py`), so the catalogue and decoder remain. The
processor is the base's file, byte for byte, at the runtime input size, and images
are the stored evidence artifacts the runtime recognizer consumes.

Checkpoints every `--checkpoint-every` steps hold model, optimizer and generator
state with the metrics up to that step. `--resume` continues a failed, interrupted
or budget-exhausted run only with exactly its recorded dataset and configuration,
and rewrites `metrics.jsonl` from the checkpoint, so no step is logged twice; for a
seed the resumed weights equal an uninterrupted run's. The cluster backend is the
same Job as `train-skill.py` (pinned PyTorch image, the runtime's transformers pins
installed in the pod); its checkpoints come back also when the pod fails.

`manifest.json` records the dataset identity and authority, the base files, the
vocabulary and ID mapping, configuration, declared and measured compute, the
SHA-256 and size of every model file, and a load check that runs
`lekiwi_rmf.object_recognition.Recognizer`, the runtime class, on a held-out image.

## Registration and promotion

Registration installs the model files to `~/.ros/lekiwi/models/NAME`, refuses a
catalogue that moved or dropped a base label, and appends an entry with
`status: experimental`, `replaces` (default: the base) and the training identity.
Status, `replaces` and `training` are governance, outside the recognizer identity,
so promotion does not change the `model_sha256` that observations and reports
record. The detector refuses an experimental recognizer; the evaluator admits it.

Promotion needs a passing schema-3 report of exactly this recognizer on its own
dataset's held-out split with human labels, and, for every suite of the recognizer
it replaces in `config/qualification_suites.yaml`, a report of this recognizer on
that suite's manifest with no label or condition regression against the suite's
baseline report (`lekiwi_rmf.perception_study.regressions`). The passing reports
become this recognizer's suites, with its own held-out suite added. Selecting it
for the robot remains a change of `config/object_perception.yaml`, deployed like
any other.
