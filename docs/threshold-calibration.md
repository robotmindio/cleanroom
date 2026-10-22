# Threshold calibration

`scripts/calibrate-thresholds.py` fits perception thresholds from labelled
[evaluator reports](object-observations.md#independent-evaluation). It never
commands the robot. Two versioned procedures exist:

| Procedure | Threshold | Tracked value | Issue |
|---|---|---|---|
| `search-min-object-px/1` | `search_min_object_px` | `config/exploration.yaml` | [#88](https://github.com/robotmindio/cleanroom/issues/88) |
| `input-quality-limits/1` | dark luma, overexposed luma, blur variance | `DARK_LUMA, BRIGHT_LUMA, BLUR_VARIANCE` in `lekiwi_rmf/object_observations.py` (arrives with #100) | [#98](https://github.com/robotmindio/cleanroom/issues/98) |

Both values are uncalibrated engineering defaults today. No real corpus large
enough to fit them exists yet; see [Recording protocol](#recording-protocol).

## Rule

Each labelled object in a report is a trial: detected when a same-label
prediction matches it (IoU ≥ 0.5, one-to-one, the evaluator's matching). A
threshold splits trials into a **trusted** side, where a miss is accepted as
evidence (search negative evidence, a `no_candidate` abstention), and a
**flagged** side.

- Search: trusted when the object's apparent size, `min(bbox width, height)`
  in pixels, is at least `search_min_object_px`.
- Input quality: trusted when mean luma ≥ dark limit, mean luma ≤ overexposed
  limit and Laplacian variance ≥ blur limit, using the `input_quality` the
  runtime records for each frame. Each limit is fitted on trials that pass the
  other two current limits.

The fitted cut is the most permissive one whose **one-sided 95% exact
(Clopper-Pearson) lower bound of scene-group recall** on the trusted side is at
least the target (`--target-recall`, default 0.90, spec 27.2). The unit of
independence is the scene group; a group fails when any of its trusted trials
is missed. Trial-level recall is reported beside it but never decides.

Cuts are scanned on `development` reports and confirmed on `heldout` reports.
Status is:

- `calibrated`: the held-out trusted side reaches the target at the chosen cut;
- `insufficient_corpus`: a split has fewer groups than zero failures would need,
  no held-out report was given, a hard condition has fewer than 20 groups
  (input quality), or the reports lack `input_quality` (`report_lacks_input_quality`);
  the result lists `additional_groups_needed`;
- `no_feasible_cut`: enough groups, but no cut reaches the target or held-out
  data does not confirm it.

Search fits each camera separately (prefix a report with `CAMERA=`) and proposes
the largest cut. Every report must evaluate the same `model_sha256`; a new
model or operating point needs a new calibration.

`search_min_object_m` and the height band are domain declarations: an image
manifest records no physical size or height, so they are reported with
`measured: false` and are covered by the recording protocol instead.

## Write gate

`--write` acts only on `calibrated`. It stores the full result under
`config/calibration/<procedure>-<held-out manifest digest>.json` and rewrites the
single tracked value with a comment naming that file. Any other status writes
nothing and exits 1. Commit the two files on a branch and merge them through a
reviewed PR; the result file is the evidence the reviewer checks.

```bash
source scripts/setup.bash
scripts/evaluate-object-recognition.py front-dev.json --split development \
  --model-name rtdetr_v2_r18 --output front-dev-report.json
scripts/evaluate-object-recognition.py front-heldout.json \
  --model-name rtdetr_v2_r18 --output front-heldout-report.json
scripts/calibrate-thresholds.py search \
  --report front=front-dev-report.json --report front=front-heldout-report.json \
  --output search-calibration.json --write
```

Repeat the two evaluations per camera and pass every report. `input-quality`
takes the same reports.

## Sample sizes

Scene groups needed for the 95% lower bound to reach the target, by number of
failed groups:

| Target recall | 0 failures | 1 | 2 | 3 |
|---|---|---|---|---|
| 0.90 | 29 | 46 | 61 | 76 |
| 0.95 | 59 | 93 | 124 | 153 |

A 1% bound on an error rate needs at least 299 failure-free groups (spec 27.3).
Each split needs these counts on the trusted side of the cut, per camera.
Frames, crops or cameras of one arrangement stay in one group.

## Recording protocol

Record on the robot with the qualified model and operating point frozen,
cameras at their tracked resolution, and the base and arm stationary for each
capture. Use the evaluation recording in
[object observations](object-observations.md#independent-evaluation); label
with the manifest schema there (every instance of every vocabulary label, in
every image). One scene group is one placement of objects in one session:
moving the robot to a new viewpoint of the same placement is the same group.

### Search (`search_min_object_px`), per camera (`front`, `wrist`, `astra_color`)

- At least 60 development and 60 held-out groups per camera: twice the
  zero-failure minimum, so a few misses still leave a feasible cut.
- Spread apparent size evenly over 8–16, 16–24, 24–32, 32–48, 48–96 and
  > 96 px (at least 10 groups per bin per split), by placing objects of the
  declared minimum size (`search_min_object_m`, 0.08 m) and larger at
  distances up to `search_max_range_m` (4 m).
- Place objects at the band's lower edge (floor, 0.02 m) and upper edge (0.3 m)
  and in between, in at least three rooms or lighting setups.
- Include at least 20 groups per split with no vocabulary object present, so
  false positives are counted, and at least 10 with two similar instances.

### Input quality (cleanroom#98), across the three cameras

- At least 20 groups per split for each hard condition tag: `dark` (lamps off,
  dusk), `overhead` (top-down view), `occlusion` (object partly hidden),
  `transparent`, `reflective`, `blur` (capture while the wrist or a person
  moves) and `similar_instances`, plus at least 30 groups per split of normal
  conditions. Tag each scene in the manifest's `conditions` list (#100).
- Cover the luma range at the frame level: under 20, 20–40, 40–80, 80–200,
  over 200 mean luma, at least 10 groups per band per split, with objects present.
- Evaluate with the evaluator that records `input_quality` per frame (#100).

## Current evidence

The only labelled real data is one development scene group (three front-camera
frames of one patterned mug, `~/.ros/lekiwi/qualification/patterned-mug-development.json`).
Both procedures return `insufficient_corpus` on it, so the defaults stay.
