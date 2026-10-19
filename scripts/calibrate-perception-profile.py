#!/usr/bin/env python3
"""Run a frozen proposal-profile calibration study on a labeled corpus; never command the robot.

Candidates are searched on the development split; the best and the current profile are compared once on
the held-out split. The content-addressed record is written to the study store, and a selected candidate
can then be applied with /robot/perception/apply_profile (parameters_json + evidence_id).
"""

import argparse
import importlib.util
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_recognition import Recognizer, use_model_runtime
from lekiwi_rmf.perception_profiles import make_profile, tracked_profiles
from lekiwi_rmf.perception_study import STORE, run_study, save


def evaluator():
    spec = importlib.util.spec_from_file_location('evaluate_object_recognition',
                                                  Path(__file__).with_name('evaluate-object-recognition.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def candidates_from(path, current):
    """A JSON list of parameter objects; each may be complete or override the current profile's values."""
    document = json.loads(Path(path).read_text())
    if not isinstance(document, list) or not 1 <= len(document) <= 32:
        raise ValueError('candidates must be a JSON list of 1..32 parameter objects')
    return [make_profile(current.backend, {**current.parameters, **item}, f'candidate-{index}')
            for index, item in enumerate(document)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--model-name', required=True)
    parser.add_argument('--model-directory')
    parser.add_argument('--current', default='', help='tracked profile in force (default: the tracked default)')
    parser.add_argument('--candidates', required=True, help='JSON list of candidate parameter overrides')
    parser.add_argument('--store', default=str(STORE))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    use_model_runtime(args.model_name)
    module = evaluator()
    recognizer = Recognizer(args.model_name, args.model_directory)
    default, tracked = tracked_profiles()
    current = tracked[recognizer.backend].get(args.current or default)
    if current is None:
        raise SystemExit(f'{args.current} is not a tracked {recognizer.backend} profile')
    manifest_sha256, _, labels, _ = module.load_scenes(args.manifest, 'heldout')
    record = run_study(lambda profile, split: module.evaluate(args.manifest, split=split, profile=profile,
                                                              recognizer=recognizer),
                       recognizer, current, candidates_from(args.candidates, current),
                       manifest_sha256=manifest_sha256, labels=labels)
    path = save(record, args.store)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'evidence_id': record['id'], 'decision': record['decision'], 'reasons': record['reasons'],
                      'candidate': record['candidate'], 'record': str(path)}))
    raise SystemExit(0 if record['decision'] == 'selected' else 1)


if __name__ == '__main__':
    main()
