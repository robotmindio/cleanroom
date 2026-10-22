#!/usr/bin/env python3
"""Fit perception thresholds from labelled evaluator reports (docs/threshold-calibration.md).

  search          search_min_object_px (config/exploration.yaml), cleanroom#88
  input-quality   dark, overexposed and blurred limits (object_observations.py), cleanroom#98

Each --report is a scripts/evaluate-object-recognition.py report, optionally prefixed CAMERA=.
Pass development and held-out reports: cuts are chosen on development and confirmed on held-out.
--write records a `calibrated` result in tracked files for a reviewed PR; any other status writes
nothing. Never commands the robot. Exit 0 only when calibrated.
"""

import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lekiwi_rmf.threshold_calibration import (  # noqa: E402
    DEFAULT_TARGET_RECALL, calibrate_input_quality, calibrate_search, load_report, write_calibration)


def reports(values):
    loaded = []
    for value in values:
        camera, separator, path = value.partition('=')
        loaded.append(load_report(path, camera) if separator else load_report(value))
    return loaded


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('procedure', choices=['search', 'input-quality'])
    parser.add_argument('--report', action='append', required=True, help='[CAMERA=]evaluator report JSON')
    parser.add_argument('--target-recall', type=float, default=DEFAULT_TARGET_RECALL)
    parser.add_argument('--output', required=True, help='calibration result JSON')
    parser.add_argument('--write', action='store_true', help='record a calibrated value in tracked files')
    parser.add_argument('--root', default=str(ROOT), help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 0 < args.target_recall < 1:
        parser.error('--target-recall must be in (0, 1)')
    loaded = reports(args.report)
    if args.procedure == 'search':
        config = yaml.safe_load((Path(args.root) / 'config/exploration.yaml').read_text())
        result = calibrate_search(loaded, config['robot_explorer']['ros__parameters'], args.target_recall)
    else:
        result = calibrate_input_quality(loaded, args.target_recall)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    written = write_calibration(args.root, result) if args.write and result['status'] == 'calibrated' else []
    print(json.dumps({'status': result['status'], 'reason': result.get('reason'),
                      'proposed_value': result['proposed_value'], 'output': str(output), 'written': written}))
    if args.write and not written:
        print(f"nothing written: status is {result['status']}", file=sys.stderr)
    raise SystemExit(0 if result['status'] == 'calibrated' else 1)


if __name__ == '__main__':
    main()
