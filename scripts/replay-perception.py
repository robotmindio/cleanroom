#!/usr/bin/env python3
"""Replay one stored observation's capture under a chosen recognizer, profile and query; never commands the robot.

The record is an observation document (/robot/observations/query) and the artifact its byte-exact evidence
(/robot/observations/evidence, or the same bytes from Knowledge). Without overrides the replay uses the
record's own operating point and reports whether it reproduced the original inference.
"""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_recognition import Recognizer, use_model_runtime
from lekiwi_rmf.perception_profiles import make_profile, tracked_profiles
from lekiwi_rmf.perception_query import make_query
from lekiwi_rmf.perception_replay import replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record')
    parser.add_argument('artifact')
    parser.add_argument('--model-name', required=True)
    parser.add_argument('--model-directory')
    parser.add_argument('--profile', default='', help='tracked profile name')
    parser.add_argument('--profile-json', default='', help='complete profile parameters as JSON')
    parser.add_argument('--labels', nargs='*', help='open-vocabulary query labels')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    use_model_runtime(args.model_name)
    recognizer = Recognizer(args.model_name, args.model_directory)
    profile = None
    if args.profile:
        profile = tracked_profiles()[1][recognizer.backend].get(args.profile)
        if profile is None:
            raise SystemExit(f'{args.profile} is not a tracked {recognizer.backend} profile')
    elif args.profile_json:
        profile = make_profile(recognizer.backend, json.loads(args.profile_json))
    record = json.loads(Path(args.record).read_text())
    result = replay(record, Path(args.artifact).read_bytes(), recognizer, profile=profile,
                    query=make_query(args.labels) if args.labels else None)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: result[key] for key in ('id', 'same_operating_point', 'reproduced')}))


if __name__ == '__main__':
    main()
