#!/usr/bin/env python3
"""Generate a reviewable CAD/stow envelope; never grants physical acceptance."""

import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lekiwi_rmf.robot_footprint import cad_footprint  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    config = yaml.safe_load((ROOT / 'config/safety_production.yaml').read_text())['safety_supervisor']['ros__parameters']
    pose = dict(zip(config['stow_joint_names'], config['stow_joint_positions']))
    result = cad_footprint(ROOT, pose, config['stow_tolerance'])
    options.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: result[k] for k in ('footprint', 'inscribed_radius_m', 'maximum_radius_m')}))


if __name__ == '__main__':
    main()
