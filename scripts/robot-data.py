#!/usr/bin/env python3
"""Audit, back up, restore and verify the robot's local data (config/data_retention.yaml).

  audit              report files whose permissions exceed their data class's mode
  backup DEST        snapshot the journals and records, with manifests of model and skill artifacts
  restore BACKUP     verify the backup, keep the replaced files, restore, rebuild indexes and verify
  verify             integrity, schema, rebuilt indexes and observation records against their evidence

Restore refuses while a store's node holds its lock; stop the robot services (scripts/ros-stop.sh)
first, since spatial memory has no lock of its own. Never commands the robot. Exit 1 on any finding.
"""

import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.robot_data import audit, backup, restore, verify  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('audit', 'backup', 'restore', 'verify'))
    parser.add_argument('path', nargs='?', type=Path, help='backup destination (backup) or backup directory (restore)')
    parser.add_argument('--home', type=Path, default=Path.home())
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0') or 0))
    parser.add_argument('--sim', action='store_true', help='the simulation stores of that domain')
    args = parser.parse_args(argv)
    if (args.path is None) == (args.command in ('backup', 'restore')):
        parser.error(f'{args.command} {"needs" if args.path is None else "takes no"} path')
    try:
        if args.command == 'audit':
            result = audit(args.home, domain=args.domain, sim=args.sim)
            failed = bool(result)
        elif args.command == 'backup':
            result = backup(args.home, args.path, domain=args.domain, sim=args.sim)
            failed = False
        elif args.command == 'restore':
            result = restore(args.path, args.home)
            failed = not result['verification']['healthy']
        else:
            result = verify(args.home, domain=args.domain, sim=args.sim)
            failed = not result['healthy']
    except (OSError, RuntimeError, ValueError) as error:
        print(f'{parser.prog}: {error}', file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
