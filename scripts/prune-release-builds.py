#!/usr/bin/env python3
"""Retire selected inactive release builds, preserving sources and test evidence.

Usage: prune-release-builds.py WORKSPACE REVISION [REVISION ...] [--apply]
Dry-run by default. Current, previous and process-referenced releases are refused.
Rebuild a retired revision with stage-release.sh before activating it again.
"""

import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import tarfile
import time


GENERATED = ('build', 'install', 'log', 'test-tmp')


def inspect_release(workspace, revision, processes=Path('/proc')):
    if not re.fullmatch('[0-9a-f]{40}', revision):
        raise ValueError('revision must be a full lowercase Git SHA')
    release = workspace / 'releases' / revision
    if release.parent.is_symlink():
        raise ValueError('releases directory must not be a symlink')
    if release.is_symlink() or not release.is_dir():
        raise ValueError(f'{revision}: expected a release directory, not a symlink')
    for name in ('current', 'previous'):
        if (workspace / name).resolve() == release:
            raise ValueError(f'{revision}: protected by {name}')
    manifest = release / 'release.json'
    if manifest.is_symlink():
        raise ValueError(f'{revision}: release manifest must not be a symlink')
    document = json.loads(manifest.read_text())
    if not isinstance(document, dict) or document.get('revision') != revision:
        raise ValueError(f'{revision}: invalid release manifest')
    marker = os.fsencode(release)
    for process in processes.glob('[0-9]*'):
        try:
            if marker in os.fsencode((process / 'cwd').resolve()):
                raise ValueError(f'{revision}: process {process.name} uses this release')
            for name in ('cmdline', 'maps', 'environ'):
                if marker in (process / name).read_bytes():
                    raise ValueError(f'{revision}: process {process.name} references this release')
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            if process.exists() and process.stat().st_uid == os.getuid():
                raise ValueError(f'cannot inspect owned process {process.name}')
            # Other users' details may be private; managed services run as this user.
            continue
    return release


def retire(release, archive):
    receipt = archive / f'{release.name}-{time.time_ns()}'
    receipt.mkdir(mode=0o700, parents=True)
    shutil.copy2(release / 'release.json', receipt / 'release.json')
    with tarfile.open(receipt / 'qualification.tar.gz', 'w:gz') as evidence:
        for name in ('release-ctest.xml', 'test_results'):
            path = release / 'build' / 'lekiwi_rmf' / name
            if path.exists():
                evidence.add(path, arcname=name)
    # Remove the seal first: an interrupted cleanup must never advertise a valid build.
    (release / 'release.json').unlink()
    for name in GENERATED:
        path = release / name
        if path.is_symlink():
            path.unlink()
        elif path.exists():
            shutil.rmtree(path)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('revisions', nargs='+')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    os.umask(0o077)
    workspace = args.workspace.resolve(strict=True)
    logs = Path(os.environ.get('LEKIWI_LOGS', Path.home() / '.ros' / 'lekiwi'))
    with ExitStack() as locks:
        for path in (logs / 'deploy.lock', workspace / 'releases' / 'stage.lock'):
            stream = locks.enter_context(path.open('a'))
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Validate the entire request before removing any generated files.
        releases = [inspect_release(workspace, revision) for revision in dict.fromkeys(args.revisions)]
        before = shutil.disk_usage(workspace).free
        receipts = []
        for release in releases:
            if args.apply:
                inspect_release(workspace, release.name)
                receipts.append(str(retire(release, workspace / 'retired-release-manifests')))
        print(json.dumps({'applied': args.apply, 'revisions': [p.name for p in releases],
                          'receipts': receipts, 'free_before': before,
                          'free_after': shutil.disk_usage(workspace).free}))


if __name__ == '__main__':
    main()
