#!/usr/bin/env python3
"""Bound what a release workspace keeps, or retire selected inactive release builds.

Usage: prune-release-builds.py WORKSPACE --retain [--need-free BYTES] [--apply]
       prune-release-builds.py WORKSPACE REVISION [REVISION ...] [--apply]
Dry-run by default. Current, previous and process-referenced releases are never touched.
--retain is the policy stage-release.sh applies before every build: it removes whole
releases, unsealed leftovers of failed staging runs and unreferenced dependency overlays.
Naming revisions removes only their generated files and keeps their sources.
Stage a removed revision again with stage-release.sh before activating it.
"""

import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import time


GENERATED = ('build', 'install', 'log', 'test-tmp')
# Sealed releases kept ready to activate besides `current` and `previous`. Those two already
# give one rollback; three more cover a short run of bad deployments. Every further release
# costs its checkout and build again and is one stage-release.sh run away.
RETAINED = 3
# Files staging itself puts into a checkout; everything else untracked there is a record.
DISPOSABLE = ('.env', '__pycache__', '.pytest_cache')


def process_references(workspace, processes=Path('/proc')):
    """Map the releases and dependency overlays running processes name to one such process."""
    pattern = re.compile(re.escape(os.fsencode(workspace))
                         + rb'/(releases/[0-9a-f]{40}|overlays/[a-z]+-[0-9a-f]{32})')
    found = {}
    for process in processes.glob('[0-9]*'):
        try:
            texts = [os.fsencode((process / 'cwd').resolve())]
            texts += [(process / name).read_bytes() for name in ('cmdline', 'maps', 'environ')]
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Other users' details may be private; managed services run as this user.
            if not process.exists() or process.stat().st_uid != os.getuid():
                continue
            # The login's own `systemd --user` and `(sd-pam)` are non-dumpable: even their owner
            # may read only the command line. Nothing in a release is such a program, so that
            # line decides; without this every SSH session made the tool refuse.
            try:
                texts = [(process / 'cmdline').read_bytes()]
            except (FileNotFoundError, ProcessLookupError):
                continue
            except PermissionError:
                raise ValueError(f'cannot inspect owned process {process.name}') from None
        for text in texts:
            for name in pattern.findall(text):
                found.setdefault(workspace / os.fsdecode(name), process.name)
    return found


def sealed_at(release):
    """When a release was sealed, or None unless its manifest is a valid seal."""
    manifest = release / 'release.json'
    try:
        document = json.loads(manifest.read_text())
    except (OSError, ValueError):
        return None
    if manifest.is_symlink() or not isinstance(document, dict) or document.get('revision') != release.name:
        return None
    return manifest.stat().st_mtime_ns


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
    if sealed_at(release) is None:
        raise ValueError(f'{revision}: invalid release manifest')
    process = process_references(workspace, processes).get(release)
    if process:
        raise ValueError(f'{revision}: process {process} references this release')
    return release


def new_receipt(release, archive):
    receipt = archive / f'{release.name}-{time.time_ns()}'
    receipt.mkdir(mode=0o700, parents=True)
    return receipt


def retire(release, archive):
    receipt = new_receipt(release, archive)
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


def worktree_record(source):
    """The anchor repository's record of a release checkout, or None if it is no Git worktree."""
    link = source / '.git'
    if not link.is_file():
        return None
    record = Path(link.read_text().removeprefix('gitdir:').strip())
    return record if record.parent.name == 'worktrees' and record.is_dir() else None


def local_files(source):
    """Untracked files of a checkout that staging did not put there, such as on-robot test records."""
    listing = subprocess.run(
        ['git', '--no-optional-locks', '-C', str(source), 'status', '--porcelain', '--ignored', '-z'],
        capture_output=True)
    if listing.returncode:
        return []  # no readable checkout: a partial one never ran anything
    names = [os.fsdecode(entry[3:]).rstrip('/') for entry in listing.stdout.split(b'\0')
             if entry[:3] in (b'?? ', b'!! ')]
    return [name for name in names
            if not set(Path(name).parts) & set(DISPOSABLE) and not name.endswith(('.pyc', '.pyo'))
            and ((source / name).is_file() or any(path.is_file() for path in (source / name).rglob('*')))]


def discard(release, archive):
    """Remove a whole release; its seal, test evidence and local files remain in a receipt."""
    receipt = retire(release, archive) if sealed_at(release) is not None else None
    source = release / 'source'
    for name in local_files(source):
        receipt = receipt or new_receipt(release, archive)
        target = receipt / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(source / name, target)
    record = worktree_record(source)
    if record:
        # Twice forced: the checkout holds ignored files, and a staging run killed while Git
        # created it leaves Git's own "initializing" lock behind.
        subprocess.run(['git', '--git-dir', str(record.parents[1]), 'worktree', 'remove',
                        '--force', '--force', str(source)], capture_output=True)
    shutil.rmtree(release)
    if record and record.exists():
        shutil.rmtree(record)  # Git could not read a broken checkout; drop its record as `worktree prune` would
    return receipt


def overlay_names(release):
    """The dependency overlays a release was built on, by its build marker and its seal."""
    marker = release / 'install/.lekiwi-overlay'
    paths = [marker.read_text().strip()] if marker.is_file() else []
    try:
        paths.append(json.loads((release / 'release.json').read_text())['dependency_overlay']['path'])
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return {Path(path).name for path in paths}


def plan_retention(workspace, processes=Path('/proc')):
    """Sort a workspace into the releases the policy keeps and what it removes; change nothing."""
    releases, overlays = workspace / 'releases', workspace / 'overlays'
    for path in (releases, overlays):
        if path.is_symlink():
            raise ValueError(f'{path.name} directory must not be a symlink')
    used = process_references(workspace, processes)
    pointers = {name: (workspace / name).resolve() for name in ('current', 'previous')}
    kept, sealed, unsealed = {}, [], []
    for release in sorted(releases.iterdir()):
        if release.is_symlink() or not release.is_dir() or not re.fullmatch('[0-9a-f]{40}', release.name):
            continue
        pointer = next((name for name, target in pointers.items()
                        if release == target or release in target.parents), None)
        record = worktree_record(release / 'source')
        if pointer:
            kept[release] = pointer
        elif release in used:
            kept[release] = f'process {used[release]}'
        elif record and (record / 'locked').is_file() and (record / 'locked').read_text().strip() != 'initializing':
            kept[release] = 'locked worktree'  # `git worktree lock` is an operator's explicit hold
        elif sealed_at(release) is None:
            unsealed.append(release)
        else:
            sealed.append(release)
    sealed.sort(key=sealed_at, reverse=True)
    kept.update(dict.fromkeys(sealed[:RETAINED], 'recent'))
    built = [path for path in (sorted(overlays.iterdir()) if overlays.is_dir() else [])
             if not path.is_symlink() and path.is_dir() and re.fullmatch('[a-z]+-[0-9a-f]{32}', path.name)]
    # The newest complete overlay is either in use or was just built for a release whose own
    # qualification then failed; rebuilding it for the corrected revision would cost hours.
    newest = max((path for path in built if (path / '.complete').is_file()),
                 key=lambda path: (path / '.complete').stat().st_mtime_ns, default=None)
    needed = set().union(*map(overlay_names, kept), {path.name for path in used})
    return {'kept': kept, 'retired': sealed[RETAINED:], 'unsealed': unsealed,
            'overlays': [path for path in built if path.name not in needed and path != newest]}


def hold(path, locks):
    """Take a lock, unless this process inherited the descriptor its caller holds it on."""
    stream = locks.enter_context(path.open('a'))
    try:
        return fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        pass
    # deploy-split.sh and stage-release.sh run this tool under their own locks. A lock belongs
    # to the open file, so relocking an inherited descriptor succeeds only below its holder.
    for entry in Path('/proc/self/fd').iterdir():
        try:
            if int(entry.name) != stream.fileno() and os.path.samefile(entry, path):
                return fcntl.flock(int(entry.name), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            continue
    sys.exit(f'{path} is held by another deployment, staging or pruning run')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('revisions', nargs='*')
    parser.add_argument('--retain', action='store_true', help='apply the retention policy')
    parser.add_argument('--need-free', type=int, default=0, metavar='BYTES',
                        help='fail unless this much space is free afterwards')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if args.retain == bool(args.revisions) or args.need_free and not args.retain:
        parser.error('name the revisions to retire, or pass --retain without revisions')
    os.umask(0o077)
    workspace = args.workspace.resolve(strict=True)
    archive = workspace / 'retired-release-manifests'
    logs = Path(os.environ.get('LEKIWI_LOGS', Path.home() / '.ros' / 'lekiwi'))
    logs.mkdir(parents=True, exist_ok=True)
    with ExitStack() as locks:
        for path in (logs / 'deploy.lock', workspace / 'releases' / 'stage.lock'):
            hold(path, locks)
        before = shutil.disk_usage(workspace).free
        receipts = []
        # Validate the entire request before removing anything.
        if args.retain:
            plan = plan_retention(workspace)
            report = {'kept': {release.name: reason for release, reason in plan['kept'].items()},
                      **{name: [path.name for path in plan[name]] for name in ('retired', 'unsealed', 'overlays')}}
            if args.apply:
                # What needs no receipt goes first: on a full disk it frees the room receipts need.
                for overlay in plan['overlays']:
                    # Remove the marker first: an interrupted cleanup must never look complete.
                    (overlay / '.complete').unlink(missing_ok=True)
                    shutil.rmtree(overlay)
                for release in plan['unsealed'] + plan['retired']:
                    receipt = discard(release, archive)
                    receipts += [str(receipt)] if receipt else []
        else:
            releases = [inspect_release(workspace, revision) for revision in dict.fromkeys(args.revisions)]
            report = {'revisions': [path.name for path in releases]}
            for release in releases:
                if args.apply:
                    inspect_release(workspace, release.name)
                    receipts.append(str(retire(release, archive)))
        after = shutil.disk_usage(workspace).free
        print(json.dumps({'applied': args.apply, **report, 'receipts': receipts,
                          'free_before': before, 'free_after': after}))
        if after < args.need_free:
            sys.exit(f'{workspace}: {after >> 20} MiB free, {args.need_free >> 20} MiB needed. Retention '
                     f"{'removed' if args.apply else 'would remove'} {len(plan['retired'])} releases, "
                     f"{len(plan['unsealed'])} unsealed directories and {len(plan['overlays'])} overlays; "
                     'free space on this filesystem and retry')


if __name__ == '__main__':
    main()
