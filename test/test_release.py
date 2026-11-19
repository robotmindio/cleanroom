"""Release staging and activation never mutate the live installation."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import pytest
import yaml


ROOT = Path(__file__).parents[1]


@pytest.fixture
def pruner():
    spec = importlib.util.spec_from_file_location('prune_release', ROOT / 'scripts/prune-release-builds.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cached_release(workspace, revision='a' * 40):
    release = workspace / 'releases' / revision
    release.mkdir(parents=True)
    (release / 'release.json').write_text(json.dumps({'revision': revision}))
    return release


@pytest.mark.parametrize('name', ['current', 'previous'])
def test_pruning_protects_active_and_rollback_releases(tmp_path, pruner, name):
    release = cached_release(tmp_path)
    (tmp_path / name).symlink_to(release)
    with pytest.raises(ValueError, match=f'protected by {name}'):
        pruner.inspect_release(tmp_path, release.name, tmp_path / 'proc')


@pytest.mark.parametrize('reference', ['cmdline', 'maps', 'environ', 'cwd'])
def test_pruning_refuses_process_references(tmp_path, pruner, reference):
    release = cached_release(tmp_path)
    processes = tmp_path / 'proc'
    process = processes / '123'
    process.mkdir(parents=True)
    for name in ('cmdline', 'maps', 'environ'):
        (process / name).write_bytes(b'')
    if reference == 'cwd':
        (process / reference).symlink_to(release)
    else:
        (process / reference).write_bytes(str(release).encode())
    with pytest.raises(ValueError, match='process 123'):
        pruner.inspect_release(tmp_path, release.name, processes)


@pytest.mark.parametrize('cmdline, refused', [(b'/usr/lib/systemd/systemd\0--user\0', False), (None, True)])
def test_pruning_judges_a_non_dumpable_owned_process_by_its_command_line(tmp_path, pruner, cmdline, refused):
    release = cached_release(tmp_path)
    process = tmp_path / 'proc' / '123'
    process.mkdir(parents=True)
    (process / 'cwd').mkdir(mode=0)  # as for the login's user manager: resolving it is denied
    (process / 'cmdline').write_bytes(str(release).encode() if cmdline is None else cmdline)
    real = type(process).resolve

    def resolve(path, *args, **kwargs):
        if path.name == 'cwd':
            raise PermissionError(13, 'Permission denied')
        return real(path, *args, **kwargs)

    import unittest.mock
    with unittest.mock.patch.object(type(process), 'resolve', resolve):
        if refused:
            with pytest.raises(ValueError, match='process 123 references'):
                pruner.inspect_release(tmp_path, release.name, tmp_path / 'proc')
        else:
            assert pruner.inspect_release(tmp_path, release.name, tmp_path / 'proc') == release


def test_pruning_preserves_sources_evidence_and_external_symlink_targets(tmp_path, pruner):
    release = cached_release(tmp_path)
    for name in ('source', 'src', 'build/lekiwi_rmf', 'log', 'test-tmp'):
        (release / name).mkdir(parents=True)
    (release / 'source/.env').write_text('private configuration')
    (release / 'src/local-change').write_text('retain source changes')
    report = release / 'build/lekiwi_rmf/release-ctest.xml'
    report.write_text('<testsuite/>')
    external = tmp_path / 'shared-install'
    external.mkdir()
    (external / 'keep').write_text('dependency underlay')
    (release / 'install').symlink_to(external)
    receipt = pruner.retire(release, tmp_path / 'receipts')
    assert json.loads((receipt / 'release.json').read_text())['revision'] == release.name
    import tarfile
    with tarfile.open(receipt / 'qualification.tar.gz') as archive:
        assert archive.extractfile('release-ctest.xml').read() == b'<testsuite/>'
    assert (release / 'source/.env').read_text() == 'private configuration'
    assert (release / 'src/local-change').read_text() == 'retain source changes'
    assert (external / 'keep').read_text() == 'dependency underlay'
    assert not (release / 'release.json').exists()
    assert all(not (release / name).exists() for name in pruner.GENERATED)


def test_pruning_dry_run_and_batch_validation_do_not_remove_builds(tmp_path, pruner, monkeypatch, capsys):
    release = cached_release(tmp_path)
    (release / 'build').mkdir()
    logs = tmp_path / 'logs'
    logs.mkdir()
    arguments = ['prune-release-builds.py', str(tmp_path), release.name]
    monkeypatch.setenv('LEKIWI_LOGS', str(logs))
    inspect = pruner.inspect_release
    monkeypatch.setattr(pruner, 'inspect_release', lambda workspace, revision: inspect(workspace, revision, tmp_path / 'proc'))
    monkeypatch.setattr(sys, 'argv', arguments)
    pruner.main()
    assert json.loads(capsys.readouterr().out)['applied'] is False
    monkeypatch.setattr(sys, 'argv', arguments + ['../outside', '--apply'])
    with pytest.raises(ValueError, match='full lowercase Git SHA'):
        pruner.main()
    assert (release / 'build').is_dir() and (release / 'release.json').is_file()


def test_pruning_refuses_release_directory_symlinks(tmp_path, pruner):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (tmp_path / 'releases').symlink_to(outside)
    with pytest.raises(ValueError, match='releases directory'):
        pruner.inspect_release(tmp_path, 'a' * 40, tmp_path / 'proc')


def test_pruning_archive_failure_keeps_the_seal_and_build(tmp_path, pruner, monkeypatch):
    release = cached_release(tmp_path)
    (release / 'build').mkdir()
    def fail(*_args, **_kwargs):
        raise OSError('archive storage unavailable')
    monkeypatch.setattr(pruner.tarfile, 'open', fail)
    with pytest.raises(OSError, match='archive storage unavailable'):
        pruner.retire(release, tmp_path / 'receipts')
    assert (release / 'release.json').is_file() and (release / 'build').is_dir()


def git(repository, *arguments):
    return subprocess.run(['git', '-C', str(repository), *arguments], check=True, capture_output=True, text=True).stdout


def retention_workspace(tmp_path):
    """Eight sealed releases (oldest first), an interrupted one and four overlays around their anchor repository."""
    anchor, workspace = tmp_path / 'anchor', tmp_path / 'workspace'
    anchor.mkdir()
    (anchor / '.gitignore').write_text('.benchmarks/\n.env\n__pycache__/\n')
    git(anchor, 'init', '-q')
    git(anchor, 'add', '.')
    git(anchor, '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-q', '-m', 'fixture')
    overlays = {name: workspace / 'overlays' / f'device-{name * 32}' for name in 'abcd'}
    for age, (name, overlay) in enumerate(overlays.items()):
        overlay.mkdir(parents=True)
        if name != 'd':  # an interrupted overlay build has no marker
            (overlay / '.complete').write_text(overlay.name)
            os.utime(overlay / '.complete', (1000 + age, 1000 + age))
    releases = [cached_release(workspace, str(index) * 40) for index in range(8)]
    releases.append(workspace / 'releases' / ('d' * 40))
    for age, release in enumerate(releases):
        git(anchor, 'worktree', 'add', '-q', '--detach', str(release / 'source'), 'HEAD')
        (release / 'build/lekiwi_rmf').mkdir(parents=True)
        (release / 'build/lekiwi_rmf/release-ctest.xml').write_text('<testsuite/>')
        (release / 'install').mkdir()
        # Only the retired release 3 names overlay b; every other release names a.
        (release / 'install/.lekiwi-overlay').write_text(str(overlays['b' if age == 3 else 'a']) + '\n')
        if age < 8:
            os.utime(release / 'release.json', (1000 + age, 1000 + age))
    (workspace / 'current').symlink_to(releases[0])
    (workspace / 'previous').symlink_to(releases[1])
    process = tmp_path / 'proc/123'
    process.mkdir(parents=True)
    for name in ('maps', 'environ'):
        (process / name).write_bytes(b'')
    (process / 'cmdline').write_bytes(f'python3\0{releases[2]}/source/tool.py\0'.encode())
    (process / 'cwd').symlink_to(tmp_path)
    git(anchor, 'worktree', 'lock', str(releases[4] / 'source'))
    # A staging run killed while Git created its checkout leaves Git's own lock behind.
    git(anchor, 'worktree', 'lock', '--reason', 'initializing', str(releases[8] / 'source'))
    record = releases[3] / 'source/.benchmarks/onboard-braking/result.json'
    record.parent.mkdir(parents=True)
    record.write_text('measured on the robot')
    (releases[3] / 'source/.env').write_text('private configuration')
    (releases[3] / 'source/__pycache__').mkdir()
    (releases[3] / 'source/__pycache__/tool.pyc').write_bytes(b'cache')
    return anchor, workspace, releases, overlays


def run_retention(pruner, tmp_path, workspace, monkeypatch, capsys, *options):
    plan = pruner.plan_retention
    monkeypatch.setenv('LEKIWI_LOGS', str(tmp_path / 'logs'))
    monkeypatch.setattr(pruner, 'plan_retention', lambda root: plan(root, tmp_path / 'proc'))
    monkeypatch.setattr(sys, 'argv', ['prune-release-builds.py', str(workspace), '--retain', *options])
    pruner.main()
    return json.loads(capsys.readouterr().out)


def test_retention_keeps_releases_in_use_and_the_newest_and_removes_the_rest(tmp_path, pruner, monkeypatch, capsys):
    anchor, workspace, releases, overlays = retention_workspace(tmp_path)
    report = run_retention(pruner, tmp_path, workspace, monkeypatch, capsys, '--apply')
    assert report['kept'] == {
        releases[0].name: 'current', releases[1].name: 'previous', releases[2].name: 'process 123',
        releases[4].name: 'locked worktree', **{releases[index].name: 'recent' for index in (5, 6, 7)}}
    assert pruner.RETAINED == 3 and report['retired'] == [releases[3].name]
    assert report['unsealed'] == [releases[8].name]
    for index in (0, 1, 2, 4, 5, 6, 7):
        assert (releases[index] / 'release.json').is_file() and (releases[index] / 'build').is_dir()
        assert (releases[index] / 'source/.git').is_file()
    assert not releases[3].exists() and not releases[8].exists()
    # The anchor repository forgets the removed checkouts, so the revisions can be staged again.
    registered = git(anchor, 'worktree', 'list', '--porcelain')
    assert str(releases[3]) not in registered and str(releases[8]) not in registered
    assert str(releases[5] / 'source') in registered
    receipt, = (workspace / 'retired-release-manifests').iterdir()
    assert report['receipts'] == [str(receipt)] and receipt.name.startswith(releases[3].name)
    assert json.loads((receipt / 'release.json').read_text())['revision'] == releases[3].name
    import tarfile
    with tarfile.open(receipt / 'qualification.tar.gz') as archive:
        assert archive.extractfile('release-ctest.xml').read() == b'<testsuite/>'
    assert sorted(str(path.relative_to(receipt)) for path in receipt.rglob('*') if path.is_file()) == [
        'qualification.tar.gz', 'release.json', 'source/.benchmarks/onboard-braking/result.json']
    # a: named by kept releases; c: the newest complete one; b: named only by the retired release.
    assert {name for name, overlay in overlays.items() if overlay.exists()} == {'a', 'c'}


def test_retention_keeps_an_overlay_a_process_still_maps(tmp_path, pruner, monkeypatch, capsys):
    _, workspace, _, overlays = retention_workspace(tmp_path)
    (tmp_path / 'proc/123/maps').write_bytes(f'7f00-7f01 r-xp {overlays["b"]}/install/lib/libdriver.so\n'.encode())
    assert overlays['b'].name not in run_retention(pruner, tmp_path, workspace, monkeypatch, capsys, '--apply')['overlays']
    assert overlays['b'].is_dir()


def test_retention_dry_run_removes_nothing(tmp_path, pruner, monkeypatch, capsys):
    anchor, workspace, releases, overlays = retention_workspace(tmp_path)
    before = sorted(str(path) for path in workspace.rglob('*'))
    report = run_retention(pruner, tmp_path, workspace, monkeypatch, capsys)
    assert report['applied'] is False and report['receipts'] == []
    assert report['retired'] == [releases[3].name] and report['unsealed'] == [releases[8].name]
    assert report['overlays'] == [overlays['b'].name, overlays['d'].name]
    assert sorted(str(path) for path in workspace.rglob('*')) == before + [str(workspace / 'releases/stage.lock')]
    assert str(releases[3] / 'source') in git(anchor, 'worktree', 'list', '--porcelain')


def test_retention_fails_early_when_the_workspace_still_lacks_space(tmp_path, pruner, monkeypatch, capsys):
    _, workspace, releases, _ = retention_workspace(tmp_path)
    with pytest.raises(SystemExit) as failure:
        run_retention(pruner, tmp_path, workspace, monkeypatch, capsys, '--apply', '--need-free', str(1 << 60))
    message = str(failure.value)
    assert str(workspace) in message and f'{1 << 40} MiB needed' in message
    assert 'removed 1 releases, 1 unsealed directories and 2 overlays' in message
    assert not releases[3].exists() and (releases[0] / 'release.json').is_file()


def test_pruning_runs_under_locks_its_caller_holds_and_refuses_anyone_elses(tmp_path):
    import fcntl
    (tmp_path / 'releases').mkdir()
    (tmp_path / 'logs').mkdir()
    command = [sys.executable, str(ROOT / 'scripts/prune-release-builds.py'), str(tmp_path), '--retain']
    environment = {**os.environ, 'LEKIWI_LOGS': str(tmp_path / 'logs')}
    for held in ('releases/stage.lock', 'logs/deploy.lock'):
        with (tmp_path / held).open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            # stage-release.sh and deploy-split.sh hand their held descriptor down to the tool.
            inherited = subprocess.run(command, env=environment, pass_fds=[lock.fileno()], capture_output=True, text=True)
            assert inherited.returncode == 0, inherited.stderr
            foreign = subprocess.run(command, env=environment, capture_output=True, text=True)
            assert foreign.returncode != 0 and 'is held by another' in foreign.stderr


@pytest.mark.parametrize("builder", ["build-lekiwi.sh", "build-native.sh"])
def test_builders_refuse_a_sealed_release_before_building(tmp_path, builder):
    (tmp_path / "install").mkdir()
    (tmp_path / "release.json").write_text("{}")
    result = subprocess.run(["bash", str(ROOT / "scripts" / builder)],
                            env={**os.environ, "LEKIWI_WS": str(tmp_path)}, capture_output=True, text=True)
    assert result.returncode != 0 and "sealed workspace" in result.stderr
    assert list((tmp_path / "install").iterdir()) == []


@pytest.fixture
def tmp_path():
    # Fixture repositories contain Git worktrees, which must survive outside /tmp.
    directory = ROOT / ".benchmarks"
    directory.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=directory, prefix="release-test-") as temporary:
        yield Path(temporary)


def write_executable(path, contents):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/usr/bin/env bash\nset -e\n" + contents)
    path.chmod(0o755)


def release_fixture(tmp_path):
    repo, workspace, binaries = (tmp_path / name for name in ("repo", "workspace", "bin"))
    (repo / "scripts").mkdir(parents=True)
    (repo / "test").mkdir()
    (repo / "config").mkdir()
    (repo / "config/device_tests.txt").write_text("test_example\n")
    # check-release validates the tracked safety acceptance with the source's own modules.
    (repo / "lekiwi_rmf").mkdir()
    for name in ("lekiwi_rmf/__init__.py", "lekiwi_rmf/geometry.py", "lekiwi_rmf/motion_guards.py",
                 "lekiwi_rmf/safety_acceptance.py", "config/safety_production.yaml",
                 "config/safety_acceptance.yaml", "config/nav2_params.yaml"):
        (repo / name).write_text((ROOT / name).read_text())
    (repo / ".gitignore").write_text(".env\n__pycache__/\n")
    (repo / "test/test_example.py").write_text("def test_example(): pass\n")
    (repo / ".env").write_text("LEKIWI_WRIST=none\n")
    for name in ("stage-release.sh", "check-release.py", "dependency-overlay-key.sh", "prune-release-builds.py"):
        path = repo / "scripts" / name
        path.write_text((ROOT / "scripts" / name).read_text())
        path.chmod(0o755)
    write_executable(repo / "scripts/build-native.sh", '''
[[ ${LEKIWI_TEST_BUILD_FAIL:-0} != 1 ]] || exit 42
[[ -z ${LEKIWI_TEST_BUILD_LOG:-} ]] || echo native >> "$LEKIWI_TEST_BUILD_LOG"
git -C "$(dirname "$0")/.." rev-parse HEAD > "$LEKIWI_WS/install/.lekiwi-native-revision"
for file in rclcpp/lib/librclcpp.so class_loader/lib/libclass_loader.so rtabmap_sync/lib/librtabmap_sync.so nav2_lifecycle_manager/lib/nav2_lifecycle_manager/lifecycle_manager nav2_smac_planner/lib/libnav2_smac_planner_lattice.so rviz_ogre_vendor/opt/rviz_ogre_vendor/lib/OGRE/RenderSystem_GL.so; do
  mkdir -p "$LEKIWI_WS/install/${file%/*}"
  printf artifact > "$LEKIWI_WS/install/$file"
done
''')
    write_executable(repo / "scripts/build-lekiwi.sh", '''
[[ ${LEKIWI_TEST_BUILD_FAIL:-0} != 1 ]] || exit 42
if [[ " $* " == *" --dependencies "* ]]; then
  [[ -z ${LEKIWI_TEST_BUILD_LOG:-} ]] || echo drivers >> "$LEKIWI_TEST_BUILD_LOG"
  exit 0
fi
[[ -z ${LEKIWI_TEST_BUILD_LOG:-} ]] || echo "package on $(cat "$LEKIWI_WS/install/setup.bash")" >> "$LEKIWI_TEST_BUILD_LOG"
[[ ${LEKIWI_TEST_PACKAGE_FAIL:-0} != 1 ]] || exit 43
mkdir -p "$LEKIWI_WS/install/lekiwi_rmf/share/lekiwi_rmf" "$LEKIWI_WS/build/lekiwi_rmf"
git -C "$(dirname "$0")/.." rev-parse HEAD > "$LEKIWI_WS/install/lekiwi_rmf/.lekiwi-source-revision"
printf '<package/>' > "$LEKIWI_WS/install/lekiwi_rmf/share/lekiwi_rmf/package.xml"
printf 'true\\n' > "$LEKIWI_WS/install/setup.bash"
''')
    write_executable(binaries / "systemctl", "exit 0\n")
    write_executable(binaries / "shellcheck", "exit 0\n")
    write_executable(binaries / "ctest", '''
while [[ $# -gt 0 ]]; do
  if [[ $1 == --output-junit ]]; then report=$2; break; fi
  shift
done
printf '<testsuite><testcase name="test_example"/></testsuite>' > "$report"
mkdir -p "${report%/*}/test_results/lekiwi_rmf"
printf '<testsuite><testcase name="test_example"/></testsuite>' > "${report%/*}/test_results/lekiwi_rmf/test_example.xunit.xml"
''')
    for args in (("init",), ("config", "user.name", "Test"),
                 ("config", "user.email", "test@example.invalid"), ("add", "."), ("commit", "-m", "fixture")):
        subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)
    revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    (workspace / "install").mkdir(parents=True)
    (workspace / "install/setup.bash").write_text("true\n")
    (workspace / "install/live-artifact").write_text("unchanged")
    (workspace / "current").symlink_to(workspace / "install", target_is_directory=True)
    # Staging applies the retention policy under the deployment lock; never contend for the real one.
    environment = {**os.environ, "PATH": f"{binaries}:{os.environ['PATH']}", "LEKIWI_LOGS": str(tmp_path / "logs")}
    return repo, workspace, revision, environment


@pytest.mark.parametrize("fail", [False, True])
@pytest.mark.parametrize("role", ["compute", "device"])
def test_staging_keeps_live_source_artifacts_and_pointer_unchanged(tmp_path, fail, role):
    repo, workspace, revision, environment = release_fixture(tmp_path)
    command = ["bash", str(repo / "scripts/stage-release.sh"), role, str(workspace), revision]
    result = subprocess.run(command, env={**environment, "LEKIWI_TEST_BUILD_FAIL": str(int(fail))},
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == (42 if fail else 0), result.stdout + result.stderr
    assert (workspace / "current").resolve() == workspace / "install"
    assert (workspace / "install/live-artifact").read_text() == "unchanged"
    assert subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip() == revision
    release = workspace / "releases" / revision
    assert (release / "release.json").exists() is not fail
    if fail:
        # A failed run frees its partial release and its checkout record at once.
        assert not release.exists() and str(release) not in git(repo, "worktree", "list", "--porcelain")
    else:
        overlay = Path((release / "install/.lekiwi-overlay").read_text().strip())
        assert overlay.parent == workspace / "overlays" and (overlay / ".complete").is_file()
        assert (overlay / "install/.lekiwi-native-revision").is_file() is (role == "compute")
        assert not (release / "install/.lekiwi-native-revision").exists()
        assert subprocess.run(command, env=environment, capture_output=True, timeout=30).returncode == 0
        cache = release / "install/lekiwi_rmf/__pycache__/runtime.cpython-312.pyc"
        cache.parent.mkdir()
        cache.write_bytes(b"runtime cache")
        assert subprocess.run(command, env=environment, capture_output=True, timeout=30).returncode == 0
        package = release / "install/lekiwi_rmf/share/lekiwi_rmf/package.xml"
        mode = package.stat().st_mode & 0o7777
        package.chmod(mode ^ 0o111)
        assert subprocess.run(command, env=environment, capture_output=True, timeout=30).returncode != 0
        package.chmod(mode)
        (release / "install/lekiwi_rmf/share/lekiwi_rmf/package.xml").write_text("changed")
        assert subprocess.run(command, env=environment, capture_output=True, timeout=30).returncode != 0


def test_release_sealing_refuses_failed_or_missing_source_tests(tmp_path):
    repo, workspace, revision, environment = release_fixture(tmp_path)
    subprocess.run(["bash", str(repo / "scripts/stage-release.sh"), "compute", str(workspace), revision],
                   env=environment, check=True, capture_output=True, timeout=30)
    spec = importlib.util.spec_from_file_location("check_release", ROOT / "scripts/check-release.py")
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    release = workspace / "releases" / revision
    report = release / "build/lekiwi_rmf/release-ctest.xml"
    original = report.read_text()
    for contents in ("<testsuite/>", '<testsuite><testcase name="test_example"><failure/></testcase></testsuite>'):
        report.write_text(contents)
        with pytest.raises(ValueError, match="every source test"):
            checker.check_release(release, revision, "compute")
    report.write_text(original)
    individual = release / "build/lekiwi_rmf/test_results/lekiwi_rmf/test_example.xunit.xml"
    for contents in ("<testsuite/>", '<testsuite><testcase name="test_example"><skipped/></testcase></testsuite>',
                     '<testsuite><testcase name="test_example"><failure/></testcase></testsuite>'):
        individual.write_text(contents)
        with pytest.raises(ValueError, match="every Python test case"):
            checker.check_release(release, revision, "compute")
    individual.unlink()
    with pytest.raises(ValueError, match="missing Python test evidence"):
        checker.check_release(release, revision, "compute")


@pytest.mark.parametrize("dependency", ["rclcpp", "rtabmap_ros"])
def test_releases_reuse_a_dependency_overlay_until_its_inputs_change(tmp_path, dependency):
    repo, workspace, first, environment = release_fixture(tmp_path)
    log = tmp_path / "builds"
    environment = {**environment, "LEKIWI_TEST_BUILD_LOG": str(log)}

    def stage(revision):
        result = subprocess.run(["bash", str(repo / "scripts/stage-release.sh"), "compute", str(workspace), revision],
                                env=environment, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        return workspace / "releases" / revision

    def commit(name, message):
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"# {message}\n")
        for args in (("add", name), ("commit", "-m", message)):
            subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)
        return subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()

    def overlay_of(release):
        return Path((release / "install/.lekiwi-overlay").read_text().strip())

    first_release = stage(first)
    second = commit("lekiwi_rmf/feature.py", "package-only change")
    second_release = stage(second)
    assert [line for line in log.read_text().splitlines() if line in ("native", "drivers")] == ["native", "drivers"]
    assert overlay_of(first_release) == overlay_of(second_release)
    third_release = stage(commit(f"thirdparty/{dependency}/fix.patch", "dependency change"))
    assert [line for line in log.read_text().splitlines() if line in ("native", "drivers")] == ["native", "drivers"] * 2
    assert overlay_of(third_release) != overlay_of(second_release)
    # An interrupted overlay build is rebuilt, never reused.
    (overlay_of(third_release) / ".complete").unlink()
    stage(commit("lekiwi_rmf/other.py", "after an interrupted dependency build"))
    assert [line for line in log.read_text().splitlines() if line in ("native", "drivers")] == ["native", "drivers"] * 3

    spec = importlib.util.spec_from_file_location("check_release", ROOT / "scripts/check-release.py")
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    checker.check_release(second_release, second, "compute")
    (overlay_of(second_release) / "install/rclcpp/lib/librclcpp.so").write_text("changed")
    with pytest.raises(ValueError, match="changed after qualification"):
        checker.check_release(second_release, second, "compute")


def test_a_retried_release_is_built_from_scratch_against_a_changed_dependency_overlay(tmp_path):
    repo, workspace, revision, environment = release_fixture(tmp_path)
    log = tmp_path / "builds"
    environment = {**environment, "LEKIWI_TEST_BUILD_LOG": str(log)}
    dependency = workspace / "src/ldlidar_stl_ros2"
    subprocess.run(["git", "clone", "-q", str(repo), str(dependency)], check=True)
    command = ["bash", str(repo / "scripts/stage-release.sh"), "compute", str(workspace), revision]
    failed = subprocess.run(command, env={**environment, "LEKIWI_TEST_PACKAGE_FAIL": "1"},
                            capture_output=True, text=True, timeout=30)
    assert failed.returncode == 43, failed.stdout + failed.stderr
    release = workspace / "releases" / revision
    first, = (workspace / "overlays").iterdir()
    assert not release.exists() and (first / ".complete").is_file()
    # A run killed outright cannot clean up: its build still names the overlay of its time.
    stale = release / "build/lekiwi_rmf/CMakeCache.txt"
    stale.parent.mkdir(parents=True)
    stale.write_text(f"rclcpp_DIR:PATH={first}/install/rclcpp\n")
    # An installed-package upgrade or dependency source change between attempts changes the key.
    git(dependency, "-c", "user.name=Test", "-c", "user.email=test@example.invalid",
        "commit", "-q", "--allow-empty", "-m", "upgrade")
    result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    second = Path((release / "install/.lekiwi-overlay").read_text().strip())
    assert second != first and not stale.exists()
    assert log.read_text().splitlines()[-1] == f"package on source {second}/install/setup.bash"


def test_staging_first_removes_superseded_releases_leftovers_and_unused_overlays(tmp_path):
    repo, workspace, first, environment = release_fixture(tmp_path)

    def stage(revision):
        # As the deployer stages on the device: the script arrives on standard input.
        with (ROOT / "scripts/stage-release.sh").open() as script:
            result = subprocess.run(["bash", "-s", "--", "device", str(workspace), revision, str(repo)],
                                    stdin=script, env=environment, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        return workspace / "releases" / revision

    def commit(name):
        (repo / name).parent.mkdir(parents=True, exist_ok=True)
        (repo / name).write_text("# change\n")
        git(repo, "add", name)
        git(repo, "commit", "-q", "-m", name)
        return git(repo, "rev-parse", "HEAD").strip()

    active = stage(first)
    (workspace / "current").unlink()
    (workspace / "current").symlink_to(active)
    old_overlay = Path((active / "install/.lekiwi-overlay").read_text().strip())
    releases = [stage(commit(f"lekiwi_rmf/change{index}.py")) for index in range(3)]
    leftover = workspace / "releases" / ("d" * 40) / "build"
    leftover.mkdir(parents=True)
    # Three other releases are sealed already: staging two more retires the two oldest of them.
    releases.append(stage(commit("thirdparty/ldlidar_stl_ros2/fix.patch")))
    releases.append(stage(commit("lekiwi_rmf/latest.py")))
    assert not leftover.parent.exists()
    assert [release.exists() for release in releases] == [False, True, True, True, True]
    assert (active / "release.json").is_file() and old_overlay.is_dir()
    registered = git(repo, "worktree", "list", "--porcelain")
    assert str(releases[0]) not in registered and str(releases[1] / "source") in registered
    assert len(list((workspace / "retired-release-manifests").iterdir())) == 1
    # Activating the newest release leaves the first overlay to `previous` alone...
    new_overlay = Path((releases[-1] / "install/.lekiwi-overlay").read_text().strip())
    assert new_overlay != old_overlay
    for name, target in (("previous", active), ("current", releases[-1])):
        (workspace / name).unlink(missing_ok=True)
        (workspace / name).symlink_to(target)
    stage(commit("lekiwi_rmf/after.py"))
    assert old_overlay.is_dir() and active.is_dir()
    # ...and once no kept release is built on it, the next staging removes it.
    (workspace / "previous").unlink()
    (workspace / "previous").symlink_to(releases[-2])
    for index in range(4):
        stage(commit(f"lekiwi_rmf/later{index}.py"))
    assert not active.exists() and not old_overlay.exists() and new_overlay.is_dir()


def test_staging_clones_only_the_materialized_vendor_revision(tmp_path):
    repo, workspace, revision, environment = release_fixture(tmp_path)
    cached = workspace / "src/ldlidar_stl_ros2"
    subprocess.run(["git", "clone", str(repo), str(cached)], check=True, capture_output=True)
    for key, value in (("user.name", "Test"), ("user.email", "test@example.invalid")):
        subprocess.run(["git", "-C", str(cached), "config", key, value], check=True)
    missing = cached / "history-only"
    missing.write_text("historical blob deliberately absent from a partial clone")
    for args in (("add", "history-only"), ("commit", "-m", "old blob")):
        subprocess.run(["git", "-C", str(cached), *args], check=True, capture_output=True)
    blob = subprocess.check_output(["git", "-C", str(cached), "rev-parse", "HEAD:history-only"], text=True).strip()
    missing.unlink()
    for args in (("add", "-u"), ("commit", "-m", "materialized HEAD"),
                 ("config", "remote.origin.promisor", "true")):
        subprocess.run(["git", "-C", str(cached), *args], check=True, capture_output=True)
    (cached / ".git/objects" / blob[:2] / blob[2:]).unlink()
    result = subprocess.run(["bash", str(repo / "scripts/stage-release.sh"), "compute", str(workspace), revision],
                            env=environment, text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert (workspace / "releases" / revision / "release.json").is_file()


def test_interrupted_staging_removes_its_partial_release_and_overlay(tmp_path):
    repo, workspace, _, environment = release_fixture(tmp_path)
    builder = repo / "scripts/build-native.sh"
    builder.write_text(builder.read_text() + 'touch "$LEKIWI_TEST_ENTERED"\nsleep 30\n')
    git(repo, "commit", "-q", "-am", "blocking build")
    revision = git(repo, "rev-parse", "HEAD").strip()
    entered = tmp_path / "entered"
    process = subprocess.Popen(["bash", str(repo / "scripts/stage-release.sh"), "compute", str(workspace), revision],
                               env={**environment, "LEKIWI_TEST_ENTERED": str(entered)}, start_new_session=True,
                               text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        deadline = time.monotonic() + 10
        while not entered.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert entered.exists() and (workspace / "releases" / revision / "source").is_dir()
    finally:
        os.killpg(process.pid, 15)  # a dropped SSH session ends the whole staging session
        stdout, stderr = process.communicate(timeout=30)
    assert process.returncode == 143, stdout + stderr
    assert not (workspace / "releases" / revision).exists() and list((workspace / "overlays").iterdir()) == []
    assert str(workspace) not in git(repo, "worktree", "list", "--porcelain")


def test_staging_finishes_its_original_commands_when_the_caller_file_changes(tmp_path):
    repo, workspace, _, environment = release_fixture(tmp_path)
    builder = repo / "scripts/build-native.sh"
    builder.write_text(builder.read_text() + '''
touch "$LEKIWI_TEST_ENTERED"
while [[ ! -e $LEKIWI_TEST_CONTINUE ]]; do sleep 0.01; done
''')
    for args in (("add", "scripts/build-native.sh"), ("commit", "-m", "blocking build")):
        subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)
    revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    entered, resume = tmp_path / "entered", tmp_path / "resume"
    script = repo / "scripts/stage-release.sh"
    process = subprocess.Popen(["bash", str(script), "compute", str(workspace), revision],
                               env={**environment, "LEKIWI_TEST_ENTERED": str(entered), "LEKIWI_TEST_CONTINUE": str(resume)},
                               text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        deadline = time.monotonic() + 5
        while not entered.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert entered.exists()
        script.write_text("exit 99\n" * 10000)
    finally:
        resume.touch()
        stdout, stderr = process.communicate(timeout=30)
    assert process.returncode == 0, stdout + stderr
    assert (workspace / "releases" / revision / "release.json").is_file()


def test_activation_retains_the_previous_release_and_refuses_an_unknown_previous_file(tmp_path):
    deploy = (ROOT / "scripts/deploy-split.sh").read_text()
    function = "activate_release() {" + deploy.split("activate_release() {", 1)[1].split("\ntrap ", 1)[0]
    before, after = tmp_path / "before", tmp_path / "after"
    before.mkdir()
    after.mkdir()
    (tmp_path / "current").symlink_to(before, target_is_directory=True)
    subprocess.run(["bash", "-ec", function + '\nactivate_release "$1" "$2"',
                    "activation", str(tmp_path), str(after)], check=True)
    assert (tmp_path / "current").resolve() == after
    assert (tmp_path / "previous").resolve() == before
    subprocess.run(["bash", "-ec", function + '\nactivate_release "$1" "$2"',
                    "activation", str(tmp_path), str(after)], check=True)
    assert (tmp_path / "previous").resolve() == before
    (tmp_path / "previous").unlink()
    (tmp_path / "previous").write_text("operator file")
    result = subprocess.run(["bash", "-ec", function + '\nactivate_release "$1" "$2"',
                             "activation", str(tmp_path), str(before)])
    assert result.returncode != 0 and (tmp_path / "current").resolve() == after
    assert (tmp_path / "previous").read_text() == "operator file"


@pytest.mark.parametrize("existing,requested", [(True, False), (False, True), (False, False)])
def test_compute_service_migration_preserves_curve_and_tailnet_options(tmp_path, existing, requested):
    deploy = (ROOT / "scripts/deploy-split.sh").read_text()
    function = "refresh_compute_service() {" + deploy.split("refresh_compute_service() {", 1)[1].split(
        "\ncompute_configuration_current()", 1)[0]
    settings = tmp_path / "settings"
    settings.write_text("LEKIWI_STACK_ARGS=" + ("start_rosbridge:=true rosbridge_address:=100.64.0.1 " if existing else "")
                        + "curve_client_secret_key_file:=/private/curve/clients/driver.key_secret\n")
    output = tmp_path / "arguments"
    installer = tmp_path / "current/source/scripts/reinstall-compute.sh"
    write_executable(installer, 'printf "%s\\n" "$@" > "$LEKIWI_TEST_OUTPUT"\n')
    script = '''
log() { :; }
die() { echo "$*" >&2; exit 1; }
grep() { command grep "${@:1:$#-1}" "$LEKIWI_TEST_SETTINGS"; }
sed() { command sed "${@:1:$#-1}" "$LEKIWI_TEST_SETTINGS"; }
compute_current=$1/current
device=device.example
rosbridge_tailnet=$2
''' + function + "\nrefresh_compute_service\n"
    subprocess.run(["bash", "-ec", script, "migration", str(tmp_path), str(requested).lower()], check=True,
                   env={**os.environ, "LEKIWI_TEST_OUTPUT": str(output), "LEKIWI_TEST_SETTINGS": str(settings)})
    assert output.read_text().splitlines() == ["--no-start", "--curve-dir", "/private/curve"] + (["--rosbridge-tailnet"] if existing or requested else [])


def test_deployment_failure_trap_preserves_the_original_failure_status():
    trap = next(line for line in (ROOT / "scripts/deploy-split.sh").read_text().splitlines()
                if line.startswith("trap ") and "deployment failed" in line)
    result = subprocess.run(["bash", "-c", trap + "\nexit 42"], capture_output=True, text=True)
    assert result.returncode == 42
    assert "previous releases and bootstrap installation are retained" in result.stderr


def test_release_check_rejects_a_stale_safety_acceptance(tmp_path):
    spec = importlib.util.spec_from_file_location("check_release", ROOT / "scripts/check-release.py")
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    source = tmp_path / "source"
    (source / "config").mkdir(parents=True)
    (source / "lekiwi_rmf").mkdir()
    for name in ("lekiwi_rmf/__init__.py", "lekiwi_rmf/geometry.py", "lekiwi_rmf/motion_guards.py",
                 "lekiwi_rmf/safety_acceptance.py", "config/safety_production.yaml",
                 "config/safety_acceptance.yaml", "config/nav2_params.yaml"):
        (source / name).write_text((ROOT / name).read_text())
    checker.check_safety_acceptance(source)
    production = source / "config/safety_production.yaml"
    parameters=yaml.safe_load(production.read_text())
    parameters['safety_supervisor']['ros__parameters']['stow_joint_positions'][0]+=.1
    production.write_text(yaml.safe_dump(parameters))
    with pytest.raises(ValueError, match="accepted arm stow differs"):
        checker.check_safety_acceptance(source)


@pytest.mark.parametrize('arm_ok, readiness_ok, strict, expected', [
    (True, True, False, 0), (False, True, False, 1),
    (True, False, False, 1), (False, True, True, 0),
])
def test_deployment_records_success_only_after_required_readiness_and_rearm(
    tmp_path, arm_ok, readiness_ok, strict, expected,
):
    deploy = (ROOT / 'scripts/deploy-split.sh').read_text()

    def function(name):
        start = deploy.index(name + '() {')
        return deploy[start:deploy.index('\n}\n', start) + 3]

    # The closing sequence of a restart; a resumed run records through the same two functions.
    final = deploy.split('# The verification above left the driver deliberately disarmed', 1)[1].split('\n', 1)[1]
    assert final.startswith('validate_robot\n') and 'record_deployment\n' in final
    assert deploy.count('( set -e; validate_robot )') == 1
    marker = tmp_path / 'deployed-revision'
    remote = tmp_path / 'remote-recorded'
    script = '''
set -e
log() { :; }
die() { echo "$*" >&2; exit 1; }
verify_release() { return 0; }
operational_ready() { [[ $LEKIWI_TEST_READY == true ]]; }
has_device_unit() { return 1; }
timeout() { echo laser; }
driver_state() { echo DISARMED; }
wait_for() { [[ $1 == 30 ]] || [[ $LEKIWI_TEST_ARM == true ]]; }
PROJECT_ROOT=/unused
record_remote() { touch "$LEKIWI_TEST_REMOTE"; }
ssh_command=(record_remote)
marker=$LEKIWI_TEST_MARKER
remote_home=/unused
remote_marker=/unused/marker
target=abcdef0123456789
device=test-peer
''' + function('validate_robot') + function('record_deployment') + '{\n' + final
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
        env={**os.environ, 'LEKIWI_TEST_READY': str(readiness_ok).lower(),
             'LEKIWI_TEST_ARM': str(arm_ok).lower(), 'LEKIWI_TEST_MARKER': str(marker),
             'LEKIWI_TEST_REMOTE': str(remote), 'LEKIWI_DISARM_ON_FAILURE': str(strict).lower()})
    assert result.returncode == expected, result.stderr
    assert marker.exists() == remote.exists() == (expected == 0)
    if not expected:
        assert marker.read_text().strip() == 'abcdef0123456789'


def test_same_revision_shortcut_checks_operational_readiness_before_success(tmp_path):
    deploy = (ROOT / 'scripts/deploy-split.sh').read_text()
    block = deploy.split('    remote_unit_active_all "${device_units[@]}"; then\n', 1)[1]
    block = block.split('\nfi\n', 1)[0]
    for ready in ('true', 'false'):
        script = '''
set -e
ros_setup() { :; }
operational_ready() { [[ $1 == 30 && $2 == true && $READY == true ]]; }
log() { echo recovery; }
target=abcdef0123456789
''' + block + '\necho continued\n'
        result = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
                                env={**os.environ, 'READY': ready, 'LEKIWI_DISARM_ON_FAILURE': 'false'})
        assert result.returncode == 0, result.stderr
        assert ('continued' in result.stdout) == (ready == 'false')
        assert ('readiness verified' in result.stdout) == (ready == 'true')
