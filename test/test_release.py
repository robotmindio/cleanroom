"""Release staging and activation never mutate the live installation."""

import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile

import pytest


ROOT = Path(__file__).parents[1]


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
    (repo / ".gitignore").write_text(".env\n__pycache__/\n")
    (repo / "test/test_example.py").write_text("def test_example(): pass\n")
    (repo / ".env").write_text("LEKIWI_WRIST=none\n")
    for name in ("stage-release.sh", "check-release.py"):
        path = repo / "scripts" / name
        path.write_text((ROOT / "scripts" / name).read_text())
        path.chmod(0o755)
    write_executable(repo / "scripts/build-native.sh", '''
[[ ${LEKIWI_TEST_BUILD_FAIL:-0} != 1 ]] || exit 42
git -C "$(dirname "$0")/.." rev-parse HEAD > "$LEKIWI_WS/install/.lekiwi-native-revision"
for file in rclcpp/lib/librclcpp.so class_loader/lib/libclass_loader.so nav2_lifecycle_manager/lib/nav2_lifecycle_manager/lifecycle_manager rviz_ogre_vendor/opt/rviz_ogre_vendor/lib/OGRE/RenderSystem_GL.so; do
  mkdir -p "$LEKIWI_WS/install/${file%/*}"
  printf artifact > "$LEKIWI_WS/install/$file"
done
''')
    write_executable(repo / "scripts/build-lekiwi.sh", '''
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
''')
    for args in (("init",), ("config", "user.name", "Test"),
                 ("config", "user.email", "test@example.invalid"), ("add", "."), ("commit", "-m", "fixture")):
        subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)
    revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    (workspace / "install").mkdir(parents=True)
    (workspace / "install/setup.bash").write_text("true\n")
    (workspace / "install/live-artifact").write_text("unchanged")
    (workspace / "current").symlink_to(workspace / "install", target_is_directory=True)
    environment = {**os.environ, "PATH": f"{binaries}:{os.environ['PATH']}"}
    return repo, workspace, revision, environment


@pytest.mark.parametrize("fail", [False, True])
def test_staging_keeps_live_source_artifacts_and_pointer_unchanged(tmp_path, fail):
    repo, workspace, revision, environment = release_fixture(tmp_path)
    command = ["bash", str(repo / "scripts/stage-release.sh"), "compute", str(workspace), revision]
    result = subprocess.run(command, env={**environment, "LEKIWI_TEST_BUILD_FAIL": str(int(fail))},
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == (42 if fail else 0), result.stdout + result.stderr
    assert (workspace / "current").resolve() == workspace / "install"
    assert (workspace / "install/live-artifact").read_text() == "unchanged"
    assert subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip() == revision
    release = workspace / "releases" / revision
    assert (release / "release.json").exists() is not fail
    if not fail:
        assert subprocess.run(command, env=environment, capture_output=True, timeout=30).returncode == 0
        cache = release / "install/lekiwi_rmf/__pycache__/runtime.cpython-312.pyc"
        cache.parent.mkdir()
        cache.write_bytes(b"runtime cache")
        assert subprocess.run(command, env=environment, capture_output=True, timeout=30).returncode == 0
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
    for contents in ("<testsuite/>", '<testsuite><testcase name="test_example"><failure/></testcase></testsuite>'):
        report.write_text(contents)
        with pytest.raises(ValueError, match="every source test"):
            checker.check_release(release, revision, "compute")


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


def test_activation_retains_the_previous_release_and_refuses_an_unknown_previous_file(tmp_path):
    deploy = (ROOT / "scripts/deploy-split.sh").read_text()
    function = "activate_release() {" + deploy.split("activate_release() {", 1)[1].split("\non_exit()", 1)[0]
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
