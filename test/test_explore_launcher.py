"""Exercise workspace selection and imports without sending a hardware goal."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("setup_status, ros_status", [(0, 0), (0, 7), (23, 0)])
@pytest.mark.parametrize("workspace_kind", ["checkout", "deployed", "override"])
def test_explore_launcher_sources_environment_and_preserves_failures(
    tmp_path, setup_status, ros_status, workspace_kind,
):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    launcher = scripts / "explore.sh"
    shutil.copyfile(Path(__file__).parents[1] / "scripts/explore.sh", launcher)
    deployed = tmp_path / "lekiwi_ws/current"
    if workspace_kind != "checkout":
        (deployed / "install").mkdir(parents=True)
        (deployed / "install/setup.bash").touch()
    workspace = {"checkout": tmp_path, "deployed": deployed,
                 "override": tmp_path / "custom"}[workspace_kind]
    (scripts / "setup.bash").write_text(
        'export EXPLORE_TEST_READY=1\n'
        'export PYTHONPATH="${LEKIWI_WS:-$PWD}/install/site-packages"\n'
        f"return {setup_status}\n"
    )
    # The checkout must not hide the selected workspace's generated interfaces.
    source_package = tmp_path / "lekiwi_rmf"
    source_package.mkdir()
    (source_package / "__init__.py").write_text('raise AssertionError("source package imported")\n')
    package = workspace / "install/site-packages/lekiwi_rmf"
    package.mkdir(parents=True)
    (package / "__init__.py").touch()
    (package / "action.py").write_text('class Explore: pass\n')
    ros = package / "explore_client.py"
    ros.write_text(
        "#!/usr/bin/python3\n"
        "import json, os, sys\n"
        "from lekiwi_rmf.action import Explore\n"
        "assert os.environ['EXPLORE_TEST_READY'] == '1'\n"
        "assert os.environ.get('LEKIWI_WS', '') == os.environ['EXPLORE_TEST_WORKSPACE']\n"
        "print(json.dumps(sys.argv[1:]))\n"
        "sys.exit(int(os.environ['EXPLORE_TEST_STATUS']))\n"
    )
    ros.chmod(0o755)
    environment = {**os.environ, "HOME": str(tmp_path),
                   "EXPLORE_TEST_STATUS": str(ros_status),
                   "EXPLORE_TEST_WORKSPACE": "" if workspace_kind == "checkout" else str(workspace)}
    environment.pop("LEKIWI_WS", None)
    if workspace_kind == "override":
        environment["LEKIWI_WS"] = str(workspace)
    result = subprocess.run(
        ["bash", str(launcher)], cwd=tmp_path.parent,
        env=environment,
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == (setup_status or ros_status), result.stderr
    if setup_status:
        assert result.stdout == ""
    else:
        arguments = json.loads(result.stdout)
        assert arguments == []
