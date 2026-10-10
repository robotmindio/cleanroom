"""Exercise the launcher with a fake ROS CLI; never send a hardware goal."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("setup_status, ros_status", [(0, 0), (0, 7), (23, 0)])
def test_explore_launcher_sources_environment_and_preserves_failures(
    tmp_path, setup_status, ros_status,
):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    launcher = scripts / "explore.sh"
    shutil.copyfile(Path(__file__).parents[1] / "scripts/explore.sh", launcher)
    (scripts / "setup.bash").write_text(
        f"export EXPLORE_TEST_READY=1\nreturn {setup_status}\n"
    )
    package = tmp_path / "lekiwi_rmf"
    package.mkdir()
    (package / "__init__.py").touch()
    binary = tmp_path / "bin"
    binary.mkdir()
    ros = package / "explore_client.py"
    ros.write_text(
        "#!/usr/bin/python3\n"
        "import json, os, sys\n"
        "assert os.environ['EXPLORE_TEST_READY'] == '1'\n"
        "print(json.dumps(sys.argv[1:]))\n"
        "sys.exit(int(os.environ['EXPLORE_TEST_STATUS']))\n"
    )
    ros.chmod(0o755)
    result = subprocess.run(
        ["bash", str(launcher)], cwd=tmp_path.parent,
        env={**os.environ, "PATH": f"{binary}:{os.environ['PATH']}",
             "EXPLORE_TEST_STATUS": str(ros_status)},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == (setup_status or ros_status), result.stderr
    if setup_status:
        assert result.stdout == ""
    else:
        arguments = json.loads(result.stdout)
        assert arguments == []
