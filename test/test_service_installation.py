"""No-root behavioral checks for systemd template rendering and host startup."""

from __future__ import annotations

import os
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).parents[1]


def test_all_units_render_with_deterministic_non_root_paths(tmp_path):
    script = r'''
set -Eeuo pipefail
PROJECT_ROOT=$1
output=$2
UNIT_DIR=$output
LEKIWI_SERVICE_USER=robot
LEKIWI_SERVICE_HOME=/srv/robot
LEKIWI_SERVICE_WORKSPACE=/srv/robot/lekiwi_ws
LEKIWI_SERVICE_LEROBOT_VENV=/srv/robot/lerobot-venv
LEKIWI_HOST_BIND_ADDRESS=127.0.0.1
LEKIWI_CURVE_SERVER_SECRET=
LEKIWI_CURVE_SERVER_PUBLIC=
LEKIWI_CURVE_AUTHORIZED_CLIENTS=
LEKIWI_CURVE_HEALTH_CLIENT_SECRET=
as_root() { "$@"; }
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$PROJECT_ROOT/scripts/service-install-common.sh"
for template in "$PROJECT_ROOT"/systemd/*.service; do
  render_systemd_unit "$template" "$output/$(basename "$template")"
done
'''
    subprocess.run(
        ["bash", "-c", script, "render-test", str(ROOT), str(tmp_path)],
        check=True,
    )
    for unit in tmp_path.glob("*.service"):
        text = unit.read_text(encoding="utf-8")
        for placeholder in (
            "@PROJECT_ROOT@", "@SERVICE_USER@", "@SERVICE_HOME@", "@WORKSPACE@",
            "@LEROBOT_VENV@", "@LEROBOT_PYTHON@", "@HOST_BIND_ADDRESS@",
            "@CURVE_SERVER_SECRET@", "@CURVE_SERVER_PUBLIC@",
            "@CURVE_AUTHORIZED_CLIENTS@", "@CURVE_HEALTH_CLIENT_SECRET@",
        ):
            assert placeholder not in text
        assert "User=robot" in text
        assert "Environment=HOME=/srv/robot" in text
        assert "WorkingDirectory=" + str(ROOT) in text
    host = (tmp_path / "lekiwi-host.service").read_text(encoding="utf-8")
    assert "LEKIWI_BIND_ADDRESS=127.0.0.1" in host
    assert "host-health-check.py --host 127.0.0.1" in host
    for unit in tmp_path.glob("*.service"):
        rendered = unit.read_text(encoding="utf-8")
        for setting in (
            "NoNewPrivileges=true",
            "PrivateTmp=true",
            "ProtectSystem=full",
            "ProtectControlGroups=true",
            "ProtectKernelTunables=true",
            "ProtectKernelModules=true",
            "ProtectKernelLogs=true",
            "LockPersonality=true",
            "RestrictSUIDSGID=true",
            "UMask=0077",
        ):
            assert setting in rendered


def test_missing_lerobot_environment_fails_once_as_configuration_error(tmp_path):
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "LEKIWI_WS": str(tmp_path / "workspace"),
        "LEKIWI_LEROBOT_VENV": str(tmp_path / "missing-venv"),
    }
    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "robot-host.sh"), "--no-cameras"],
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        timeout=5,
    )
    assert result.returncode == 78
    assert "LeRobot Python is missing" in result.stderr
    assert "retrying" not in result.stderr


def test_installer_never_uses_effective_root_as_implicit_service_user():
    helper = (ROOT / "scripts" / "service-install-common.sh").read_text(encoding="utf-8")
    assert "SUDO_USER" in helper
    assert "running as root requires --service-user USER" in helper
    assert "refusing to install robot services as root" in helper


def test_full_installer_includes_qualification_tooling_dependencies():
    """A fresh deployment must not silently omit required qualification checks."""
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")
    lines = installer.splitlines()

    for package in ("shellcheck", "python3-zmq"):
        assert package in installer
    universe_line = next(index for index, line in enumerate(lines) if "add-apt-repository -y universe" in line)
    shellcheck_line = next(index for index, line in enumerate(lines) if line.strip().startswith("shellcheck "))
    assert universe_line < shellcheck_line
