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
LEKIWI_HOST_BIND_ADDRESS=0.0.0.0
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
    assert "LEKIWI_BIND_ADDRESS=0.0.0.0" in host
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


def test_installer_reapplies_the_pinned_free_fleet_patch_on_rerun():
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")
    patch = ROOT / "thirdparty" / "free_fleet" / "0001-retry-nav2-goal-during-activation.patch"

    assert patch.is_file()
    assert 'apply_pinned_patch "$free_fleet_source" "$free_fleet_patch"' in installer
    assert '"$free_fleet_source" "$FREE_FLEET_REV" "$free_fleet_patch"' in installer


def test_simulation_installer_excludes_astra_hardware_setup():
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")

    assert 'if [[ $install_mode == full ]]; then\n  log "Fetching the pinned Orbbec Astra Pro ROS 2 driver"' in installer
    assert 'extra_source_paths+=("$astra_source")' in installer
    assert 'extra_packages+=(astra_camera astra_camera_msgs)' in installer
    assert 'Simulation-only installation: skipping Astra driver and udev setup' in installer


def test_service_installers_support_an_unauthenticated_split_zmq_transport():
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")

    assert 'if [[ -n $CURVE_DIR_ARG ]]; then' in device
    assert 'STACK_ARGS="camera_source:=remote remote_ip:=$REMOTE"' in compute


def test_device_installer_finds_ros_packages_under_sudo_root_path():
    installer = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")

    assert "source /opt/ros/jazzy/setup.bash" in installer
    assert 'source "$LEKIWI_SERVICE_WORKSPACE/install/setup.bash"' in installer


def test_deploy_order_fails_closed_around_the_device_restart():
    deploy = (ROOT / "scripts" / "deploy-split.sh").read_text(encoding="utf-8")

    disarm = deploy.index("\ndisarm\n")
    stop_stack = deploy.index("stop lekiwi-stack.service", disarm)
    stop_host = deploy.index("stop lekiwi-host.service", stop_stack)
    start_host = deploy.index("start lekiwi-host.service", stop_host)
    start_stack = deploy.index("start lekiwi-stack.service", start_host)
    assert disarm < stop_stack < stop_host < start_host < start_stack
    assert "git merge --ff-only" in deploy
    assert "deploy-inhibit-auto-arm" in deploy
    assert "reset --hard" not in deploy


def test_deploy_sudoers_are_limited_by_machine_role():
    script = ROOT / "scripts" / "install-deploy-sudoers.sh"
    compute = subprocess.run(
        ["bash", str(script), "compute", "--user", "nobody", "--print"],
        check=True, text=True, capture_output=True,
    ).stdout
    device = subprocess.run(
        ["bash", str(script), "device", "--user", "nobody", "--print"],
        check=True, text=True, capture_output=True,
    ).stdout

    assert "lekiwi-stack.service" in compute
    assert "lekiwi-host.service" not in compute
    assert "lekiwi-host.service" in device
    assert "lekiwi-cameras.service" in device
    assert "lekiwi-stack.service" not in device
    assert "NOPASSWD" in compute and "NOPASSWD" in device
    assert "daemon-reload" not in compute + device


def test_managed_build_prefers_system_cmake_and_starts_clean():
    builder = (ROOT / "scripts" / "build-lekiwi.sh").read_text(encoding="utf-8")

    assert "PATH=/usr/bin:/bin:$PATH" in builder
    assert '-DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local"' in builder
    assert 'rm -rf -- "$workspace/build/lekiwi_rmf"' in builder
