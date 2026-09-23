"""No-root behavioral checks for systemd template rendering and host startup."""

from __future__ import annotations

import getpass
import hashlib
import os
import pathlib
import re
import shutil
import stat
import subprocess
import time

import pytest


ROOT = pathlib.Path(__file__).parents[1]


def test_runtime_helpers_share_device_and_calibration_checks(tmp_path):
    calibration = tmp_path / "camera.yaml"
    calibration.write_text("image_width: 640\ncamera_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n")
    script = r'''
set -Eeuo pipefail
source "$1/scripts/lib/runtime-common.sh"
[[ $(first_match "$2") == "$2" ]]
camera_calibration_valid "$3"
wait_for 1 test -s "$3"
'''
    subprocess.run(
        ["bash", "-c", script, "runtime-test", str(ROOT), str(ROOT / "README.md"), str(calibration)],
        check=True,
    )


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
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
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


def test_headless_host_refuses_to_answer_the_calibration_prompt_without_a_calibration(tmp_path):
    venv = tmp_path / "venv"
    calibration = tmp_path / "home/.cache/huggingface/lerobot/calibration/robots/lekiwi/lekiwi_1.json"
    stdin_log = tmp_path / "host-stdin"
    # The fake host records what it was piped, then loses the calibration and exits
    # like a dropped motor bus: the next attempt must stop instead of prompting.
    _executable(venv / "bin" / "python", f'cat > "{stdin_log}"\nrm -f "{calibration}"\nexit 1\n')
    _executable(venv / "bin" / "lerobot-calibrate", "exit 1\n")
    fakes = tmp_path / "bin"
    _executable(fakes / "fuser", "exit 1\n")
    _executable(fakes / "sleep", "exit 0\n")
    port = tmp_path / "ttyACM0"
    port.write_text("", encoding="utf-8")
    environment = {k: v for k, v in os.environ.items() if not k.startswith(("HF_", "LEKIWI_"))}
    environment.update(
        HOME=str(tmp_path / "home"), LEKIWI_LEROBOT_VENV=str(venv), LEKIWI_PORT=str(port),
        PATH=f"{fakes}:{os.environ['PATH']}",
    )

    def run():
        return subprocess.run(
            ["bash", str(ROOT / "scripts" / "robot-host.sh"), "--no-cameras"],
            cwd=ROOT, env=environment, text=True, capture_output=True, timeout=10,
        )

    missing = run()
    assert missing.returncode == 78
    assert "motor calibration is missing" in missing.stderr
    assert not stdin_log.exists()

    calibration.parent.mkdir(parents=True)
    calibration.write_text("{}", encoding="utf-8")
    lost = run()
    assert lost.returncode == 78
    assert stdin_log.read_text(encoding="utf-8") == "\n"
    assert "retrying the motor-bus connection" in lost.stderr

    unit = (ROOT / "systemd" / "lekiwi-host.service").read_text(encoding="utf-8")
    assert "RestartPreventExitStatus=78" in unit


def test_installer_never_uses_effective_root_as_implicit_service_user():
    helper = (ROOT / "scripts" / "lib" / "service-install-common.sh").read_text(encoding="utf-8")
    assert "SUDO_USER" in helper
    assert "running as root requires --service-user USER" in helper
    assert "refusing to install robot services as root" in helper


def test_unit_validation_ignores_unrelated_systemd_units():
    helper = (ROOT / "scripts" / "lib" / "service-install-common.sh").read_text(encoding="utf-8")

    assert 'systemd-analyze verify --recursive-errors=no "$UNIT_DIR/$unit"' in helper


def test_startup_disarm_is_tracked_in_the_launch_default():
    launch = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert '"auto_arm_on_startup", default_value="false"' in launch


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
    # One implementation of the pinned checkout, shared with install-pi.sh.
    assert "checkout_pinned() {" not in installer
    assert "reset --hard" not in installer


def test_downloads_are_pinned_and_rejected_on_a_checksum_mismatch(tmp_path):
    payload = tmp_path / "upstream.deb"
    payload.write_bytes(b"package")
    fakes = tmp_path / "bin"
    _executable(fakes / "curl", f'cp "{payload}" "$3"\n')  # curl -fL -o DEST URL
    script = r'''
set -Eeuo pipefail
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$1/scripts/thirdparty-common.sh"
download_verified https://example.invalid/a.deb "$2" "$3"
'''
    good = hashlib.sha256(b"package").hexdigest()

    def download(digest):
        return subprocess.run(
            ["bash", "-c", script, "download", str(ROOT), digest, str(tmp_path / "out.deb")],
            env={**os.environ, "PATH": f"{fakes}:{os.environ['PATH']}"}, capture_output=True, text=True,
        )

    assert download(good).returncode == 0
    assert (tmp_path / "out.deb").read_bytes() == b"package"
    rejected = download("0" * 64)
    assert rejected.returncode != 0 and "checksum mismatch" in rejected.stderr
    assert not (tmp_path / "out.deb").exists()

    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")
    assert "api.github.com" not in installer and "/latest/" not in installer
    assert installer.count("download_verified") == 2
    for package in ("nudged", "pycdr2", "rosbags"):
        assert re.search(rf"\b{package}==[0-9.]+", installer), package


@pytest.mark.skipif(shutil.which("git") is None, reason="git is required")
def test_pinned_checkout_reverses_only_its_patch_and_keeps_other_local_edits(tmp_path):
    script = r'''
set -Eeuo pipefail
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$1/scripts/thirdparty-common.sh"
git() { command git -c user.name=test -c user.email=test@example.invalid "$@"; }
cd "$2"
git init -q upstream
printf 'one\n' > upstream/a.txt
printf 'two\n' > upstream/b.txt
git -C upstream add .
git -C upstream commit -qm base
printf 'patched\n' > upstream/a.txt
git -C upstream diff > fix.patch
git -C upstream checkout -q a.txt
revision=$(git -C upstream rev-parse HEAD)

checkout_pinned "$PWD/upstream" "$PWD/dest" "$revision" "$PWD/fix.patch" >/dev/null 2>&1
apply_pinned_patch dest "$PWD/fix.patch" "the test patch"
# A rerun over the patched tree, with an unrelated local edit beside it.
printf 'mine\n' > dest/b.txt
checkout_pinned "$PWD/upstream" "$PWD/dest" "$revision" "$PWD/fix.patch" >/dev/null 2>&1
apply_pinned_patch dest "$PWD/fix.patch" "the test patch"
[[ $(cat dest/a.txt) == patched ]]
[[ $(cat dest/b.txt) == mine ]]

# Without the known patch, local changes are refused rather than discarded.
if (checkout_pinned "$PWD/upstream" "$PWD/dest" "$revision" >/dev/null 2>&1); then exit 1; fi
[[ $(cat dest/b.txt) == mine ]]
'''
    subprocess.run(["bash", "-c", script, "pinned-checkout", str(ROOT), str(tmp_path)], check=True)


def test_simulation_installer_excludes_astra_hardware_setup():
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")

    assert 'if [[ $install_mode == full ]]; then\n  log "Fetching the pinned Orbbec Astra Pro ROS 2 driver"' in installer
    assert 'extra_source_paths+=("$astra_source")' in installer
    assert 'extra_packages+=(astra_camera astra_camera_msgs)' in installer
    assert 'Simulation-only installation: skipping Astra driver and udev setup' in installer


def test_split_compute_installs_and_starts_moveit_by_default():
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")
    reinstall = (ROOT / "scripts" / "reinstall-compute.sh").read_text(encoding="utf-8")
    workstation = (ROOT / "scripts" / "workstation-up.sh").read_text(encoding="utf-8")

    assert '"ros-$ROS_DISTRO-moveit"' in installer
    assert "start_moveit:=true" in compute
    assert "load_lekiwi_env" in reinstall
    assert "install-compute-services.sh" in reinstall
    assert '--remote "$remote"' in reinstall
    # The installer restarts a changed stack; the wrapper must not restart it again.
    assert "systemctl restart" not in compute
    assert "systemctl restart" not in reinstall
    assert "start_moveit:=true" in workstation


def test_service_installers_support_an_unauthenticated_split_zmq_transport():
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")

    assert 'if [[ -n $CURVE_DIR_ARG ]]; then' in device
    assert 'STACK_ARGS="camera_source:=remote remote_ip:=$REMOTE laser_source:=ld06 lidar_source:=remote start_moveit:=true"' in compute
    assert compute.count("--curve-dir does not contain") == 1


def test_remote_stack_has_no_local_host_dependency():
    stack = (ROOT / "systemd" / "lekiwi-stack.service").read_text(encoding="utf-8")
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")

    assert "Requires=lekiwi-host.service" not in stack
    assert "PartOf=lekiwi-host.service" not in stack
    assert "After=network-online.target" in stack
    assert '"Requires=lekiwi-host.service"' in compute
    assert '"PartOf=lekiwi-host.service"' in compute


def test_standard_installers_start_and_relay_the_host_lidar_without_an_opt_in():
    installer = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")

    assert "--remote-lidar" not in installer
    assert "units=(lekiwi-host.service lekiwi-lidar.service)" in device
    assert 'as_root systemctl enable --now "${units[@]}"' in device
    assert "ldlidar_stl_ros2 is unavailable; the standard device installation requires the LD06 driver" in device
    assert 'install-deploy-sudoers.sh" device --user "$LEKIWI_SERVICE_USER"' in device
    assert 'install-deploy-sudoers.sh" compute --user "$LEKIWI_SERVICE_USER"' in installer
    assert "record_service_fingerprint device" in device
    assert "record_service_fingerprint compute" in installer


def test_device_installer_skips_the_zenoh_service_without_its_binary():
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    pi_installer = (ROOT / "scripts" / "install-pi.sh").read_text(encoding="utf-8")
    common = (ROOT / "scripts" / "thirdparty-common.sh").read_text(encoding="utf-8")
    workstation_installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")

    assert 'command -v zenoh-bridge-ros2dds' in device
    assert "skipping lekiwi-zenoh.service" in device
    assert "[[ $zenoh_available == true ]] && units+=(lekiwi-zenoh.service)" in device
    # The README-prescribed device installer provides the binary the unit runs.
    assert 'install_zenoh_bridge "$HOME/.local/bin"' in pi_installer
    assert 'install_zenoh_bridge "$HOME/.local/bin"' in workstation_installer
    assert "install_zenoh_bridge() {" in common


def test_installed_units_that_change_are_queued_for_restart(tmp_path):
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    script = r'''
set -Eeuo pipefail
PROJECT_ROOT=$1
UNIT_DIR=$2
LEKIWI_SERVICE_USER=robot
LEKIWI_SERVICE_HOME=/srv/robot
LEKIWI_SERVICE_WORKSPACE=/srv/robot/lekiwi_ws
LEKIWI_SERVICE_LEROBOT_VENV=/srv/robot/lerobot-venv
as_root() { "$@"; }
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"

LEKIWI_HOST_BIND_ADDRESS=0.0.0.0
install_unit lekiwi-host.service
[[ ${#CHANGED_UNITS[@]} -eq 0 ]]   # first installation: enable --now starts it
install_unit lekiwi-host.service
[[ ${#CHANGED_UNITS[@]} -eq 0 ]]   # identical rerun: nothing to restart
LEKIWI_HOST_BIND_ADDRESS=10.0.0.5
install_unit lekiwi-host.service
[[ ${CHANGED_UNITS[*]} == lekiwi-host.service ]]
'''
    subprocess.run(["bash", "-c", script, "unit-change", str(ROOT), str(tmp_path)], check=True)
    assert "restart_changed_units" in device


def test_compute_stack_restarts_only_when_its_configuration_changes(tmp_path):
    """Replays the compute installer's configuration and start sequence against a fake systemctl."""
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")
    assert 'install_unit_config "$stack_env" lekiwi-stack.service' in compute
    assert 'install_unit_config "$topology_conf" lekiwi-stack.service' in compute
    assert 'remove_unit_config "$topology_conf" lekiwi-stack.service' in compute
    assert compute.index("restart_changed_units") < compute.index("enable --now lekiwi-stack.service")

    calls = tmp_path / "systemctl.log"
    fakes = tmp_path / "bin"
    _executable(fakes / "systemctl", f'echo "$*" >> "{calls}"\n')
    script = r'''
set -Eeuo pipefail
PROJECT_ROOT=$1
etc=$2
as_root() { "$@"; }
log() { :; }
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
install() { # install <launch arguments> [drop-in]
  CHANGED_UNITS=()
  install_unit_config "$etc/lekiwi-stack" lekiwi-stack.service "LEKIWI_STACK_ARGS=$1"
  if [[ -n ${2:-} ]]; then
    install_unit_config "$etc/topology.conf" lekiwi-stack.service "[Unit]"
  else
    remove_unit_config "$etc/topology.conf" lekiwi-stack.service
  fi
  restart_changed_units
  echo "--"  >> "$etc/../systemctl.log"
}
install "remote_ip:=10.0.0.2"             # first installation
install "remote_ip:=10.0.0.2"             # identical rerun
install "remote_ip:=10.0.0.3"             # new device address
install "remote_ip:=10.0.0.3" local       # topology drop-in added
install "remote_ip:=10.0.0.3" local       # identical rerun
install "remote_ip:=10.0.0.3"             # drop-in removed
'''
    (tmp_path / "etc").mkdir()
    subprocess.run(
        ["bash", "-c", script, "compute-restart", str(ROOT), str(tmp_path / "etc")],
        env={**os.environ, "PATH": f"{fakes}:{os.environ['PATH']}"}, check=True,
    )
    runs = calls.read_text(encoding="utf-8").split("--\n")[:-1]
    restart = "try-restart lekiwi-stack.service\n"
    assert runs == [restart, "", restart, restart, "", restart]


def test_sensor_services_keep_retrying_after_intermittent_usb_resets():
    for name in ("lekiwi-astra.service", "lekiwi-cameras.service", "lekiwi-lidar.service", "lekiwi-zenoh.service"):
        unit = (ROOT / "systemd" / name).read_text(encoding="utf-8")
        assert "StartLimitIntervalSec=0" in unit
        assert "Restart=always" in unit


def test_pi_and_manual_split_startup_include_the_ld06():
    pi_installer = (ROOT / "scripts" / "install-pi.sh").read_text(encoding="utf-8")
    pi_up = (ROOT / "scripts" / "pi-up.sh").read_text(encoding="utf-8")
    workstation_up = (ROOT / "scripts" / "workstation-up.sh").read_text(encoding="utf-8")
    lidar = (ROOT / "scripts" / "ros-lidar.sh").read_text(encoding="utf-8")

    assert "Installing the pinned LD06 ROS driver" in pi_installer
    assert "ldlidar_stl_ros2_node" in pi_installer
    assert "start_recorded lidar scripts/ros-lidar.sh" in pi_up
    assert "start_recorded astra scripts/ros-astra.sh" in pi_up
    assert "start_recorded zenoh scripts/ros-zenoh.sh" in pi_up
    assert "laser_source:=ld06 lidar_source:=remote" in workstation_up
    assert "start_moveit:=true" in workstation_up
    assert "waiting for LD06 serial port" in lidar


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
    # A stale compute configuration is reinstalled only once the robot is disarmed and
    # its stack stopped, never started early, and its sudo need is checked up front.
    refresh = deploy.index("\n  refresh_compute_service\n")
    assert stop_stack < refresh < stop_host
    assert deploy.count("refresh_compute_service\n") == 1
    assert '"$project_root/scripts/reinstall-compute.sh" --no-start' in deploy
    assert deploy.index("sudo -n true") < disarm
    assert "lekiwi-lidar.service" in deploy
    # The zenoh bridge is required and preflighted before anything is stopped;
    # Astra and the cameras are skipped by the device installer without their ROS packages.
    assert "device_units=(lekiwi-host.service lekiwi-lidar.service lekiwi-zenoh.service)" in deploy
    assert "for unit in lekiwi-lidar.service lekiwi-zenoh.service; do" in deploy
    assert 'if remote_unit_exists "$unit"; then device_units+=("$unit"); fi' in deploy
    assert ".lekiwi-source-revision" in deploy
    assert "expected_service_fingerprint" in deploy
    assert "canonical /scan is not the LD06 frame" in deploy
    assert 'awk \'NF && $1 != "---" { print $1; exit }\'' in deploy
    assert "has_nopasswd_systemctl" in deploy
    assert 'compute_sudoers=$(sudo -n -l)' in deploy
    assert 'git -C "$project_root" merge --ff-only' in deploy
    # The deployer runs from any directory: every git call names its repository.
    assert not re.search(r"(?<![\w-])git (?!-C )", deploy)
    assert "cannot fetch origin within 30 seconds" in deploy
    assert "LEKIWI_ROBOT_HOST" in deploy
    assert "load_lekiwi_env" in deploy
    assert "Refreshing stale compute service configuration" in deploy
    assert "reinstall-compute.sh" in deploy
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
    assert "lekiwi-astra.service" in device
    assert "lekiwi-cameras.service" in device
    assert "lekiwi-lidar.service" in device
    assert "lekiwi-zenoh.service" in device
    assert "lekiwi-stack.service" not in device
    assert "NOPASSWD" in compute and "NOPASSWD" in device
    assert "daemon-reload" not in compute + device


def test_managed_build_prefers_system_cmake_and_starts_clean():
    builder = (ROOT / "scripts" / "build-lekiwi.sh").read_text(encoding="utf-8")

    assert "PATH=/usr/bin:/bin:$PATH" in builder
    assert '-DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local"' in builder
    assert 'rm -rf -- "$workspace/build/lekiwi_rmf"' in builder
    assert '.lekiwi-source-revision' in builder


def test_service_fingerprint_covers_installed_service_behavior():
    revision = (ROOT / "scripts" / "lib" / "service-install-revision.sh").read_text(encoding="utf-8")

    for source in (
        "systemd/lekiwi-stack.service",
        "systemd/lekiwi-astra.service",
        "scripts/ros-astra.sh",
        "systemd/lekiwi-lidar.service",
        "scripts/ros-lidar.sh",
        "scripts/lib/service-install-common.sh",
        "scripts/lib/runtime-common.sh",
        "scripts/install-deploy-sudoers.sh",
        "scripts/install-device-network.sh",
    ):
        assert source in revision


def _network_checkout(tmp_path: pathlib.Path, env_file: str | None = None) -> pathlib.Path:
    """A copy of the network installers, so the developer's own .env cannot leak in."""
    checkout = tmp_path / "checkout"
    for name in ("install-device-network.sh", "install-wifi-regdom.sh", "lib/runtime-common.sh"):
        target = checkout / "scripts" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "scripts" / name, target)
    if env_file is not None:
        (checkout / ".env").write_text(env_file, encoding="utf-8")
    return checkout


def _network_path(tmp_path: pathlib.Path, *, nmcli: bool) -> tuple[str, pathlib.Path]:
    """A PATH of only what the installers need, with iw and nmcli replaced by fakes."""
    tools = tmp_path / "tools"
    tools.mkdir(exist_ok=True)
    for command in ("bash", "cat", "dirname", "getent", "grep", "id", "install"):
        link = tools / command
        if not link.exists():
            link.symlink_to(shutil.which(command))
    iw_log = tmp_path / "iw.log"
    _executable(tools / "iw", f'echo "$*" >> "{iw_log}"\n')
    if nmcli:
        _executable(tools / "nmcli", "exit 0\n")
    elif (tools / "nmcli").exists():
        (tools / "nmcli").unlink()
    return str(tools), iw_log


def _network_install(tmp_path, checkout, *args, nmcli=True, user=None):
    path, _ = _network_path(tmp_path, nmcli=nmcli)
    environment = {k: v for k, v in os.environ.items() if k != "LEKIWI_WIFI_COUNTRY"}
    environment.update(PATH=path, LEKIWI_NETWORK_ROOT=str(tmp_path / "root"))
    return subprocess.run(
        [str(checkout / "scripts" / "install-device-network.sh"), "--user", user or getpass.getuser(), *args],
        env=environment, capture_output=True, text=True,
    )


@pytest.mark.skipif(getpass.getuser() == "root", reason="the installer refuses to grant network control to root")
def test_wifi_country_follows_the_configuration_on_every_rerun(tmp_path):
    regdom = tmp_path / "root/etc/modprobe.d/lekiwi-cfg80211-regdom.conf"
    iw_log = tmp_path / "iw.log"

    default = _network_checkout(tmp_path / "default")
    assert _network_install(tmp_path, default).returncode == 0
    assert "options cfg80211 ieee80211_regdom=ID" in regdom.read_text()
    assert iw_log.read_text().splitlines() == ["reg set ID"]

    configured = _network_checkout(tmp_path / "configured", "LEKIWI_WIFI_COUNTRY=MX\n")
    for _ in range(2):  # a rerun without --country keeps the configured country
        result = _network_install(tmp_path, configured)
        assert result.returncode == 0, result.stderr
        assert "ieee80211_regdom=MX" in regdom.read_text()
    assert iw_log.read_text().splitlines()[-2:] == ["reg set MX", "reg set MX"]

    assert _network_install(tmp_path, configured, "--country", "DE").returncode == 0
    assert "ieee80211_regdom=DE" in regdom.read_text()
    rejected = _network_install(tmp_path, configured, "--country", "idn")
    assert rejected.returncode != 0
    assert "ieee80211_regdom=DE" in regdom.read_text()

    invalid = _network_checkout(tmp_path / "invalid", "LEKIWI_WIFI_COUNTRY=id\n")
    assert _network_install(tmp_path, invalid).returncode != 0
    assert "ieee80211_regdom=DE" in regdom.read_text()

    # The installers hand the configured country on explicitly.
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    workstation = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")
    assert '--country "${LEKIWI_WIFI_COUNTRY:-ID}"' in device
    assert 'install-wifi-regdom.sh" "${LEKIWI_WIFI_COUNTRY:-ID}"' in workstation
    assert device.count('load_lekiwi_env "$PROJECT_ROOT/.env"') == 1
    assert workstation.count('load_lekiwi_env "$PROJECT_ROOT/.env"') == 1


@pytest.mark.skipif(getpass.getuser() == "root", reason="the installer refuses to grant network control to root")
def test_wifi_country_is_set_without_network_manager_and_boot_overrides_are_reported(tmp_path):
    checkout = _network_checkout(tmp_path)
    cmdline = tmp_path / "root/proc/cmdline"
    cmdline.parent.mkdir(parents=True)
    cmdline.write_text("console=tty1 cfg80211.ieee80211_regdom=GB rootwait\n")

    result = _network_install(tmp_path, checkout, nmcli=False)

    assert result.returncode == 0, result.stderr
    assert "ieee80211_regdom=ID" in (tmp_path / "root/etc/modprobe.d/lekiwi-cfg80211-regdom.conf").read_text()
    assert (tmp_path / "iw.log").read_text().splitlines() == ["reg set ID"]
    assert "cfg80211.ieee80211_regdom=GB" in result.stderr
    assert not (tmp_path / "root/etc/NetworkManager").exists()
    assert not (tmp_path / "root/etc/polkit-1").exists()


@pytest.mark.skipif(getpass.getuser() == "root", reason="the installer refuses to grant network control to root")
def test_device_network_installer_disables_power_saving_and_scopes_polkit(tmp_path):
    user = getpass.getuser()
    checkout = _network_checkout(tmp_path)
    for _ in range(2):  # re-running is idempotent
        result = _network_install(tmp_path, checkout)
        assert result.returncode == 0, result.stderr

    powersave = (tmp_path / "root/etc/NetworkManager/conf.d/zz-lekiwi-wifi-powersave-off.conf").read_text()
    assert "[connection]" in powersave and "wifi.powersave = 2" in powersave

    rule = (tmp_path / "root/etc/polkit-1/rules.d/50-lekiwi-networkmanager.rules").read_text()
    assert f'subject.user == "{user}"' in rule
    assert "org.freedesktop.NetworkManager.network-control" in rule
    assert "org.freedesktop.NetworkManager.settings.modify.system" in rule
    # Only the three named actions: no wildcard match on the NetworkManager namespace.
    assert "indexOf" in rule and "startsWith" not in rule

    assert _network_install(tmp_path, checkout, user="root").returncode != 0


def test_torque_on_failure_key_is_validated_and_reaches_both_machines(tmp_path):
    env_file = tmp_path / ".env"

    def load(value):
        env_file.write_text(f"LEKIWI_DISARM_ON_FAILURE={value}\n")
        return subprocess.run(
            ["bash", "-c", 'source "$1/scripts/lib/runtime-common.sh"; load_lekiwi_env "$2" && echo "${LEKIWI_DISARM_ON_FAILURE:-unset}"',
             "test", str(ROOT), str(env_file)],
            env={k: v for k, v in os.environ.items() if k != "LEKIWI_DISARM_ON_FAILURE"},
            capture_output=True, text=True,
        )

    assert load("true").stdout.strip() == "true"
    assert load("false").stdout.strip() == "false"
    assert load("yes").returncode != 0

    # Both sides must pass the same opt-in on, and default to holding torque.
    stack = (ROOT / "scripts" / "ros-start.sh").read_text(encoding="utf-8")
    host = (ROOT / "scripts" / "robot-host.sh").read_text(encoding="utf-8")
    assert "disarm_on_failure:=true" in stack
    assert '--safety.disarm_on_failure="${LEKIWI_DISARM_ON_FAILURE:-false}"' in host


def _executable(path: pathlib.Path, body: str) -> pathlib.Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def test_port_probes_match_the_exact_local_port(tmp_path):
    listing = {
        "only-lookalikes": "LISTEN 0 4096 0.0.0.0:55550 0.0.0.0:*\nLISTEN 0 4096 [::]:15557 [::]:*\n",
        "motion-only": "LISTEN 0 4096 0.0.0.0:5555 0.0.0.0:*\n",
        "both": "LISTEN 0 4096 0.0.0.0:5555 0.0.0.0:*\nLISTEN 0 4096 [::]:5557 [::]:*\n",
    }

    def probe(name: str, function: str) -> int:
        fake = tmp_path / name / "ss"
        _executable(fake, f"cat <<'OUT'\nState Recv-Q Send-Q Local Peer\n{listing[name]}OUT\n")
        return subprocess.run(
            ["bash", "-c", f'source "{ROOT}/scripts/lib/runtime-common.sh"; {function}'],
            env={**os.environ, "PATH": f"{fake.parent}:{os.environ['PATH']}"},
        ).returncode

    assert probe("only-lookalikes", "lekiwi_motion_port_listening") != 0
    assert probe("motion-only", "lekiwi_motion_port_listening") == 0
    assert probe("motion-only", "lekiwi_safety_ports_listening") != 0
    assert probe("both", "lekiwi_safety_ports_listening") == 0


def test_log_pruning_removes_only_stale_files_under_the_ros_log_directory(tmp_path):
    unit = (ROOT / "systemd" / "lekiwi-ros-logrotate.service").read_text(encoding="utf-8")
    # Pruning runs after logrotate even when logrotate fails, and logrotate gets its state directory.
    assert "ExecStopPost=-@PROJECT_ROOT@/scripts/prune-ros-logs.sh @SERVICE_HOME@/.ros/log" in unit
    assert "ExecStartPre=/usr/bin/mkdir -p @SERVICE_HOME@/.ros/lekiwi" in unit
    assert "--state @SERVICE_HOME@/.ros/lekiwi/" in unit
    command = [str(ROOT / "scripts" / "prune-ros-logs.sh"), str(tmp_path / ".ros" / "log")]

    log = tmp_path / ".ros" / "log"
    outside = tmp_path / "elsewhere"
    outside.mkdir()
    old = time.time() - 6 * 86400
    recent = time.time() - 3600
    files = {
        "loose_old.log": old,
        "loose_new.log": recent,
        "2026-01-01/launch.log.1": old,
        "2026-01-01/launch.log": old,
        "2026-01-02/launch.log": recent,
        "2026-01-02/rotated.log.1": old,
        "2026-01-02/held_open.log": old,
        # A first run after a long time has thousands of stale files to test.
        **{f"2026-01-01/backlog_{n}.log": old for n in range(500)},
    }
    for name, mtime in files.items():
        path = log / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("x", encoding="utf-8")
        os.utime(path, (mtime, mtime))
    (log / "2026-01-03").mkdir()
    os.utime(log / "2026-01-03", (old, old))
    (log / "latest").symlink_to("2026-01-02")
    (outside / "keep.log").write_text("x", encoding="utf-8")
    os.utime(outside / "keep.log", (old, old))
    (log / "linked").symlink_to(outside)

    # A quiet node's open log is stale by mtime but must survive the prune.
    with (log / "2026-01-02" / "held_open.log").open():
        subprocess.run(command, check=True)
        # Emptying a directory refreshes its mtime; a later run removes it once it has stayed empty.
        os.utime(log / "2026-01-01", (old, old))
        subprocess.run(command, check=True)

    remaining = {str(p.relative_to(log)) for p in log.rglob("*") if not p.is_dir() or p.is_symlink()}
    assert remaining == {
        "loose_new.log", "2026-01-02/launch.log", "2026-01-02/held_open.log", "latest", "linked",
    }
    assert not (log / "2026-01-01").exists()
    assert not (log / "2026-01-03").exists()
    assert (outside / "keep.log").exists()


@pytest.mark.skipif(not os.access("/usr/sbin/logrotate", os.X_OK), reason="logrotate is required")
def test_log_rotation_install_creates_the_state_directory_for_the_service_user(tmp_path):
    user = getpass.getuser()
    group = subprocess.run(["id", "-gn", user], check=True, capture_output=True, text=True).stdout.strip()
    calls = tmp_path / "as_root.log"
    script = r'''
set -Eeuo pipefail
PROJECT_ROOT=$1
UNIT_DIR=$2/units
LEKIWI_SERVICE_USER=$3
LEKIWI_SERVICE_HOME=$2/home
LEKIWI_SERVICE_WORKSPACE=$2/home/lekiwi_ws
LEKIWI_SERVICE_LEROBOT_VENV=
calls=$4
as_root() { printf '%s\n' "$*" >> "$calls"; [[ $1 != tee ]] || cat >/dev/null; }
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
install_log_rotation
'''
    subprocess.run(["bash", "-c", script, "log-rotation", str(ROOT), str(tmp_path), user, str(calls)], check=True)
    commands = calls.read_text(encoding="utf-8").splitlines()
    assert f"install -d -o {user} -g {group} -m 0755 {tmp_path}/home/.ros/lekiwi" in commands
    for installer in ("install-compute-services.sh", "install-device-services.sh"):
        text = (ROOT / "scripts" / installer).read_text(encoding="utf-8")
        assert text.index("install_log_rotation") < text.index("systemctl daemon-reload")


def test_rotation_config_compresses_rotated_launch_logs_at_once():
    config = (ROOT / "systemd" / "lekiwi-ros-logrotate.conf").read_text(encoding="utf-8")

    assert "20*/*.log" in config
    assert "\n    compress\n" in config
    assert "delaycompress" not in config
    # Both installers get the timer's binary from the shared helper, which refuses to install without it.
    helper = (ROOT / "scripts" / "lib" / "service-install-common.sh").read_text(encoding="utf-8")
    assert "[[ -x /usr/sbin/logrotate ]]" in helper


def test_stack_survives_repeated_self_heal_kills_and_host_gets_time_to_pass_its_gate():
    stack = (ROOT / "systemd" / "lekiwi-stack.service").read_text(encoding="utf-8")
    host = (ROOT / "systemd" / "lekiwi-host.service").read_text(encoding="utf-8")

    assert "StartLimitIntervalSec=0" in stack
    assert "StartLimitBurst" not in stack
    # The ExecStartPost gate probes up to 120 times, one second each plus a one second sleep.
    timeout = int(re.search(r"^TimeoutStartSec=(\d+)s$", host, re.MULTILINE).group(1))
    assert timeout > 2 * 120


def test_long_running_startup_children_do_not_inherit_the_start_lock():
    for name in ("up.sh", "pi-up.sh", "workstation-up.sh"):
        script = (ROOT / "scripts" / name).read_text(encoding="utf-8")
        launches = [line for line in script.splitlines() if "setsid" in line and "&" in line]
        assert launches, name
        assert all("9>&-" in line for line in launches), name
        assert "flock -n 9" in script


def test_ros_stop_leaves_units_owned_by_systemd_alone(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    fake_systemctl = tmp_path / "bin" / "systemctl"
    _executable(fake_systemctl, 'exit 0\n')  # every unit reports active
    sentinel = subprocess.Popen(["sleep", "60"], start_new_session=True)
    try:
        kinds = ("stack", "host", "astra", "cameras", "lidar", "zenoh")
        for kind in kinds:
            (runtime / f"{kind}.pid").write_text(f"{sentinel.pid}\n", encoding="utf-8")
        result = subprocess.run(
            ["bash", str(ROOT / "scripts" / "ros-stop.sh")],
            env={**os.environ, "LEKIWI_RUNTIME_DIR": str(runtime),
                 "PATH": f"{fake_systemctl.parent}:{os.environ['PATH']}"},
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, result.stderr
        for unit in ("stack", "host", "astra", "cameras", "lidar", "zenoh"):
            assert f"lekiwi-{unit}.service is active -- left running" in result.stdout
        assert sentinel.poll() is None
        assert all((runtime / f"{kind}.pid").exists() for kind in kinds)
    finally:
        sentinel.kill()
        sentinel.wait()


def test_sync_calibration_uses_the_configured_robot_and_gives_up(tmp_path):
    scripts = tmp_path / "scripts"
    for name in ("sync-calibration.sh", "lib/runtime-common.sh"):
        target = scripts / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text((ROOT / "scripts" / name).read_text(encoding="utf-8"), encoding="utf-8")
    fakes = tmp_path / "bin"
    calls = tmp_path / "rsync-calls"
    _executable(fakes / "rsync", f'printf "%s\\n" "$*" >> "{calls}"\nexit 1\n')
    _executable(fakes / "sleep", "exit 0\n")
    environment = {k: v for k, v in os.environ.items() if k != "LEKIWI_ROBOT_HOST"}
    environment.update(HOME=str(tmp_path / "home"), PATH=f"{fakes}:{os.environ['PATH']}")

    def run(**extra):
        return subprocess.run(
            ["bash", str(scripts / "sync-calibration.sh")],
            env={**environment, **extra}, capture_output=True, text=True, timeout=30,
        )

    assert run().returncode == 2
    assert not calls.exists()

    result = run(LEKIWI_ROBOT_HOST="robot.example")
    assert result.returncode == 1
    lines = calls.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 5 * 5  # five files, five attempts
    assert all(" robot.example:" in line for line in lines)


def test_calibration_starts_again_exactly_the_services_it_stopped(tmp_path):
    scripts = tmp_path / "scripts"
    for name in ("calibrate.sh", "lib/runtime-common.sh", "lib/service-install-common.sh"):
        target = scripts / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text((ROOT / "scripts" / name).read_text(encoding="utf-8"), encoding="utf-8")
    (scripts / "setup.bash").write_text("", encoding="utf-8")
    fakes = tmp_path / "bin"
    log = tmp_path / "systemctl.log"
    _executable(fakes / "systemctl", f'''
if [[ $1 == is-active ]]; then
  [[ $3 == lekiwi-host.service ]]   # only the motor host service is running
  exit
fi
echo "$*" >> "{log}"
''')
    _executable(fakes / "sudo", 'exec "$@"\n')
    _executable(fakes / "sleep", "exit 0\n")
    _executable(fakes / "pgrep", "exit 1\n")
    _executable(fakes / "fuser", "exit 1\n")
    home = tmp_path / "home"
    motor_file = home / ".cache/huggingface/lerobot/calibration/robots/lekiwi/lekiwi_1.json"

    def run(host_script: str):
        _executable(scripts / "robot-host.sh", host_script)
        _executable(scripts / "ros-stop.sh", "exit 0\n")
        log.write_text("", encoding="utf-8")
        result = subprocess.run(
            ["bash", str(scripts / "calibrate.sh"), "motor"],
            env={**os.environ, "HOME": str(home), "LEKIWI_LOGS": str(tmp_path / "logs"),
                 "PATH": f"{fakes}:{os.environ['PATH']}"},
            capture_output=True, text=True, timeout=30,
        )
        return result, log.read_text(encoding="utf-8").splitlines()

    failed, calls = run("exit 1\n")
    assert failed.returncode != 0
    assert calls == ["stop lekiwi-host.service", "start lekiwi-host.service"]

    motor_file.parent.mkdir(parents=True)
    finished, calls = run(f'echo "{{}}" > "{motor_file}"\n')
    assert finished.returncode == 0, finished.stderr
    assert calls == ["stop lekiwi-host.service", "start lekiwi-host.service"]
