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


@pytest.mark.parametrize('activation_failure',[False,True])
@pytest.mark.parametrize('existing_profile',[False,True])
def test_wifi_switch_arms_rollback_before_activation_and_retains_original(tmp_path,activation_failure,existing_profile):
    fakes=tmp_path/'bin'
    log=tmp_path/'calls'
    _executable(fakes/'nmcli',r'''
printf 'nmcli %s\n' "$*" >> "$LEKIWI_TEST_LOG"
if [[ $1 == -g && $2 == GENERAL.CON-UUID ]]; then echo original; exit; fi
if [[ $1 == -g && $2 == connection.autoconnect-priority ]]; then echo 50; exit; fi
if [[ $1 == -g && $2 == 802-11-wireless.ssid ]]; then echo 'house 5GHz'; exit; fi
if [[ $1 == -g ]]; then
  [[ -f $LEKIWI_TEST_CLONED ]] || exit 10
  echo target; exit
fi
if [[ $1 == connection && $2 == clone ]]; then touch "$LEKIWI_TEST_CLONED"; fi
if [[ $1 == --wait ]]; then exit "$LEKIWI_TEST_FAIL"; fi
while (( $# )); do
  if [[ $1 == 802-11-wireless.channel && ${2:-} == 0 ]]; then exit 2; fi
  shift
done
''')
    _executable(fakes/'sudo','shift\nexec "$@"\n')
    for name in ('systemd-run','systemctl'):
        _executable(fakes/name,'printf "%s %s\\n" "${0##*/}" "$*" >> "$LEKIWI_TEST_LOG"\n')
    if existing_profile:
        (tmp_path/'cloned').touch()
    result=subprocess.run(['bash',str(ROOT/'scripts/switch-device-wifi.sh'),'house 5GHz'],
        env={**os.environ,'PATH':f'{fakes}:{os.environ["PATH"]}',
             'LEKIWI_TEST_LOG':str(log),'LEKIWI_TEST_CLONED':str(tmp_path/'cloned'),
             'LEKIWI_TEST_FAIL':str(int(activation_failure))},capture_output=True,text=True)
    calls=log.read_text().splitlines()
    timer=next(i for i,s in enumerate(calls) if s.startswith('systemd-run'))
    activation=next(i for i,s in enumerate(calls) if s.startswith('nmcli --wait'))
    assert timer<activation and calls[timer].endswith('/usr/bin/nmcli connection up uuid original')
    assert not any('modify uuid original' in s or 'delete' in s or 'psk' in s for s in calls)
    assert any('band a' in s and 'autoconnect-priority 51' in s for s in calls)
    assert any('connection clone' in s for s in calls) is not existing_profile
    assert ('systemctl stop lekiwi-wifi-rollback.timer' in calls) is not activation_failure
    assert (result.returncode!=0) is activation_failure


@pytest.mark.parametrize("help_text,expected", [("colcon build", False), ("--allow-overriding", True)])
def test_native_builder_accepts_colcon_without_optional_override_extension(tmp_path, help_text, expected):
    _executable(tmp_path / "colcon", f'[[ "$*" == "build --help" ]] && echo "{help_text}"\n')
    result = _bash('source "$LIB/build-common.sh"; colcon_supports_overriding',
                   PATH=f"{tmp_path}:{os.environ['PATH']}")
    assert (result.returncode == 0) is expected


def test_pi5_usb_current_config_is_idempotent(tmp_path):
    script = (ROOT / "scripts" / "enable-pi5-usb-current.sh").read_text(encoding="utf-8")
    update = script.split("<<'PY'\n", 1)[1].split("\nPY", 1)[0]
    config = tmp_path / "config.txt"
    config.write_text("[all]\nother_setting=1\n", encoding="utf-8")
    subprocess.run(["python3", "-c", update, str(config)], check=True)
    first = config.read_text(encoding="utf-8")
    config.write_text(first.replace("# BEGIN cleanroom", "[all]\n\n[all]\n# BEGIN cleanroom"), encoding="utf-8")
    subprocess.run(["python3", "-c", update, str(config)], check=True)
    assert config.read_text(encoding="utf-8") == first
    assert first.count("usb_max_current_enable=1") == 1


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
        ["bash", str(ROOT / "scripts" / "robot-host.sh")],
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
            ["bash", str(ROOT / "scripts" / "robot-host.sh")],
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

    assert _unit("lekiwi-host.service")["Service"]["RestartPreventExitStatus"] == ["78"]


@pytest.mark.skipif(os.geteuid() == 0, reason="an explicit account is resolved as a non-root caller")
def test_service_user_must_be_an_existing_non_root_account():
    def resolve(name):
        return _bash('source "$LIB/runtime-common.sh"; source "$LIB/service-install-common.sh"\n'
                     'resolve_service_user "$1"; echo "$LEKIWI_SERVICE_USER $LEKIWI_SERVICE_HOME"', name)

    user = getpass.getuser()
    accepted = resolve(user)
    assert accepted.returncode == 0, accepted.stderr
    assert accepted.stdout.split() == [user, os.path.expanduser("~")]
    for name, reason in (("root", "refusing to install robot services as root"),
                         ("Not-A-User", "invalid service user"),
                         ("no_such_lekiwi_user", "service user does not exist")):
        rejected = resolve(name)
        assert rejected.returncode != 0 and reason in rejected.stderr, name


def test_unit_validation_checks_only_the_named_units(tmp_path):
    calls = tmp_path / "calls"
    _executable(tmp_path / "bin" / "systemd-analyze", f'echo "$*" >> "{calls}"\n')
    result = _bash('as_root() { "$@"; }; source "$LIB/runtime-common.sh"; source "$LIB/service-install-common.sh"\n'
                   'UNIT_DIR=/units; verify_systemd_units a.service b.service',
                   PATH=f"{tmp_path / 'bin'}:{os.environ['PATH']}")
    assert result.returncode == 0, result.stderr
    # Dependencies outside this installer can be absent by design; never recurse into them.
    assert calls.read_text().splitlines() == [
        "verify --recursive-errors=no /units/a.service", "verify --recursive-errors=no /units/b.service",
    ]


def test_full_installer_includes_qualification_tooling_dependencies():
    """A fresh deployment must not silently omit required qualification checks."""
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")
    lines = installer.splitlines()

    for package in ("shellcheck", "python3-zmq"):
        assert package in installer
    universe_line = next(index for index, line in enumerate(lines) if "add-apt-repository -y universe" in line)
    shellcheck_line = next(index for index, line in enumerate(lines) if line.strip().startswith("shellcheck "))
    assert universe_line < shellcheck_line


@pytest.mark.skipif(shutil.which("git") is None, reason="git is required")
def test_checkout_with_patches_applies_every_tracked_patch_and_reruns(tmp_path):
    script = r'''
set -Eeuo pipefail
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$1/scripts/thirdparty-common.sh"
THIRDPARTY_PATCH_ROOT=$2/thirdparty
git() { command git -c user.name=test -c user.email=test@example.invalid "$@"; }
cd "$2"
git init -q upstream
printf 'one\n' > upstream/a.txt
printf 'one\n' > upstream/b.txt
git -C upstream add .
git -C upstream commit -qm base
revision=$(git -C upstream rev-parse HEAD)
mkdir -p thirdparty/dep thirdparty/plain
printf 'two\n' > upstream/a.txt
git -C upstream diff > thirdparty/dep/0001-first.patch
git -C upstream checkout -q a.txt
printf 'three\n' > upstream/b.txt
git -C upstream diff > thirdparty/dep/0002-second.patch
git -C upstream checkout -q b.txt

checkout_with_patches dep "$PWD/upstream" "$PWD/dest" "$revision" >/dev/null 2>&1
[[ $(cat dest/a.txt) == two && $(cat dest/b.txt) == three ]]
checkout_with_patches dep "$PWD/upstream" "$PWD/dest" "$revision" >/dev/null 2>&1
[[ $(cat dest/a.txt) == two && $(cat dest/b.txt) == three ]]
apply_thirdparty_patches dep dest
[[ $(cat dest/a.txt) == two && $(cat dest/b.txt) == three ]]
# A dependency without tracked patches is a plain pinned checkout.
checkout_with_patches plain "$PWD/upstream" "$PWD/clean" "$revision" >/dev/null 2>&1
[[ $(cat clean/a.txt) == one && -z $(git -C clean status --porcelain) ]]
# A conflicting edit names the patch that could not be applied.
printf 'mine\n' > clean/a.txt
if message=$(apply_thirdparty_patches dep clean 2>&1); then exit 1; fi
[[ $message == *"dep/0001-first.patch"* ]]
'''
    subprocess.run(["bash", "-c", script, "patched-checkout", str(ROOT), str(tmp_path)], check=True)


def test_installers_and_builders_apply_every_tracked_third_party_patch_set():
    scripts = {name: (ROOT / "scripts" / name).read_text(encoding="utf-8")
               for name in ("install.sh", "install-pi.sh", "build-lekiwi.sh", "build-native.sh")}
    assert "checkout_with_patches free_fleet " in scripts["install.sh"]
    assert "checkout_with_patches ros2_astra_camera " in scripts["install.sh"]
    for name in ("install.sh", "install-pi.sh"):
        assert "checkout_with_patches ldlidar_stl_ros2 " in scripts[name]
    for dependency in ("ldlidar_stl_ros2", "ros2_astra_camera"):
        assert f"apply_thirdparty_patches {dependency} " in scripts["build-lekiwi.sh"]
    assert 'checkout_with_patches "$dependency"' in scripts["build-native.sh"]
    assert 'apply_thirdparty_patches "$dependency"' in scripts["build-native.sh"]
    # Every tracked patch belongs to a dependency some script checks out.
    native = re.search(r"for dependency in ([a-z_0-9 ]+); do", scripts["build-native.sh"]).group(1).split()
    used = {"free_fleet", "ros2_astra_camera", "ldlidar_stl_ros2", *native}
    assert {patch.parent.name for patch in (ROOT / "thirdparty").glob("*/*.patch")} == used


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
def test_pinned_checkout_reverses_known_patches_and_keeps_other_local_edits(tmp_path):
    script = r'''
set -Eeuo pipefail
die() { printf '%s\n' "$*" >&2; exit 1; }
source "$1/scripts/thirdparty-common.sh"
git() { command git -c user.name=test -c user.email=test@example.invalid "$@"; }
cd "$2"
git init -q upstream
printf 'one\n' > upstream/a.txt
printf 'two\n' > upstream/b.txt
printf 'three\n' > upstream/c.txt
git -C upstream add .
git -C upstream commit -qm base
printf 'patched\n' > upstream/a.txt
git -C upstream diff > first.patch
git -C upstream checkout -q a.txt
printf 'patched too\n' > upstream/b.txt
git -C upstream diff > second.patch
git -C upstream checkout -q b.txt
revision=$(git -C upstream rev-parse HEAD)

checkout_pinned "$PWD/upstream" "$PWD/dest" "$revision" "$PWD/first.patch" "$PWD/second.patch" >/dev/null 2>&1
apply_pinned_patch dest "$PWD/first.patch" "the first test patch"
apply_pinned_patch dest "$PWD/first.patch" "the first test patch"
apply_pinned_patch dest "$PWD/second.patch" "the second test patch"
# A rerun over the patched tree, with an unrelated local edit beside it.
printf 'mine\n' > dest/c.txt
checkout_pinned "$PWD/upstream" "$PWD/dest" "$revision" "$PWD/first.patch" "$PWD/second.patch" >/dev/null 2>&1
apply_pinned_patch dest "$PWD/first.patch" "the first test patch"
apply_pinned_patch dest "$PWD/second.patch" "the second test patch"
[[ $(cat dest/a.txt) == patched ]]
[[ $(cat dest/b.txt) == 'patched too' ]]
[[ $(cat dest/c.txt) == mine ]]

# Without the known patch, local changes are refused rather than discarded.
if (checkout_pinned "$PWD/upstream" "$PWD/dest" "$revision" >/dev/null 2>&1); then exit 1; fi
[[ $(cat dest/c.txt) == mine ]]

# An unrelated edit alone is not mistaken for an already-applied known patch.
checkout_pinned "$PWD/upstream" "$PWD/other" "$revision" >/dev/null 2>&1
printf 'mine\n' > other/c.txt
if (checkout_pinned "$PWD/upstream" "$PWD/other" "$revision" "$PWD/first.patch" "$PWD/second.patch" >/dev/null 2>&1); then exit 1; fi
[[ $(cat other/c.txt) == mine ]]
'''
    subprocess.run(["bash", "-c", script, "pinned-checkout", str(ROOT), str(tmp_path)], check=True)


def test_simulation_installer_excludes_astra_hardware_setup():
    installer = (ROOT / "scripts" / "install.sh").read_text(encoding="utf-8")

    assert 'if [[ $install_mode == full ]]; then\n  log "Fetching the pinned Orbbec Astra Pro ROS 2 driver"' in installer
    assert 'extra_source_paths+=("$astra_source")' in installer
    assert 'extra_packages+=(astra_camera astra_camera_msgs)' in installer
    builder = (ROOT / "scripts/build-lekiwi.sh").read_text()
    assert 'packages+=(astra_camera_msgs astra_camera)' in builder
    assert 'Simulation-only installation: skipping Astra driver and udev setup' in installer


@pytest.mark.parametrize("total_kb,available_kb,serial", [
    (64_000_000, 2_000_000, True),     # a large host whose memory is in use elsewhere
    (16_000_000, 12_000_000, False),
])
def test_builds_run_serially_when_little_memory_is_available(tmp_path, total_kb, available_kb, serial):
    meminfo = tmp_path / "meminfo"
    meminfo.write_text(f"MemTotal: {total_kb} kB\nMemFree: 1000 kB\nMemAvailable: {available_kb} kB\n")
    result = _bash('source "$LIB/build-common.sh"; low_available_memory "$1"', str(meminfo))
    assert (result.returncode == 0) is serial


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


def test_mapper_shutdown_signals_the_launcher_without_interrupting_its_save():
    service = _unit("lekiwi-stack.service")["Service"]
    assert service["KillMode"] == ["mixed"] and service["KillSignal"] == ["SIGINT"]
    runner = (ROOT / 'scripts/test-navigation.py').read_text()
    assert 'os.kill(stack.pid,signal.SIGINT)' in runner
    assert re.search(r'try:\s+stack\.wait\(timeout=45\)', runner)
    deploy = (ROOT / 'scripts/deploy-split.sh').read_text()
    assert 'systemctl show -P KillMode lekiwi-stack.service' in deploy
    assert deploy.index('kill -INT "$stack_pid"') < deploy.index('sudo -n /usr/bin/systemctl stop lekiwi-stack.service')


def test_ros_stop_interrupts_only_the_stack_launcher_so_it_can_shut_its_nodes_down(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    _executable(tmp_path / "bin" / "systemctl", "exit 3\n")
    signals = tmp_path / "node-signals"
    # A node in the launcher's process group: ros2 launch, not ros-stop, must stop it.
    node = (
        "import os, signal, sys, time\n"
        "record = lambda number, _: open(sys.argv[1], 'a').write(f'{number}\\n')\n"
        "signal.signal(signal.SIGINT, record)\n"
        "signal.signal(signal.SIGTERM, record)\n"
        "while os.getppid() == int(sys.argv[2]):\n    time.sleep(0.05)\n"
    )
    launcher = subprocess.Popen(
        ["bash", "-c", 'python3 -c "$1" "$2" "$$" & trap "exit 0" INT; wait',
         "ros2 launch lekiwi_rmf bringup.launch.py", node, str(signals)],
        start_new_session=True,
    )
    time.sleep(0.5)
    (runtime / "stack.pid").write_text(f"{launcher.pid}\n", encoding="utf-8")
    stop = subprocess.Popen(
        ["bash", str(ROOT / "scripts" / "ros-stop.sh")],
        env={**os.environ, "LEKIWI_RUNTIME_DIR": str(runtime), "PATH": f"{tmp_path / 'bin'}:{os.environ['PATH']}"},
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        assert launcher.wait(timeout=10) == 0
        stdout, stderr = stop.communicate(timeout=30)
        assert stop.returncode == 0, stderr
        assert "stopping recorded stack" in stdout
        assert not signals.exists()
    finally:
        for process in (launcher, stop):
            if process.poll() is None:
                process.kill()
                process.wait()
        if launcher.pid:
            subprocess.run(["pkill", "-KILL", "-g", str(launcher.pid)], check=False)


def test_rviz_exports_the_selected_collision_plugin_in_its_parameter_file():
    script = (ROOT / 'scripts/rviz.sh').read_text()
    assert 'config["collision_detector"] = "lekiwi_rmf/RestFCL"' in script
    keys = script.split('keys = (', 1)[1].split(')', 1)[0]
    assert '"collision_detector"' in keys
    display = (ROOT / 'src/rest_motion_planning_display.cpp').read_text()
    assert display.index('declare_parameter<std::string>') < display.index('MotionPlanningDisplay::onInitialize();')
    assert 'Class: lekiwi_rmf/MotionPlanning' in (ROOT / 'config/lekiwi.rviz').read_text()


def test_build_reuses_its_checkout_cache_and_removes_a_foreign_cache(tmp_path):
    workspace = tmp_path / 'workspace'
    build = workspace / 'build/lekiwi_rmf'
    build.mkdir(parents=True)
    cache = build / 'CMakeCache.txt'
    cache.write_text('CMAKE_HOME_DIRECTORY:INTERNAL=/source/current\n')

    def run(project_root):
        result = _bash('source "$LIB/build-common.sh"; remove_foreign_build_cache "$1" "$2"',
                       str(workspace), project_root)
        assert result.returncode == 0, result.stderr

    run('/source/current')
    assert cache.exists()
    run('/source/other')
    assert not build.exists()
    run('/source/other')  # nothing left to remove


def test_service_installers_support_an_unauthenticated_split_zmq_transport():
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")

    assert 'if [[ -n $CURVE_DIR_ARG ]]; then' in device
    assert 'STACK_ARGS="profile:=split remote_ip:=$REMOTE start_moveit:=true"' in compute
    assert compute.count("--curve-dir does not contain") == 1


def test_remote_stack_has_no_local_host_dependency():
    stack = _unit("lekiwi-stack.service")["Unit"]
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")

    assert "Requires" not in stack and "PartOf" not in stack
    assert "network-online.target" in " ".join(stack["After"]).split()
    # Only the all-in-one topology binds the stack to a local host, through a drop-in.
    assert '"Requires=lekiwi-host.service"' in compute
    assert '"PartOf=lekiwi-host.service"' in compute


def test_standard_installers_start_and_relay_the_host_lidar_without_an_opt_in():
    installer = (ROOT / "scripts" / "install-compute-services.sh").read_text(encoding="utf-8")
    device = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")

    assert "--remote-lidar" not in installer
    assert "units=(lekiwi-host.service lekiwi-lidar.service)" in device
    assert 'as_root systemctl enable --now --no-block "${units[@]}"' in device
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
    # A stopped, changed unit must start once, not start and then restart.
    assert device.index("restart_changed_units --no-block\n") < device.index(
        'enable --now --no-block "${units[@]}"'
    )



def test_device_restart_does_not_wait_for_hardware_units(tmp_path):
    # Unpowered servos keep lekiwi-host from starting; the installer must still
    # reach the sudoers grant, the Wi-Fi country and the fingerprint.
    script = r'''
set -Eeuo pipefail
PROJECT_ROOT=$1
calls=$2/calls
log() { :; }
as_root() { printf '%s\n' "$*" >> "$calls"; }
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
CHANGED_UNITS=(lekiwi-host.service)
restart_changed_units --no-block
'''
    subprocess.run(["bash", "-c", script, "no-block", str(ROOT), str(tmp_path)], check=True)
    calls = (tmp_path / "calls").read_text(encoding="utf-8")
    assert calls == "systemctl try-restart --no-block lekiwi-host.service\n"

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
        unit = _unit(name)
        assert unit["Unit"]["StartLimitIntervalSec"] == ["0"], name
        assert unit["Service"]["Restart"] == ["always"], name


def test_pi_and_manual_split_startup_include_the_ld06():
    pi_installer = (ROOT / "scripts" / "install-pi.sh").read_text(encoding="utf-8")
    pi_up = (ROOT / "scripts" / "pi-up.sh").read_text(encoding="utf-8")
    workstation_up = (ROOT / "scripts" / "workstation-up.sh").read_text(encoding="utf-8")
    lidar = (ROOT / "scripts" / "ros-lidar.sh").read_text(encoding="utf-8")

    assert "Installing the pinned LD06 ROS driver" in pi_installer
    build = (ROOT / "scripts" / "build-lekiwi.sh").read_text(encoding="utf-8")
    assert 'packages+=(ldlidar_stl_ros2)' in build
    assert "ldlidar_stl_ros2_node" in pi_installer
    assert "start_recorded lidar scripts/ros-lidar.sh" in pi_up
    assert "start_recorded astra scripts/ros-astra.sh" in pi_up
    assert "start_recorded zenoh scripts/ros-zenoh.sh" in pi_up
    assert "profile:=split" in workstation_up
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
    stop_host = deploy.index("lekiwi-zenoh.service lekiwi-host.service; do", stop_stack)
    start_host = deploy.index("start lekiwi-host.service", stop_host)
    start_stack = deploy.index("start lekiwi-stack.service", start_host)
    assert disarm < stop_stack < stop_host < start_host < start_stack
    stage_device = deploy.index('nice -n 10 bash -s -- device')
    stage_compute = deploy.index('nice -n 10 "$project_root/scripts/stage-release.sh" compute')
    verify = deploy.index('verify "$compute_release" "$target" compute', stage_compute)
    activate = deploy.index('\nactivate_release "$workspace" "$compute_release"')
    assert stage_device < stage_compute < verify < disarm
    assert stop_host < activate < start_host
    # A stale compute configuration is reinstalled only once the robot is disarmed and
    # its stack stopped, never started early, and its sudo need is checked up front.
    refresh = deploy.index("\n  refresh_compute_service\n")
    assert stop_host < refresh < start_host
    assert deploy.count("refresh_compute_service\n") == 1
    refresh_device = deploy.index('"${ssh_command[@]}" sudo -n "${remote_installer[@]}"', stop_host)
    assert stop_host < refresh_device < start_host
    assert '--bind-address "$remote_bind_address" --no-start' in deploy
    device_installer = (ROOT / "scripts" / "install-device-services.sh").read_text(encoding="utf-8")
    assert 'as_root systemctl enable "${units[@]}" lekiwi-ros-logrotate.timer' in device_installer
    assert '"$compute_current/source/scripts/reinstall-compute.sh" "${installer_args[@]}"' in deploy
    assert 'local installer_args=(--no-start) stack_arguments=()' in deploy
    assert deploy.index("sudo -n true") < disarm
    assert disarm < deploy.index('sudo -n /usr/bin/systemctl reboot') < stop_host
    assert 'vcgencmd get_config usb_max_current_enable' in deploy
    sudoers = (ROOT / "scripts" / "install-deploy-sudoers.sh").read_text(encoding="utf-8")
    assert 'commands+=("$systemctl reboot")' in sudoers
    assert "lekiwi-lidar.service" in deploy
    # The zenoh bridge is required and preflighted before anything is stopped;
    # Astra and the cameras are skipped by the device installer without their ROS packages.
    assert "device_units=(lekiwi-host.service lekiwi-lidar.service lekiwi-zenoh.service)" in deploy
    assert "for unit in lekiwi-lidar.service lekiwi-zenoh.service; do" in deploy
    assert 'if remote_unit_exists "$unit"; then device_units+=("$unit"); fi' in deploy
    assert "check-release.py" in deploy
    assert "expected_service_fingerprint" in deploy
    assert "canonical /scan is not the LD06 frame" in deploy
    assert 'awk \'NF && $1 != "---" { print $1; exit }\'' in deploy
    assert "has_nopasswd_systemctl" in deploy
    assert 'compute_sudoers=$(sudo -n -l)' in deploy
    # The deployer runs from any directory: every git call names its repository.
    assert not re.search(r"(?<![\w-])git (?!-C )", deploy)
    assert "cannot fetch origin within 30 seconds" in deploy
    assert "LEKIWI_ROBOT_HOST" in deploy
    assert "load_lekiwi_env" in deploy
    assert "Refreshing stale compute service configuration" in deploy
    assert "reinstall-compute.sh" in deploy
    assert "reset --hard" not in deploy


def test_deploy_bundle_transfers_exact_revision_without_device_origin(tmp_path):
    source, device = tmp_path/'source', tmp_path/'device'
    def git(repository, *args):
        return subprocess.run(['git', '-C', str(repository), *args], check=True,
                              capture_output=True, text=True).stdout.strip()
    source.mkdir()
    git(source, 'init')
    git(source, 'config', 'user.name', 'Test')
    git(source, 'config', 'user.email', 'test@example.invalid')
    (source/'value').write_text('before')
    git(source, 'add', 'value')
    git(source, 'commit', '-m', 'before')
    subprocess.run(['git', 'clone', str(source), str(device)], check=True, capture_output=True)
    (source/'value').write_text('after')
    git(source, 'commit', '-am', 'after')
    target = git(source, 'rev-parse', 'HEAD')
    # The local "ssh" runs its command string in a shell, like the remote login shell.
    result = _bash('''source "$LIB/runtime-common.sh"; source "$LIB/deploy-common.sh"
ssh_command=(bash -c 'if [[ $# == 1 ]]; then eval "$1"; else "$@"; fi' --)
transfer_device_revision "$1" "$2" "$3" "${ssh_command[@]}"
git -C "$2" merge --ff-only "$3"
transfer_device_revision "$1" "$2" "$3" "${ssh_command[@]}"''', str(source), str(device), target)
    assert result.returncode == 0, result.stderr
    assert git(device, 'rev-parse', 'HEAD') == target
    assert (device/'value').read_text() == 'after'


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


def test_managed_build_prefers_system_cmake_and_records_its_revision():
    builder = (ROOT / "scripts" / "build-lekiwi.sh").read_text(encoding="utf-8")

    assert "PATH=/usr/bin:/bin:$PATH" in builder
    assert '-DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local"' in builder
    assert 'remove_foreign_build_cache "$workspace" "$project_root"' in builder
    assert '.lekiwi-source-revision' in builder


@pytest.mark.parametrize("role,sources", [
    ("compute", ("systemd/lekiwi-stack.service", "scripts/lib/service-install-common.sh",
                 "scripts/lib/runtime-common.sh", "scripts/install-deploy-sudoers.sh",
                 "scripts/setup-zenoh-tls.sh", "scripts/install-wifi-powersave.sh",
                 "scripts/install-compute-services.sh")),
    ("device", ("systemd/lekiwi-astra.service", "scripts/ros-astra.sh", "systemd/lekiwi-lidar.service",
                "scripts/ros-lidar.sh", "scripts/lib/runtime-common.sh", "scripts/install-device-network.sh",
                "scripts/install-wifi-powersave.sh", "config/dds_socket_buffers.conf")),
])
def test_service_fingerprint_changes_with_each_installed_service_input(tmp_path, role, sources):
    checkout = tmp_path / "checkout"
    shutil.copytree(ROOT / "systemd", checkout / "systemd")
    shutil.copytree(ROOT / "scripts", checkout / "scripts")
    (checkout / "config").mkdir()
    shutil.copy2(ROOT / "config" / "dds_socket_buffers.conf", checkout / "config")

    def fingerprint():
        result = _bash('PROJECT_ROOT=$1; source "$LIB/runtime-common.sh"; source "$LIB/service-install-revision.sh"\n'
                       'service_fingerprint "$2"', str(checkout), role)
        assert result.returncode == 0, result.stderr
        return result.stdout.strip()

    baseline = fingerprint()
    assert re.fullmatch(r"[0-9a-f]{64}", baseline)
    for source in sources:
        path = checkout / source
        original = path.read_bytes()
        path.write_bytes(original + b"\n")
        assert fingerprint() != baseline, source
        path.write_bytes(original)
    # A deployment that changes nothing the services run does not refresh them.
    (checkout / "scripts" / "foxglove.sh").write_text("changed\n")
    assert fingerprint() == baseline
    assert 'as_root "$PROJECT_ROOT/scripts/install-wifi-powersave.sh"' in (
        ROOT / "scripts" / "install-compute-services.sh"
    ).read_text(encoding="utf-8")


def _network_checkout(tmp_path: pathlib.Path, env_file: str | None = None) -> pathlib.Path:
    """A copy of the network installers, so the developer's own .env cannot leak in."""
    checkout = tmp_path / "checkout"
    for name in ("install-device-network.sh", "install-wifi-regdom.sh", "install-wifi-powersave.sh", "lib/runtime-common.sh"):
        target = checkout / "scripts" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "scripts" / name, target)
    (checkout/'config').mkdir()
    shutil.copy2(ROOT/'config/dds_socket_buffers.conf',checkout/'config/dds_socket_buffers.conf')
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
    buffers=tmp_path/'root/etc/sysctl.d/60-lekiwi-dds-socket-buffers.conf'
    assert buffers.read_text()==(ROOT/'config/dds_socket_buffers.conf').read_text()

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
    assert "safety_policy:=strict" in stack
    assert '--safety.disarm_on_failure="${LEKIWI_DISARM_ON_FAILURE:-false}"' in host


def _bash(body: str, *args: str, **env: str) -> subprocess.CompletedProcess:
    """Run a bash snippet that sources what it needs from $LIB (scripts/lib)."""
    return subprocess.run(
        ["bash", "-c", "set -Eeuo pipefail\nLIB=$0\n" + body, str(ROOT / "scripts" / "lib"), *args],
        env={**os.environ, **env}, capture_output=True, text=True, timeout=60,
    )


def _unit(name: str) -> dict[str, dict[str, list[str]]]:
    """A tracked systemd unit as {section: {key: [values in order]}}."""
    sections: dict[str, dict[str, list[str]]] = {}
    current: dict[str, list[str]] = {}
    for line in (ROOT / "systemd" / name).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith(("#", ";")):
            continue
        if line.startswith("[") and line.endswith("]"):
            current = sections.setdefault(line[1:-1], {})
        else:
            key, _, value = line.partition("=")
            current.setdefault(key.strip(), []).append(value.strip())
    return sections


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
    service = _unit("lekiwi-ros-logrotate.service")["Service"]
    # Pruning runs after logrotate even when logrotate fails, and logrotate gets its state directory.
    assert service["ExecStopPost"] == ["-@PROJECT_ROOT@/scripts/prune-ros-logs.sh @SERVICE_HOME@/.ros/log"]
    assert service["ExecStartPre"] == ["/usr/bin/mkdir -p @SERVICE_HOME@/.ros/lekiwi"]
    assert any("--state @SERVICE_HOME@/.ros/lekiwi/" in command for command in service["ExecStart"])
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


def test_stack_retries_startup_and_host_gets_time_to_pass_its_gate():
    stack = _unit("lekiwi-stack.service")["Unit"]
    host = _unit("lekiwi-host.service")["Service"]

    assert stack["StartLimitIntervalSec"] == ["0"] and "StartLimitBurst" not in stack
    # The ExecStartPost gate probes up to 120 times, one second each plus a one second sleep.
    timeout, = host["TimeoutStartSec"]
    assert re.fullmatch(r"\d+s", timeout) and int(timeout[:-1]) > 2 * 120


LAUNCHERS = ("up.sh", "pi-up.sh", "workstation-up.sh", "sim-up.sh")


def _launcher(tmp_path, body, *args, **env):
    """Run body after sourcing scripts/lib/launcher.sh with private log/runtime dirs."""
    script = 'set -Eeuo pipefail\nsource "$LAUNCHER_LIB"\n' + body
    return subprocess.run(
        ["bash", "-c", script, "launcher-test", *args],
        env={**os.environ, "LAUNCHER_LIB": str(ROOT / "scripts/lib/launcher.sh"),
             "LEKIWI_LOGS": str(tmp_path / "logs"), **env},
        capture_output=True, text=True, timeout=30,
    )


def test_launcher_init_serializes_startup_and_children_never_hold_the_lock(tmp_path):
    runtime = tmp_path / "logs" / "runtime"
    # The first launcher starts a long-running child, then keeps the lock while a
    # second launcher of the same name tries to start.
    result = _launcher(tmp_path, r'''
launcher_init demo
[[ $LEKIWI_RUNTIME_DIR == "$RUNTIME_DIR" && $(stat -c %a "$RUNTIME_DIR") == 700 ]]
start_recorded child sleep 30
start_recorded other --log other-name bash -c 'echo started'
second=0
bash -c 'source "$LAUNCHER_LIB"; launcher_init demo; echo second-started' > "$LOGS/second" 2>&1 || second=$?
[[ $second == 0 && $(<"$LOGS/second") == *"startup is already in progress"* ]]
launcher_release
# Released: the child does not keep the lock alive.
bash -c 'source "$LAUNCHER_LIB"; launcher_init demo; echo third-started' > "$LOGS/third" 2>&1
[[ $(<"$LOGS/third") == third-started ]]
''')
    assert result.returncode == 0, result.stderr
    child = int((runtime / "child.pid").read_text())
    try:
        # The recorded PID leads its own session and process group, which ros-stop signals.
        assert os.getsid(child) == child and os.getpgid(child) == child
        fds = {os.readlink(f"/proc/{child}/fd/{fd}") for fd in os.listdir(f"/proc/{child}/fd")}
        assert not any(target.endswith("demo-start.lock") for target in fds)
    finally:
        os.killpg(child, 9)
    deadline = time.monotonic() + 5
    while not (tmp_path / "logs" / "other-name.log").read_text() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert (tmp_path / "logs" / "other-name.log").read_text() == "started\n"
    assert (runtime / "other.pid").exists()


def test_launchers_start_long_running_children_only_through_start_recorded():
    for name in LAUNCHERS:
        script = (ROOT / "scripts" / name).read_text(encoding="utf-8")
        assert "launcher_init " in script and "setsid" not in script, name
    workstation = (ROOT / "scripts" / "workstation-up.sh").read_text(encoding="utf-8")
    # A shared workstation's other stacks are never discovered by name.
    assert "pgrep" not in workstation
    assert "start_recorded rviz scripts/rviz.sh" in workstation


@pytest.mark.parametrize("kind,name", [
    ("stack", "ros2 launch lekiwi_rmf bringup.launch.py profile:=split"),
    ("stack", "bash scripts/ros-start.sh profile:=split"),
    ("host", "bash scripts/robot-host.sh"),
    ("host", "python3 -m lerobot.robots.lekiwi.lekiwi_host"),
    ("rviz", "bash scripts/rviz.sh"),
    ("rviz", "rviz2 -d lekiwi.rviz"),
    ("astra", "ros2 launch lekiwi_rmf pi_astra.launch.py"),
    ("cameras", "ros2 launch launch/pi_cameras.launch.py"),
    ("lidar", "ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node"),
    ("zenoh", "zenoh-bridge-ros2dds -c config/zenoh_device.json5"),
])
def test_recorded_running_identifies_each_kind_by_its_recorded_pid(tmp_path, kind, name):
    sentinel = subprocess.Popen(["bash", "-c", f'exec -a "{name}" sleep 60'])
    runtime = tmp_path / "logs" / "runtime"
    runtime.mkdir(parents=True)
    try:
        (runtime / f"{kind}.pid").write_text(f"{sentinel.pid}\n", encoding="utf-8")
        other = "zenoh" if kind != "zenoh" else "host"
        (runtime / f"{other}.pid").write_text(f"{sentinel.pid}\n", encoding="utf-8")
        result = _launcher(tmp_path, '''
launcher_dirs
recorded_running "$1"
! recorded_running "$2"
! recorded_running missing
''', kind, other)
        assert result.returncode == 0, result.stderr
    finally:
        sentinel.kill()
        sentinel.wait()
    # A recorded PID that has exited is not running, whatever it was.
    result = _launcher(tmp_path, 'launcher_dirs\n! recorded_running "$1"\n', kind)
    assert result.returncode == 0, result.stderr


def test_stack_service_refusal_depends_on_the_unit_state(tmp_path):
    fakes = tmp_path / "bin"
    _executable(fakes / "systemctl", '[[ $1 == is-active && $3 == lekiwi-stack.service ]] && exit "$UNIT_STATE"\nexit 3\n')
    for state, expected in (("0", 1), ("3", 0)):
        result = _launcher(tmp_path, "refuse_while_stack_service_runs\n",
                           PATH=f"{fakes}:{os.environ['PATH']}", UNIT_STATE=state)
        assert result.returncode == expected, result.stderr
    assert "stop it first" in _launcher(tmp_path, "refuse_while_stack_service_runs\n",
                                        PATH=f"{fakes}:{os.environ['PATH']}", UNIT_STATE="0").stderr


def test_ros_stop_leaves_unit_owned_stack_alone(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    fake_systemctl = tmp_path / "bin" / "systemctl"
    unit_cgroup = next(
        line.split("::", 1)[1]
        for line in pathlib.Path(f"/proc/{os.getpid()}/cgroup").read_text().splitlines()
        if line.startswith("0::")
    )
    _executable(
        fake_systemctl,
        'case "$1" in is-active) exit 0 ;; show) printf "%s\\n" "$FAKE_UNIT_CGROUP" ;; esac\n',
    )
    sentinel = subprocess.Popen(
        ["bash", "-c", 'exec -a "ros2 launch lekiwi_rmf bringup.launch.py" sleep 60'],
        start_new_session=True,
    )
    try:
        (runtime / "stack.pid").write_text(f"{sentinel.pid}\n", encoding="utf-8")
        result = subprocess.run(
            ["bash", str(ROOT / "scripts" / "ros-stop.sh")],
            env={**os.environ, "LEKIWI_RUNTIME_DIR": str(runtime),
                 "FAKE_UNIT_CGROUP": unit_cgroup,
                 "PATH": f"{fake_systemctl.parent}:{os.environ['PATH']}"},
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, result.stderr
        assert "lekiwi-stack.service owns recorded stack" in result.stdout
        assert sentinel.poll() is None
        assert (runtime / "stack.pid").exists()
    finally:
        sentinel.kill()
        sentinel.wait()


def test_ros_stop_stops_recorded_sim_when_stack_unit_is_active(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    fake_systemctl = tmp_path / "bin" / "systemctl"
    _executable(
        fake_systemctl,
        'case "$1" in is-active) exit 0 ;; show) printf "%s\\n" "$FAKE_UNIT_CGROUP" ;; esac\n',
    )
    sentinel = subprocess.Popen(
        ["bash", "-c", 'exec -a "ros2 launch lekiwi_rmf bringup.launch.py" sleep 60'],
        start_new_session=True,
    )
    (runtime / "stack.pid").write_text(f"{sentinel.pid}\n", encoding="utf-8")
    sentinel_cgroup = next(
        line.split("::", 1)[1]
        for line in pathlib.Path(f"/proc/{sentinel.pid}/cgroup").read_text().splitlines()
        if line.startswith("0::")
    )
    stop = subprocess.Popen(
        ["bash", str(ROOT / "scripts" / "ros-stop.sh")],
        env={**os.environ, "LEKIWI_RUNTIME_DIR": str(runtime),
             "FAKE_UNIT_CGROUP": f"{sentinel_cgroup}/unrelated.service",
             "PATH": f"{fake_systemctl.parent}:{os.environ['PATH']}"},
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        deadline = time.monotonic() + 5
        while sentinel.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        assert sentinel.poll() is not None
        stdout, stderr = stop.communicate(timeout=5)
        assert stop.returncode == 0, stderr
        assert "stopping recorded stack" in stdout
        assert not (runtime / "stack.pid").exists()
    finally:
        if sentinel.poll() is None:
            sentinel.kill()
            sentinel.wait()
        if stop.poll() is None:
            stop.kill()
            stop.wait()


def test_ros_stop_stops_a_recorded_rviz_launcher_and_refuses_a_reused_pid(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    fakes = tmp_path / "bin"
    _executable(fakes / "systemctl", "exit 3\n")
    # workstation-up.sh records rviz.sh itself, before it execs rviz2.
    launcher = subprocess.Popen(["bash", "-c", 'exec -a "bash scripts/rviz.sh" sleep 60'],
                                start_new_session=True)
    # A recorded PID now reused by an unrelated program must never be signalled.
    unrelated = subprocess.Popen(["sleep", "60"], start_new_session=True)
    try:
        (runtime / "rviz.pid").write_text(f"{launcher.pid}\n", encoding="utf-8")
        (runtime / "host.pid").write_text(f"{unrelated.pid}\n", encoding="utf-8")
        result = subprocess.run(
            ["bash", str(ROOT / "scripts" / "ros-stop.sh")],
            env={**os.environ, "LEKIWI_RUNTIME_DIR": str(runtime),
                 "PATH": f"{fakes}:{os.environ['PATH']}"},
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, result.stderr
        assert f"stopping recorded rviz (PID {launcher.pid})" in result.stdout
        assert launcher.wait(timeout=5) is not None
        assert not (runtime / "rviz.pid").exists()
        assert f"refusing to signal unrecognised PID {unrelated.pid}" in result.stderr
        assert unrelated.poll() is None
    finally:
        for process in (launcher, unrelated):
            if process.poll() is None:
                process.kill()
                process.wait()


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
