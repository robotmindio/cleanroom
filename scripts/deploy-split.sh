#!/usr/bin/env bash
# Deploy one pushed revision to the compute machine and its remote device host.
# Parse the complete command group before long builds; edits to this caller
# checkout must not change the running deployment sequence.
{
set -Eeuo pipefail

usage() {
  echo "usage: $0 [[USER@]DEVICE] [--remote-repo PATH] [--workspace PATH] [--remote-workspace PATH]" >&2
  exit 2
}
die() { echo "$0: $*" >&2; exit 1; }
log() { printf '\n==> %s\n' "$*"; }

original_args=("$@")
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
PROJECT_ROOT=$project_root
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
load_lekiwi_env "$PROJECT_ROOT/.env"
device=${LEKIWI_ROBOT_HOST:-}
if [[ ${1:-} != --* && -n ${1:-} ]]; then
  device=$1
  shift
fi
[[ -n $device ]] || die "set LEKIWI_ROBOT_HOST in $project_root/.env or pass [USER@]DEVICE"
[[ $device =~ ^([a-z_][a-z0-9_-]*@)?[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
  die "device must be a hostname or address, optionally prefixed by USER@"
device_address=${device#*@}

remote_repo=""
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
remote_workspace=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --remote-repo) [[ $# -ge 2 ]] || usage; remote_repo=$2; shift 2 ;;
    --workspace) [[ $# -ge 2 ]] || usage; workspace=$2; shift 2 ;;
    --remote-workspace) [[ $# -ge 2 ]] || usage; remote_workspace=$2; shift 2 ;;
    *) usage ;;
  esac
done

# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/lib/service-install-revision.sh"
logs=${LEKIWI_LOGS:-$HOME/.ros/lekiwi}
mkdir -p "$logs"
if [[ ${LEKIWI_DEPLOY_LOCKED:-} != 1 ]]; then
  export LEKIWI_DEPLOY_LOCKED=1
  exec flock -n "$logs/deploy.lock" "$0" "${original_args[@]}"
fi

ssh_command=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$device")
ssh_interactive=(ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$device")
# shellcheck disable=SC2016 # HOME must expand on the device, not compute.
remote_home=$("${ssh_command[@]}" 'printf %s "$HOME"') || die "cannot reach $device with key-based SSH"
: "${remote_repo:=$remote_home/cleanroom}"
: "${remote_workspace:=$remote_home/lekiwi_ws}"
for path in "$workspace" "$remote_home" "$remote_repo" "$remote_workspace"; do
  [[ $path =~ ^/[A-Za-z0-9._/-]+$ ]] || die "deployment paths must be absolute and contain no whitespace: $path"
done
workspace=$(realpath -e "$workspace") || die "local workspace not found"
remote_workspace=$("${ssh_command[@]}" realpath -e "$remote_workspace") || die "device workspace not found"
[[ $workspace =~ ^/[A-Za-z0-9._/-]+$ && $remote_workspace =~ ^/[A-Za-z0-9._/-]+$ ]] || \
  die "resolved workspace paths must contain no whitespace"

require_clean() { # require_clean <repository> [description]
  local repository=$1 description=${2:-$1}
  [[ -z $(git -C "$repository" status --porcelain) ]] || die "$description has uncommitted or untracked files"
}
transfer_device_revision() (
  # The compute revision is already verified against origin. The device can
  # receive those same Git objects over SSH when its DNS/Internet is unavailable.
  local previous bundle remote_bundle
  local exclusions=()
  previous=$("${ssh_command[@]}" git -C "$remote_repo" rev-parse HEAD)
  [[ $previous =~ ^[0-9a-f]{40}$ ]] || die "invalid device revision"
  [[ $previous != "$target" ]] || return 0
  if git -C "$project_root" cat-file -e "$previous^{commit}" 2>/dev/null; then
    exclusions=("^$previous")
  fi
  bundle=$(mktemp)
  remote_bundle=$("${ssh_command[@]}" mktemp /tmp/lekiwi-source.XXXXXXXX.bundle)
  [[ $remote_bundle =~ ^/tmp/lekiwi-source\.[A-Za-z0-9]+\.bundle$ ]] || die "invalid device bundle path"
  trap 'rm -f -- "$bundle"; "${ssh_command[@]}" rm -f -- "$remote_bundle"' EXIT
  git -C "$project_root" bundle create "$bundle" HEAD "${exclusions[@]}"
  "${ssh_command[@]}" "cat > '$remote_bundle'" < "$bundle"
  "${ssh_command[@]}" git -C "$remote_repo" fetch "$remote_bundle" HEAD
)
ros_setup() {
  export LEKIWI_WS=$workspace
  [[ ! -L $workspace/current ]] || export LEKIWI_WS=$workspace/current
  set +u
  # shellcheck source=/dev/null
  source "$project_root/scripts/setup.bash"
  set -u
  # The ros2 daemon binds its network interface once. One started before a Wi-Fi change
  # answers nothing, and `ros2 service type` then blocks forever. A wedged daemon ignores
  # both `daemon stop` and SIGTERM, so kill it; the next ros2 command starts a fresh one.
  timeout 10 ros2 daemon stop >/dev/null 2>&1 || pkill -KILL -u "$(id -u)" -f 'ros2cli.daemon' || true
}
disarm() {
  local response deadline=$((SECONDS + 90))
  wait_for 30 ros2 service type /safety/disarm || die "/safety/disarm is unavailable"
  # Starting the Astra resets the USB hub it shares with the motor-bus adapter, which
  # leaves the host's torque endpoint silent for up to ~15 s. Disarming is idempotent,
  # so retry through that window instead of failing the deployment on it.
  until response=$(timeout 30 ros2 service call /safety/disarm std_srvs/srv/Trigger '{}' 2>&1) &&
    [[ $response == *"success=True"* ]]; do
    (( SECONDS < deadline )) || die "motor host did not confirm torque-off: $response"
    sleep 3
  done
}
remote_unit_exists() {
  "${ssh_command[@]}" /usr/bin/systemctl cat "$1" >/dev/null 2>&1
}
remote_unit_active() {
  "${ssh_command[@]}" /usr/bin/systemctl is-active --quiet "$1"
}
remote_unit_active_all() {
  local unit
  for unit in "$@"; do remote_unit_active "$unit" || return 1; done
}
# shellcheck disable=SC2016 # $device expands in the remote shell.
remote_front_camera_present() {
  "${ssh_command[@]}" 'for device in /dev/v4l/by-id/*WEBCAM*-video-index0; do [[ -e $device ]] && exit 0; done; exit 1'
}
has_nopasswd_systemctl() { # has_nopasswd_systemctl <sudo -l output> <action> <unit>
  local rules=$1 action=$2 unit=$3
  [[ $rules == *"NOPASSWD:"*"/usr/bin/systemctl $action $unit"* ]]
}
refresh_compute_service() {
  log "Refreshing stale compute service configuration"
  local setting curve_directory
  local installer_args=(--no-start) stack_arguments=()
  read -r -a stack_arguments <<<"$(sed -n 's/^LEKIWI_STACK_ARGS=//p' /etc/default/lekiwi-stack)"
  for setting in "${stack_arguments[@]}"; do
    if [[ $setting == curve_client_secret_key_file:=* ]]; then
      curve_directory=${setting#*=}
      [[ $curve_directory == */clients/driver.key_secret ]] || die "unexpected compute CURVE key path"
      curve_directory=${curve_directory%/clients/driver.key_secret}
      [[ $curve_directory =~ ^/[A-Za-z0-9._/-]+$ ]] || die "invalid compute CURVE directory"
      installer_args+=(--curve-dir "$curve_directory")
    fi
  done
  if grep -Fq 'start_rosbridge:=true' /etc/default/lekiwi-stack &&
     grep -Fq 'rosbridge_address:=' /etc/default/lekiwi-stack; then
    installer_args+=(--rosbridge-tailnet)
  fi
  # The stack is stopped here; preserve its optional tailnet access on migration.
  LEKIWI_ROBOT_HOST=${device#*@} LEKIWI_WS=$compute_current \
    "$compute_current/source/scripts/reinstall-compute.sh" "${installer_args[@]}"
}
compute_configuration_current() {
  grep -Fq "remote_ip:=$device_address" /etc/default/lekiwi-stack &&
    grep -Fq 'laser_source:=ld06 lidar_source:=remote' /etc/default/lekiwi-stack &&
    [[ $(cat "$service_marker" 2>/dev/null || true) == "$expected_service_fingerprint" &&
       $(systemctl show -P WorkingDirectory lekiwi-stack.service) == "$workspace/current/source" ]]
}
compute_tls_current() {
  local file
  for file in ca.crt compute.crt compute.key; do
    [[ -r /etc/lekiwi/zenoh-tls/$file ]] || return 1
  done
  for file in ca.crt device.crt device.key; do
    "${ssh_command[@]}" test -r "/etc/lekiwi/zenoh-tls/$file" || return 1
  done
}
verify_compute_configuration() {
  compute_configuration_current || die "compute service configuration did not refresh"
  compute_tls_current || die "zenoh TLS identities did not refresh"
}
# The device installer skips Astra and the cameras when their ROS package is
# missing, so those two are deployed only where they are installed.
device_units=(lekiwi-host.service lekiwi-lidar.service lekiwi-zenoh.service)
has_device_unit() { [[ " ${device_units[*]} " == *" $1 "* ]]; }

log "Preflighting source revisions and deployment permissions"
require_clean "$project_root" "local repository"
branch=$(git -C "$project_root" symbolic-ref --quiet --short HEAD) || die "local repository must be on a branch"
timeout 30 git -C "$project_root" fetch --quiet origin || die "cannot fetch origin within 30 seconds"
upstream=$(git -C "$project_root" rev-parse --verify '@{upstream}') || die "$branch has no upstream"
target=$(git -C "$project_root" rev-parse HEAD)
[[ $target == "$upstream" ]] || die "local HEAD is not the pushed upstream revision"

"${ssh_command[@]}" git -C "$remote_repo" rev-parse --git-dir >/dev/null || die "remote repository not found: $remote_repo"
[[ -z $("${ssh_command[@]}" git -C "$remote_repo" status --porcelain) ]] || \
  die "remote repository has uncommitted or untracked files"
if ! "${ssh_command[@]}" timeout 30 git -C "$remote_repo" fetch --quiet origin; then
  log "Device origin fetch failed; transferring the pushed revision over SSH"
  transfer_device_revision
fi
"${ssh_command[@]}" git -C "$remote_repo" cat-file -e "$target^{commit}" || \
  die "device does not have revision $target"
compute_release=$workspace/releases/$target
device_release=$remote_workspace/releases/$target
compute_current=$workspace/current
device_current=$remote_workspace/current
[[ ! -e $compute_current || -L $compute_current ]] || die "release pointer is not a symlink: $compute_current"
"${ssh_command[@]}" "test ! -e '$device_current' || test -L '$device_current'" || \
  die "device release pointer is not a symlink"

[[ -d $workspace/install ]] || die "local workspace is not installed: $workspace"
"${ssh_command[@]}" test -d "$remote_workspace/install" || \
  die "device workspace is not installed: $remote_workspace"
/usr/bin/systemctl cat lekiwi-stack.service >/dev/null 2>&1 || die "lekiwi-stack.service is not installed"
remote_unit_exists lekiwi-host.service || die "lekiwi-host.service is not installed"
[[ $(systemctl show -P User lekiwi-stack.service) == "$(id -un)" ]] || \
  die "run deployment as the compute service account"
[[ $("${ssh_command[@]}" systemctl show -P User lekiwi-host.service) == $("${ssh_command[@]}" id -un) ]] || \
  die "connect as the device service account"
for unit in lekiwi-lidar.service lekiwi-zenoh.service; do
  remote_unit_exists "$unit" || \
    die "$unit is not installed; rerun scripts/install-device-services.sh on $device"
done
for unit in lekiwi-astra.service lekiwi-cameras.service; do
  if remote_unit_exists "$unit"; then device_units+=("$unit"); fi
done

expected_service_fingerprint=$(service_fingerprint compute) || die "cannot calculate service configuration fingerprint"
service_marker=$logs/service-fingerprint-compute
remote_service_marker=$remote_home/.ros/lekiwi/service-fingerprint-device
# A stale compute configuration is reinstalled only after the robot is disarmed and
# its stack stopped (below). Check sudo access before changing the robot state.
refresh_compute=false
if ! compute_configuration_current || ! compute_tls_current; then
  refresh_compute=true
  if ! sudo -n true 2>/dev/null; then
    [[ -t 0 ]] || die "compute service refresh needs an interactive terminal for sudo"
    sudo -v || die "compute service refresh needs sudo access"
  fi
fi

device_sudoers=$("${ssh_command[@]}" sudo -n -l) || \
  die "device sudoers grant is missing; rerun scripts/install-device-services.sh on $device"
remote_model=$("${ssh_command[@]}" 'tr -d "\0" </proc/device-tree/model 2>/dev/null || true')
remote_is_pi5=false
if [[ $remote_model == *"Raspberry Pi 5"* ]]; then
  remote_is_pi5=true
  if [[ $device_sudoers != *"/usr/local/sbin/lekiwi-enable-pi5-usb-current"* || \
        $device_sudoers != *"/usr/bin/systemctl reboot"* ]] || \
    ! "${ssh_command[@]}" cmp -s "$remote_repo/scripts/enable-pi5-usb-current.sh" \
      /usr/local/sbin/lekiwi-enable-pi5-usb-current; then
    bootstrap_ssh=("${ssh_command[@]}")
    bootstrap_sudo=(sudo -n)
    if ! "${ssh_command[@]}" sudo -n true 2>/dev/null; then
      [[ -t 0 ]] || die "Pi 5 privilege setup needs an interactive terminal for sudo"
      bootstrap_ssh=("${ssh_interactive[@]}")
      bootstrap_sudo=(sudo)
    fi
    log "Ensuring Pi 5 USB helper and deployment permissions (sudo may prompt)"
    "${bootstrap_ssh[@]}" "${bootstrap_sudo[*]} /usr/bin/install -o root -g root -m 0755 '$remote_repo/scripts/enable-pi5-usb-current.sh' /usr/local/sbin/lekiwi-enable-pi5-usb-current && ${bootstrap_sudo[*]} '$remote_repo/scripts/install-deploy-sudoers.sh' device --user \"\$(id -un)\"" || \
      die "could not install the Pi 5 USB helper and deployment permission"
    device_sudoers=$("${ssh_command[@]}" sudo -n -l) || die "Pi deployment sudo grant did not install"
    [[ $device_sudoers == *"/usr/local/sbin/lekiwi-enable-pi5-usb-current"* && \
       $device_sudoers == *"/usr/bin/systemctl reboot"* ]] || die "Pi deployment sudo grant is incomplete"
  fi
fi
if [[ $refresh_compute == false ]]; then
  compute_sudoers=$(sudo -n -l) || die "compute sudoers grant is missing; rerun scripts/install-compute-services.sh"
fi
for action in start stop reset-failed; do
  [[ $refresh_compute == true ]] || has_nopasswd_systemctl "$compute_sudoers" "$action" lekiwi-stack.service || \
    die "compute sudoers grant is missing; rerun scripts/install-compute-services.sh"
  for unit in "${device_units[@]}"; do
    has_nopasswd_systemctl "$device_sudoers" "$action" "$unit" || \
      die "device sudoers grant is missing; rerun scripts/install-device-services.sh on $device"
  done
done

[[ $refresh_compute == true ]] || verify_compute_configuration
pi_reboot_needed=false
if [[ $remote_is_pi5 == true ]]; then
  log "Persisting Pi 5 USB current setting (5 V / 5 A supply required)"
  "${ssh_command[@]}" sudo -n /usr/local/sbin/lekiwi-enable-pi5-usb-current
  if [[ $("${ssh_command[@]}" vcgencmd get_config usb_max_current_enable) != usb_max_current_enable=1 ]]; then
    pi_reboot_needed=true
  fi
fi
expected_device_service_fingerprint=$(service_fingerprint device) || die "cannot calculate device service configuration fingerprint"
refresh_device=false
if [[ $("${ssh_command[@]}" "cat '$remote_service_marker' 2>/dev/null || true") != "$expected_device_service_fingerprint" ||
      $("${ssh_command[@]}" systemctl show -P WorkingDirectory lekiwi-host.service) != "$device_current/source" ]]; then
  refresh_device=true
  if ! "${ssh_command[@]}" sudo -n true 2>/dev/null; then
    [[ -t 0 ]] || die "device service refresh needs an interactive terminal for sudo"
  fi
  remote_service_user=$("${ssh_command[@]}" systemctl show -P User lekiwi-host.service)
  [[ $remote_service_user =~ ^[a-z_][a-z0-9_-]*$ ]] || die "invalid device service user: $remote_service_user"
  read -r -a remote_environment <<<"$("${ssh_command[@]}" systemctl show -P Environment lekiwi-host.service)"
  remote_lerobot_venv=""
  remote_bind_address=0.0.0.0
  remote_curve_dir=""
  for setting in "${remote_environment[@]}"; do
    case $setting in
      LEKIWI_LEROBOT_VENV=*) remote_lerobot_venv=${setting#*=} ;;
      LEKIWI_BIND_ADDRESS=*) remote_bind_address=${setting#*=} ;;
      LEKIWI_CURVE_SERVER_SECRET=*)
        [[ -z ${setting#*=} ]] || remote_curve_dir=${setting#*=}
        ;;
    esac
  done
  [[ $remote_lerobot_venv =~ ^/[A-Za-z0-9._/-]+$ ]] || die "invalid device LeRobot venv path"
  [[ $remote_bind_address =~ ^[0-9.]+$ ]] || die "invalid device bind address"
  if [[ -n $remote_curve_dir ]]; then
    [[ $remote_curve_dir == */server.key_secret ]] || die "invalid device CURVE key path"
    remote_curve_dir=${remote_curve_dir%/server.key_secret}
    [[ $remote_curve_dir =~ ^/[A-Za-z0-9._/-]+$ ]] || die "invalid device CURVE directory"
  fi
fi

marker=$logs/deployed-revision
remote_marker=$remote_home/.ros/lekiwi/deployed-revision
verify_release() {
  [[ $(readlink -f "$compute_current") == "$compute_release" ]] &&
    [[ $("${ssh_command[@]}" readlink -f "$device_current") == "$device_release" ]] &&
    /usr/bin/python3 "$compute_release/source/scripts/check-release.py" verify "$compute_release" "$target" compute &&
    "${ssh_command[@]}" /usr/bin/python3 "$device_release/source/scripts/check-release.py" verify "$device_release" "$target" device
}
if [[ $refresh_compute == false && $refresh_device == false &&
      -L $compute_current && $(readlink -f "$compute_current") == "$compute_release" &&
      $("${ssh_command[@]}" readlink -f "$device_current") == "$device_release" &&
      $(cat "$marker" 2>/dev/null || true) == "$target" &&
      $("${ssh_command[@]}" "cat '$remote_marker' 2>/dev/null || true") == "$target" ]] &&
    /usr/bin/systemctl is-active --quiet lekiwi-stack.service &&
    remote_unit_active_all "${device_units[@]}" && verify_release; then
  echo "already deployed ${target:0:12}; services and sealed releases are current"
  exit 0
fi

/usr/bin/systemctl is-active --quiet lekiwi-stack.service || die "lekiwi-stack.service must be running before deployment"
remote_unit_active lekiwi-host.service || die "lekiwi-host.service must be running before deployment"

# Build and qualify persistent detached worktrees before touching live services.
# A staging failure leaves both the active source and install prefixes unchanged.
log "Staging revision ${target:0:12} on the device"
# The anchor checkout stays on its existing revision. Run the pushed staging tool
# from compute so initial migration does not require updating live device source.
"${ssh_command[@]}" nice -n 10 bash -s -- device "$remote_workspace" "$target" "$remote_repo" < "$project_root/scripts/stage-release.sh"
log "Staging revision ${target:0:12} on compute"
nice -n 10 "$project_root/scripts/stage-release.sh" compute "$workspace" "$target"
/usr/bin/python3 "$compute_release/source/scripts/check-release.py" verify "$compute_release" "$target" compute
"${ssh_command[@]}" /usr/bin/python3 "$device_release/source/scripts/check-release.py" verify "$device_release" "$target" device
load_lekiwi_env "$compute_release/source/.env"

activate_release() { # activate_release <workspace> <release>, only after services stop
  local root=$1 release=$2 previous temporary=$1/.current-$$
  [[ ! -e $root/previous || -L $root/previous ]] || return 1
  previous=$(readlink -f "$root/current" 2>/dev/null || true)
  if [[ -L $root/current && $previous != "$release" ]]; then
    ln -s "$previous" "$root/.previous-$$"
    mv -Tf "$root/.previous-$$" "$root/previous"
  fi
  ln -s "$release" "$temporary"
  mv -Tf "$temporary" "$root/current"
}

trap 'echo "$0: deployment failed; previous releases and bootstrap installation are retained; no automatic rollback or resume" >&2' EXIT

log "Confirming torque-off and stopping the compute stack"
ros_setup
disarm
# Older installed units broadcast SIGINT and interrupt RTAB-Map's save when
# launch forwards it again. Close their launcher before the first unit refresh.
if [[ $(systemctl show -P KillMode lekiwi-stack.service) != mixed ]]; then
  stack_pid=$(systemctl show -P MainPID lekiwi-stack.service)
  [[ $stack_pid =~ ^[1-9][0-9]*$ ]] || die "compute launcher PID is missing"
  kill -INT "$stack_pid"
  deadline=$((SECONDS + 45))
  while kill -0 "$stack_pid" 2>/dev/null; do
    (( SECONDS < deadline )) || die "compute launcher did not finish its database save"
    sleep 0.2
  done
fi
sudo -n /usr/bin/systemctl stop lekiwi-stack.service
if [[ $pi_reboot_needed == true ]]; then
  log "Rebooting the Pi to apply its USB current setting"
  "${ssh_command[@]}" sudo -n /usr/bin/systemctl reboot || true
  deadline=$((SECONDS + 60))
  until ! "${ssh_command[@]}" true >/dev/null 2>&1; do
    (( SECONDS < deadline )) || die "Pi did not begin rebooting"
    sleep 2
  done
  wait_for 180 "${ssh_command[@]}" true || die "Pi did not reconnect after reboot"
  [[ $("${ssh_command[@]}" vcgencmd get_config usb_max_current_enable) == usb_max_current_enable=1 ]] || \
    die "Pi USB current setting did not apply after reboot"
fi

log "Stopping device services"
for unit in lekiwi-cameras.service lekiwi-astra.service lekiwi-lidar.service lekiwi-zenoh.service lekiwi-host.service; do
  if has_device_unit "$unit"; then
    "${ssh_command[@]}" sudo -n /usr/bin/systemctl stop "$unit"
  fi
done

log "Selecting the qualified releases while both stacks are stopped"
activate_release "$workspace" "$compute_release"
"${ssh_command[@]}" bash -se < <(declare -f activate_release; printf 'activate_release %q %q\n' "$remote_workspace" "$device_release")
if [[ $refresh_compute == true ]]; then
  refresh_compute_service
  verify_compute_configuration
fi

if [[ $refresh_device == true ]]; then
  log "Refreshing stale device service configuration"
  remote_installer=("$device_current/source/scripts/install-device-services.sh" --service-user "$remote_service_user" \
    --workspace "$device_current" --lerobot-venv "$remote_lerobot_venv" \
    --bind-address "$remote_bind_address" --no-start)
  [[ -z $remote_curve_dir ]] || remote_installer+=(--curve-dir "$remote_curve_dir")
  if "${ssh_command[@]}" sudo -n true 2>/dev/null; then
    "${ssh_command[@]}" sudo -n "${remote_installer[@]}"
  else
    "${ssh_interactive[@]}" sudo "${remote_installer[@]}"
  fi
  [[ $("${ssh_command[@]}" "cat '$remote_service_marker' 2>/dev/null || true") == "$expected_device_service_fingerprint" ]] || \
    die "device service configuration did not refresh"
  for unit in lekiwi-astra.service lekiwi-cameras.service; do
    if remote_unit_exists "$unit" && ! has_device_unit "$unit"; then device_units+=("$unit"); fi
  done
fi

log "Starting and validating device services"
"${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed lekiwi-host.service
"${ssh_command[@]}" sudo -n /usr/bin/systemctl start lekiwi-host.service
remote_unit_active lekiwi-host.service || die "lekiwi-host.service did not become active"
for unit in lekiwi-astra.service lekiwi-cameras.service; do
  has_device_unit "$unit" || continue
  "${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed "$unit"
  "${ssh_command[@]}" sudo -n /usr/bin/systemctl start "$unit"
  remote_unit_active "$unit" || die "$unit did not become active"
done
"${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed lekiwi-lidar.service
"${ssh_command[@]}" sudo -n /usr/bin/systemctl start lekiwi-lidar.service
remote_unit_active lekiwi-lidar.service || die "lekiwi-lidar.service did not become active"
# Restart, not start: the bridge reads its allow-list from the source tree.
"${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed lekiwi-zenoh.service
"${ssh_command[@]}" sudo -n /usr/bin/systemctl restart lekiwi-zenoh.service
remote_unit_active lekiwi-zenoh.service || die "lekiwi-zenoh.service did not become active"

log "Starting the compute stack disarmed"
sudo -n /usr/bin/systemctl reset-failed lekiwi-stack.service
sudo -n /usr/bin/systemctl start lekiwi-stack.service
wait_for 120 /usr/bin/systemctl is-active --quiet lekiwi-stack.service || \
  die "lekiwi-stack.service did not become active"
ros_setup
wait_for 120 ros2 service type /safety/disarm || die "updated driver did not appear"
disarm
driver_state=$(timeout 10 ros2 topic echo --once /safety/driver_state --field data | tr -d "'[:space:]-")
[[ $driver_state == DISARMED ]] || die "updated driver is not disarmed: $driver_state"
wait_for 30 sh -c "ros2 topic info /hardware/diagnostics | grep -Eq 'Publisher count: [1-9]'" || \
  die "updated driver is not publishing motor health"
if ! has_device_unit lekiwi-cameras.service; then
  log "lekiwi-cameras.service is not installed on the device; skipping the camera check"
elif remote_front_camera_present; then
  timeout 30 ros2 topic echo --once /pi/camera/front/image_raw/compressed >/dev/null || \
    die "attached front camera did not reach compute"
else
  log "No front camera is attached; its independent service remains waiting"
fi
if has_device_unit lekiwi-astra.service; then
  timeout 30 ros2 topic echo --once /camera/depth/points >/dev/null || \
    die "device Astra point cloud did not reach compute"
else
  log "lekiwi-astra.service is not installed on the device; skipping the point-cloud check"
fi
lidar_frame=$(timeout 30 ros2 topic echo --once --field header.frame_id /scan | \
  awk 'NF && $1 != "---" { print $1; exit }') || \
  die "device LD06 scan did not reach compute"
[[ $lidar_frame == laser ]] || die "canonical /scan is not the LD06 frame: $lidar_frame"

verify_release || die "qualified release changed during deployment"
log "Recording the verified deployment revision"
printf '%s\n' "$target" > "$marker"
"${ssh_command[@]}" "mkdir -p '$remote_home/.ros/lekiwi' && printf '%s\\n' '$target' > '$remote_marker'"
trap - EXIT
# The verification above left the driver deliberately disarmed. A domestic robot stays
# armed, so arm it again; only LEKIWI_DISARM_ON_FAILURE=true keeps it disarmed for an operator.
outcome="robot remains disarmed"
if [[ ${LEKIWI_DISARM_ON_FAILURE:-false} != true ]]; then
  if wait_for 60 sh -c "ros2 service call /safety/arm std_srvs/srv/Trigger '{}' | grep -q 'success=True'"; then
    outcome="robot armed"
  else
    outcome="robot could not be armed (run: ros2 service call /safety/arm std_srvs/srv/Trigger '{}')"
  fi
fi
echo "deployed ${target:0:12} to compute and $device; $outcome"
exit
}
