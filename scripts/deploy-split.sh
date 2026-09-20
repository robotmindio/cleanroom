#!/usr/bin/env bash
# Deploy one pushed revision to the compute machine and its remote device host.
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
# shellcheck disable=SC2016 # HOME must expand on the device, not compute.
remote_home=$("${ssh_command[@]}" 'printf %s "$HOME"') || die "cannot reach $device with key-based SSH"
: "${remote_repo:=$remote_home/cleanroom}"
: "${remote_workspace:=$remote_home/lekiwi_ws}"
for path in "$workspace" "$remote_home" "$remote_repo" "$remote_workspace"; do
  [[ $path =~ ^/[A-Za-z0-9._/-]+$ ]] || die "deployment paths must be absolute and contain no whitespace: $path"
done

require_clean() { # require_clean <repository> [description]
  local repository=$1 description=${2:-$1}
  [[ -z $(git -C "$repository" status --porcelain) ]] || die "$description has uncommitted or untracked files"
}
ros_setup() {
  export LEKIWI_WS=$workspace
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
  LEKIWI_ROBOT_HOST=${device#*@} LEKIWI_WS=$workspace \
    "$project_root/scripts/reinstall-compute.sh"
}
# The device installer skips Astra and the cameras when their ROS package is
# missing, so those two are deployed only where they are installed.
device_units=(lekiwi-host.service lekiwi-lidar.service lekiwi-zenoh.service)
has_device_unit() { [[ " ${device_units[*]} " == *" $1 "* ]]; }

log "Preflighting source revisions and deployment permissions"
require_clean "$project_root" "local repository"
branch=$(git symbolic-ref --quiet --short HEAD) || die "local repository must be on a branch"
timeout 30 git fetch --quiet origin || die "cannot fetch origin within 30 seconds"
upstream=$(git rev-parse --verify '@{upstream}') || die "$branch has no upstream"
before=$(git rev-parse HEAD)
git merge --ff-only "$upstream"
if [[ $before != $(git rev-parse HEAD) && ${LEKIWI_DEPLOY_REFRESHED:-} != 1 ]]; then
  export LEKIWI_DEPLOY_REFRESHED=1
  exec "$project_root/scripts/deploy-split.sh" "${original_args[@]}"
fi
require_clean "$project_root" "updated local repository"
target=$(git rev-parse HEAD)
[[ $target == $(git rev-parse '@{upstream}') ]] || die "local HEAD is not the pushed upstream revision"

"${ssh_command[@]}" test -d "$remote_repo/.git" || die "remote repository not found: $remote_repo"
[[ -z $("${ssh_command[@]}" git -C "$remote_repo" status --porcelain) ]] || \
  die "remote repository has uncommitted or untracked files"
remote_branch=$("${ssh_command[@]}" git -C "$remote_repo" symbolic-ref --quiet --short HEAD) || \
  die "remote repository must be on a branch"
[[ $remote_branch == "$branch" ]] || die "branch mismatch: local $branch, device $remote_branch"
"${ssh_command[@]}" timeout 30 git -C "$remote_repo" fetch --quiet origin || \
  die "device cannot fetch origin within 30 seconds"
"${ssh_command[@]}" git -C "$remote_repo" merge --ff-only "$target"
[[ $("${ssh_command[@]}" git -C "$remote_repo" rev-parse HEAD) == "$target" ]] || \
  die "device did not reach revision $target"

[[ -d $workspace/install ]] || die "local workspace is not installed: $workspace"
"${ssh_command[@]}" test -d "$remote_workspace/install" || \
  die "device workspace is not installed: $remote_workspace"
/usr/bin/systemctl cat lekiwi-stack.service >/dev/null 2>&1 || die "lekiwi-stack.service is not installed"
remote_unit_exists lekiwi-host.service || die "lekiwi-host.service is not installed"
for unit in lekiwi-lidar.service lekiwi-zenoh.service; do
  remote_unit_exists "$unit" || \
    die "$unit is not installed; rerun scripts/install-device-services.sh on $device"
done
for unit in lekiwi-astra.service lekiwi-cameras.service; do
  if remote_unit_exists "$unit"; then device_units+=("$unit"); fi
done
if ! grep -Fq "remote_ip:=$device_address" /etc/default/lekiwi-stack \
  || ! grep -Fq 'laser_source:=ld06 lidar_source:=remote' /etc/default/lekiwi-stack; then
  refresh_compute_service
fi
if ! grep -Fq "remote_ip:=$device_address" /etc/default/lekiwi-stack \
  || ! grep -Fq 'laser_source:=ld06 lidar_source:=remote' /etc/default/lekiwi-stack; then
  die "compute service configuration did not refresh"
fi

compute_sudoers=$(sudo -n -l) || die "compute sudoers grant is missing; rerun scripts/install-compute-services.sh"
device_sudoers=$("${ssh_command[@]}" sudo -n -l) || \
  die "device sudoers grant is missing; rerun scripts/install-device-services.sh on $device"
for action in start stop reset-failed; do
  has_nopasswd_systemctl "$compute_sudoers" "$action" lekiwi-stack.service || \
    die "compute sudoers grant is missing; rerun scripts/install-compute-services.sh"
  for unit in "${device_units[@]}"; do
    has_nopasswd_systemctl "$device_sudoers" "$action" "$unit" || \
      die "device sudoers grant is missing; rerun scripts/install-device-services.sh on $device"
  done
done

expected_service_fingerprint=$(service_fingerprint compute) || die "cannot calculate service configuration fingerprint"
service_marker=$logs/service-fingerprint-compute
remote_service_marker=$remote_home/.ros/lekiwi/service-fingerprint-device
if [[ $(cat "$service_marker" 2>/dev/null || true) != "$expected_service_fingerprint" ]]; then
  refresh_compute_service
fi
[[ $(cat "$service_marker" 2>/dev/null || true) == "$expected_service_fingerprint" ]] || \
  die "compute service configuration did not refresh"
# The sensor bridge is mutual TLS; a missing identity would leave the robot without sensors.
for file in ca.crt compute.crt compute.key; do
  [[ -r /etc/lekiwi/zenoh-tls/$file ]] || die "missing /etc/lekiwi/zenoh-tls/$file; run scripts/reinstall-compute.sh"
done
for file in ca.crt device.crt device.key; do
  "${ssh_command[@]}" test -r "/etc/lekiwi/zenoh-tls/$file" || \
    die "missing /etc/lekiwi/zenoh-tls/$file on $device; run scripts/reinstall-compute.sh"
done
expected_device_service_fingerprint=$(service_fingerprint device) || die "cannot calculate device service configuration fingerprint"
[[ $("${ssh_command[@]}" "cat '$remote_service_marker' 2>/dev/null || true") == "$expected_device_service_fingerprint" ]] || \
  die "device service configuration is stale; rerun scripts/install-device-services.sh on $device"

marker=$logs/deployed-revision
remote_marker=$remote_home/.ros/lekiwi/deployed-revision
workspace_revision() { cat "$1/install/lekiwi_rmf/.lekiwi-source-revision" 2>/dev/null || true; }
remote_workspace_revision() {
  "${ssh_command[@]}" "cat '$remote_workspace/install/lekiwi_rmf/.lekiwi-source-revision' 2>/dev/null || true"
}
if [[ $(cat "$marker" 2>/dev/null || true) == "$target" && \
      $("${ssh_command[@]}" "cat '$remote_marker' 2>/dev/null || true") == "$target" && \
      $(workspace_revision "$workspace") == "$target" && \
      $(remote_workspace_revision) == "$target" ]] && \
    /usr/bin/systemctl is-active --quiet lekiwi-stack.service && \
    remote_unit_active_all "${device_units[@]}"; then
  echo "already deployed ${target:0:12}; services and both workspaces are current"
  exit 0
fi

/usr/bin/systemctl is-active --quiet lekiwi-stack.service || die "lekiwi-stack.service must be running before deployment"
remote_unit_active lekiwi-host.service || die "lekiwi-host.service must be running before deployment"

on_exit() {
  local code=$?
  echo "$0: deployment stopped safely; services are not automatically rolled back or resumed" >&2
  return "$code"
}
trap on_exit EXIT

log "Confirming torque-off and stopping the compute stack"
ros_setup
disarm
sudo -n /usr/bin/systemctl stop lekiwi-stack.service

log "Stopping device services"
if remote_unit_active lekiwi-cameras.service; then
  "${ssh_command[@]}" sudo -n /usr/bin/systemctl stop lekiwi-cameras.service
fi
if remote_unit_active lekiwi-lidar.service; then
  "${ssh_command[@]}" sudo -n /usr/bin/systemctl stop lekiwi-lidar.service
fi
"${ssh_command[@]}" sudo -n /usr/bin/systemctl stop lekiwi-host.service

log "Building revision ${target:0:12} on the device"
"${ssh_command[@]}" env LEKIWI_WS="$remote_workspace" "$remote_repo/scripts/build-lekiwi.sh"

log "Building revision ${target:0:12} on compute"
LEKIWI_WS=$workspace "$project_root/scripts/build-lekiwi.sh"

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

log "Recording the verified deployment revision"
printf '%s\n' "$target" > "$marker"
"${ssh_command[@]}" "mkdir -p '$remote_home/.ros/lekiwi' && printf '%s\\n' '$target' > '$remote_marker'"
[[ $("${ssh_command[@]}" git -C "$remote_repo" rev-parse HEAD) == "$target" ]] || \
  die "device revision changed during deployment"
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
