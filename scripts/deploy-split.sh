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
device=${LEKIWI_ROBOT_HOST:-}
if [[ $# -gt 0 && $1 != --* ]]; then
  device=$1
  shift
elif [[ -z $device && -r $project_root/.env ]]; then
  mapfile -t configured_hosts < <(sed -n 's/^LEKIWI_ROBOT_HOST=//p' "$project_root/.env")
  [[ ${#configured_hosts[@]} -eq 1 ]] ||
    die ".env must contain exactly one LEKIWI_ROBOT_HOST"
  device=${configured_hosts[0]}
fi
[[ -n $device ]] || die "set LEKIWI_ROBOT_HOST in $project_root/.env or pass [USER@]DEVICE"
[[ $device =~ ^([a-z_][a-z0-9_-]*@)?[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
  die "device must be a hostname or address, optionally prefixed by USER@"

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

cd "$project_root"
PROJECT_ROOT=$project_root
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
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
}
disarm() {
  local response
  wait_for 30 ros2 service type /safety/disarm || die "/safety/disarm is unavailable"
  response=$(timeout 20 ros2 service call /safety/disarm std_srvs/srv/Trigger '{}') || \
    die "disarm request did not complete"
  [[ $response == *"success=True"* ]] || die "motor host did not confirm torque-off: $response"
}
remote_unit_exists() {
  "${ssh_command[@]}" /usr/bin/systemctl cat "$1" >/dev/null 2>&1
}
remote_unit_active() {
  "${ssh_command[@]}" /usr/bin/systemctl is-active --quiet "$1"
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
  touch "$logs/deploy-inhibit-auto-arm"
  LEKIWI_ROBOT_HOST=${device#*@} LEKIWI_WS=$workspace \
    "$project_root/scripts/reinstall-compute.sh"
}
device_units=(lekiwi-host.service lekiwi-astra.service lekiwi-cameras.service lekiwi-lidar.service)

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
for unit in lekiwi-astra.service lekiwi-cameras.service lekiwi-lidar.service; do
  remote_unit_exists "$unit" || \
    die "$unit is not installed; rerun scripts/install-device-services.sh on $device"
done
if ! grep -Fq 'laser_source:=ld06 lidar_source:=remote' /etc/default/lekiwi-stack; then
  refresh_compute_service
fi
grep -Fq 'laser_source:=ld06 lidar_source:=remote' /etc/default/lekiwi-stack || \
  die "compute service configuration did not refresh"

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
    remote_unit_active lekiwi-host.service && \
    remote_unit_active lekiwi-astra.service && \
    remote_unit_active lekiwi-cameras.service && \
    remote_unit_active lekiwi-lidar.service; then
  echo "already deployed ${target:0:12}; services and both workspaces are current"
  exit 0
fi

/usr/bin/systemctl is-active --quiet lekiwi-stack.service || die "lekiwi-stack.service must be running before deployment"
remote_unit_active lekiwi-host.service || die "lekiwi-host.service must be running before deployment"

deploy_inhibit=$logs/deploy-inhibit-auto-arm
touch "$deploy_inhibit"
completed=false
on_exit() {
  local code=$?
  if [[ $completed != true ]]; then
    echo "$0: deployment stopped safely; services are not automatically rolled back or resumed" >&2
    echo "$0: auto-arm remains inhibited by $deploy_inhibit" >&2
  fi
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
"${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed lekiwi-astra.service
"${ssh_command[@]}" sudo -n /usr/bin/systemctl start lekiwi-astra.service
remote_unit_active lekiwi-astra.service || die "lekiwi-astra.service did not become active"
"${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed lekiwi-cameras.service
"${ssh_command[@]}" sudo -n /usr/bin/systemctl start lekiwi-cameras.service
remote_unit_active lekiwi-cameras.service || die "lekiwi-cameras.service did not become active"
"${ssh_command[@]}" sudo -n /usr/bin/systemctl reset-failed lekiwi-lidar.service
"${ssh_command[@]}" sudo -n /usr/bin/systemctl start lekiwi-lidar.service
remote_unit_active lekiwi-lidar.service || die "lekiwi-lidar.service did not become active"

log "Starting the compute stack with auto-arm inhibited"
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
if remote_front_camera_present; then
  timeout 30 ros2 topic echo --once /pi/camera/front/image_raw/compressed >/dev/null || \
    die "attached front camera did not reach compute"
else
  log "No front camera is attached; its independent service remains waiting"
fi
timeout 30 ros2 topic echo --once /camera/depth/points >/dev/null || \
  die "device Astra point cloud did not reach compute"
lidar_frame=$(timeout 30 ros2 topic echo --once --field header.frame_id /scan | \
  awk 'NF && $1 != "---" { print $1; exit }') || \
  die "device LD06 scan did not reach compute"
[[ $lidar_frame == laser ]] || die "canonical /scan is not the LD06 frame: $lidar_frame"

log "Recording the verified deployment revision"
printf '%s\n' "$target" > "$marker"
"${ssh_command[@]}" "mkdir -p '$remote_home/.ros/lekiwi' && printf '%s\\n' '$target' > '$remote_marker'"
[[ $("${ssh_command[@]}" git -C "$remote_repo" rev-parse HEAD) == "$target" ]] || \
  die "device revision changed during deployment"
rm -f -- "$deploy_inhibit"
completed=true
trap - EXIT
echo "deployed ${target:0:12} to compute and $device; robot remains disarmed"
