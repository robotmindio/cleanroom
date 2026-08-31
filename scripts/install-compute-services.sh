#!/usr/bin/env bash
# Install the compute-side boot service (lekiwi-stack.service): the ROS
# bringup -- Nav2, RTAB-Map, RMF, rosbridge -- wherever it should run. The
# robot's own computer and a separate desk machine are both fine, and the
# device half (motors + cameras) can sit on either one.
#
# Usage: scripts/install-compute-services.sh [--remote DEVICE_ADDR]
#        [--service-user USER] [--workspace PATH] [--curve-dir PATH]
#   no --remote and no LEKIWI_ROBOT_HOST in .env: the device side runs here too; the stack is
#                 ordered after lekiwi-host.service and starts once its ZMQ
#                 port answers (camera_source:=local).
#   --remote    : motors and cameras live on DEVICE_ADDR instead; compressed
#                 frames stream from there and relays in the bringup expand
#                 them into the same canonical topics. The device-side LD06
#                 service is relayed by default too.
#
# Re-run any time the split changes; both installers are idempotent.
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system
# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/runtime-common.sh"
load_lekiwi_env "$PROJECT_ROOT/.env"

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

REMOTE=${LEKIWI_ROBOT_HOST:-}
SERVICE_USER_ARG=""
WORKSPACE_ARG=""
CURVE_DIR_ARG=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --remote)
      [[ $# -ge 2 ]] || die "--remote needs a device address"
      REMOTE=$2
      shift 2
      ;;
    --service-user)
      [[ $# -ge 2 ]] || die "--service-user needs a user name"
      SERVICE_USER_ARG=$2
      shift 2
      ;;
    --workspace)
      [[ $# -ge 2 ]] || die "--workspace needs an absolute path"
      WORKSPACE_ARG=$2
      shift 2
      ;;
    --curve-dir)
      [[ $# -ge 2 ]] || die "--curve-dir needs an absolute key directory"
      CURVE_DIR_ARG=$2
      shift 2
      ;;
    *) die "unknown argument: $1 (usage: $0 [--remote DEVICE_ADDR] [--service-user USER] [--workspace PATH] [--curve-dir PATH])" ;;
  esac
done
[[ -z $REMOTE || $REMOTE =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
  die "--remote must be a hostname or IPv4 address without whitespace"

# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/service-install-common.sh"
# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/service-install-revision.sh"
resolve_service_user "$SERVICE_USER_ARG"
resolve_service_paths "$WORKSPACE_ARG" "" true false

install_unit() { render_systemd_unit "$PROJECT_ROOT/systemd/$1" "$UNIT_DIR/$1"; }

host_unit="$UNIT_DIR/lekiwi-host.service"
cameras_unit="$UNIT_DIR/lekiwi-cameras.service"
lidar_unit="$UNIT_DIR/lekiwi-lidar.service"
topology_dir="$UNIT_DIR/lekiwi-stack.service.d"
topology_conf="$topology_dir/topology.conf"

if [[ -n $REMOTE ]]; then
  # There is no local host to depend on in this topology.
  log "Remote topology: stack reaches the host at $REMOTE over the network"
  if [[ -e $topology_conf ]]; then
    as_root rm -f "$topology_conf"
  fi
  if [[ -f $host_unit ]]; then
    log "warning: device services are also installed on this machine -- an"
    log "unusual split. If the devices are actually here, drop --remote."
  fi
  STACK_ARGS="camera_source:=remote remote_ip:=$REMOTE laser_source:=ld06 lidar_source:=remote"
  if [[ -n $CURVE_DIR_ARG ]]; then
    [[ $CURVE_DIR_ARG == /* && $CURVE_DIR_ARG != *[[:space:]]* ]] || \
      die "--curve-dir must be an absolute path without whitespace"
    curve_client_secret="$CURVE_DIR_ARG/clients/driver.key_secret"
    curve_server_public="$CURVE_DIR_ARG/server.key"
    [[ -f $curve_client_secret && -f $curve_server_public ]] || \
      die "--curve-dir does not contain clients/driver.key_secret and server.key"
    STACK_ARGS="$STACK_ARGS curve_client_secret_key_file:=$curve_client_secret curve_server_public_key_file:=$curve_server_public"
  fi
else
  # All-in-one: make the host and stack one lifecycle group.
  as_root mkdir -p "$topology_dir"
  printf '%s\n' "[Unit]" "Requires=lekiwi-host.service" \
    "PartOf=lekiwi-host.service" "After=lekiwi-host.service" |
    as_root tee "$topology_conf" >/dev/null
  if [[ -f $cameras_unit ]]; then
    # The camera publisher service owns this machine's USB cameras. Letting the
    # stack open them again would fight it for the devices (v4l2 allows one
    # reader), so the stack takes the same compressed stream a remote machine
    # would -- over loopback.
    log "Camera publisher service found here: stack takes its frames over"
    log "loopback, so each USB device keeps exactly one reader."
    STACK_ARGS="camera_source:=remote remote_ip:=127.0.0.1 laser_source:=ld06 lidar_source:=remote"
  else
    if [[ ! -f $host_unit ]]; then
      log "No device services here yet. If the motors and cameras end up on"
      log "this machine, also run: scripts/install-device-services.sh"
      log "Until something answers on :5555 the stack will keep retrying."
    fi
    # The standard device installer owns the LD06 serial port through its
    # service, even when this compute stack shares the same machine.
    [[ -f $lidar_unit ]] && STACK_ARGS="laser_source:=ld06 lidar_source:=remote" || STACK_ARGS=""
  fi
  if [[ -n $CURVE_DIR_ARG ]]; then
    [[ $CURVE_DIR_ARG == /* && $CURVE_DIR_ARG != *[[:space:]]* ]] || \
      die "--curve-dir must be an absolute path without whitespace"
    curve_client_secret="$CURVE_DIR_ARG/clients/driver.key_secret"
    curve_server_public="$CURVE_DIR_ARG/server.key"
    [[ -f $curve_client_secret && -f $curve_server_public ]] || \
      die "--curve-dir does not contain clients/driver.key_secret and server.key"
    STACK_ARGS="${STACK_ARGS:+$STACK_ARGS }curve_client_secret_key_file:=$curve_client_secret curve_server_public_key_file:=$curve_server_public"
  fi
fi

if [[ -n $REMOTE ]]; then
  log "Installing lekiwi-stack.service (--remote $REMOTE, device LD06)"
elif [[ $STACK_ARGS == *remote* ]]; then
  log "Installing lekiwi-stack.service (loopback camera_source:=remote)"
else
  log "Installing lekiwi-stack.service (local cameras)"
fi
install_unit lekiwi-stack.service
log "Validating rendered systemd unit"
verify_systemd_units lekiwi-stack.service

stack_env=/etc/default/lekiwi-stack
printf '# Written by scripts/install-compute-services.sh.\nLEKIWI_STACK_ARGS=%s\n' \
  "$STACK_ARGS" | as_root tee "$stack_env" >/dev/null

log "Reloading systemd and enabling lekiwi-stack.service"
as_root systemctl daemon-reload
as_root systemctl enable --now lekiwi-stack.service

log "Granting $LEKIWI_SERVICE_USER non-interactive deployment control"
as_root "$PROJECT_ROOT/scripts/install-deploy-sudoers.sh" compute --user "$LEKIWI_SERVICE_USER"
record_service_fingerprint compute

cat <<EOF
Done. Check on it with:
  systemctl status lekiwi-stack.service
  journalctl -u lekiwi-stack.service -f

Extra launch arguments (slam_mode:=localization and friends) go into
$(printf %q "$stack_env") as LEKIWI_STACK_ARGS, then:
  sudo systemctl restart lekiwi-stack.service
EOF
if [[ -n $REMOTE ]]; then
  cat <<EOF
Camera calibration lives on the device machine (ros-cameras.sh refuses to
publish without it) and crosses the network with the frames, so nothing
camera-related is needed on this one.
EOF
fi
