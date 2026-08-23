#!/usr/bin/env bash
# Install the compute-side boot service (lekiwi-stack.service): the ROS
# bringup -- Nav2, RTAB-Map, RMF, rosbridge -- wherever it should run. The
# robot's own computer and a separate desk machine are both fine, and the
# device half (motors + cameras) can sit on either one.
#
# Usage: scripts/install-compute-services.sh [--remote DEVICE_ADDR]
#   no --remote : the device side runs on this machine too; the stack is
#                 ordered after lekiwi-host.service and starts once its ZMQ
#                 port answers (camera_source:=local).
#   --remote    : motors and cameras live on DEVICE_ADDR instead; compressed
#                 frames stream from there and relays in the bringup expand
#                 them into the same canonical topics (camera_source:=remote).
#
# Re-run any time the split changes; both installers are idempotent.
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

REMOTE=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --remote)
      [[ $# -ge 2 ]] || die "--remote needs a device address"
      REMOTE=$2
      shift 2
      ;;
    *) die "unknown argument: $1 (usage: $0 [--remote DEVICE_ADDR])" ;;
  esac
done
[[ -z $REMOTE || $REMOTE != -* ]] || die "--remote looks like a flag: $REMOTE"

WORKSPACE="${LEKIWI_WS:-$HOME/lekiwi_ws}"
[[ -d $WORKSPACE/install ]] || die "no ROS workspace at $WORKSPACE -- run scripts/install.sh on this machine first"

SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)
as_root() { # as_root <command...>
  [[ $EUID -eq 0 ]] || command -v sudo >/dev/null || die "sudo is required to $1"
  "${SUDO[@]}" "$@"
}

install_unit() { # install_unit <name>
  sed -e "s|@PROJECT_ROOT@|$PROJECT_ROOT|g" -e "s|@SERVICE_USER@|$(id -un)|g" \
    "$PROJECT_ROOT/systemd/$1" | as_root tee "$UNIT_DIR/$1" >/dev/null
}

host_unit="$UNIT_DIR/lekiwi-host.service"
topology_dir="$UNIT_DIR/lekiwi-stack.service.d"
topology_conf="$topology_dir/topology.conf"

if [[ -n $REMOTE ]]; then
  # There is no local host to depend on: clear the base unit's Requires,
  # PartOf and After so systemd does not wait for (or try to pull in) a unit
  # that may not even exist here.
  log "Remote topology: stack reaches the host at $REMOTE over the network"
  as_root mkdir -p "$topology_dir"
  printf '%s\n' "[Unit]" "Requires=" "PartOf=" "After=" |
    as_root tee "$topology_conf" >/dev/null
  if [[ -f $host_unit ]]; then
    log "warning: device services are also installed on this machine -- an"
    log "unusual split. If the devices are actually here, drop --remote."
  fi
else
  # All-in-one: restore the base dependency set in case a previous remote
  # configuration left the drop-in behind.
  if [[ -e $topology_conf ]]; then
    log "Removing stale topology override $(printf %q "$topology_conf")"
    as_root rm -f "$topology_conf"
  fi
  if [[ ! -f $host_unit ]]; then
    log "No device services here yet. If the motors and cameras end up on"
    log "this machine, also run: scripts/install-device-services.sh"
    log "Until something answers on :5555 the stack will keep retrying."
  fi
fi

if [[ -n $REMOTE ]]; then
  log "Installing lekiwi-stack.service (camera_source:=remote remote_ip:=$REMOTE)"
else
  log "Installing lekiwi-stack.service (local cameras)"
fi
install_unit lekiwi-stack.service

stack_env=/etc/default/lekiwi-stack
{
  printf '# Written by scripts/install-compute-services.sh.\n'
  if [[ -n $REMOTE ]]; then
    printf 'LEKIWI_STACK_ARGS=camera_source:=remote remote_ip:=%s\n' "$REMOTE"
  else
    printf 'LEKIWI_STACK_ARGS=\n'
  fi
} | as_root tee "$stack_env" >/dev/null

log "Reloading systemd and enabling lekiwi-stack.service"
as_root systemctl daemon-reload
as_root systemctl enable --now lekiwi-stack.service

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
