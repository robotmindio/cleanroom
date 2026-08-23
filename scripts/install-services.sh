#!/usr/bin/env bash
# Install the boot services: lekiwi-host everywhere, plus lekiwi-stack on a
# machine with a ROS workspace -- the service-shaped versions of
# scripts/pi-up.sh and scripts/up.sh. RViz stays manual (scripts/rviz.sh);
# it needs a desktop session, which a system service must not assume.
# Usage: scripts/install-services.sh [--now]
#   --now  also start the units immediately, not just at the next boot
#
# The role is detected like up.sh draws it: a ROS workspace here means this
# machine reads the cameras itself through ROS, so the host runs without them
# (LEKIWI_HOST_ARGS=--no-cameras); anywhere else is a robot Pi running the
# full host. Override either side later in /etc/default/lekiwi-host or
# /etc/default/lekiwi-stack.
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

[[ ${1:-} == "" || ${1:-} == --now ]] || die "unknown argument: $1 (usage: $0 [--now])"
START_NOW=false
[[ ${1:-} == --now ]] && START_NOW=true

WORKSPACE="${LEKIWI_WS:-$HOME/lekiwi_ws}"
if [[ -d $WORKSPACE/install ]]; then
  stack=true
  host_args="--no-cameras"
else
  stack=false
  host_args=""
fi

SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)
as_root() { # as_root <command...>
  [[ $EUID -eq 0 ]] || command -v sudo >/dev/null || die "sudo is required to $1"
  "${SUDO[@]}" "$@"
}

install_unit() { # install_unit <name>
  local name=$1 target="$UNIT_DIR/$1"
  sed -e "s|@PROJECT_ROOT@|$PROJECT_ROOT|g" -e "s|@SERVICE_USER@|$(id -un)|g" \
    "$PROJECT_ROOT/systemd/$name" | as_root tee "$target" >/dev/null
}

log "Installing lekiwi-host.service (host args: '${host_args:-none}')"
install_unit lekiwi-host.service
default_file=/etc/default/lekiwi-host
if [[ -n $host_args ]]; then
  printf 'LEKIWI_HOST_ARGS=%s\n' "$host_args" | as_root "write $default_file" tee "$default_file" >/dev/null
elif [[ -f $default_file ]] && grep -q '^LEKIWI_HOST_ARGS=' "$default_file"; then
  log "Removing stale $(printf %q "$default_file") override"
  as_root rm -f "$default_file"
fi

units=(lekiwi-host.service)
if "$stack"; then
  log "Installing lekiwi-stack.service (ROS workspace found in $(printf %q "$WORKSPACE"))"
  install_unit lekiwi-stack.service
  units+=(lekiwi-stack.service)
else
  log "No ROS workspace -- skipping lekiwi-stack.service (this machine is a robot Pi)"
fi

log "Reloading systemd and enabling: ${units[*]}"
as_root systemctl daemon-reload
as_root systemctl enable "${units[@]}"
if "$START_NOW"; then
  as_root systemctl start "${units[@]}"
fi

cat <<EOF
Done. The unit files live in /etc/systemd/system; edit
$(printf %q "$PROJECT_ROOT")/systemd/*.service to change them, not the copies.

Check on them:
  systemctl status ${units[*]}
  journalctl -u lekiwi-host.service -f

Start now without rebooting:
  sudo systemctl start ${units[*]}

Notes:
- One-time setup from HARDWARE.md (motors calibrated) has to exist already;
  the service cannot run the interactive calibration for you.
- scripts/ros-stop.sh knows nothing about systemd and would fight the
  restart policy. Stop these with 'sudo systemctl stop' instead, or disable
  them before going back to the manual scripts.
EOF
"$stack" || cat <<'EOF'
- The workstation talks to this host over ZMQ :5555; give it this machine's
  address (hostname -I).
EOF
