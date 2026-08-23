#!/usr/bin/env bash
# Install the device-side boot services on whatever machine the robot's USB
# devices are plugged into -- a Raspberry Pi, a NUC, anything with the
# hardware. The names follow the hardware, not the board:
#
#   lekiwi-host.service     the LeRobot motor bus, served on ZMQ :5555
#   lekiwi-cameras.service  v4l2_camera publishers for this machine's cameras
#
# Cameras are read here by ROS nodes and never by the motor host: one reader
# per device, and a stalled camera frame must not take the motor bus down.
# When the ROS stack runs on another machine it picks the compressed frames up
# over the network -- point scripts/install-compute-services.sh there at this
# one (hostname -I).
#
# Re-run any time; both installers are idempotent.
# Usage: scripts/install-device-services.sh
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

for arg in "$@"; do
  die "unknown argument: $arg (usage: $0)"
done

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

first_match() { # first existing path matching a glob, empty if none
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}

log "Installing lekiwi-host.service"
install_unit lekiwi-host.service
# Older installers flipped camera ownership through this file; the host is
# now always camera-less and nothing reads it anymore.
if [[ -f /etc/default/lekiwi-host ]]; then
  log "Removing obsolete $(printf %q /etc/default/lekiwi-host)"
  as_root rm -f /etc/default/lekiwi-host
fi

log "Installing lekiwi-cameras.service"
install_unit lekiwi-cameras.service
if [ -z "$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')" ]; then
  log "warning: no front camera found -- lekiwi-cameras will keep failing"
  log "until one is attached (set LEKIWI_FRONT in ros-cameras.sh for odd hardware)."
fi
calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
if ! grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$calibration" 2>/dev/null; then
  log "warning: front-camera calibration missing or invalid: $(printf %q "$calibration")"
  log "The cameras service refuses to start without it."
  log "Run scripts/calibrate-camera.sh on this machine first (stop its service while calibrating)."
fi

log "Reloading systemd and enabling services"
as_root systemctl daemon-reload
as_root systemctl enable --now lekiwi-host.service lekiwi-cameras.service

cat <<EOF
Done. Check on them with:
  systemctl status lekiwi-host.service lekiwi-cameras.service
  journalctl -u lekiwi-host.service -f

The host binds :5555 once every servo answers; that socket is what the
compute side waits for.

If the ROS stack will live on another machine, give it this one's address:
  $(hostname -I 2>/dev/null | awk '{print $1}')
There, run: scripts/install-compute-services.sh --remote <address>

One-time setup from HARDWARE.md (motor calibration) has to exist already;
the service cannot run the interactive calibration for you.
EOF
