#!/usr/bin/env bash
# Install only the LD06 device-side service without restarting motor/camera services.
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system
SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)
as_root() { "${SUDO[@]}" "$@"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
log() { printf '\n==> %s\n' "$*"; }

# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/service-install-common.sh"
resolve_service_user "${1:-}"
resolve_service_paths "" "" true false

(
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  # shellcheck source=/dev/null
  source "$LEKIWI_SERVICE_WORKSPACE/install/setup.bash"
  ros2 pkg prefix ldlidar_stl_ros2 >/dev/null
) || die "ldlidar_stl_ros2 is unavailable; build the device workspace first"

log "Installing lekiwi-lidar.service"
render_systemd_unit "$PROJECT_ROOT/systemd/lekiwi-lidar.service" "$UNIT_DIR/lekiwi-lidar.service"
verify_systemd_units lekiwi-lidar.service
as_root systemctl daemon-reload
as_root systemctl enable --now lekiwi-lidar.service
