#!/usr/bin/env bash
# Install only the device-side Astra publisher without touching motors, lidar, or V4L2 cameras.
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
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/service-install-revision.sh"
resolve_service_user "${1:-}"
resolve_service_paths "" "" true false

(
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  # shellcheck source=/dev/null
  source "$LEKIWI_SERVICE_WORKSPACE/install/setup.bash"
  ros2 pkg prefix astra_camera >/dev/null
) || die "astra_camera is unavailable; build the device workspace first"

log "Installing lekiwi-astra.service"
render_systemd_unit "$PROJECT_ROOT/systemd/lekiwi-astra.service" "$UNIT_DIR/lekiwi-astra.service"
verify_systemd_units lekiwi-astra.service
as_root systemctl daemon-reload
as_root systemctl enable --now lekiwi-astra.service
as_root "$PROJECT_ROOT/scripts/install-deploy-sudoers.sh" device --user "$LEKIWI_SERVICE_USER"
record_service_fingerprint device
