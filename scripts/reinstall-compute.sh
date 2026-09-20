#!/usr/bin/env bash
# Reinstall the split compute service from this checkout's .env. The installer
# restarts lekiwi-stack.service itself.
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
load_lekiwi_env "$PROJECT_ROOT/.env"
remote=${LEKIWI_ROBOT_HOST:-}
[[ -n $remote ]] || {
  echo "$0: set LEKIWI_ROBOT_HOST in $PROJECT_ROOT/.env" >&2
  exit 2
}

service_user=${SUDO_USER:-$(id -un)}
service_home=$(getent passwd "$service_user" | cut -d: -f6)
workspace=${LEKIWI_WS:-$service_home/lekiwi_ws}
as_root=()
[[ $EUID -eq 0 ]] || as_root=(sudo)

# sudo drops the environment, so the host is passed explicitly rather than
# left for the installer to find.
"${as_root[@]}" "$PROJECT_ROOT/scripts/install-compute-services.sh" \
  --service-user "$service_user" --workspace "$workspace" --remote "$remote" "$@"
grep -Fq 'start_moveit:=true' /etc/default/lekiwi-stack
systemctl is-active --quiet lekiwi-stack.service
echo "compute service reinstalled; MoveIt enabled by default"
