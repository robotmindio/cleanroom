#!/usr/bin/env bash
# Reinstall and restart the split compute service from this checkout's .env.
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
remote=${LEKIWI_ROBOT_HOST:-}
if [[ -z $remote && -r $PROJECT_ROOT/.env ]]; then
  mapfile -t configured_hosts < <(sed -n 's/^LEKIWI_ROBOT_HOST=//p' "$PROJECT_ROOT/.env")
  [[ ${#configured_hosts[@]} -eq 1 ]] || {
    echo "$0: .env must contain exactly one LEKIWI_ROBOT_HOST" >&2
    exit 2
  }
  remote=${configured_hosts[0]}
fi
[[ $remote =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
  echo "$0: set LEKIWI_ROBOT_HOST in $PROJECT_ROOT/.env" >&2
  exit 2
}

service_user=${SUDO_USER:-$(id -un)}
service_home=$(getent passwd "$service_user" | cut -d: -f6)
workspace=${LEKIWI_WS:-$service_home/lekiwi_ws}
as_root=()
[[ $EUID -eq 0 ]] || as_root=(sudo)

"${as_root[@]}" "$PROJECT_ROOT/scripts/install-compute-services.sh" \
  --service-user "$service_user" --workspace "$workspace" --remote "$remote" "$@"
"${as_root[@]}" systemctl restart lekiwi-stack.service
grep -Fq 'start_moveit:=true' /etc/default/lekiwi-stack
systemctl is-active --quiet lekiwi-stack.service
echo "compute service reinstalled; MoveIt enabled by default"
