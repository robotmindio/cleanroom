#!/usr/bin/env bash
# One-time least-privilege sudo setup for scripts/deploy-split.sh.
set -Eeuo pipefail

usage() {
  echo "usage: $0 compute|device [--user USER] [--print]" >&2
  exit 2
}

[[ $# -ge 1 ]] || usage
role=$1
shift
[[ $role == compute || $role == device ]] || usage

requested_user=""
print_only=false
while [[ $# -gt 0 ]]; do
  case $1 in
    --user) [[ $# -ge 2 ]] || usage; requested_user=$2; shift 2 ;;
    --print) print_only=true; shift ;;
    *) usage ;;
  esac
done

if [[ -n $requested_user ]]; then
  deploy_user=$requested_user
elif [[ $EUID -eq 0 ]]; then
  deploy_user=${SUDO_USER:-}
else
  deploy_user=$(id -un)
fi
if [[ ! $deploy_user =~ ^[a-z_][a-z0-9_-]*$ ]] || ! getent passwd "$deploy_user" >/dev/null; then
  echo "$0: a valid non-root deployment user is required" >&2
  exit 1
fi
[[ $(id -u "$deploy_user") -ne 0 ]] || {
  echo "$0: refusing to grant deployment commands to root" >&2
  exit 1
}

systemctl=/usr/bin/systemctl
if [[ $role == compute ]]; then
  units=(lekiwi-stack.service)
else
  units=(lekiwi-host.service lekiwi-astra.service lekiwi-cameras.service lekiwi-lidar.service)
fi

commands=()
for action in start stop restart reset-failed; do
  for unit in "${units[@]}"; do
    commands+=("$systemctl $action $unit")
  done
done
rule="$deploy_user ALL=(root) NOPASSWD: $(IFS=', '; echo "${commands[*]}")"

if [[ $print_only == true ]]; then
  printf '%s\n' "$rule"
  exit 0
fi

tmp=$(mktemp)
trap 'rm -f -- "$tmp"' EXIT
printf '%s\n' '# Managed by scripts/install-deploy-sudoers.sh.' "$rule" > "$tmp"
chmod 0440 "$tmp"
visudo_bin=$(command -v visudo || true)
[[ -n $visudo_bin ]] || { echo "$0: visudo is required" >&2; exit 1; }
"$visudo_bin" -cf "$tmp"

destination=/etc/sudoers.d/lekiwi-deploy-$role-$deploy_user
if [[ $EUID -eq 0 ]]; then
  /usr/bin/install -o root -g root -m 0440 "$tmp" "$destination"
else
  sudo /usr/bin/install -o root -g root -m 0440 "$tmp" "$destination"
fi
echo "installed $destination"
