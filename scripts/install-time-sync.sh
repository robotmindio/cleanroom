#!/usr/bin/env bash
# Install the split robot's shared time reference. chrony steps only during the
# first updates after boot (makestep limit 3); later corrections slew.
#
# Usage: scripts/install-time-sync.sh device | compute [DEVICE_ADDR]
# config/chrony-ROLE.conf is a template; this site's addresses come from .env:
#   LEKIWI_TIME_SERVERS (device): the compute host's addresses, preferred one first
#   LEKIWI_TIME_CLIENTS (compute): the IPv4 addresses or subnets allowed to follow it
# Without the key, the addresses of an already installed configuration are kept.
set -Eeuo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
[[ ( $# == 1 || ( $# == 2 && $1 == compute ) ) && ( $1 == compute || $1 == device ) ]] || \
  die "usage: $0 device | compute [DEVICE_ADDR]"
role=$1
load_lekiwi_env "$PROJECT_ROOT/.env"
case $role in
  device) key=LEKIWI_TIME_SERVERS directive=server ;;
  compute) key=LEKIWI_TIME_CLIENTS directive=allow ;;
esac
addresses=()
if [[ -n ${!key:-} ]]; then
  IFS=, read -r -a addresses <<<"${!key}"
elif as_root grep -q "^$directive " /etc/chrony/chrony.conf 2>/dev/null; then
  log "$key is not set in .env; keeping the installed $directive addresses"
  # shellcheck disable=SC2016 # $1 and $2 are awk fields.
  mapfile -t addresses < <(as_root awk -v directive="$directive" '$1 == directive { print $2 }' /etc/chrony/chrony.conf)
elif [[ $role == device ]]; then
  # The device has no clock of its own; unsynchronized, the driver rejects its telemetry.
  die "set LEKIWI_TIME_SERVERS=<compute address>[,<address>...] in $PROJECT_ROOT/.env"
elif [[ ${2:-} =~ ^[0-9]+([.][0-9]+){3}$ ]]; then
  addresses=("$2")
elif [[ -n ${2:-} ]]; then
  # chrony resolves a name in `allow` once, at start; before the network is up that loses it.
  die "set LEKIWI_TIME_CLIENTS=<device IPv4 or subnet>[,...] in $PROJECT_ROOT/.env"
fi
for address in "${addresses[@]}"; do
  [[ $address =~ ^[A-Za-z0-9][A-Za-z0-9._-]*(/[0-9]{1,2})?$ ]] || die "invalid time-sync address: $address"
done
config=""
while IFS= read -r line; do
  case $line in
    *@FIRST_ADDRESS@*) [[ -z ${addresses[0]:-} ]] || config+=${line/@FIRST_ADDRESS@/${addresses[0]}}$'\n' ;;
    *@OTHER_ADDRESS@*)
      for address in "${addresses[@]:1}"; do config+=${line/@OTHER_ADDRESS@/$address}$'\n'; done ;;
    *) config+=$line$'\n' ;;
  esac
done < "$PROJECT_ROOT/config/chrony-$role.conf"
if ! command -v chronyd >/dev/null; then
  # Prevent the package's default makestep policy from running during installation.
  as_root systemctl mask --runtime chrony.service
  as_root env NEEDRESTART_MODE=l DEBIAN_FRONTEND=noninteractive apt-get install -y chrony
fi
as_root install -d -m 0755 /etc/chrony
install_unit_config /etc/chrony/chrony.conf chrony.service "${config%$'\n'}"
as_root chronyd -p -f /etc/chrony/chrony.conf >/dev/null
as_root rm -f /etc/systemd/timesyncd.conf.d/60-lekiwi.conf
# Package installation may already have removed timesyncd.
if systemctl cat systemd-timesyncd.service >/dev/null 2>&1; then
  as_root systemctl disable --now systemd-timesyncd.service
fi
as_root systemctl unmask --runtime chrony.service
# shellcheck disable=SC2119 # Restart only chrony when its configuration changed.
restart_changed_units
as_root systemctl enable --now chrony.service
