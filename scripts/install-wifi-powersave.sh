#!/usr/bin/env bash
# Disable Wi-Fi power saving on either end of the robot's telemetry link.
# LEKIWI_NETWORK_ROOT=DIR writes under DIR without changing the live network (tests only).
set -Eeuo pipefail

root=${LEKIWI_NETWORK_ROOT:-}
[[ -n $root || $EUID -eq 0 ]] || { echo 'run as root' >&2; exit 1; }

if ! command -v nmcli >/dev/null; then
  echo 'NetworkManager is not installed -- no Wi-Fi power-saving setting to install'
  exit 0
fi

conf=$root/etc/NetworkManager/conf.d/zz-lekiwi-wifi-powersave-off.conf
install -d -m 0755 "${conf%/*}"
# wifi.powersave=2 disables it; zz- overrides the distribution default.
printf '%s\n' '# Managed by scripts/install-wifi-powersave.sh.' '[connection]' 'wifi.powersave = 2' |
  install -m 0644 /dev/stdin "$conf"

if [[ -z $root ]]; then
  systemctl reload NetworkManager
  if command -v iw >/dev/null; then
    iw dev | awk '$1 == "Interface" {print $2}' | while IFS= read -r interface; do
      iw dev "$interface" set power_save off
      printf 'Wi-Fi power saving disabled on %s\n' "$interface"
    done
  else
    echo 'warning: iw is not installed; the setting applies on the next Wi-Fi connect' >&2
  fi
fi
