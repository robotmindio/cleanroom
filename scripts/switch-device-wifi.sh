#!/usr/bin/env bash
# Move the device to a supplied 5 GHz SSID using its saved Wi-Fi credentials.
# Keep the original profile as fallback. Never reads or prints the password.
set -Eeuo pipefail
[[ $# -ge 1 && $# -le 2 ]] || { echo "usage: $0 SSID [INTERFACE]" >&2; exit 2; }
ssid=$1
interface=${2:-wlan0}
original=$(nmcli -g GENERAL.CON-UUID device show "$interface")
[[ -n $original && $original != -- ]] || { echo 'no active Wi-Fi profile to clone' >&2; exit 1; }
profile=lekiwi-5ghz
if nmcli -g connection.uuid connection show "$profile" >/dev/null 2>&1; then
  echo "$profile already exists; review that saved profile before replacing it" >&2
  exit 1
fi
nmcli connection clone uuid "$original" "$profile"
target=$(nmcli -g connection.uuid connection show "$profile")
nmcli connection modify uuid "$target" 802-11-wireless.ssid "$ssid" \
  802-11-wireless.band a 802-11-wireless.bssid "" 802-11-wireless.channel 0 \
  802-11-wireless.powersave 2 connection.autoconnect-priority 10
sudo -n systemd-run --quiet --collect --unit=lekiwi-wifi-rollback --on-active=45s \
  /usr/bin/nmcli connection up uuid "$original"
nmcli --wait 25 connection up uuid "$target"
sudo -n systemctl stop lekiwi-wifi-rollback.timer
echo '5 GHz profile active; original profile retained for fallback'
