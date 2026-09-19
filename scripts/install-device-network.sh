#!/usr/bin/env bash
# Network settings for the machine that owns the robot's USB devices. Run as root.
#
#   - Wi-Fi power saving off. Ubuntu and Raspberry Pi OS enable it by default; the radio
#     then sleeps between beacons and every packet waits up to hundreds of milliseconds,
#     which stalls ROS discovery, the camera streams and the torque link.
#   - A polkit rule that lets the deployment user manage NetworkManager over SSH without a
#     password (nmcli connection up/modify, Wi-Fi scans). It grants no shell access.
#
# Usage: sudo scripts/install-device-network.sh --user USER
# LEKIWI_NETWORK_ROOT=DIR writes under DIR and skips NetworkManager checks (tests only).
set -Eeuo pipefail

die() { printf 'error: %s\n' "$*" >&2; exit 1; }

[[ ${1:-} == --user && $# -eq 2 ]] || die "usage: $0 --user USER"
deploy_user=$2
if [[ ! $deploy_user =~ ^[a-z_][a-z0-9_-]*$ ]] || ! getent passwd "$deploy_user" >/dev/null; then
  die "a valid non-root deployment user is required"
fi
[[ $(id -u "$deploy_user") -ne 0 ]] || die "refusing to grant network control to root"

root=${LEKIWI_NETWORK_ROOT:-}
if [[ -z $root ]]; then
  [[ $EUID -eq 0 ]] || die "run as root"
  if ! command -v nmcli >/dev/null; then
    printf 'NetworkManager is not installed -- nothing to configure\n'
    exit 0
  fi
fi

nm_conf=$root/etc/NetworkManager/conf.d/zz-lekiwi-wifi-powersave-off.conf
polkit_rule=$root/etc/polkit-1/rules.d/50-lekiwi-networkmanager.rules

install -d -m 0755 "${nm_conf%/*}" "${polkit_rule%/*}"

# wifi.powersave: 2 = disable. The zz- prefix sorts after the distribution's
# default-wifi-powersave-on.conf, and later files win.
printf '%s\n' '# Managed by scripts/install-device-network.sh.' '[connection]' 'wifi.powersave = 2' |
  install -m 0644 /dev/stdin "$nm_conf"

cat <<EOF | install -m 0644 /dev/stdin "$polkit_rule"
// Managed by scripts/install-device-network.sh.
polkit.addRule(function(action, subject) {
  var allowed = [
    "org.freedesktop.NetworkManager.settings.modify.system",
    "org.freedesktop.NetworkManager.network-control",
    "org.freedesktop.NetworkManager.wifi.scan"
  ];
  if (subject.user == "$deploy_user" && allowed.indexOf(action.id) >= 0) {
    return polkit.Result.YES;
  }
});
EOF

if [[ -z $root ]]; then
  systemctl reload NetworkManager
  printf 'installed %s and %s\n' "$nm_conf" "$polkit_rule"
  printf 'Power saving is off from the next Wi-Fi connect or reboot.\n'
fi
