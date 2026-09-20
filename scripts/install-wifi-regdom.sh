#!/usr/bin/env bash
# Set the Wi-Fi regulatory country. Run as root; idempotent.
#
# Without one the kernel uses the world domain "00", which allows only passive
# scanning on 5 GHz (a 5 GHz network shows up only now and then) and which the
# Raspberry Pi's brcmfmac firmware rejects ("Firmware rejected country setting").
#
# Usage: sudo scripts/install-wifi-regdom.sh [CC]   two-letter ISO 3166 country (default ID)
# LEKIWI_NETWORK_ROOT=DIR writes under DIR and skips the live change (tests only).
set -Eeuo pipefail

die() { printf 'error: %s\n' "$*" >&2; exit 1; }

country=${1:-ID}
[[ $# -le 1 && $country =~ ^[A-Z]{2}$ ]] || die "usage: $0 [CC] (two uppercase letters)"
root=${LEKIWI_NETWORK_ROOT:-}
[[ -n $root || $EUID -eq 0 ]] || die "run as root"

conf=$root/etc/modprobe.d/lekiwi-cfg80211-regdom.conf
install -d -m 0755 "${conf%/*}"
# cfg80211 reads this on every load, so the country survives reboots.
printf '%s\n' '# Managed by scripts/install-wifi-regdom.sh.' "options cfg80211 ieee80211_regdom=$country" |
  install -m 0644 /dev/stdin "$conf"

# The modprobe option only takes effect at the next module load; apply it now too.
if [[ -z $root ]]; then
  if command -v iw >/dev/null; then
    iw reg set "$country"
  else
    printf 'warning: iw is not installed; country %s applies at the next reboot\n' "$country"
  fi
fi
