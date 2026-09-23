#!/usr/bin/env bash
# Set the Wi-Fi regulatory country, now and at every boot. Run as root; idempotent.
#
# Without one the kernel uses the world domain "00", which allows only passive
# scanning on 5 GHz (a 5 GHz network shows up only now and then) and which the
# Raspberry Pi's brcmfmac firmware rejects ("Firmware rejected country setting").
#
# Usage: sudo scripts/install-wifi-regdom.sh [CC]
#   CC  two-letter ISO 3166 country; default LEKIWI_WIFI_COUNTRY from the
#       repository .env, else ID. Every installer resolves it the same way, so a
#       rerun never falls back to a different country than the configured one.
# LEKIWI_NETWORK_ROOT=DIR writes under DIR instead of / (tests only).
set -Eeuo pipefail

die() { printf 'error: %s\n' "$*" >&2; exit 1; }

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
load_lekiwi_env "$PROJECT_ROOT/.env"

country=${1:-${LEKIWI_WIFI_COUNTRY:-ID}}
[[ $# -le 1 && $country =~ ^[A-Z]{2}$ ]] || die "usage: $0 [CC] (two uppercase letters)"
root=${LEKIWI_NETWORK_ROOT:-}
[[ -n $root || $EUID -eq 0 ]] || die "run as root"

conf=$root/etc/modprobe.d/lekiwi-cfg80211-regdom.conf
install -d -m 0755 "${conf%/*}"
# cfg80211 reads this on every load, so the country survives reboots.
printf '%s\n' '# Managed by scripts/install-wifi-regdom.sh.' "options cfg80211 ieee80211_regdom=$country" |
  install -m 0644 /dev/stdin "$conf"
printf 'Wi-Fi regulatory country: %s (%s)\n' "$country" "$conf"

# A kernel command-line value (raspi-config writes one into cmdline.txt) overrides
# the modprobe option at boot. Editing the boot configuration is left to a person.
boot_country=$(grep -o 'cfg80211\.ieee80211_regdom=[A-Za-z0-9]*' "$root/proc/cmdline" 2>/dev/null || true)
boot_country=${boot_country##*=}
if [[ -n $boot_country && $boot_country != "$country" ]]; then
  printf 'warning: the kernel command line sets cfg80211.ieee80211_regdom=%s, which replaces %s at\n' \
    "$boot_country" "$country" >&2
  printf 'every boot; change it in the boot command line (raspi-config: Localisation > WLAN Country)\n' >&2
fi

# The modprobe option only takes effect at the next module load; apply it now too.
if command -v iw >/dev/null; then
  iw reg set "$country"
else
  printf 'warning: iw is not installed; country %s applies at the next reboot\n' "$country"
fi
