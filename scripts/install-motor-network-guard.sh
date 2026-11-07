#!/usr/bin/env bash
# Opt-in Tailnet-only motor transport. CURVE authentication is still required.
# LEKIWI_NETWORK_ROOT=DIR installs files without changing a live host (tests).
set -Eeuo pipefail
# shellcheck source=/dev/null
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/runtime-common.sh"
[[ $# -eq 0 ]] || die "usage: $0"
root=${LEKIWI_NETWORK_ROOT:-}
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -z $root ]]; then
  [[ $EUID -eq 0 ]] || die "run as root"
  [[ -d /sys/class/net/tailscale0 ]] || die "tailscale0 is required before restricting motor ports"
  /usr/sbin/nft --check -f "$project/config/motor-network.nft"
fi
install -d -m 0755 "$root/etc/lekiwi" "$root/etc/systemd/system"
install -m 0644 "$project/config/motor-network.nft" "$root/etc/lekiwi/motor-network.nft"
install -m 0644 "$project/systemd/lekiwi-motor-network.service" "$root/etc/systemd/system/lekiwi-motor-network.service"
if [[ -z $root ]]; then
  systemctl daemon-reload
  systemctl enable lekiwi-motor-network.service
  systemctl reload-or-restart lekiwi-motor-network.service
fi
