#!/usr/bin/env bash
# Install the split robot's shared time reference without stepping live clocks.
set -Eeuo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
[[ $# == 1 && ( $1 == compute || $1 == device ) ]] || die "usage: $0 compute|device"
role=$1
if ! command -v chronyd >/dev/null; then
  # Prevent the package's default makestep policy from running during installation.
  as_root systemctl mask --runtime chrony.service
  as_root env NEEDRESTART_MODE=l DEBIAN_FRONTEND=noninteractive apt-get install -y chrony
fi
as_root install -d -m 0755 /etc/chrony
install_unit_config /etc/chrony/chrony.conf chrony.service \
  "$(cat "$PROJECT_ROOT/config/chrony-$role.conf")"
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
