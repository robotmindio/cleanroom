#!/usr/bin/env bash
# Keep device capture clocks aligned without changing sensor freshness limits.
set -Eeuo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
[[ $# == 0 ]] || die "usage: $0"
as_root install -d -m 0755 /etc/systemd/timesyncd.conf.d
install_unit_config /etc/systemd/timesyncd.conf.d/60-lekiwi.conf systemd-timesyncd.service \
  "$(cat "$PROJECT_ROOT/config/timesyncd.conf")"
# shellcheck disable=SC2119 # Restart only the time service when its configuration changed.
restart_changed_units
as_root systemctl enable --now systemd-timesyncd.service
