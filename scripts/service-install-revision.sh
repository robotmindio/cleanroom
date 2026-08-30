#!/usr/bin/env bash
# Source from a service installer after PROJECT_ROOT, as_root(), and die().

service_fingerprint() {
  (
    cd "$PROJECT_ROOT" || exit
    LC_ALL=C sha256sum systemd/*.service scripts/ros-lidar.sh \
      scripts/service-install-common.sh scripts/install-deploy-sudoers.sh | \
      sha256sum | awk '{print $1}'
  )
}

record_service_fingerprint() {
  local fingerprint marker group
  fingerprint=$(service_fingerprint) || die "cannot calculate service configuration fingerprint"
  marker="$LEKIWI_SERVICE_HOME/.ros/lekiwi/service-fingerprint"
  group=$(id -gn "$LEKIWI_SERVICE_USER") || die "cannot determine service group"
  as_root install -d -o "$LEKIWI_SERVICE_USER" -g "$group" -m 0755 "${marker%/*}"
  printf '%s\n' "$fingerprint" | as_root tee "$marker" >/dev/null
  as_root chown "$LEKIWI_SERVICE_USER:$group" "$marker"
}
