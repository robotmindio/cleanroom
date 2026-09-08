#!/usr/bin/env bash
# Source from a service installer after PROJECT_ROOT, as_root(), and die().

service_fingerprint() {
  local role=$1
  local sources=()
  case $role in
    compute)
      sources=(systemd/lekiwi-stack.service scripts/service-install-common.sh \
        scripts/runtime-common.sh scripts/install-deploy-sudoers.sh)
      ;;
    device)
      sources=(systemd/lekiwi-host.service systemd/lekiwi-astra.service \
        systemd/lekiwi-cameras.service systemd/lekiwi-lidar.service \
        scripts/ros-astra.sh scripts/ros-cameras.sh scripts/ros-lidar.sh \
        scripts/service-install-common.sh scripts/runtime-common.sh \
        scripts/install-deploy-sudoers.sh)
      ;;
    *) die "service fingerprint role must be compute or device" ;;
  esac
  (
    cd "$PROJECT_ROOT" || exit
    LC_ALL=C sha256sum "${sources[@]}" | sha256sum | awk '{print $1}'
  )
}

record_service_fingerprint() {
  local role=$1 fingerprint marker group
  fingerprint=$(service_fingerprint "$role") || die "cannot calculate service configuration fingerprint"
  marker="$LEKIWI_SERVICE_HOME/.ros/lekiwi/service-fingerprint-$role"
  group=$(id -gn "$LEKIWI_SERVICE_USER") || die "cannot determine service group"
  as_root install -d -o "$LEKIWI_SERVICE_USER" -g "$group" -m 0755 "${marker%/*}"
  printf '%s\n' "$fingerprint" | as_root tee "$marker" >/dev/null
  as_root chown "$LEKIWI_SERVICE_USER:$group" "$marker"
}
