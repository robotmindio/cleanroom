#!/usr/bin/env bash
# Install the device-side boot services on whatever machine the robot's USB
# devices are plugged into -- a Raspberry Pi, a NUC, anything with the
# hardware. The names follow the hardware, not the board:
#
#   lekiwi-host.service     the LeRobot motor bus, motion on :5555 and torque safety on :5557
#   lekiwi-astra.service    Astra Pro RGB-D publisher
#   lekiwi-cameras.service  v4l2_camera publishers for this machine's cameras
#   lekiwi-lidar.service    private LD06 scan publisher for the compute stack
#   lekiwi-zenoh.service    exports the sensor topics to the compute stack
#                           (needs zenoh-bridge-ros2dds; skipped without it)
#
# Cameras are read here by ROS nodes and never by the motor host: one reader
# per device, and a stalled camera frame must not take the motor bus down.
# When the ROS stack runs on another machine it picks the compressed frames up
# over the network -- point scripts/install-compute-services.sh there at this
# one (hostname -I).
#
# Re-run any time; both installers are idempotent. A service whose installed
# unit changed (new --bind-address or --curve-dir) is restarted to pick it up.
# Usage: scripts/install-device-services.sh [--service-user USER]
#        [--workspace PATH] [--lerobot-venv PATH] [--bind-address IPV4]
#        [--curve-dir PATH] [--no-start]
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system
# shellcheck source=/dev/null
source "$PROJECT_ROOT/scripts/lib/runtime-common.sh"
load_lekiwi_env "$PROJECT_ROOT/.env"

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

SERVICE_USER_ARG=""
WORKSPACE_ARG=""
LEROBOT_VENV_ARG=""
HOST_BIND_ADDRESS_ARG=""
CURVE_DIR_ARG=""
START_SERVICES=true
while [[ $# -gt 0 ]]; do
  case $1 in
    --service-user)
      [[ $# -ge 2 ]] || die "--service-user needs a user name"
      SERVICE_USER_ARG=$2; shift 2 ;;
    --workspace)
      [[ $# -ge 2 ]] || die "--workspace needs an absolute path"
      WORKSPACE_ARG=$2; shift 2 ;;
    --lerobot-venv)
      [[ $# -ge 2 ]] || die "--lerobot-venv needs an absolute path"
      LEROBOT_VENV_ARG=$2; shift 2 ;;
    --bind-address)
      [[ $# -ge 2 ]] || die "--bind-address needs an explicit IPv4 interface address"
      HOST_BIND_ADDRESS_ARG=$2; shift 2 ;;
    --curve-dir)
      [[ $# -ge 2 ]] || die "--curve-dir needs an absolute key directory"
      CURVE_DIR_ARG=$2; shift 2 ;;
    --no-start) START_SERVICES=false; shift ;;
    *) die "unknown argument: $1 (usage: $0 [--service-user USER] [--workspace PATH] [--lerobot-venv PATH] [--bind-address IPV4] [--curve-dir PATH] [--no-start])" ;;
  esac
done

# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/lib/service-install-common.sh"
# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/lib/service-install-revision.sh"
resolve_service_user "$SERVICE_USER_ARG"
resolve_service_paths "$WORKSPACE_ARG" "$LEROBOT_VENV_ARG" false

# Keep CURVE enabled on routine service refreshes once keys are installed.
[[ -n $CURVE_DIR_ARG || ! -d $LEKIWI_SERVICE_HOME/.config/lekiwi/curve ]] || \
  CURVE_DIR_ARG=$LEKIWI_SERVICE_HOME/.config/lekiwi/curve
LEKIWI_HOST_BIND_ADDRESS=${HOST_BIND_ADDRESS_ARG:-0.0.0.0}
LEKIWI_HOST_BIND_ADDRESS=$(python3 -c '
import ipaddress, sys
address = ipaddress.IPv4Address(sys.argv[1])
if address.is_multicast:
    raise SystemExit(2)
print(address)
' "$LEKIWI_HOST_BIND_ADDRESS") || \
  die "--bind-address must be an IPv4 address (multicast is forbidden)"
export LEKIWI_HOST_BIND_ADDRESS
LEKIWI_CURVE_SERVER_SECRET=""
LEKIWI_CURVE_SERVER_PUBLIC=""
LEKIWI_CURVE_AUTHORIZED_CLIENTS=""
LEKIWI_CURVE_HEALTH_CLIENT_SECRET=""
if [[ -n $CURVE_DIR_ARG ]]; then
  curve_dir=$CURVE_DIR_ARG
  [[ $curve_dir == /* && $curve_dir != *[[:space:]]* ]] || \
    die "--curve-dir must be an absolute path without whitespace"
  LEKIWI_CURVE_SERVER_SECRET="$curve_dir/server.key_secret"
  LEKIWI_CURVE_SERVER_PUBLIC="$curve_dir/server.key"
  LEKIWI_CURVE_AUTHORIZED_CLIENTS="$curve_dir/authorized_clients"
  LEKIWI_CURVE_HEALTH_CLIENT_SECRET="$curve_dir/clients/health.key_secret"
  for key_path in "$LEKIWI_CURVE_SERVER_SECRET" "$LEKIWI_CURVE_SERVER_PUBLIC" "$LEKIWI_CURVE_HEALTH_CLIENT_SECRET"; do
    [[ -f $key_path ]] || die "missing CURVE certificate: $key_path (run $LEKIWI_SERVICE_LEROBOT_VENV/bin/python $PROJECT_ROOT/scripts/generate-zmq-keys.py $curve_dir as $LEKIWI_SERVICE_USER)"
  done
  [[ -d $LEKIWI_CURVE_AUTHORIZED_CLIENTS ]] || die "missing authorized-client directory: $LEKIWI_CURVE_AUTHORIZED_CLIENTS"
fi
export LEKIWI_CURVE_SERVER_SECRET LEKIWI_CURVE_SERVER_PUBLIC
export LEKIWI_CURVE_AUTHORIZED_CLIENTS LEKIWI_CURVE_HEALTH_CLIENT_SECRET

ros_package_available() { # ros_package_available <package>
  (
    set +u
    # The documented sudo invocation has root's minimal PATH. Source ROS in a
    # subshell before probing or an installed package is silently missed.
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    if [[ -f $LEKIWI_SERVICE_WORKSPACE/install/setup.bash ]]; then
      # shellcheck source=/dev/null
      source "$LEKIWI_SERVICE_WORKSPACE/install/setup.bash"
    fi
    ros2 pkg prefix "$1" >/dev/null 2>&1
  )
}

if ! ros_package_available ldlidar_stl_ros2; then
  die "ldlidar_stl_ros2 is unavailable; the standard device installation requires the LD06 driver"
fi

log "Installing lekiwi-host.service"
install_unit lekiwi-host.service
# Older installers flipped camera ownership through this file; the host is
# now always camera-less and nothing reads it anymore.
if [[ -f /etc/default/lekiwi-host ]]; then
  log "Removing obsolete $(printf %q /etc/default/lekiwi-host)"
  as_root rm -f /etc/default/lekiwi-host
fi

camera_ros_available=false
ros_package_available v4l2_camera && camera_ros_available=true
if [[ $camera_ros_available == true ]]; then
  log "Installing lekiwi-cameras.service"
  install_unit lekiwi-cameras.service
else
  # Trixie/Raspberry Pi OS has no ROS apt packages. Do not enable a service
  # whose ExecStart cannot exist, and clean up an older attempted install.
  log "ROS v4l2_camera is unavailable; skipping lekiwi-cameras.service"
  if [[ -f $UNIT_DIR/lekiwi-cameras.service ]]; then
    as_root systemctl disable --now lekiwi-cameras.service 2>/dev/null || true
    as_root rm -f "$UNIT_DIR/lekiwi-cameras.service"
  fi
fi
astra_ros_available=false
ros_package_available astra_camera && astra_ros_available=true
if [[ $astra_ros_available == true ]]; then
  log "Installing lekiwi-astra.service"
  install_unit lekiwi-astra.service
else
  log "ROS astra_camera is unavailable; skipping lekiwi-astra.service"
fi

log "Installing lekiwi-lidar.service"
install_unit lekiwi-lidar.service
# ros-zenoh.sh runs the bridge from the service user's ~/.local/bin, where
# scripts/install.sh and scripts/install-pi.sh put it.
zenoh_available=false
if PATH="$LEKIWI_SERVICE_HOME/.local/bin:$PATH" command -v zenoh-bridge-ros2dds >/dev/null; then
  zenoh_available=true
  log "Installing lekiwi-zenoh.service"
  install_unit lekiwi-zenoh.service
  if [[ ! -r /etc/lekiwi/zenoh-tls/device.key ]]; then
    log "warning: no zenoh TLS identity yet; the bridge will not start until the compute"
    log "machine runs scripts/install-compute-services.sh (or scripts/setup-zenoh-tls.sh)."
  fi
else
  # Without the binary the unit would crash-loop every RestartSec. The compute
  # stack then receives no device sensors, so say so loudly.
  log "warning: zenoh-bridge-ros2dds not found for $LEKIWI_SERVICE_USER; skipping lekiwi-zenoh.service"
  log "Sensors will not reach the compute stack. Run scripts/install-pi.sh (or scripts/install.sh) as that user, then rerun this installer."
  if [[ -f $UNIT_DIR/lekiwi-zenoh.service ]]; then
    as_root systemctl disable --now lekiwi-zenoh.service 2>/dev/null || true
    as_root rm -f "$UNIT_DIR/lekiwi-zenoh.service"
  fi
fi
log "Installing ROS log rotation"
install_log_rotation
if [[ $camera_ros_available == true && -z "$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')" ]]; then
  log "no front camera found -- lekiwi-cameras will wait without affecting Astra"
  log "(set LEKIWI_FRONT for other hardware)."
fi
calibration="${LEKIWI_CAMERA_INFO:-$LEKIWI_SERVICE_HOME/.ros/camera_info/lekiwi_front.yaml}"
if ! grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$calibration" 2>/dev/null; then
  log "warning: front-camera calibration missing or invalid: $(printf %q "$calibration")"
  log "The cameras service will wait without it."
  log "Run scripts/calibrate-camera.sh on this machine first (stop its service while calibrating)."
fi

units=(lekiwi-host.service lekiwi-lidar.service)
[[ $zenoh_available == true ]] && units+=(lekiwi-zenoh.service)
[[ $astra_ros_available == true ]] && units+=(lekiwi-astra.service)
[[ $camera_ros_available == true ]] && units+=(lekiwi-cameras.service)
log "Validating rendered systemd units"
verify_systemd_units "${units[@]}"
verify_systemd_units lekiwi-ros-logrotate.service lekiwi-ros-logrotate.timer

log "Reloading systemd and enabling services"
log "Enabling Pi 5 USB current setting by default (5 V / 5 A supply required)"
as_root install -o root -g root -m 0755 \
  "$PROJECT_ROOT/scripts/enable-pi5-usb-current.sh" /usr/local/sbin/lekiwi-enable-pi5-usb-current
as_root /usr/local/sbin/lekiwi-enable-pi5-usb-current
as_root systemctl daemon-reload
# Restart first: try-restart skips a stopped unit, which enable --now then starts
# once, already with the new configuration, instead of starting it twice.
# --no-block: lekiwi-host only starts once every servo answers, and systemd keeps
# retrying it. Unpowered servos must not abort the installation before the sudoers
# grant, the Wi-Fi country and the configuration fingerprint below.
restart_changed_units --no-block
if [[ $START_SERVICES == true ]]; then
  as_root systemctl enable --now --no-block "${units[@]}" lekiwi-ros-logrotate.timer
else
  as_root systemctl enable "${units[@]}" lekiwi-ros-logrotate.timer
fi

log "Granting $LEKIWI_SERVICE_USER non-interactive deployment control"
as_root "$PROJECT_ROOT/scripts/install-deploy-sudoers.sh" device --user "$LEKIWI_SERVICE_USER"
log "Setting the Wi-Fi country, disabling power saving and granting network control"
as_root "$PROJECT_ROOT/scripts/install-device-network.sh" --user "$LEKIWI_SERVICE_USER" \
  --country "${LEKIWI_WIFI_COUNTRY:-ID}"
record_service_fingerprint device
for unit in "${units[@]}"; do
  log "$unit: $(systemctl is-active "$unit" 2>/dev/null || true)"
done

if [[ -f $UNIT_DIR/lekiwi-stack.service ]]; then
  log "A ROS stack service is also installed here -- re-run"
  log "scripts/install-compute-services.sh so it takes this machine's"
  log "camera stream over loopback instead of opening the devices again;"
  log "v4l2 allows one reader per camera."
fi

cat <<EOF
Done. Check on them with:
  systemctl status lekiwi-host.service lekiwi-cameras.service lekiwi-lidar.service
  journalctl -u lekiwi-host.service -f

The host binds :5555 (motion) and :5557 (physical torque safety) once every
servo answers; the compute side requires both before it arms.

If the ROS stack will live on another machine, give it this one's address:
  $(hostname -I 2>/dev/null | awk '{print $1}')
There, run: scripts/install-compute-services.sh --remote <address>

One-time setup from HARDWARE.md (motor calibration) has to exist already;
the service cannot run the interactive calibration for you.
EOF
