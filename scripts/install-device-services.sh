#!/usr/bin/env bash
# Install the device-side boot services on whatever machine the robot's USB
# devices are plugged into -- a Raspberry Pi, a NUC, anything with the
# hardware. The names follow the hardware, not the board:
#
#   lekiwi-host.service     the LeRobot motor bus, motion on :5555 and torque safety on :5557
#   lekiwi-cameras.service  v4l2_camera publishers for this machine's cameras
#   lekiwi-lidar.service    private LD06 scan publisher for the compute stack
#
# Cameras are read here by ROS nodes and never by the motor host: one reader
# per device, and a stalled camera frame must not take the motor bus down.
# When the ROS stack runs on another machine it picks the compressed frames up
# over the network -- point scripts/install-compute-services.sh there at this
# one (hostname -I).
#
# Re-run any time; both installers are idempotent.
# Usage: scripts/install-device-services.sh [--service-user USER]
#        [--workspace PATH] [--lerobot-venv PATH] [--bind-address IPV4]
#        [--curve-dir PATH]
set -Eeuo pipefail

PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UNIT_DIR=/etc/systemd/system

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

SERVICE_USER_ARG=""
WORKSPACE_ARG=""
LEROBOT_VENV_ARG=""
HOST_BIND_ADDRESS_ARG=""
CURVE_DIR_ARG=""
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
    *) die "unknown argument: $1 (usage: $0 [--service-user USER] [--workspace PATH] [--lerobot-venv PATH] [--bind-address IPV4] [--curve-dir PATH])" ;;
  esac
done

SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)
as_root() { # as_root <command...>
  [[ $EUID -eq 0 ]] || command -v sudo >/dev/null || die "sudo is required to $1"
  "${SUDO[@]}" "$@"
}

# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/service-install-common.sh"
# shellcheck disable=SC1091 # PROJECT_ROOT is resolved above, not a fixed source path.
source "$PROJECT_ROOT/scripts/service-install-revision.sh"
resolve_service_user "$SERVICE_USER_ARG"
resolve_service_paths "$WORKSPACE_ARG" "$LEROBOT_VENV_ARG" false
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
  curve_dir=${CURVE_DIR_ARG:-"$LEKIWI_SERVICE_HOME/.ros/lekiwi/curve"}
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

install_unit() { render_systemd_unit "$PROJECT_ROOT/systemd/$1" "$UNIT_DIR/$1"; }

if ! (
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  if [[ -f $LEKIWI_SERVICE_WORKSPACE/install/setup.bash ]]; then
    # shellcheck source=/dev/null
    source "$LEKIWI_SERVICE_WORKSPACE/install/setup.bash"
  fi
  ros2 pkg prefix ldlidar_stl_ros2 >/dev/null 2>&1
) ; then
  die "ldlidar_stl_ros2 is unavailable; the standard device installation requires the LD06 driver"
fi

first_match() { # first existing path matching a glob, empty if none
  # shellcheck disable=SC2086 # Deliberately expand the caller-supplied glob.
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}

log "Installing lekiwi-host.service"
install_unit lekiwi-host.service
# Older installers flipped camera ownership through this file; the host is
# now always camera-less and nothing reads it anymore.
if [[ -f /etc/default/lekiwi-host ]]; then
  log "Removing obsolete $(printf %q /etc/default/lekiwi-host)"
  as_root rm -f /etc/default/lekiwi-host
fi

camera_ros_available=false
if (
  set +u
  # The documented sudo invocation has root's minimal PATH. Source ROS in a
  # subshell before probing or an installed camera package is silently missed.
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  if [[ -f $LEKIWI_SERVICE_WORKSPACE/install/setup.bash ]]; then
    # shellcheck source=/dev/null
    source "$LEKIWI_SERVICE_WORKSPACE/install/setup.bash"
  fi
  ros2 pkg prefix v4l2_camera >/dev/null 2>&1
); then
  camera_ros_available=true
fi
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

log "Installing lekiwi-lidar.service"
install_unit lekiwi-lidar.service
if [[ $camera_ros_available == true && -z "$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')" ]]; then
  log "warning: no front camera found -- lekiwi-cameras will keep failing"
  log "until one is attached (set LEKIWI_FRONT in ros-cameras.sh for odd hardware)."
fi
calibration="${LEKIWI_CAMERA_INFO:-$LEKIWI_SERVICE_HOME/.ros/camera_info/lekiwi_front.yaml}"
if ! grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$calibration" 2>/dev/null; then
  log "warning: front-camera calibration missing or invalid: $(printf %q "$calibration")"
  log "The cameras service refuses to start without it."
  log "Run scripts/calibrate-camera.sh on this machine first (stop its service while calibrating)."
fi

units=(lekiwi-host.service lekiwi-lidar.service)
[[ $camera_ros_available == true ]] && units+=(lekiwi-cameras.service)
log "Validating rendered systemd units"
verify_systemd_units "${units[@]}"

log "Reloading systemd and enabling services"
as_root systemctl daemon-reload
if [[ $camera_ros_available == true ]]; then
  as_root systemctl enable --now lekiwi-host.service lekiwi-cameras.service
else
  as_root systemctl enable --now lekiwi-host.service
fi
as_root systemctl enable --now lekiwi-lidar.service

log "Granting $LEKIWI_SERVICE_USER non-interactive deployment control"
as_root "$PROJECT_ROOT/scripts/install-deploy-sudoers.sh" device --user "$LEKIWI_SERVICE_USER"
record_service_fingerprint

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
