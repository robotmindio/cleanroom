#!/usr/bin/env bash
# Bring up the full ROS stack against the real robot.
# Usage: scripts/ros-start.sh [extra launch args...]
#   scripts/ros-start.sh                                    # local mapping/navigation only
#   scripts/ros-start.sh slam_mode:=localization            # drive a map you already built
#   scripts/ros-start.sh start_rmf:=false                   # Nav2 only
# Override per machine: LEKIWI_FRONT, LEKIWI_WS. The Astra serial is pinned
# in config/hardware.yaml so the safety camera cannot be selected ad hoc.
# The LeRobot host must already be running -- `scripts/robot-host.sh` on the robot.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# Leave a precise ownership record for ros-stop.sh. This remains valid across
# exec because the launcher replaces this shell in the same PID; systemd and
# up.sh may supply a tighter runtime directory explicitly.
runtime_dir="${LEKIWI_RUNTIME_DIR:-${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/runtime}"
mkdir -p "$runtime_dir"
chmod 700 "$runtime_dir"
printf '%s\n' "$$" > "$runtime_dir/stack.pid"
# ROS's own setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES and friends),
# so `set -u` has to stand down for the sourcing.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

# systemd starts this script directly, not scripts/up.sh. Apply the same
# bounded default-database policy in both startup paths.
scripts/rtabmap-db-maintenance.py "$@"

# /dev/videoN shifts on every USB re-enumeration and on a laptop video0 is the built-in
# webcam, so resolve the front camera by its device name -- same glob as robot-host.sh.
# A workstation using a remote LeKiwi host has no local camera at all.
first_match() { # first existing path matching a glob, empty if none
  # shellcheck disable=SC2086 # Deliberately expand the caller-supplied glob.
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}
camera_source=""
for arg in "$@"; do
  case "$arg" in
    camera_source:=remote) camera_source=remote ;;
    camera_source:=local) camera_source=local ;;
    remote_ip:=*) [[ -n $camera_source ]] || camera_source=remote ;;
  esac
done
: "${camera_source:=local}"

camera_calibration_valid() {
  local calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
  [ -s "$calibration" ] \
    && grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$calibration" \
    && grep -A3 '^camera_matrix:' "$calibration" | grep -qE '^[[:space:]]*data:.*[1-9]'
}

# Geometry and odometry measurements are machine-local, not source-controlled. The
# calibration tools write only numeric KEY=VALUE lines; reject anything malformed rather
# than sourcing a user file into this launcher.
calibration_args=()
has_launch_arg() { # has_launch_arg <key>
  local key=$1 arg
  for arg in "$@"; do [[ $arg == "$key":=* ]] && return 0; done
  return 1
}
load_launch_calibration() {
  local file="${LEKIWI_LAUNCH_CALIBRATION:-$HOME/.ros/lekiwi_launch_calibration.conf}"
  local key value
  [ -r "$file" ] || return 0
  while IFS='=' read -r key value; do
    case "$key" in camera_height|camera_pitch|xy_velocity_scale|yaw_velocity_scale) ;;
      *) continue ;;
    esac
    [[ $value =~ ^[0-9]+([.][0-9]+)?$ ]] || continue
    has_launch_arg "$key" "$@" || calibration_args+=("$key:=$value")
  done < "$file"
}
load_launch_calibration "$@"

require_camera_calibration() {
  local calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
  if ! camera_calibration_valid; then
    echo "$0: camera calibration is missing or invalid: $calibration" >&2
    echo "Launching the calibration program now." >&2
    scripts/calibrate-camera.sh "$calibration"
  fi
  if ! camera_calibration_valid; then
    echo "$0: camera calibration was not saved or is invalid: $calibration" >&2
    exit 1
  fi
}

if [[ $camera_source == local ]]; then
  FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
  [ -n "$FRONT" ] || { echo "$0: no front camera found -- set LEKIWI_FRONT or pass camera_source:=remote" >&2; exit 1; }
  # The wrist camera is optional: unplugged, or LEKIWI_WRIST=none, and the stack runs without it.
  WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"
  require_camera_calibration
else
  FRONT=none
  WRIST=none
fi

front_camera_info="file://${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
wrist_camera_info="file://${LEKIWI_WRIST_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_wrist.yaml}"

exec ros2 launch lekiwi_rmf bringup.launch.py mode:=real \
  camera_source:="$camera_source" \
  camera_device:="$FRONT" wrist_camera_device:="${WRIST:-none}" \
  camera_info_url:="$front_camera_info" wrist_camera_info_url:="$wrist_camera_info" \
  "${calibration_args[@]}" "$@"
