#!/usr/bin/env bash

# Source-only runtime helpers; this is not a user command. Scripts piped to a
# remote shell (stage-release.sh) cannot source it and keep their own die().

log() { printf '\n==> %s\n' "$*"; }
die() { echo "$0: $*" >&2; exit 1; }

load_lekiwi_env() {
  local file=${1:-.env} line value
  [ -r "$file" ] || return 0
  while IFS= read -r line || [ -n "$line" ]; do
    case $line in
      LEKIWI_ROBOT_HOST=*)
        value=${line#*=}
        value=${value%$'\r'}
        [[ $value =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
          echo "$file: LEKIWI_ROBOT_HOST must be a hostname or IPv4 address" >&2
          return 1
        }
        [[ -n ${LEKIWI_ROBOT_HOST:-} ]] || export LEKIWI_ROBOT_HOST=$value
        ;;
      LEKIWI_DISARM_ON_FAILURE=*)
        value=${line#*=}
        value=${value%$'\r'}
        [[ $value == true || $value == false ]] || {
          echo "$file: LEKIWI_DISARM_ON_FAILURE must be true or false" >&2
          return 1
        }
        [[ -n ${LEKIWI_DISARM_ON_FAILURE:-} ]] || export LEKIWI_DISARM_ON_FAILURE=$value
        ;;
      LEKIWI_WIFI_COUNTRY=*)
        value=${line#*=}
        value=${value%$'\r'}
        [[ $value =~ ^[A-Z]{2}$ ]] || {
          echo "$file: LEKIWI_WIFI_COUNTRY must be two uppercase letters (ISO 3166)" >&2
          return 1
        }
        [[ -n ${LEKIWI_WIFI_COUNTRY:-} ]] || export LEKIWI_WIFI_COUNTRY=$value
        ;;
    esac
  done < "$file"
}

first_match() { # first existing path matching a glob, empty if none
  # shellcheck disable=SC2086 # Deliberately expand the caller-supplied glob.
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}

wait_for() { # wait_for <seconds> <command...>: retry until it succeeds or the time is up
  local deadline=$((SECONDS + $1)) remaining; shift
  while true; do
    # A program that blocks (ros2 against a wedged daemon) must not outlive the wait, so
    # each attempt gets only the time that is left. timeout cannot run a shell function;
    # the functions passed here only probe local state and return at once.
    remaining=$((deadline - SECONDS))
    if [[ $(type -t "$1") == file ]]; then
      timeout -k 2 "$((remaining > 1 ? remaining : 1))" "$@" >/dev/null 2>&1 && return 0
    else
      "$@" >/dev/null 2>&1 && return 0
    fi
    (( SECONDS < deadline )) || return 1
    sleep 1
  done
}

reset_ros2_daemon() {
  # The ros2 daemon binds its network interface once. One started before a Wi-Fi change
  # answers nothing, and `ros2 service type` then blocks forever. A wedged daemon ignores
  # both `daemon stop` and SIGTERM, so kill it; the next ros2 command starts a fresh one.
  timeout 10 ros2 daemon stop >/dev/null 2>&1 || pkill -KILL -u "$(id -u)" -f 'ros2cli.daemon' || true
}

tcp_port_listening() { # tcp_port_listening <port>: exact local port, not :55550
  grep -Eq ":$1[[:space:]]" < <(ss -tln 2>/dev/null)
}

lekiwi_motion_port_listening() {
  tcp_port_listening 5555
}

lekiwi_safety_ports_listening() {
  lekiwi_motion_port_listening && tcp_port_listening 5557
}

source_device_ros_env() { # ROS and DDS settings for a device service; run from the repository root
  local workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
  if [ -f "$workspace/install/setup.bash" ]; then
    set +u # ROS's own setup scripts read unset variables
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    # shellcheck source=/dev/null
    source "$workspace/install/setup.bash"
    set -u
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$PWD/config/cyclonedds.xml"
  else
    # shellcheck source=/dev/null
    source scripts/setup-pi.bash
  fi
}

camera_calibration_valid() {
  local file="${1:-${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}}"
  [ -s "$file" ] \
    && grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$file" \
    && grep -A3 '^camera_matrix:' "$file" | grep -qE '^[[:space:]]*data:.*[1-9]'
}

require_camera_calibration() {
  local calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
  if ! camera_calibration_valid "$calibration"; then
    echo "$0: camera calibration is missing or invalid: $calibration" >&2
    echo "Launching the calibration program now." >&2
    scripts/calibrate-camera.sh "$calibration"
  fi
  if ! camera_calibration_valid "$calibration"; then
    echo "$0: camera calibration was not saved or is invalid: $calibration" >&2
    exit 1
  fi
}
