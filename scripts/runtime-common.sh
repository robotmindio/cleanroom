#!/usr/bin/env bash

first_match() { # first existing path matching a glob, empty if none
  # shellcheck disable=SC2086 # Deliberately expand the caller-supplied glob.
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}

wait_for() { # wait_for <seconds> <command...>
  local deadline=$((SECONDS + $1)); shift
  until "$@" >/dev/null 2>&1; do
    (( SECONDS < deadline )) || return 1
    sleep 1
  done
}

lekiwi_motion_port_listening() {
  ss -tln 2>/dev/null | grep -q ':5555'
}

lekiwi_safety_ports_listening() {
  lekiwi_motion_port_listening && ss -tln 2>/dev/null | grep -q ':5557'
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
