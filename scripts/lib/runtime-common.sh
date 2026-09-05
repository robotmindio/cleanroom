#!/usr/bin/env bash

# Source-only runtime helpers; this is not a user command.

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

camera_calibration_valid() {
  local file="${1:-${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}}"
  [ -s "$file" ] \
    && grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$file" \
    && grep -A3 '^camera_matrix:' "$file" | grep -qE '^[[:space:]]*data:.*[1-9]'
}
