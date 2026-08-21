#!/usr/bin/env bash
# Bring up the full ROS stack against the real robot.
# Usage: scripts/ros-start.sh [extra launch args...]
#   scripts/ros-start.sh                                    # mapping, RMF and rosbridge on
#   scripts/ros-start.sh slam_mode:=localization            # drive a map you already built
#   scripts/ros-start.sh start_rmf:=false                   # Nav2 only
# Override per machine: LEKIWI_FRONT, LEKIWI_WS
# The LeRobot host must already be running -- `scripts/robot-host.sh` on the robot.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's own setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES and friends),
# so `set -u` has to stand down for the sourcing.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

# /dev/videoN shifts on every USB re-enumeration and on a laptop video0 is the built-in
# webcam, so resolve the front camera by its device name -- same glob as robot-host.sh.
# A workstation using a remote LeKiwi host has no local camera at all.
first_match() { # first existing path matching a glob, empty if none
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

if [[ $camera_source == local ]]; then
  FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
  [ -n "$FRONT" ] || { echo "$0: no front camera found -- set LEKIWI_FRONT or pass camera_source:=remote" >&2; exit 1; }
  # The wrist camera is optional: unplugged, or LEKIWI_WRIST=none, and the stack runs without it.
  WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"
else
  FRONT=none
  WRIST=none
fi

exec ros2 launch lekiwi_rmf bringup.launch.py mode:=real \
  camera_source:="$camera_source" camera_device:="$FRONT" wrist_camera_device:="${WRIST:-none}" "$@"
