#!/usr/bin/env bash
# Everything, in order: LeRobot host, ROS stack, RViz.
# Usage: scripts/up.sh [extra launch args...]     # e.g. slam_mode:=localization
# Stop it all with scripts/ros-stop.sh
#
# Only for the wired robot, where the motors hang off this machine. With a Pi on the
# robot the host runs there (see HARDWARE.md) and this machine runs scripts/ros-start.sh.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"

# A launch takes a few seconds to appear in pgrep. Serialize this whole startup window so
# two near-simultaneous invocations cannot both pass the "no stack" check and bind the
# same ROS/rosbridge resources.
exec 9>"$LOGS/up-start.lock"
if ! flock -n 9; then
  echo "$0: startup is already in progress" >&2
  exit 0
fi

wait_for() { # wait_for <seconds> <command...>
  local deadline=$((SECONDS + $1)); shift
  until "$@" >/dev/null 2>&1; do
    [ $SECONDS -lt $deadline ] || return 1
    sleep 1
  done
}

# A listening TCP port alone is not enough: a stale or unrelated process can bind it,
# leaving the ROS driver to discover much later that no LeRobot host is available.
host_up() {
  ss -tln | grep -q ':5555' && pgrep -f '[l]erobot\.robots\.lekiwi\.lekiwi_host' >/dev/null
}

first_match() { # first existing path matching a glob, empty if none
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}

camera_calibration_valid() {
  local calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
  # A missing calibration makes v4l2_camera publish an all-zero CameraInfo message.
  # That looks superficially healthy, but free_space and RTAB-Map cannot use it.
  [ -s "$calibration" ] \
    && grep -qE '^image_width:[[:space:]]*[1-9][0-9]*' "$calibration" \
    && grep -A3 '^camera_matrix:' "$calibration" | grep -qE '^[[:space:]]*data:.*[1-9]'
}

require_camera_calibration() {
  local calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
  if ! camera_calibration_valid; then
    echo "camera calibration is missing or invalid: $calibration" >&2
    echo "Launching the calibration program now." >&2
    scripts/calibrate-camera.sh "$calibration"
  fi
  if ! camera_calibration_valid; then
    echo "camera calibration was not saved or is invalid: $calibration" >&2
    exit 1
  fi
}

require_free_cameras() {
  local front wrist
  front="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
  wrist="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"
  for device in "$front" "$wrist"; do
    [ -z "$device" ] || [ "$device" = none ] || ! fuser -s "$(readlink -f "$device")" || {
      echo "camera is already in use: $device -- stop the process holding it before starting ROS" >&2
      exit 1
    }
  done
}

# A second stack is not obvious from the outside -- both drivers publish and /odom simply
# arrives at twice the rate -- so stop before starting rather than doubling up.
if pgrep -f 'ros2 launch lekiwi_rmf' >/dev/null; then
  echo "$0: a stack is already running -- scripts/ros-stop.sh first" >&2
  exit 1
fi

require_camera_calibration
require_free_cameras

# The ZMQ command socket is the honest ready signal: the host binds it only after
# connect() has found all nine servos. It also tells us a host you started by hand is
# already there, so this leaves it alone.
if host_up; then
  echo "host: already running"
else
  # ROS reads the local cameras directly. Keeping them out of the LeRobot host avoids
  # two V4L2 clients fighting over the same USB camera, which otherwise leaves ROS with
  # no images while the motor host can also die on a delayed camera read.
  setsid scripts/robot-host.sh --no-cameras > "$LOGS/host.log" 2>&1 &
  wait_for 90 host_up || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    exit 1
  }
  echo "host: up"
fi

setsid scripts/ros-start.sh "$@" > "$LOGS/stack.log" 2>&1 &
wait_for 120 grep -q 'Connected to LeKiwi host' "$LOGS/stack.log" || {
  echo "driver never reached the host -- see $LOGS/stack.log" >&2
  exit 1
}
echo "stack: up"

setsid scripts/rviz.sh > "$LOGS/rviz.log" 2>&1 &
# Background services inherit file descriptors. Release this lock explicitly so it does
# not remain held for the lifetime of the host, stack, or RViz process.
flock -u 9
exec 9>&-
echo "rviz: starting (it waits for the cameras)"
echo "logs in $LOGS -- stop everything with scripts/ros-stop.sh"
