#!/usr/bin/env bash
# Publish this machine's cameras as ROS topics, for a stack running elsewhere.
# Usage: scripts/ros-cameras.sh
# Override per machine: LEKIWI_FRONT, LEKIWI_WRIST, LEKIWI_CAMERA_INFO,
# LEKIWI_WRIST_CAMERA_INFO
#
# Every camera in this system is read by v4l2_camera on the machine it is
# plugged into -- the same nodes whether the ROS stack is local (which then
# launches them itself) or remote (this script). Only compressed frames cross
# the network; the remote side expands them back into /camera/... topics.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/runtime-common.sh

# The Pi runs ros-base only (scripts/setup-pi.bash); a machine with a
# workspace gets the same DDS settings through setup.bash. Either way the
# transport must match the workstation's or discovery is one-sided.
if [ -f "${LEKIWI_WS:-$HOME/lekiwi_ws}/install/setup.bash" ]; then
  set +u # ROS's own setup scripts read unset variables
  # shellcheck source=/dev/null
  source scripts/setup.bash
  set -u
else
  # shellcheck source=/dev/null
  source scripts/setup-pi.bash
fi

calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
front_camera_info="file://${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
wrist_camera_info="file://${LEKIWI_WRIST_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_wrist.yaml}"

# The optional 2D cameras must never make the independently owned Astra service
# failed or unavailable. Wait here until a calibrated front camera exists; an
# intentional service stop still stops the loop.
while :; do
  FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
  if [ -z "$FRONT" ]; then
    echo "$0: waiting for front camera (set LEKIWI_FRONT for other hardware)" >&2
    sleep 5
    continue
  fi
  if ! camera_calibration_valid "$calibration"; then
    echo "$0: waiting for valid front-camera calibration: $calibration" >&2
    sleep 5
    continue
  fi
  # The wrist camera is optional: unplugged or LEKIWI_WRIST=none runs without it.
  WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"
  # shellcheck disable=SC2093 # ros2 launch becomes this service's main process.
  exec ros2 launch launch/pi_cameras.launch.py \
    front_device:="$FRONT" wrist_device:="${WRIST:-none}" \
    camera_info_url:="$front_camera_info" wrist_camera_info_url:="$wrist_camera_info" "$@"
done
