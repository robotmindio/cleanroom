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

FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
[ -n "$FRONT" ] || { echo "$0: no front camera found -- set LEKIWI_FRONT" >&2; exit 1; }
# The wrist camera is optional: unplugged or LEKIWI_WRIST=none runs without it.
WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"

# A missing calibration makes v4l2_camera publish an all-zero CameraInfo that
# looks healthy but is useless to free_space and RTAB-Map. Fail before anyone
# starts trusting those frames.
calibration="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
camera_calibration_valid "$calibration" || {
    echo "$0: camera calibration is missing or invalid: $calibration" >&2
    echo "Run scripts/calibrate-camera.sh on this machine first." >&2
    exit 1
  }

front_camera_info="file://${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
wrist_camera_info="file://${LEKIWI_WRIST_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_wrist.yaml}"
exec ros2 launch launch/pi_cameras.launch.py \
  front_device:="$FRONT" wrist_device:="${WRIST:-none}" \
  camera_info_url:="$front_camera_info" wrist_camera_info_url:="$wrist_camera_info" "$@"
