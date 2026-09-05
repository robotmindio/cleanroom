#!/usr/bin/env bash
# Calibrate a local camera and save the result for the real-robot stack.
# Usage: scripts/calibrate-camera.sh [output-yaml]
#        scripts/calibrate-camera.sh --wrist [output-yaml]
# The calibration belongs to the machine the camera is plugged into. Stop that
# machine's camera publisher first (the lekiwi-cameras service, or
# scripts/ros-cameras.sh) -- it holds the device and this script opens it too.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh

target=front
case "${1:-}" in
  --wrist) target=wrist; shift ;;
  --front) shift ;;
esac
[ "$#" -le 1 ] || { echo "usage: $0 [--wrist] [output-yaml]" >&2; exit 2; }

if [ "$target" = wrist ]; then
  CALIBRATION="${1:-${LEKIWI_WRIST_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_wrist.yaml}}"
  DEVICE="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"
  CAMERA=lekiwi_wrist
  NAMESPACE=/camera/wrist
  FRAME=wrist_camera_optical_frame
  SIZE='[352, 288]'
else
  CALIBRATION="${1:-${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}}"
  DEVICE="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
  CAMERA=lekiwi_front
  NAMESPACE=/camera/front
  FRAME=front_camera_optical_frame
  SIZE='[640, 480]'
fi

set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

[ -n "$DEVICE" ] || { echo "$0: no $target camera found -- set LEKIWI_${target^^}" >&2; exit 1; }
[ ! -e "$CALIBRATION" ] || [ -w "$CALIBRATION" ] || {
  echo "$0: cannot write $CALIBRATION" >&2
  exit 1
}
mkdir -p "$(dirname "$CALIBRATION")"

# A GUI crash immediately after Save still leaves a complete ROS calibration archive.
# Reuse only a very recent one: this is recovery for an interrupted calibration, not a
# substitute for deliberately calibrating a different camera later.
archive=/tmp/calibrationdata.tar.gz
archive_matches_camera() {
  tar -xOf "$archive" ost.yaml 2>/dev/null | grep -qE "^camera_name:[[:space:]]*$CAMERA$"
}
if [ -s "$archive" ] && find "$archive" -mmin -60 -print -quit | grep -q . \
  && tar -tzf "$archive" | grep -qx 'ost.yaml' && archive_matches_camera; then
  tar -xOf "$archive" ost.yaml > "$CALIBRATION.tmp"
  mv "$CALIBRATION.tmp" "$CALIBRATION"
  echo "Recovered the recently saved camera calibration to $CALIBRATION"
  exit 0
fi

if fuser -s "$(readlink -f "$DEVICE")"; then
  echo "$0: $target camera is already in use: $DEVICE" >&2
  echo "Stop the existing stack before calibrating." >&2
  exit 1
fi

echo "Starting the $target camera at $SIZE for calibration."
echo "In the calibration window, collect samples, click Calibrate, then Save."
echo "Do not click Commit: v4l2_camera does not provide the set_camera_info service."
echo "After Save reports success, this script closes the window and saves the YAML."
echo "The result will be saved to $CALIBRATION."

# camera_calibration's Save button writes this archive. Its Commit button instead calls
# a set_camera_info service, which v4l2_camera intentionally does not implement.
calibration_stamp=$(mktemp)

scripts/camera-supervisor.sh \
  --device "$DEVICE" --name "${target}_camera" --namespace "$NAMESPACE" \
  --camera-name "$CAMERA" --frame "$FRAME" --size "$SIZE" \
  --camera-info-url "file://$CALIBRATION" &
camera_pid=$!
cleanup() {
  kill -INT "$camera_pid" 2>/dev/null || true
  wait "$camera_pid" 2>/dev/null || true
  rm -f "$calibration_stamp"
}
trap cleanup EXIT

for _ in {1..20}; do
  ros2 topic info "$NAMESPACE/image_raw" >/dev/null 2>&1 && break
  sleep 1
done
ros2 topic info "$NAMESPACE/image_raw" >/dev/null 2>&1 || {
  echo "$0: camera did not publish $NAMESPACE/image_raw" >&2
  exit 1
}

ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --camera_name "$CAMERA" --no-service-check \
  --ros-args --remap image:="$NAMESPACE/image_raw" --remap camera:="$NAMESPACE" &
calibrator_pid=$!

stop_process_tree() { # stop_process_tree <signal> <pid>
  local signal=$1 pid=$2 descendant
  for descendant in $(pgrep -P "$pid" 2>/dev/null || true); do
    stop_process_tree "$signal" "$descendant"
  done
  kill "-$signal" "$pid" 2>/dev/null || true
}

# Save writes a complete archive immediately. Commit tries to call a
# set_camera_info service that v4l2_camera does not provide and can leave the
# GUI stuck forever, so make Save the single terminal action.
saved=0
while kill -0 "$calibrator_pid" 2>/dev/null; do
  if [ -s "$archive" ] && [ "$archive" -nt "$calibration_stamp" ] && archive_matches_camera; then
    saved=1
    echo "Calibration Save detected; closing the calibration window."
    stop_process_tree TERM "$calibrator_pid"
    sleep 1
    stop_process_tree KILL "$calibrator_pid"
    break
  fi
  sleep 1
done
wait "$calibrator_pid" 2>/dev/null || true

[ "$saved" = 1 ] || { [ -s "$archive" ] && [ "$archive" -nt "$calibration_stamp" ] && archive_matches_camera; } || {
  echo "$0: no calibration was saved; run it again and click Save after Calibrate." >&2
  exit 1
}
tar -xOf "$archive" ost.yaml > "$CALIBRATION.tmp"
mv "$CALIBRATION.tmp" "$CALIBRATION"
echo "Saved camera calibration to $CALIBRATION"
