#!/usr/bin/env bash
# Calibrate the local front camera and save the result for the real-robot stack.
# Usage: scripts/calibrate-camera.sh [output-yaml]
set -Eeuo pipefail

cd "$(dirname "$0")/.."
CALIBRATION="${1:-${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}}"

set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

first_match() {
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0
}

FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
[ -n "$FRONT" ] || { echo "$0: no front camera found -- set LEKIWI_FRONT" >&2; exit 1; }
[ ! -e "$CALIBRATION" ] || [ -w "$CALIBRATION" ] || {
  echo "$0: cannot write $CALIBRATION" >&2
  exit 1
}
mkdir -p "$(dirname "$CALIBRATION")"

# A GUI crash immediately after Save still leaves a complete ROS calibration archive.
# Reuse only a very recent one: this is recovery for an interrupted calibration, not a
# substitute for deliberately calibrating a different camera later.
archive=/tmp/calibrationdata.tar.gz
if [ -s "$archive" ] && find "$archive" -mmin -60 -print -quit | grep -q . \
  && tar -tzf "$archive" | grep -qx 'ost.yaml'; then
  tar -xOf "$archive" ost.yaml > "$CALIBRATION.tmp"
  mv "$CALIBRATION.tmp" "$CALIBRATION"
  echo "Recovered the recently saved camera calibration to $CALIBRATION"
  exit 0
fi

if fuser -s "$(readlink -f "$FRONT")"; then
  echo "$0: front camera is already in use: $FRONT" >&2
  echo "Stop the existing stack before calibrating." >&2
  exit 1
fi

echo "Starting the front camera at 640x480 for calibration."
echo "In the calibration window, collect samples, click Calibrate, then Save."
echo "Do not click Commit: v4l2_camera does not provide the set_camera_info service."
echo "After Save reports success, close the window with q or Escape."
echo "The result will be saved to $CALIBRATION."

# camera_calibration's Save button writes this archive. Its Commit button instead calls
# a set_camera_info service, which v4l2_camera intentionally does not implement.
calibration_stamp=$(mktemp)

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __node:=front_camera -r __ns:=/camera/front \
  -p video_device:="$FRONT" \
  -p camera_info_url:="file://$CALIBRATION" \
  -p camera_frame_id:=front_camera_optical_frame \
  -p camera_name:=lekiwi_front \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p image_size:='[640, 480]' &
camera_pid=$!
cleanup() {
  kill -INT "$camera_pid" 2>/dev/null || true
  wait "$camera_pid" 2>/dev/null || true
  rm -f "$calibration_stamp"
}
trap cleanup EXIT

for _ in {1..20}; do
  ros2 topic info /camera/front/image_raw >/dev/null 2>&1 && break
  sleep 1
done
ros2 topic info /camera/front/image_raw >/dev/null 2>&1 || {
  echo "$0: camera did not publish /camera/front/image_raw" >&2
  exit 1
}

set +e
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --camera_name lekiwi_front --no-service-check \
  --ros-args --remap image:=/camera/front/image_raw --remap camera:=/camera/front
calibrator_status=$?
set -e

[ -s "$archive" ] && [ "$archive" -nt "$calibration_stamp" ] || {
  echo "$0: no calibration was saved; run it again and click Save after Calibrate." >&2
  exit 1
}
tar -xOf "$archive" ost.yaml > "$CALIBRATION.tmp"
mv "$CALIBRATION.tmp" "$CALIBRATION"
echo "Saved camera calibration to $CALIBRATION"
if [ "$calibrator_status" -ne 0 ]; then
  echo "Calibration UI exited after saving; continuing with the saved result."
fi
