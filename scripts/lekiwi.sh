#!/usr/bin/env bash
# Motor calibration and ZMQ host for a LeKiwi, with this machine's camera paths.
# Usage: scripts/lekiwi.sh calibrate|host
# Override per machine: LEKIWI_PORT, LEKIWI_FRONT, LEKIWI_WRIST, LEKIWI_ID
set -Eeuo pipefail

BIN="${LEKIWI_WS:-$HOME/lekiwi_ws}/.venv-lerobot/bin"
ID="${LEKIWI_ID:-lekiwi_1}"

# /dev/ttyACM0 and /dev/videoN are renumbered by every USB re-enumeration -- a bumped
# cable moves the motor bus to ttyACM1 and shifts both cameras. by-id names follow the
# device. Adjust the globs for your own hardware, or set LEKIWI_PORT/FRONT/WRIST.
first_match() { # first existing path matching a glob, empty if none
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
}
PORT="${LEKIWI_PORT:-$(first_match '/dev/serial/by-id/*USB_Single_Serial*')}"
FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"

require() {
  for var in "$@"; do
    [ -n "${!var}" ] || { echo "$0: no device found for $var -- set LEKIWI_$var" >&2; exit 1; }
  done
}

# LeRobot's stock lekiwi config hardcodes /dev/video0 and /dev/video2, which on a
# laptop grabs the built-in webcam. Name the devices explicitly instead.
# fourcc: without it OpenCV negotiates uncompressed YUYV, and two 640x480@30 YUYV
# streams do not fit in USB 2.0 bandwidth -- the second camera opens but never
# delivers a frame. warmup_s: the front webcam needs ~1.1 s for its first frame,
# more than LeRobot's 1 s default. Later frames arrive at 40 ms.
CAMERAS="{front: {type: opencv, index_or_path: $FRONT, width: 640, height: 480, fps: 30, fourcc: MJPG, rotation: 180, warmup_s: 3},
          wrist: {type: opencv, index_or_path: $WRIST, width: 480, height: 640, fps: 30, fourcc: MJPG, rotation: 90, warmup_s: 3}}"

case "${1:-}" in
  # ponytail: calibration only talks to the motor bus, so skip the cameras entirely.
  calibrate)
    require PORT
    exec "$BIN/lerobot-calibrate" --robot.type=lekiwi --robot.id="$ID" \
      --robot.port="$PORT" --robot.cameras='{}'
    ;;
  host)
    require PORT FRONT WRIST
    exec "$BIN/python" -m lerobot.robots.lekiwi.lekiwi_host \
      --robot.id="$ID" --robot.port="$PORT" --robot.cameras="$CAMERAS" \
      --host.connection_time_s=86400
    ;;
  *)
    echo "usage: $0 calibrate|host" >&2
    exit 2
    ;;
esac
