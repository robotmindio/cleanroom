#!/usr/bin/env bash
# Motor calibration and ZMQ host for a LeKiwi, with this machine's camera paths.
# Usage: scripts/lekiwi.sh calibrate|host
# Override per machine: LEKIWI_PORT, LEKIWI_FRONT, LEKIWI_WRIST, LEKIWI_ID
set -Eeuo pipefail

BIN="${LEKIWI_WS:-$HOME/lekiwi_ws}/.venv-lerobot/bin"
ID="${LEKIWI_ID:-lekiwi_1}"
PORT="${LEKIWI_PORT:-/dev/ttyACM0}"
FRONT="${LEKIWI_FRONT:-/dev/video2}"
WRIST="${LEKIWI_WRIST:-/dev/video4}"

# LeRobot's stock lekiwi config hardcodes /dev/video0 and /dev/video2, which on a
# laptop grabs the built-in webcam. Name the devices explicitly instead.
# warmup_s: the front webcam takes ~1.1 s to produce its first frame and LeRobot's
# default 1 s warmup times out on it. Later frames arrive at 40 ms.
CAMERAS="{front: {type: opencv, index_or_path: $FRONT, width: 640, height: 480, fps: 30, rotation: 180, warmup_s: 3},
          wrist: {type: opencv, index_or_path: $WRIST, width: 480, height: 640, fps: 30, rotation: 90, warmup_s: 3}}"

case "${1:-}" in
  # ponytail: calibration only talks to the motor bus, so skip the cameras entirely.
  calibrate)
    exec "$BIN/lerobot-calibrate" --robot.type=lekiwi --robot.id="$ID" \
      --robot.port="$PORT" --robot.cameras='{}'
    ;;
  host)
    exec "$BIN/python" -m lerobot.robots.lekiwi.lekiwi_host \
      --robot.id="$ID" --robot.port="$PORT" --robot.cameras="$CAMERAS" \
      --host.connection_time_s=86400
    ;;
  *)
    echo "usage: $0 calibrate|host" >&2
    exit 2
    ;;
esac
