#!/usr/bin/env bash
# Motor calibration and ZMQ robot host for a LeKiwi, with this machine's camera paths.
# Usage: scripts/robot-host.sh [calibrate|--no-cameras]
# Override per machine: LEKIWI_PORT, LEKIWI_FRONT, LEKIWI_WRIST, LEKIWI_ID
set -Eeuo pipefail

# install.sh puts the venv in the workspace; install-pi.sh puts it in ~/lerobot-venv,
# because the Pi has no ROS workspace. Take whichever exists.
BIN="${LEKIWI_WS:-$HOME/lekiwi_ws}/.venv-lerobot/bin"
[ -x "$BIN/python" ] || BIN="${LEKIWI_LEROBOT_VENV:-$HOME/lerobot-venv}/bin"
ID="${LEKIWI_ID:-lekiwi_1}"
READ_RETRIES="${LEKIWI_READ_RETRIES:-5}"
RESTART_DELAY="${LEKIWI_HOST_RESTART_DELAY:-3}"

# This mirrors LeRobot's default calibration location. Keep the environment overrides
# so a machine using a shared/custom Hugging Face cache gets the same behaviour.
HF_CACHE="${HF_HOME:-$HOME/.cache/huggingface}"
LEROBOT_HOME="${HF_LEROBOT_HOME:-$HF_CACHE/lerobot}"
CALIBRATION_DIR="${HF_LEROBOT_CALIBRATION:-$LEROBOT_HOME/calibration}"
CALIBRATION_FILE="$CALIBRATION_DIR/robots/lekiwi/$ID.json"

# /dev/ttyACM0 and /dev/videoN are renumbered by every USB re-enumeration -- a bumped
# cable moves the motor bus to ttyACM1 and shifts both cameras. by-id names follow the
# device. Adjust the globs for your own hardware, or set LEKIWI_PORT/FRONT/WRIST.
first_match() { # first existing path matching a glob, empty if none
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
  return 0 # a miss is not an error here; require() reports it with the variable name
}
PORT="${LEKIWI_PORT:-$(first_match '/dev/serial/by-id/*USB_Single_Serial*')}"
FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"

require() {
  for var in "$@"; do
    [ -n "${!var}" ] || { echo "$0: no device found for $var -- set LEKIWI_$var" >&2; exit 1; }
  done
}

require_port_access() {
  local resolved
  resolved="$(readlink -f "$PORT")"
  if [ ! -r "$resolved" ] || [ ! -w "$resolved" ]; then
    echo "$0: cannot read/write $resolved (expected a dialout-accessible serial device)" >&2
    echo "Add $USER to dialout, log out and back in, then retry." >&2
    exit 1
  fi
}

# LeRobot's stock lekiwi config hardcodes /dev/video0 and /dev/video2, which on a
# laptop grabs the built-in webcam. Name the devices explicitly instead.
# fourcc: without it OpenCV negotiates uncompressed YUYV, and two 640x480@30 YUYV
# streams do not fit in USB 2.0 bandwidth -- the second camera opens but never
# delivers a frame. warmup_s: the front webcam needs ~1.1 s for its first frame,
# more than LeRobot's 1 s default. Later frames arrive at 40 ms.
# rotation 0: LeRobot's stock lekiwi config rotates the front camera 180, which assumes
# their mounting. This mast holds the camera upright -- frames read straight off the
# device come out right way up, so rotating them puts the optical frame 180 out of step
# with the URDF and RTAB-Map corrects poses in the wrong direction.
# LEKIWI_WRIST=none (or LEKIWI_FRONT=none) drops that camera: the wrist cable runs along
# the arm and falls off the bus under movement, and one dead read thread takes the whole
# host down with it.
entries=""
[ "$FRONT" = none ] || entries="front: {type: opencv, index_or_path: $FRONT, width: 640, height: 480, fps: 30, fourcc: MJPG, rotation: 0, warmup_s: 3}"
[ "$WRIST" = none ] || entries="${entries:+$entries,
          }wrist: {type: opencv, index_or_path: $WRIST, width: 480, height: 640, fps: 30, fourcc: MJPG, rotation: 90, warmup_s: 3}"
CAMERAS="{$entries}"

run_host_once() {
  # The servos lose their calibration registers on every power cycle, so connect() stops
  # to ask whether to reuse ~/.cache/.../lekiwi_1.json. Empty answer = reuse it. Without
  # this the host dies on EOFError whenever it runs without a terminal.
  printf '\n' | "$BIN/python" -m lerobot.robots.lekiwi.lekiwi_host \
    --robot.id="$ID" --robot.port="$PORT" --robot.cameras="$1" \
    --robot.num_read_retries="$READ_RETRIES" \
    --host.connection_time_s=86400
}

run_host() {
  # A second host on the same bus is the confusing failure: both talk over each other and
  # the handshake dies with "[TxRxResult] Incorrect status packet!", which reads like a
  # broken cable or a wrong port. Refuse instead. Ask who holds the device rather than
  # matching process names -- `pgrep -f` also matches the shell that typed the name.
  if fuser -s "$(readlink -f "$PORT")" 2>/dev/null; then
    echo "$0: a LeKiwi host already owns $PORT -- stop it first" >&2
    exit 1
  fi
  # A dropped motor-bus packet used to terminate the host permanently and leave ROS
  # connected to an empty ZMQ port. Keep this small supervisor alive instead. Each new
  # initial startup may arm after telemetry, but a ROS driver that has observed a
  # link loss requires an explicit safety/arm call after telemetry returns. A host
  # reconnect can therefore never resume motion unexpectedly.
  trap 'exit 0' INT TERM HUP
  while true; do
    if run_host_once "$1"; then
      status=0
    else
      status=$?
    fi
    echo "LeKiwi host exited (status $status); retrying the motor-bus connection in ${RESTART_DELAY}s." >&2
    sleep "$RESTART_DELAY"
  done
}

run_calibration() {
  "$BIN/lerobot-calibrate" --robot.type=lekiwi --robot.id="$ID" \
    --robot.port="$PORT" --robot.cameras='{}'
}

case "${1:-}" in
  # Calibration only talks to the motor bus, so skip the cameras entirely.
  calibrate)
    require PORT
    require_port_access
    exec "$BIN/lerobot-calibrate" --robot.type=lekiwi --robot.id="$ID" \
      --robot.port="$PORT" --robot.cameras='{}'
    ;;
  --no-cameras)
    require PORT
    require_port_access
    run_host '{}'
    ;;
  '')
    require PORT
    require_port_access
    if [ ! -f "$CALIBRATION_FILE" ]; then
      echo "No calibration found for $ID; starting calibration first."
      run_calibration
      [ -f "$CALIBRATION_FILE" ] || {
        echo "$0: calibration completed but did not create $CALIBRATION_FILE" >&2
        exit 1
      }
    fi
    # The known wrist camera is auto-detected. Set LEKIWI_WRIST=none to leave it
    # out of a direct LeRobot host (e.g. when conserving USB bandwidth).
    require FRONT
    [ "$WRIST" = none ] || require WRIST
    run_host "$CAMERAS"
    ;;
  *)
    echo "usage: $0 [calibrate|--no-cameras]" >&2
    exit 2
    ;;
esac
