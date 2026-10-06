#!/usr/bin/env bash
# Motor calibration and ZMQ motor host for a LeKiwi. The host serves no cameras:
# ROS camera nodes own the USB cameras (scripts/ros-cameras.sh).
# Usage: scripts/robot-host.sh [calibrate|--telemetry-fault-test]
# Override per machine: LEKIWI_PORT, LEKIWI_ID
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
load_lekiwi_env
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

# Systemd supplies LEKIWI_LEROBOT_VENV explicitly.  Interactive use retains
# the two documented defaults, but failure to find either is configuration
# failure, not a reason to enter the motor-bus reconnect loop.
if [[ -n ${LEKIWI_LEROBOT_VENV:-} ]]; then
  BIN="$LEKIWI_LEROBOT_VENV/bin"
else
  BIN="${LEKIWI_WS:-$HOME/lekiwi_ws}/.venv-lerobot/bin"
  [[ -x "$BIN/python" ]] || BIN="$HOME/lerobot-venv/bin"
fi
[[ -x "$BIN/python" ]] || {
  echo "$0: LeRobot Python is missing: $BIN/python" >&2
  echo "Set LEKIWI_LEROBOT_VENV or install the documented virtual environment." >&2
  exit 78
}
[[ -x "$BIN/lerobot-calibrate" ]] || {
  echo "$0: LeRobot calibration executable is missing: $BIN/lerobot-calibrate" >&2
  exit 78
}
ID="${LEKIWI_ID:-lekiwi_1}"
READ_RETRIES="${LEKIWI_READ_RETRIES:-5}"
RESTART_DELAY="${LEKIWI_HOST_RESTART_DELAY:-3}"
BIND_ADDRESS="${LEKIWI_BIND_ADDRESS:-0.0.0.0}"
HOST_PROGRAM=scripts/torque-host.py
CURVE_SERVER_SECRET="${LEKIWI_CURVE_SERVER_SECRET:-}"
CURVE_AUTHORIZED_CLIENTS="${LEKIWI_CURVE_AUTHORIZED_CLIENTS:-}"
if [[ -n $CURVE_SERVER_SECRET || -n $CURVE_AUTHORIZED_CLIENTS ]]; then
  [[ -n $CURVE_SERVER_SECRET && -n $CURVE_AUTHORIZED_CLIENTS ]] || {
    echo "$0: both LEKIWI_CURVE_SERVER_SECRET and LEKIWI_CURVE_AUTHORIZED_CLIENTS are required" >&2
    exit 78
  }
  CURVE_ARGS=(
    --curve.server_secret_key_file="$CURVE_SERVER_SECRET"
    --curve.authorized_clients_dir="$CURVE_AUTHORIZED_CLIENTS"
  )
else
  CURVE_ARGS=()
fi

# This mirrors LeRobot's default calibration location. Keep the environment overrides
# so a machine using a shared/custom Hugging Face cache gets the same behaviour.
HF_CACHE="${HF_HOME:-$HOME/.cache/huggingface}"
LEROBOT_HOME="${HF_LEROBOT_HOME:-$HF_CACHE/lerobot}"
CALIBRATION_DIR="${HF_LEROBOT_CALIBRATION:-$LEROBOT_HOME/calibration}"
CALIBRATION_FILE="$CALIBRATION_DIR/robots/lekiwi/$ID.json"

# /dev/ttyACM0 is renumbered by every USB re-enumeration -- a bumped cable moves the
# motor bus to ttyACM1. The by-id name follows the device. Adjust the glob for your own
# hardware, or set LEKIWI_PORT.
PORT="${LEKIWI_PORT:-$(first_match '/dev/serial/by-id/*USB_Single_Serial*')}"

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

run_host_once() {
  # The servos lose their calibration registers on every power cycle, so connect() stops
  # to ask whether to reuse ~/.cache/.../lekiwi_1.json. Empty answer = reuse it. Without
  # this the host dies on EOFError whenever it runs without a terminal. Without the file
  # that same answer would start an unattended calibration of a moving robot: refuse, and
  # exit 78 so systemd (RestartPreventExitStatus=78) stops retrying.
  [ -f "$CALIBRATION_FILE" ] || {
    echo "$0: motor calibration is missing: $CALIBRATION_FILE" >&2
    echo "Calibrate at the robot first: scripts/calibrate.sh motor" >&2
    exit 78
  }
  printf '\n' | "$BIN/python" "$HOST_PROGRAM" \
    --robot.id="$ID" --robot.port="$PORT" \
    --robot.num_read_retries="$READ_RETRIES" \
    --safety.bind_address="$BIND_ADDRESS" \
    --safety.disarm_on_failure="${LEKIWI_DISARM_ON_FAILURE:-false}" \
    "${CURVE_ARGS[@]}"
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
  # connected to an empty ZMQ port. Keep this small supervisor alive instead. A restarted
  # host starts torque-off; the ROS driver re-arms itself once telemetry returns, or in
  # the strict mode (LEKIWI_DISARM_ON_FAILURE=true) waits for an explicit safety/arm.
  trap 'exit 0' INT TERM HUP
  while true; do
    if run_host_once; then
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
    run_calibration
    ;;
  ''|--telemetry-fault-test)
    [[ ${1:-} != --telemetry-fault-test ]] || HOST_PROGRAM=scripts/test-host-telemetry.py
    require PORT
    require_port_access
    run_host
    ;;
  *)
    echo "usage: $0 [calibrate|--telemetry-fault-test]" >&2
    exit 2
    ;;
esac
