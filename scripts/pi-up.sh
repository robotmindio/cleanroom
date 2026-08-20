#!/usr/bin/env bash
# Start everything that runs on the robot Pi: the motor host and camera publisher.
# Usage: scripts/pi-up.sh
set -Eeuo pipefail

cd "$(dirname "$0")/.."
LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"

wait_for() { # wait_for <seconds> <command...>
  local deadline=$((SECONDS + $1)); shift
  until "$@" >/dev/null 2>&1; do
    [ "$SECONDS" -lt "$deadline" ] || return 1
    sleep 1
  done
}

host_up() { ss -tln | grep -q ':5555'; }

first_match() {
  set -- $1
  [ -e "$1" ] && printf '%s' "$1"
}

FRONT="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"
[ -n "$FRONT" ] || { echo "$0: no front camera found -- set LEKIWI_FRONT" >&2; exit 1; }

# The host deliberately carries motors only here. The camera ROS nodes own the webcam
# devices, keeping a stalled frame from taking motor control down with it.
if ss -tln | grep -q ':5555'; then
  echo "host: already running"
else
  setsid scripts/robot-host.sh --no-cameras >"$LOGS/host.log" 2>&1 &
  wait_for 90 host_up || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    exit 1
  }
  echo "host: up"
fi

# shellcheck source=/dev/null
source scripts/setup-pi.bash
camera_up() { pgrep -f 'ros2 launch .*pi_cameras\.launch\.py' >/dev/null; }

if camera_up; then
  echo "cameras: already running"
else
  setsid ros2 launch "$(pwd)/launch/pi_cameras.launch.py" \
    front_device:="$FRONT" wrist_device:="${WRIST:-none}" >"$LOGS/pi-cameras.log" 2>&1 &
  wait_for 20 camera_up || {
    echo "camera publisher did not stay up -- see $LOGS/pi-cameras.log" >&2
    tail -10 "$LOGS/pi-cameras.log" >&2
    exit 1
  }
  echo "cameras: up"
fi

echo "Pi ready -- logs in $LOGS"
