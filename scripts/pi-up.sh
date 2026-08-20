#!/usr/bin/env bash
# Start everything that runs on the robot Pi: the motor-and-camera host.
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

if ss -tln | grep -q ':5555'; then
  echo "host: already running"
else
  setsid scripts/robot-host.sh >"$LOGS/host.log" 2>&1 &
  wait_for 90 host_up || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    exit 1
  }
  echo "host: up"
fi

echo "Pi ready -- logs in $LOGS"
