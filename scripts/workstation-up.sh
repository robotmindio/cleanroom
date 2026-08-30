#!/usr/bin/env bash
# Start everything that runs on the workstation for a robot Pi at the given address.
# Usage: scripts/workstation-up.sh <pi-ip-or-hostname> [extra ROS launch args...]
set -Eeuo pipefail

[[ $# -ge 1 ]] || {
  echo "usage: $0 <pi-ip-or-hostname> [extra ROS launch args...]" >&2
  exit 2
}

cd "$(dirname "$0")/.."
PI_IP=$1
shift
LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"

exec 9>"$LOGS/workstation-up-start.lock"
if ! flock -n 9; then
  echo "$0: startup is already in progress" >&2
  exit 0
fi

wait_for() { # wait_for <seconds> <command...>
  local deadline=$((SECONDS + $1)); shift
  until "$@" >/dev/null 2>&1; do
    [ "$SECONDS" -lt "$deadline" ] || return 1
    sleep 1
  done
}

if pgrep -f 'ros2 launch lekiwi_rmf' >/dev/null; then
  echo "$0: a ROS stack is already running -- scripts/ros-stop.sh first" >&2
  exit 1
fi

setsid scripts/ros-start.sh remote_ip:="$PI_IP" camera_source:=remote \
  laser_source:=ld06 lidar_source:=remote "$@" \
  >"$LOGS/stack.log" 2>&1 &
wait_for 120 grep -q 'Connected to LeKiwi host' "$LOGS/stack.log" || {
  echo "driver never reached the Pi host -- see $LOGS/stack.log" >&2
  exit 1
}
echo "stack: up"

setsid scripts/rviz.sh >"$LOGS/rviz.log" 2>&1 &
flock -u 9
exec 9>&-
echo "rviz: starting"
echo "logs in $LOGS -- stop the workstation stack with scripts/ros-stop.sh"
