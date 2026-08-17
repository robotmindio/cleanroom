#!/usr/bin/env bash
# Everything, in order: LeRobot host, ROS stack, RViz.
# Usage: scripts/up.sh [extra launch args...]     # e.g. slam_mode:=localization
# Stop it all with scripts/ros-stop.sh
#
# Only for the wired robot, where the motors hang off this machine. With a Pi on the
# robot the host runs there (see HARDWARE.md) and this machine runs scripts/ros-start.sh.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"

wait_for() { # wait_for <seconds> <command...>
  local deadline=$((SECONDS + $1)); shift
  until "$@" >/dev/null 2>&1; do
    [ $SECONDS -lt $deadline ] || return 1
    sleep 1
  done
}

host_up() { ss -tln | grep -q ':5555'; }

# A second stack is not obvious from the outside -- both drivers publish and /odom simply
# arrives at twice the rate -- so stop before starting rather than doubling up.
if pgrep -f 'ros2 launch lekiwi_rmf' >/dev/null; then
  echo "$0: a stack is already running -- scripts/ros-stop.sh first" >&2
  exit 1
fi

# The ZMQ command socket is the honest ready signal: the host binds it only after
# connect() has found all nine servos. It also tells us a host you started by hand is
# already there, so this leaves it alone.
if host_up; then
  echo "host: already running"
else
  nohup scripts/lekiwi.sh host > "$LOGS/host.log" 2>&1 &
  wait_for 90 host_up || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    exit 1
  }
  echo "host: up"
fi

nohup scripts/ros-start.sh "$@" > "$LOGS/stack.log" 2>&1 &
wait_for 120 grep -q 'Connected to LeKiwi host' "$LOGS/stack.log" || {
  echo "driver never reached the host -- see $LOGS/stack.log" >&2
  exit 1
}
echo "stack: up"

nohup scripts/rviz.sh > "$LOGS/rviz.log" 2>&1 &
echo "rviz: starting (it waits for the cameras)"
echo "logs in $LOGS -- stop everything with scripts/ros-stop.sh"
