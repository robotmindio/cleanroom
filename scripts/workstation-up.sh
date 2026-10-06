#!/usr/bin/env bash
# Start everything that runs on the workstation for a robot Pi at the given address.
# Usage: scripts/workstation-up.sh [robot-host] [extra ROS launch args...]
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
# shellcheck source=/dev/null
source scripts/lib/launcher.sh
load_lekiwi_env
ROBOT_HOST=${LEKIWI_ROBOT_HOST:-}
if [[ ${1:-} != *:=* && -n ${1:-} ]]; then
  ROBOT_HOST=$1
  shift
fi
[[ -n $ROBOT_HOST ]] || {
  echo "usage: $0 [robot-host] [extra ROS launch args...] (or set LEKIWI_ROBOT_HOST in .env)" >&2
  exit 2
}
launcher_init workstation-up

# Only this launcher's recorded processes count: another robot's stack on a
# shared workstation has the same executable names and is left alone.
refuse_while_stack_service_runs
if recorded_running stack || recorded_running rviz; then
  echo "$0: a recorded LeKiwi stack is already running -- scripts/ros-stop.sh first" >&2
  exit 1
fi

start_recorded stack scripts/ros-start.sh profile:=split remote_ip:="$ROBOT_HOST" start_moveit:=true "$@"
wait_for 120 grep -q 'Connected to LeKiwi host' "$LOGS/stack.log" || {
  echo "driver never reached the Pi host -- see $LOGS/stack.log" >&2
  exit 1
}
echo "stack: up"

# No recorded RViz was running at startup. Clear a stale record so rviz.sh cannot
# mistake it for an RViz to replace, or delete the record written here.
rm -f -- "$RUNTIME_DIR/rviz.pid"
start_recorded rviz scripts/rviz.sh
launcher_release
echo "rviz: starting"
echo "logs in $LOGS -- stop the workstation stack with scripts/ros-stop.sh"
