#!/usr/bin/env bash
# Explicitly re-arm a running real-robot stack after inspecting the robot.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
# ROS's setup.bash reads unset variables, so disable nounset while sourcing it.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

# ros2 service call has no built-in timeout and blocks forever if the driver node is
# down, and every ros2 command blocks on a wedged daemon. Probe within 10 s; on failure
# replace the daemon and give the driver (its launch entry respawns it) 30 s to appear,
# then fail loudly instead of hanging.
if ! timeout -k 2 10 ros2 service type /safety/arm >/dev/null 2>&1; then
  reset_ros2_daemon
  if ! wait_for 30 ros2 service type /safety/arm; then
    echo "$0: /safety/arm is not available after 40s -- is lekiwi-stack.service up?" >&2
    echo "  systemctl status lekiwi-stack.service" >&2
    echo "  journalctl -u lekiwi-stack.service -n 50 --no-pager" >&2
    exit 1
  fi
fi

response=$(timeout 30 ros2 service call /safety/arm std_srvs/srv/Trigger '{}') || {
  status=$?
  printf '%s\n' "$response" >&2
  exit "$status"
}
printf '%s\n' "$response"
# ros2 exits zero for a delivered request even when the driver rejects it.
grep -qE '^std_srvs[.]srv[.]Trigger_Response[(]success=True, message=' <<< "$response"
