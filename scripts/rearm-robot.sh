#!/usr/bin/env bash
# Explicitly re-arm a running real-robot stack after inspecting the robot.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/runtime-common.sh
# ROS's setup.bash reads unset variables, so disable nounset while sourcing it.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

# ros2 service call has no built-in timeout and blocks forever if the driver
# node is down. Fail loud within 30s instead -- that node's launch entry now
# respawns on its own, but a driver that keeps crashing (or a stopped stack)
# still needs a clear signal here instead of a silent hang.
if ! wait_for 30 ros2 service type /safety/arm; then
  echo "$0: /safety/arm is not available after 30s -- is lekiwi-stack.service up?" >&2
  echo "  systemctl status lekiwi-stack.service" >&2
  echo "  journalctl -u lekiwi-stack.service -n 50 --no-pager" >&2
  exit 1
fi

exec ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
