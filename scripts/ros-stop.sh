#!/usr/bin/env bash
# Stop a bringup and every node it left behind, plus the LeRobot host if one is running.
# Usage: scripts/ros-stop.sh
#
# `ros2 launch` relays SIGINT to its nodes and waits for their teardown. Targeting only
# its PID avoids killing unrelated ROS applications on the same workstation.
set -Eeuo pipefail

for pid in $(pgrep -f 'ros2 launch lekiwi_rmf' || true); do
  kill -INT "$pid" 2>/dev/null || true
done

# ponytail: 15 s covers Nav2's lifecycle teardown.
sleep 15

# The host is what scripts/up.sh starts first, so this is what takes it down. It holds the
# motor bus, and leaving it behind is what makes the next `lekiwi.sh host` refuse to start.
if pkill -f 'lerobot.robots.lekiwi.lekiwi_host'; then
  echo "ROS stack and LeRobot host stopped."
else
  echo "ROS stack stopped."
fi
