#!/usr/bin/env bash
# Stop a bringup and every node it left behind, plus the LeRobot host if one is running.
# Usage: scripts/ros-stop.sh
#
# `ros2 launch` relays SIGINT to its nodes and waits for their teardown. The exact-tree
# fallback below handles old launches started through nohup, which inherit ignored SIGINT.
set -Eeuo pipefail

mapfile -t launch_pids < <(pgrep -f 'ros2 launch lekiwi_rmf' || true)
for pid in "${launch_pids[@]}"; do
  kill -INT "$pid" 2>/dev/null || true
done

# ponytail: 15 s covers Nav2's lifecycle teardown.
sleep 15

stop_tree() { # stop_tree <signal> <pid>
  local signal=$1 pid=$2 child
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    stop_tree "$signal" "$child"
  done
  kill "-$signal" "$pid" 2>/dev/null || true
}

# Do not sweep /opt/ros: only terminate descendants of launchers found above.
for pid in "${launch_pids[@]}"; do
  kill -0 "$pid" 2>/dev/null || continue
  stop_tree TERM "$pid"
done
sleep 3
for pid in "${launch_pids[@]}"; do
  kill -0 "$pid" 2>/dev/null || continue
  stop_tree KILL "$pid"
done

# The host is what scripts/up.sh starts first, so this is what takes it down. It holds the
# motor bus, and leaving it behind is what makes the next `lekiwi.sh host` refuse to start.
if pkill -f 'lerobot.robots.lekiwi.lekiwi_host'; then
  echo "ROS stack and LeRobot host stopped."
else
  echo "ROS stack stopped."
fi
