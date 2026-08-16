#!/usr/bin/env bash
# Stop a bringup and every node it left behind.
# Usage: scripts/ros-stop.sh
#
# `ros2 launch` only shuts its nodes down on SIGINT to the whole process group;
# killing the launcher alone orphans ~20 nodes that keep their DDS participants.
# Enough orphans accumulate and `ros2 topic list` hangs forever with no error --
# the nodes still talk to each other, so the stack looks healthy while introspection
# is dead.
set -Eeuo pipefail

for pid in $(pgrep -f 'ros2 launch lekiwi_rmf' || true); do
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
  [ -n "$pgid" ] && kill -INT "-$pgid" 2>/dev/null || true
done

# ponytail: 15 s covers Nav2's lifecycle teardown; the sweep below catches the rest.
sleep 15

# Match on the install prefixes rather than on open sockets: every node of this stack
# runs out of /opt/ros or the workspace, and nothing else on the machine does. The
# Zenoh bridge is the exception -- the installer puts it in ~/.local/bin.
# The ros2 CLI daemon is meant to outlive a run -- killing it only costs a slow
# first command.
orphans=$(pgrep -f "/opt/ros/jazzy|${LEKIWI_WS:-$HOME/lekiwi_ws}/install|zenoh-bridge-ros2dds" | grep -v "^$$\$" || true)
for pid in $orphans; do
  grep -qa ros2cli.daemon "/proc/$pid/cmdline" 2>/dev/null && continue
  kill -9 "$pid" 2>/dev/null || true
done

echo "ROS stack stopped."
