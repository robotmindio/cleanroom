#!/usr/bin/env bash
# Start a managed headless simulation after proving this host can render Ogre2.
# Usage: scripts/sim-up.sh [extra ROS launch args...]
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/launcher.sh
launcher_init sim-up

# Do not silently stop a recorded real stack just because it happens to share
# the default runtime directory. Its owner must choose to stop it explicitly.
if [[ -e $RUNTIME_DIR/stack.pid ]]; then
  echo "$0: a recorded LeKiwi stack exists; inspect it or run scripts/ros-stop.sh first" >&2
  exit 1
fi

scripts/sim-renderer-check.py

# The new session execs ros2 launch, making its PID both the recorded stack
# identity and its process-group leader for ros-stop.sh, which then force-stops
# a stuck Gazebo child without sweeping unrelated ROS processes on a shared server.
# ROS setup scripts reference optional variables, so -u stays off in there.
# shellcheck disable=SC2016 # Expanded by the launched shell.
start_recorded stack --log sim-stack \
  bash -c 'source scripts/setup.bash && exec ros2 launch lekiwi_rmf bringup.launch.py profile:=sim "$@"' \
  sim-stack "$@"

launcher_release
echo "simulation: starting (PID $(<"$RUNTIME_DIR/stack.pid"))"
echo "logs: $LOGS/sim-stack.log"
echo "stop with: scripts/ros-stop.sh"
