#!/usr/bin/env bash
# Internal process-group leader for sim-up.sh.  It records only its own launch,
# so ros-stop.sh can force-stop a stuck external Gazebo child without sweeping
# unrelated ROS processes on a shared server.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
runtime_dir="${LEKIWI_RUNTIME_DIR:-${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/runtime}"
mkdir -p "$runtime_dir"
chmod 700 "$runtime_dir"
printf '%s\n' "$$" > "$runtime_dir/stack.pid"

# ROS setup scripts reference optional variables while -u is active.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

exec ros2 launch lekiwi_rmf bringup.launch.py mode:=sim "$@"
