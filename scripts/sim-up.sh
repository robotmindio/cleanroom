#!/usr/bin/env bash
# Start a managed headless simulation after proving this host can render Ogre2.
# Usage: scripts/sim-up.sh [extra ROS launch args...]
set -Eeuo pipefail

cd "$(dirname "$0")/.."
logs_dir="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
runtime_dir="${LEKIWI_RUNTIME_DIR:-$logs_dir/runtime}"
mkdir -p "$logs_dir" "$runtime_dir"
chmod 700 "$runtime_dir"

exec 9>"$logs_dir/sim-up-start.lock"
if ! flock -n 9; then
  echo "$0: simulation startup is already in progress" >&2
  exit 0
fi

# Do not silently stop a recorded real stack just because it happens to share
# the default runtime directory. Its owner must choose to stop it explicitly.
if [[ -e $runtime_dir/stack.pid ]]; then
  echo "$0: a recorded LeKiwi stack exists; inspect it or run scripts/ros-stop.sh first" >&2
  exit 1
fi

scripts/sim-renderer-check.py

# sim-start.sh execs ros2 launch in this new session, making its PID both the
# recorded stack identity and its process-group leader for ros-stop.sh.
setsid scripts/sim-start.sh "$@" 9>&- >"$logs_dir/sim-stack.log" 2>&1 &
stack_pid=$!
printf '%s\n' "$stack_pid" > "$runtime_dir/stack.pid"

flock -u 9
exec 9>&-
echo "simulation: starting (PID $stack_pid)"
echo "logs: $logs_dir/sim-stack.log"
echo "stop with: scripts/ros-stop.sh"
