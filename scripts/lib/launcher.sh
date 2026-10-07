#!/usr/bin/env bash
# Source-only helpers shared by the manual launchers (up.sh, pi-up.sh,
# workstation-up.sh, sim-up.sh) and ros-stop.sh. Process ownership is tracked by
# recorded PIDs, never by process-name sweeps: a shared workstation can run
# another robot's stack with the same executable names.

launcher_dirs() { # sets LOGS and RUNTIME_DIR from LEKIWI_LOGS / LEKIWI_RUNTIME_DIR
  LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
  RUNTIME_DIR="${LEKIWI_RUNTIME_DIR:-$LOGS/runtime}"
}

launcher_init() { # launcher_init <name>: create the directories and hold <name>-start.lock on fd 9
  launcher_dirs
  mkdir -p "$LOGS" "$RUNTIME_DIR"
  chmod 700 "$RUNTIME_DIR"
  export LEKIWI_RUNTIME_DIR="$RUNTIME_DIR"
  # A launch takes a few seconds to become visible. Serialize the whole startup
  # window so two near-simultaneous invocations cannot both start a stack. Every
  # long-running child closes fd 9 (start_recorded does), so the lock is released
  # when this shell exits or calls launcher_release.
  exec 9>"$LOGS/$1-start.lock"
  if ! flock -n 9; then
    echo "$0: startup is already in progress" >&2
    exit 0
  fi
}

launcher_release() { # release the startup lock before returning to the caller
  flock -u 9
  exec 9>&-
}

start_recorded() { # start_recorded <kind> [--log <name>] <command...>: own session, PID in <kind>.pid
  local kind=$1 log_name=$1
  shift
  if [[ ${1:-} == --log ]]; then
    log_name=$2
    shift 2
  fi
  # A non-leader background child makes setsid exec in place, so $! is the new
  # session and process-group leader that ros-stop.sh signals.
  setsid "$@" >"$LOGS/$log_name.log" 2>&1 9>&- &
  printf '%s\n' "$!" > "$RUNTIME_DIR/$kind.pid"
}

process_signature() { # process_signature <kind>: extended regex for that kind's command line
  case $1 in
    stack) echo 'bringup\.launch\.py|scripts/ros-start\.sh' ;;
    host) echo 'robot-host\.sh|torque-host\.py|lerobot\.robots\.lekiwi\.lekiwi_host' ;;
    rviz) echo 'rviz2|scripts/rviz\.sh' ;;
    astra) echo 'ros-astra\.sh|pi_astra\.launch\.py|astra_camera_node' ;;
    cameras) echo 'ros-cameras\.sh|pi_cameras\.launch\.py' ;;
    lidar) echo 'ros-lidar\.sh|ldlidar_stl_ros2_node' ;;
    zenoh) echo 'ros-zenoh\.sh|zenoh-bridge-ros2dds' ;;
    *) return 1 ;;
  esac
}

pid_matches() { # pid_matches <pid> <kind>: the live PID still runs that kind of process
  local pid=$1 signature command
  signature=$(process_signature "$2") || return 1
  [ -r "/proc/$pid/cmdline" ] || return 1
  command=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  [[ $command =~ $signature ]]
}

recorded_running() { # recorded_running <kind>: <kind>.pid names a live process of that kind
  local pid
  [ -r "$RUNTIME_DIR/$1.pid" ] || return 1
  pid=$(<"$RUNTIME_DIR/$1.pid")
  [[ $pid =~ ^[1-9][0-9]*$ ]] && pid_matches "$pid" "$1"
}

refuse_while_stack_service_runs() {
  # A boot service owns its stack; starting another one would put two drivers on one robot.
  if command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet lekiwi-stack.service; then
    echo "$0: lekiwi-stack.service is running -- stop it first: sudo systemctl stop lekiwi-stack.service" >&2
    exit 1
  fi
}
