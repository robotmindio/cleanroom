#!/usr/bin/env bash
# Stop only processes recorded by this repository's launchers.
#
# A process-name sweep can kill a different robot's Nav2/RViz instance on a
# shared workstation. up.sh, pi-up.sh and ros-start.sh record their own
# process-group leaders, so cleanup remains complete for this stack without that
# collateral damage. Usage: scripts/ros-stop.sh
set -Eeuo pipefail

runtime_dir="${LEKIWI_RUNTIME_DIR:-${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/runtime}"

pid_matches() { # pid_matches <pid> <stack|host|rviz|astra|cameras|lidar|zenoh>
  local pid=$1 kind=$2 command
  [ -r "/proc/$pid/cmdline" ] || return 1
  command=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  case "$kind" in
    stack) [[ $command == *"ros2 launch lekiwi_rmf"* || $command == *"bringup.launch.py"* ]] ;;
    host) [[ $command == *"robot-host.sh"* || $command == *"torque-host.py"* || $command == *"lerobot.robots.lekiwi.lekiwi_host"* ]] ;;
    rviz) [[ $command == *"rviz2"* ]] ;;
    astra) [[ $command == *"ros-astra.sh"* || $command == *"pi_astra.launch.py"* ]] ;;
    cameras) [[ $command == *"ros-cameras.sh"* || $command == *"pi_cameras.launch.py"* ]] ;;
    lidar) [[ $command == *"ros-lidar.sh"* || $command == *"ldlidar_stl_ros2"* ]] ;;
    zenoh) [[ $command == *"ros-zenoh.sh"* || $command == *"zenoh-bridge-ros2dds"* ]] ;;
  esac
}

stop_recorded() { # stop_recorded <file> <kind>
  local file=$1 kind=$2 pid pgid deadline grouped=0
  [ -r "$file" ] || return 1
  pid=$(<"$file")
  if [[ ! $pid =~ ^[1-9][0-9]*$ ]] || ! kill -0 "$pid" 2>/dev/null; then
    rm -f -- "$file"
    return 1
  fi
  if ! pid_matches "$pid" "$kind"; then
    echo "$0: refusing to signal unrecognised PID $pid from $file" >&2
    return 1
  fi
  pgid=$(ps -o pgid= -p "$pid" | tr -d '[:space:]')
  if [[ $pgid == "$pid" ]]; then grouped=1; fi
  echo "stopping recorded $kind (PID $pid)"
  if (( grouped )); then kill -INT -- "-$pid" 2>/dev/null || true; else kill -INT "$pid" 2>/dev/null || true; fi
  deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    if (( grouped )); then kill -0 -- "-$pid" 2>/dev/null || break; else kill -0 "$pid" 2>/dev/null || break; fi
    sleep 1
  done
  if (( grouped )); then
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  else
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  fi
  sleep 3
  if (( grouped )); then
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  else
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  fi
  rm -f -- "$file"
  return 0
}

# A systemd unit owns its cgroup and restart policy; do not fight it with raw
# signals. The explicit command below keeps the ownership boundary visible.
unit_active() { # unit_active <unit>
  command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet "$1"
}

stopped=0
stop_kind() { # stop_kind <kind> [owning-unit]
  local kind=$1 unit=${2:-}
  if [[ -n $unit ]] && unit_active "$unit"; then
    echo "$unit is active -- left running; stop it with: sudo systemctl stop $unit"
  elif stop_recorded "$runtime_dir/$kind.pid" "$kind"; then
    stopped=1
  fi
}

stop_kind rviz
stop_kind stack lekiwi-stack.service
stop_kind host lekiwi-host.service
stop_kind astra lekiwi-astra.service
stop_kind cameras lekiwi-cameras.service
stop_kind lidar lekiwi-lidar.service
stop_kind zenoh lekiwi-zenoh.service

if (( stopped )); then
  echo "Recorded LeKiwi processes stopped."
else
  echo "No recorded LeKiwi process groups were running."
fi
