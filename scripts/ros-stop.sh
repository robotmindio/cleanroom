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

stop_recorded() { # stop_recorded <file> <kind> [owning-unit]
  local file=$1 kind=$2 unit=${3:-} pid pgid deadline grouped=0 ownership_status
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
  if [[ -n $unit ]]; then
    if unit_owns_pid "$pid" "$unit"; then
      echo "$unit owns recorded $kind (PID $pid) -- left running; stop it with: sudo systemctl stop $unit"
      return 1
    else
      ownership_status=$?
      if (( ownership_status == 2 )); then
        echo "$0: cannot verify whether $unit owns recorded $kind (PID $pid); left it running" >&2
        return 1
      fi
    fi
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

# Leave a process alone only when an active unit actually owns its cgroup.
unit_active() { # unit_active <unit>
  command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet "$1"
}

unit_owns_pid() { # 0: unit owns pid, 1: it does not, 2: ownership cannot be checked
  local pid=$1 unit=$2 unit_cgroup pid_cgroups cgroup
  unit_active "$unit" || return 1
  unit_cgroup=$(systemctl show --property=ControlGroup --value "$unit" 2>/dev/null) || return 2
  [[ -n $unit_cgroup && -r /proc/$pid/cgroup ]] || return 2
  pid_cgroups=$(awk -F: '{print $3}' "/proc/$pid/cgroup") || return 2
  [[ -n $pid_cgroups ]] || return 2
  while IFS= read -r cgroup; do
    [[ $cgroup == "$unit_cgroup" || $cgroup == "$unit_cgroup/"* ]] && return 0
  done <<<"$pid_cgroups"
  return 1
}

stopped=0
stop_kind() { # stop_kind <kind> [owning-unit]
  local kind=$1 unit=${2:-}
  if stop_recorded "$runtime_dir/$kind.pid" "$kind" "$unit"; then
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
