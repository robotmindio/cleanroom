#!/usr/bin/env bash
# Stop a bringup and every node it left behind, plus the LeRobot host if one is running.
# Usage: scripts/ros-stop.sh
#
# `ros2 launch` relays SIGINT to its nodes and waits for their teardown. The exact-tree
# fallback below handles old launches started through nohup, which inherit ignored SIGINT.
set -Eeuo pipefail

mapfile -t launch_pids < <(pgrep -f 'ros2 launch lekiwi_rmf' || true)
mapfile -t pi_camera_pids < <(pgrep -f 'ros2 launch .*pi_cameras\.launch\.py' || true)
# Processes owned by the boot units are not ours to kill: systemd would undo it
# (restart) or, for a clean exit, leave the unit silently down. Either way the
# fix belongs to systemctl.
service_managed=false
if command -v systemctl >/dev/null && systemctl is-active --quiet lekiwi-cameras.service; then
  if (( ${#pi_camera_pids[@]} )); then
    echo "lekiwi-cameras.service is active -- leaving its camera publisher running"
    echo "(stop it with: sudo systemctl stop lekiwi-cameras.service)"
    service_managed=true
  fi
  pi_camera_pids=()
fi
mapfile -t rviz_pids < <(pgrep -f '[r]viz2 -d .*lekiwi\.rviz' || true)
# A launcher can die before relaying SIGINT, leaving its Python nodes re-parented to the
# user manager. Match this stack's executables, not every Python ROS node.
mapfile -t driver_pids < <(pgrep -f '[/]lekiwi_rmf/lib/lekiwi_rmf/lekiwi_driver([[:space:]]|$)' || true)
mapfile -t fleet_adapter_pids < <(pgrep -f '[/]free_fleet_adapter/lib/free_fleet_adapter/fleet_adapter\.py([[:space:]]|$)' || true)
# Calibration starts this node outside a launch process. Include it so Ctrl-C or this
# stop script never leaves the front camera busy for the next `scripts/up.sh`.
mapfile -t calibration_camera_pids < <(pgrep -f '[/]v4l2_camera/v4l2_camera_node.*__ns:=/camera/front' || true)
launch_pids+=("${pi_camera_pids[@]}" "${rviz_pids[@]}" "${driver_pids[@]}" "${fleet_adapter_pids[@]}" "${calibration_camera_pids[@]}")
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
# motor bus, and leaving it behind is what makes the next `robot-host.sh` refuse to start.
# robot-host.sh supervises a crashed LeRobot process, therefore stop the supervisor too
# or it would immediately reconnect after this script stops its child.
host_stopped=false
if command -v systemctl >/dev/null && systemctl is-active --quiet lekiwi-host.service; then
  echo "lekiwi-host.service is active -- left running; stop it with:"
  echo "  sudo systemctl stop lekiwi-host.service"
else
  # `pkill -f` considers a terminal command line too, so it can kill the shell which
  # invoked this script merely because that command mentioned robot-host.sh. Restrict the
  # selection to actual bash processes and inspect their argument vector instead.
  host_wrapper_pids=()
  for pid in $(pgrep -x bash 2>/dev/null || true); do
    cmdline=$(tr '\0' ' ' 2>/dev/null < "/proc/$pid/cmdline" || true)
    case "$cmdline" in
      *" scripts/robot-host.sh "*|*"/scripts/robot-host.sh "*) host_wrapper_pids+=("$pid") ;;
    esac
  done
  for pid in "${host_wrapper_pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null && host_stopped=true || true
  done
  if pkill -TERM -f '[l]erobot\.robots\.lekiwi\.lekiwi_host'; then
    host_stopped=true
  fi
  sleep 1
  for pid in "${host_wrapper_pids[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
fi
if "$host_stopped"; then
  echo "ROS stack and LeRobot host stopped."
else
  echo "ROS stack stopped."
fi
