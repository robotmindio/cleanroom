#!/usr/bin/env bash
# Everything, in order: LeRobot host, ROS stack, RViz.
# Usage: scripts/up.sh [extra launch args...]     # e.g. slam_mode:=localization
# Stop it all with scripts/ros-stop.sh
#
# Only for the wired robot, where the motors hang off this machine. With a Pi on the
# robot the host runs there (see HARDWARE.md) and this machine runs scripts/ros-start.sh.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
# shellcheck source=/dev/null
source scripts/lib/launcher.sh
launcher_init up

# Keep bounded, repository-owned startup diagnostics. Stack logs are replaced
# per launch, but dated crash copies and host logs otherwise accumulate forever.
find "$LOGS" -type f \( -name '*.log.*' -o -name '*.log-*' \) -mtime +14 -delete 2>/dev/null || true

recorded_host_up() {
  recorded_running host && lekiwi_safety_ports_listening
}

service_host_up() {
  command -v systemctl >/dev/null 2>&1 \
    && systemctl is-active --quiet lekiwi-host.service \
    && lekiwi_safety_ports_listening
}

host_up() {
  recorded_host_up || service_host_up
}

WRIST="${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}"

require_free_cameras() {
  local front wrist
  front="${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}"
  wrist="$WRIST"
  for device in "$front" "$wrist"; do
    [ -z "$device" ] || [ "$device" = none ] || ! fuser -s "$(readlink -f "$device")" || {
      echo "camera is already in use: $device -- stop the process holding it before starting ROS" >&2
      exit 1
    }
  done
}

# A launcher can die before it relays SIGINT, leaving its own nodes re-parented.
# Process-name sweeps are unsafe on a shared workstation: another robot can have
# the same Nav2 executable names.  Stop only this launcher's recorded process
# groups; an unrecorded stack is deliberately left for its owner to manage.
refuse_while_stack_service_runs
if [[ -e $RUNTIME_DIR/stack.pid || -e $RUNTIME_DIR/host.pid || -e $RUNTIME_DIR/rviz.pid ]]; then
  echo "Stopping this launcher's recorded stack before restart." >&2
  scripts/ros-stop.sh
fi

require_camera_calibration
require_free_cameras

# Only clean up a host this invocation created. A manually started or systemd
# managed host is intentionally left alone, but a failed up.sh must never leave
# its own retrying host holding the serial bus forever.
host_started_here=0
host_launcher_pid=""
stop_host_started_here() {
  (( host_started_here )) || return 0
  kill -TERM -- "-$host_launcher_pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$host_launcher_pid" 2>/dev/null || true
  wait "$host_launcher_pid" 2>/dev/null || true
  rm -f -- "$RUNTIME_DIR/host.pid"
  host_started_here=0
}

stack_started_here=0
stack_launcher_pid=""
stop_stack_started_here() {
  (( stack_started_here )) || return 0
  kill -TERM -- "-$stack_launcher_pid" 2>/dev/null || true
  local deadline=$((SECONDS + 10))
  while kill -0 -- "-$stack_launcher_pid" 2>/dev/null && (( SECONDS < deadline )); do sleep 1; done
  kill -KILL -- "-$stack_launcher_pid" 2>/dev/null || true
  wait "$stack_launcher_pid" 2>/dev/null || true
  rm -f -- "$RUNTIME_DIR/stack.pid"
  stack_started_here=0
}

# The ZMQ command socket is the honest ready signal: the host binds it only after
# connect() has found all nine servos. It must belong either to this launcher's
# recorded process group or to the repository-managed systemd service.
if host_up; then
  echo "host: already running"
elif lekiwi_motion_port_listening; then
  echo "host on TCP 5555 lacks the required torque-safety endpoint on TCP 5557" >&2
  echo "restart it from this repository (or restart lekiwi-host.service) before launching ROS." >&2
  exit 1
else
  # ROS reads the local cameras directly; the motor host serves none.
  start_recorded host scripts/robot-host.sh
  host_launcher_pid=$(<"$RUNTIME_DIR/host.pid")
  host_started_here=1
  wait_for 90 host_up || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    stop_host_started_here
    exit 1
  }
  echo "host: up"
fi

start_recorded stack scripts/ros-start.sh "$@"
stack_launcher_pid=$(<"$RUNTIME_DIR/stack.pid")
stack_started_here=1
wait_for 120 grep -q 'Connected to LeKiwi host' "$LOGS/stack.log" || {
  echo "driver never reached the host -- see $LOGS/stack.log" >&2
  stop_stack_started_here
  stop_host_started_here
  exit 1
}
echo "stack: up"

# RViz adds an OpenGL renderer plus two raw-image subscriptions.  On the 4 GB
# robot computer that can starve the camera safety path; use scripts/rviz.sh
# explicitly from a workstation when visualisation is needed.
rm -f -- "$RUNTIME_DIR/rviz.pid"
launcher_release
echo "rviz: not started automatically; run scripts/rviz.sh on a workstation when needed"
echo "logs in $LOGS -- stop everything with scripts/ros-stop.sh"
