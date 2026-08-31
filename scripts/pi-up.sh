#!/usr/bin/env bash
# Start everything that runs on this device machine: the motor-bus host,
# camera publishers, and LD06 scan publisher.
# Usage: scripts/pi-up.sh
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/runtime-common.sh
LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"

exec 9>"$LOGS/pi-up-start.lock"
if ! flock -n 9; then
  echo "$0: startup is already in progress" >&2
  exit 0
fi

host_up() { ss -tln | grep -q ':5555' && ss -tln | grep -q ':5557'; }
motion_host_up() { ss -tln | grep -q ':5555'; }
cameras_up() { pgrep -f '[v]4l2_camera_node' >/dev/null; }
lidar_up() {
  systemctl is-active --quiet lekiwi-lidar.service 2>/dev/null ||
    pgrep -f '[r]os-lidar.sh|[l]dlidar_stl_ros2_node' >/dev/null
}

# Motors and cameras are separate processes on purpose: one reader per USB
# device, and a stalled camera frame must never abort the motor host.
if host_up; then
  echo "host: already running"
elif motion_host_up; then
  echo "host on TCP 5555 lacks torque safety on TCP 5557; restart it from this repository first" >&2
  exit 1
else
  setsid scripts/robot-host.sh --no-cameras >"$LOGS/host.log" 2>&1 &
  wait_for 90 host_up || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    exit 1
  }
  echo "host: up"
fi

if cameras_up; then
  echo "cameras: already running"
else
  setsid scripts/ros-cameras.sh >"$LOGS/cameras.log" 2>&1 &
  # The camera nodes appear within seconds of the launch starting; a missing
  # calibration or camera makes ros-cameras.sh fail fast with the reason.
  wait_for 30 cameras_up || {
    echo "camera publishers did not come up -- see $LOGS/cameras.log" >&2
    tail -5 "$LOGS/cameras.log" >&2
    exit 1
  }
  echo "cameras: up"
fi

if lidar_up; then
  echo "lidar: already running"
else
  setsid scripts/ros-lidar.sh >"$LOGS/lidar.log" 2>&1 &
  echo "lidar: starting (waits for the LD06 serial port)"
fi

flock -u 9
exec 9>&-
echo "device side ready -- logs in $LOGS"
