#!/usr/bin/env bash
# Start everything that runs on this device machine: the motor-bus host,
# Astra and camera publishers, LD06 scan publisher, and the sensor bridge.
# Usage: scripts/pi-up.sh
# Each process it starts is recorded under the runtime directory, so
# scripts/ros-stop.sh stops them; boot services are left to systemd.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"
RUNTIME_DIR="${LEKIWI_RUNTIME_DIR:-$LOGS/runtime}"
mkdir -p "$RUNTIME_DIR"
chmod 700 "$RUNTIME_DIR"

# The lock is held by this shell alone: every long-running child below closes
# descriptor 9, so a failed start releases it when this script exits and the
# next run is not told that startup is still in progress.
exec 9>"$LOGS/pi-up-start.lock"
if ! flock -n 9; then
  echo "$0: startup is already in progress" >&2
  exit 0
fi

start_recorded() { # start_recorded <kind> <command...>: own session, PID recorded for ros-stop.sh
  local kind=$1
  shift
  setsid "$@" >"$LOGS/$kind.log" 2>&1 9>&- &
  printf '%s\n' "$!" > "$RUNTIME_DIR/$kind.pid"
}

astra_built() {
  [[ -x ${LEKIWI_WS:-$HOME/lekiwi_ws}/install/astra_camera/lib/astra_camera/astra_camera_node ]]
}
astra_up() {
  systemctl is-active --quiet lekiwi-astra.service 2>/dev/null ||
    pgrep -f '[a]stra_camera_node|[r]os-astra.sh' >/dev/null
}
cameras_up() { pgrep -f '[v]4l2_camera_node' >/dev/null; }
lidar_up() {
  systemctl is-active --quiet lekiwi-lidar.service 2>/dev/null ||
    pgrep -f '[r]os-lidar.sh|[l]dlidar_stl_ros2_node' >/dev/null
}

# Motors and cameras are separate processes on purpose: one reader per USB
# device, and a stalled camera frame must never abort the motor host.
if lekiwi_safety_ports_listening; then
  echo "host: already running"
elif lekiwi_motion_port_listening; then
  echo "host on TCP 5555 lacks torque safety on TCP 5557; restart it from this repository first" >&2
  exit 1
else
  start_recorded host scripts/robot-host.sh --no-cameras
  wait_for 90 lekiwi_safety_ports_listening || {
    echo "host did not come up -- see $LOGS/host.log" >&2
    tail -5 "$LOGS/host.log" >&2
    exit 1
  }
  echo "host: up"
fi

# Astra shares a USB hub with the motor-bus adapter; starting it after the host
# matches the boot-service order, and the host reconnects after the hub resets.
if ! astra_built; then
  echo "astra: astra_camera is not built in ${LEKIWI_WS:-$HOME/lekiwi_ws} -- skipping"
elif astra_up; then
  echo "astra: already running"
else
  start_recorded astra scripts/ros-astra.sh
  echo "astra: starting"
fi

if cameras_up; then
  echo "cameras: already running"
else
  start_recorded cameras scripts/ros-cameras.sh
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
  start_recorded lidar scripts/ros-lidar.sh
  echo "lidar: starting (waits for the LD06 serial port)"
fi

# The only way the sensors above reach the compute machine: DDS stays local.
if systemctl is-active --quiet lekiwi-zenoh.service 2>/dev/null ||
    pgrep -f '[z]enoh-bridge-ros2dds' >/dev/null; then
  echo "sensor bridge: already running"
else
  start_recorded zenoh scripts/ros-zenoh.sh
  echo "sensor bridge: starting"
fi

flock -u 9
exec 9>&-
echo "device side ready -- logs in $LOGS"
