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
# shellcheck source=/dev/null
source scripts/lib/launcher.sh
launcher_init pi-up

astra_built() {
  # A staged release keeps the driver in its shared dependency overlay.
  local workspace=${LEKIWI_WS:-$HOME/lekiwi_ws} overlay
  overlay=$(cat "$workspace/install/.lekiwi-overlay" 2>/dev/null || true)
  [[ -x $workspace/install/astra_camera/lib/astra_camera/astra_camera_node ||
     ( -n $overlay && -x $overlay/install/astra_camera/lib/astra_camera/astra_camera_node ) ]]
}
# This device is dedicated to one robot, so a process of the kind running under
# systemd or another shell also counts as running; nothing here is signalled.
running() { # running <kind> <unit>
  systemctl is-active --quiet "$2" 2>/dev/null ||
    pgrep -f -- "$(process_signature "$1")" >/dev/null
}
# Readiness, not ownership: the publishers exist only once the launch has started them.
cameras_up() { pgrep -f '[v]4l2_camera_node' >/dev/null; }

# Motors and cameras are separate processes on purpose: one reader per USB
# device, and a stalled camera frame must never abort the motor host.
if lekiwi_safety_ports_listening; then
  echo "host: already running"
elif lekiwi_motion_port_listening; then
  echo "host on TCP 5555 lacks torque safety on TCP 5557; restart it from this repository first" >&2
  exit 1
else
  start_recorded host scripts/robot-host.sh
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
elif running astra lekiwi-astra.service; then
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

if running lidar lekiwi-lidar.service; then
  echo "lidar: already running"
else
  start_recorded lidar scripts/ros-lidar.sh
  echo "lidar: starting (waits for the LD06 serial port)"
fi

# The only way the sensors above reach the compute machine: DDS stays local.
if running zenoh lekiwi-zenoh.service; then
  echo "sensor bridge: already running"
else
  start_recorded zenoh scripts/ros-zenoh.sh
  echo "sensor bridge: starting"
fi

launcher_release
echo "device side ready -- logs in $LOGS"
