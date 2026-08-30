#!/usr/bin/env bash
# Publish the LD06 attached to this device machine for the compute stack.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
if [ -f "$workspace/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  # shellcheck source=/dev/null
  source "$workspace/install/setup.bash"
  set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$PWD/config/cyclonedds.xml"
else
  # shellcheck source=/dev/null
  source scripts/setup-pi.bash
fi

default_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
legacy_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0
PORT=${LEKIWI_LIDAR_PORT:-$default_port}
if [ ! -e "$PORT" ] && [ -e "$legacy_port" ]; then PORT=$legacy_port; fi
if [ ! -e "$PORT" ]; then
  echo "$0: waiting for LD06 serial port: $PORT" >&2
  while [ ! -e "$PORT" ]; do
    if [ -z "${LEKIWI_LIDAR_PORT:-}" ] && [ -e "$legacy_port" ]; then PORT=$legacy_port; break; fi
    sleep 2
  done
fi

exec ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD06 -p topic_name:=/pi/lidar/scan \
  -p frame_id:=laser -p port_name:="$PORT" -p port_baudrate:=230400
