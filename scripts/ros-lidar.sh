#!/usr/bin/env bash
# Publish the LD06 attached to this device machine for a remote ROS stack.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
if [ -f "${LEKIWI_WS:-$HOME/lekiwi_ws}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source scripts/setup.bash
else
  # shellcheck source=/dev/null
  source scripts/setup-pi.bash
fi

default_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
legacy_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if0-port0
PORT=${LEKIWI_LIDAR_PORT:-$default_port}
if [ ! -e "$PORT" ] && [ -e "$legacy_port" ]; then PORT=$legacy_port; fi
[ -e "$PORT" ] || { echo "$0: LD06 serial port is absent: $PORT" >&2; exit 1; }

exec ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD06 -p topic_name:=/pi/lidar/scan \
  -p frame_id:=laser -p port_name:="$PORT" -p port_baudrate:=230400
