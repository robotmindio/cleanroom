#!/usr/bin/env bash
# Publish the USB-attached Astra Pro independently of motors, lidar, and V4L2 cameras.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
source_device_ros_env

exec ros2 launch lekiwi_rmf pi_astra.launch.py
