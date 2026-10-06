#!/usr/bin/env bash
# Publish the USB-attached Astra Pro independently of motors, lidar, and V4L2 cameras.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
# shellcheck source=/dev/null
source scripts/lib/self-heal.sh

source_device_ros_env

self_heal
exec ros2 launch lekiwi_rmf pi_astra.launch.py
