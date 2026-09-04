#!/usr/bin/env bash
# Publish the USB-attached Astra Pro independently of motors, lidar, and V4L2 cameras.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/runtime-common.sh

if [ -f "${LEKIWI_WS:-$HOME/lekiwi_ws}/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source scripts/setup.bash
  set -u
else
  # shellcheck source=/dev/null
  source scripts/setup-pi.bash
fi

exec ros2 launch lekiwi_rmf pi_astra.launch.py
