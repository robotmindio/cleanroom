#!/usr/bin/env bash
# Keyboard driving, in a shell that does not need the ROS environment sourced first.
# Usage: scripts/teleop.sh
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's setup.bash reads unset variables, so `set -u` has to stand down for it.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

exec ros2 run lekiwi_rmf teleop.py "$@"
