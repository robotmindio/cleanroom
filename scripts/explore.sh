#!/usr/bin/env bash
# Start attended exploration with the tracked server limits; Ctrl-C cancels it.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's setup.bash reads unset variables.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

exec ros2 action send_goal /robot/explore lekiwi_rmf/action/Explore \
  '{revisit_known: true, max_duration_sec: 0.0, max_radius_m: 0.0}' --feedback
