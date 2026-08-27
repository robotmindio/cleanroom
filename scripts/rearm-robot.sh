#!/usr/bin/env bash
# Explicitly re-arm a running real-robot stack after inspecting the robot.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's setup.bash reads unset variables, so disable nounset while sourcing it.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

exec ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
