#!/usr/bin/env bash
# RViz on a running stack, with Nav2's stock view: map, costmaps, robot model, TF,
# plans, and the goal/initial-pose tools.
# Usage: scripts/rviz.sh [extra rviz2 args...]
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's setup.bash reads unset variables, so `set -u` has to stand down for it.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

# ponytail: Nav2 ships a view that covers everything this stack publishes except the
# camera -- add an Image display on /camera/front/image_raw if you want frames too.
exec rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz "$@"
