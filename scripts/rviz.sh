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

# Nav2's stock view with its Gazebo Realsense group swapped for the front camera.
# Read from the source tree, not the install space, so an edit takes effect without a
# rebuild.
#
# One Image display only: a second one aborts RViz before the window opens, with
# "Cannot create GL vertex buffer" out of Ogre. Not a GPU shortage -- it also happens
# under LIBGL_ALWAYS_SOFTWARE, and with both displays on the same live topic.
#
# On this machine even the single display comes up unchecked and never subscribes:
# Ogre cannot give the image panel its render texture, so RViz disables the display.
# Until that is fixed, camera frames come from its own process:
#   ros2 run rqt_image_view rqt_image_view /camera/front/image_raw
exec rviz2 -d config/lekiwi.rviz "$@"
