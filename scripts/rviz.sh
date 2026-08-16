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
# An Image display only comes up enabled if `Window Geometry: QMainWindow State` holds a
# dock widget with the display's exact name -- RViz binds the display's enabled state to
# its panel, and Qt drops a dock the saved layout never mentions. Renaming the display in
# the YAML is not enough; the name is also a length-prefixed UTF-16 string inside that
# hex blob, which is why config/lekiwi.rviz carries a patched one. Rename the display and
# the camera goes dark again.
#
# The inherited dock is saved hidden, so the panel does not appear on its own: untick and
# re-tick Front Camera to pop it out, then File > Save Config to keep it. For frames
# without the ceremony:
#   ros2 run rqt_image_view rqt_image_view /camera/front/image_raw
exec rviz2 -d config/lekiwi.rviz "$@"
