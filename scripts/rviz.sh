#!/usr/bin/env bash
# RViz on a running stack: map, costmaps, robot model, TF, plans, the goal/initial-pose
# tools, and both camera panels.
# Usage: scripts/rviz.sh [extra rviz2 args...]
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's setup.bash reads unset variables, so `set -u` has to stand down for it.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

# Read from the source tree, not the install space, so an edit takes effect without a
# rebuild.
#
# The camera panels in config/lekiwi.rviz are more delicate than they look. An Image
# display is only enabled when `Window Geometry: QMainWindow State` -- Qt's saved dock
# layout, stored as a hex blob -- holds a dock of the same name, because RViz ties the
# display's enabled state to its panel. So the config borrows the layout from
# rtabmap_examples, which budgets for two image docks, and the dock names inside the blob
# are patched to match the display names. Three things break it:
#
#   - a panel set the layout does not budget for (Nav2's side panels) squeezes the image
#     docks to zero height, and a zero-height render window aborts Ogre with
#     "Cannot create GL vertex buffer";
#   - two displays sharing one name leaves Qt's dock matching ambiguous, and the panels
#     came back hidden on about half the launches;
#   - renaming a display in the YAML alone silently un-ticks it, since the blob still
#     carries the old name.
#
# Editing displays is safe; editing panels, display names, or the window state is not.
#
# Starting before the cameras publish also loses the panels, so wait for them. A camera
# that never appears is not fatal -- the other panel and the whole navigation view still
# come up.
for topic in $(grep -o '/camera/[a-z]*/image_raw' config/lekiwi.rviz | sort -u); do
  for _ in $(seq 30); do
    [ -n "$(ros2 topic info "$topic" 2>/dev/null | grep 'Publisher count: [1-9]')" ] && break
    sleep 1
  done
done
exec rviz2 -d config/lekiwi.rviz "$@"
