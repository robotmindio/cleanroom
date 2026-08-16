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

# Nav2's stock view minus its Gazebo Realsense group. Read from the source tree, not the
# install space, so an edit takes effect without a rebuild.
#
# No camera here, deliberately. This GPU cannot give RViz a second render window beside
# the Nav2 scene: make an Image panel visible and Ogre aborts the process with "Cannot
# create GL vertex buffer" or GLX "failed to create drawable" -- reproducible with the
# map displays off, under LIBGL_ALWAYS_SOFTWARE, and with 7 GB of RAM free. (The same
# panels open fine in rtabmap_examples' lighter config, so it is the combination.)
# Leave an Image display in the config and RViz just shows it unticked, subscribing to
# nothing, which is worse than not offering it. Cameras come from scripts/cameras.sh.
exec rviz2 -d config/lekiwi.rviz "$@"
