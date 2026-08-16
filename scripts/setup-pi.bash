#!/usr/bin/env bash
# Source this on the robot's Pi before launching the cameras; do not execute it.
# The Pi runs ros-base only, so this sources ROS itself rather than a colcon workspace.

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  printf 'ROS is not installed on this Pi. Run scripts/install-pi.sh on Ubuntu 24.04.\n' >&2
  return 1 2>/dev/null
  exit 1
fi

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Same DDS settings as the workstation: without a matching participant policy the two
# machines discover each other inconsistently.
export CYCLONEDDS_URI="file://$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/cyclonedds.xml"
