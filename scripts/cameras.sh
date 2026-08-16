#!/usr/bin/env bash
# One viewer window per camera topic that is actually being published.
# Usage: scripts/cameras.sh [topic...]        # default: every /**/image_raw with a publisher
#
# Not an RViz display: on this hardware RViz aborts with "Cannot create GL vertex buffer"
# as soon as an Image panel opens next to the Nav2 scene, and comes up with the display
# silently unticked when the panel is hidden instead. rqt_image_view draws in Qt, in its
# own process, so a camera window cannot take the navigation view down with it.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

topics=("$@")
if [ ${#topics[@]} -eq 0 ]; then
  # `ros2 topic list` alone also lists topics whose only endpoint is a subscriber
  mapfile -t topics < <(ros2 topic find sensor_msgs/msg/Image | grep -E '/image_raw$' || true)
fi
[ ${#topics[@]} -gt 0 ] || { echo "$0: no camera is publishing -- is the stack up?" >&2; exit 1; }

for topic in "${topics[@]}"; do
  echo "opening $topic"
  ros2 run rqt_image_view rqt_image_view "$topic" &
done
wait
