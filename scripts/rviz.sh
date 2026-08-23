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
#   - the blob also carries a pixel height per dock, and if the visible docks in one
#     column add up to more than the column, Qt gives up on the overflowing ones and
#     floats them off the bottom of the screen -- which reads as "the cameras are gone",
#     with the displays un-ticked to match. The left column is 978 px: Displays 470 plus
#     two 209 px camera docks and their handles leaves room to spare, so growing Displays
#     is what to be careful about;
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
mkdir -p "${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"

# `up.sh` and `workstation-up.sh` both deliberately return while RViz is coming up.  A
# second start can otherwise get here while the first one is still waiting for topics,
# so neither sees an rviz2 process to replace and two expensive RViz instances appear.
# Keep the lock only while starting: after exec the normal "replace the old RViz" path
# below remains available to a later, intentional `scripts/rviz.sh` invocation.
exec 9>"${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/rviz-start.lock"
if ! flock -n 9; then
  echo "rviz: a launch is already in progress"
  exit 0
fi

mapfile -t old_rviz_pids < <(pgrep -f '[r]viz2 -d .*lekiwi\.rviz' || true)
for pid in "${old_rviz_pids[@]}"; do
  # A previous RViz keeps its in-memory, user-modified dock state even after a new stack
  # starts. Replace it so the fresh copy below is the one visible layout.
  kill -TERM "$pid" 2>/dev/null || true
done

# Do not start a second OpenGL renderer until the old one has released its display and
# camera subscriptions.  On a normal close this is near-instant; SIGKILL is reserved
# for a hung renderer after a short grace period.
for _ in $(seq 25); do
  still_running=false
  for pid in "${old_rviz_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      still_running=true
      break
    fi
  done
  "$still_running" || break
  sleep 0.2
done
for pid in "${old_rviz_pids[@]}"; do
  kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
done

topics=$(grep -o '/camera/[a-z]*/image_raw' config/lekiwi.rviz | sort -u)
for topic in $topics; do
  for _ in $(seq 30); do
    # plain `... && break` ends the script under `set -e` on the first miss, and `grep -q`
    # quits early enough to break the pipe, which `set -o pipefail` then reports as failure
    if ros2 topic info "$topic" 2>/dev/null | grep 'Publisher count: [1-9]' >/dev/null; then
      break
    fi
    sleep 1
  done
done

# RViz writes the layout back into whatever file it was given when it closes, panels and
# all. Close it once with the camera docks hidden and the config is quietly ruined for
# every launch after -- which is exactly how they went missing. Run from a copy so the
# checked-in file is the only source of truth and RViz's own saves land in the scratch one.
run_config="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/lekiwi.rviz"
mkdir -p "$(dirname "$run_config")"
cp config/lekiwi.rviz "$run_config"

# The MoveIt RViz plugin is a separate ROS node. Unlike move_group it does not inherit
# these parameters from the launch file, so give it the same generated URDF and SRDF.
# Without them the Planning panel cannot construct its robot model and emits a misleading
# robot_description_semantic error even though move_group itself is healthy.
package_share="$(ros2 pkg prefix lekiwi_rmf)/share/lekiwi_rmf"
robot_description="$(xacro "$package_share/urdf/lekiwi.urdf.xacro" sim:=false)"
robot_description_semantic="$(< "$package_share/config/lekiwi.srdf")"

# The Planning panel's interactive markers solve IK inside RViz, which loads the solver
# from this parameter; a plain -p cannot carry a nested map, so it goes through a
# generated params file instead.
run_params="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/rviz-moveit-params.yaml"
{
  printf '/**:\n  ros__parameters:\n    robot_description_kinematics:\n'
  sed 's/^/      /' "$package_share/config/kinematics.yaml"
} > "$run_params"

# Do not let the startup lock leak into RViz itself.
exec 9>&-
exec rviz2 -d "$run_config" "$@" --ros-args \
  -p "robot_description:=$robot_description" \
  -p "robot_description_semantic:=$robot_description_semantic" \
  --params-file "$run_params"
