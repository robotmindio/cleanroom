#!/usr/bin/env bash
# RViz on a running stack: map, costmaps, robot model, TF, plans, the goal/initial-pose
# tools, and live camera displays.
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
logs_dir="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
runtime_dir="${LEKIWI_RUNTIME_DIR:-$logs_dir/runtime}"
run_config="$logs_dir/lekiwi.rviz"
rviz_pid_file="$runtime_dir/rviz.pid"
mkdir -p "$logs_dir" "$runtime_dir"

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

# Replace only the RViz process started by this repository. A broad process
# search can kill a different robot's RViz on a shared workstation.
old_rviz_pids=()
if [[ -r "$rviz_pid_file" ]]; then
  old_rviz_pid=$(<"$rviz_pid_file")
  if [[ $old_rviz_pid =~ ^[1-9][0-9]*$ ]] && kill -0 "$old_rviz_pid" 2>/dev/null; then
    old_rviz_command=$(tr '\0' ' ' < "/proc/$old_rviz_pid/cmdline" 2>/dev/null || true)
    if [[ $old_rviz_command == *"rviz2"* && $old_rviz_command == *"-d $run_config"* ]]; then
      old_rviz_pids=("$old_rviz_pid")
    else
      echo "rviz: recorded PID $old_rviz_pid is not this stack's RViz; leaving it alone" >&2
    fi
  else
    rm -f -- "$rviz_pid_file"
  fi
fi
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

topics=$(rg -o '/camera(?:/[a-z_]+)+/image_raw' config/lekiwi.rviz | sort -u)
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
cp config/lekiwi.rviz "$run_config"

# The MoveIt RViz plugin is a separate ROS node. Unlike move_group it does not inherit
# these parameters from the launch file, so give it the same generated URDF and SRDF.
# Without them the Planning panel cannot construct its robot model and emits a misleading
# robot_description_semantic error even though move_group itself is healthy.
package_share="$(ros2 pkg prefix lekiwi_rmf)/share/lekiwi_rmf"
robot_description="$(xacro "$package_share/urdf/lekiwi.urdf.xacro" sim:=false)"
robot_description_semantic="$(< "$package_share/config/lekiwi.srdf")"

# The Planning panel gets its robot model, planned-path markers, and IK solver all through
# ROS parameters. A plain -p cannot carry these: the nested kinematics map defeats it, and
# the expanded URDF/SRDF are multi-line XML whose spaces and apostrophes (e.g. the
# gripper's comment) an override rule cannot tolerate. So everything goes through a
# generated params file instead, with the XML as YAML literal blocks.
run_params="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}/rviz-moveit-params.yaml"
velocity_scale="$(awk '$1 == "default_velocity_scaling_factor:" { print $2; exit }' "$package_share/config/joint_limits.yaml")"
acceleration_scale="$(awk '$1 == "default_acceleration_scaling_factor:" { print $2; exit }' "$package_share/config/joint_limits.yaml")"
{
  printf '/**:\n  ros__parameters:\n'
  printf '    robot_description: |-\n'
  printf '%s\n' "$robot_description" | sed 's/^/      /'
  printf '    robot_description_semantic: |-\n'
  printf '%s\n' "$robot_description_semantic" | sed 's/^/      /'
  printf '    robot_description_kinematics:\n'
  sed 's/^/      /' "$package_share/config/kinematics.yaml"
  # The Motion Planning panel is its own ROS node.  It reads these defaults from
  # its *own* parameters (not move_group's), otherwise it silently falls back to
  # 0.1 for both controls even when joint_limits.yaml says otherwise.
  printf '    robot_description_planning:\n'
  sed 's/^/      /' "$package_share/config/joint_limits.yaml"
} > "$run_params"

# Do not let the startup lock leak into RViz itself.
exec 9>&-
# MoveIt's panel queries these names before it loads the rest of its configuration.
# Supply them as explicit process overrides as well as in the generated YAML; the
# RViz node then has them declared when MotionPlanningFrame reads them.
printf '%s\n' "$$" > "$rviz_pid_file"
exec rviz2 -d "$run_config" "$@" --ros-args --params-file "$run_params" \
  -p "robot_description_planning.default_velocity_scaling_factor:=$velocity_scale" \
  -p "robot_description_planning.default_acceleration_scaling_factor:=$acceleration_scale"
