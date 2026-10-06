#!/usr/bin/env bash
# Rebuild only this repository into the workspace used by the managed services.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
project_root=$PWD
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
[[ $workspace == /* && -d $workspace/install ]] || {
  echo "$0: installed workspace not found: $workspace" >&2
  exit 1
}
die() { echo "$0: $*" >&2; exit 1; }
[[ ! -L $workspace/current ]] || die "stage a release instead of rebuilding the bootstrap workspace"

# shellcheck source=scripts/thirdparty-common.sh
source "$project_root/scripts/thirdparty-common.sh"

base_paths=("$project_root")
packages=(lekiwi_rmf)
lidar_source=$workspace/src/ldlidar_stl_ros2
if [[ -d $lidar_source/.git ]]; then
  for patch in "$project_root/thirdparty/ldlidar_stl_ros2/"*.patch; do
    apply_pinned_patch "$lidar_source" "$patch" "the LD06 native fixes"
  done
  base_paths+=("$lidar_source")
  packages+=(ldlidar_stl_ros2)
fi

# Rebuild the USB camera driver where it is installed, including device deploys.
astra_source=$workspace/src/ros2_astra_camera
if [[ -d $astra_source/.git ]]; then
  for patch in "$project_root/thirdparty/ros2_astra_camera/"*.patch; do
    apply_pinned_patch "$astra_source" "$patch" "the Astra native camera fixes"
  done
  base_paths+=("$astra_source")
  packages+=(astra_camera_msgs astra_camera)
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$workspace/install/setup.bash"
set -u

# Keep user-installed CMake/Protobuf copies from overriding the ROS packages.
PATH=/usr/bin:/bin:$PATH
export PATH
cache="$workspace/build/lekiwi_rmf/CMakeCache.txt"
if [[ -f $cache && $(awk -F= '$1 == "CMAKE_HOME_DIRECTORY:INTERNAL" {print $2}' "$cache") != "$project_root" ]]; then
  # CMake cannot reuse a build from another checkout; otherwise its normal
  # incremental build avoids unnecessary CPU contention with robot callbacks.
  rm -rf -- "$workspace/build/lekiwi_rmf"
fi

parallel_args=()
# Other applications can consume most RAM even on a large compute host.
# Size the compile batch from available memory to avoid swapping ROS callbacks.
if (( $(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo) < 8000 )); then
  parallel_args=(--parallel-workers 1)
  export MAKEFLAGS=-j1
fi

colcon --log-base "$workspace/log" build \
  --base-paths "${base_paths[@]}" \
  --packages-select "${packages[@]}" \
  --build-base "$workspace/build" \
  --install-base "$workspace/install" \
  "${parallel_args[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local" \
    -DPython3_EXECUTABLE=/usr/bin/python3

installed_driver=$workspace/install/lekiwi_rmf/lib/lekiwi_rmf/lekiwi_driver
if [[ ! -x $installed_driver ]] || ! cmp -s lekiwi_rmf/driver.py "$installed_driver"; then
  echo "$0: build completed without installing the current driver" >&2
  exit 1
fi
if [[ -d $lidar_source/.git &&
      ! -x $workspace/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node ]]; then
  echo "$0: build completed without installing the LD06 driver" >&2
  exit 1
fi

# The split deployer can skip an otherwise disruptive rebuild when both
# workspaces already contain this exact source revision.
git -C "$project_root" rev-parse HEAD > "$workspace/install/lekiwi_rmf/.lekiwi-source-revision"
