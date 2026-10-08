#!/usr/bin/env bash
# Rebuild only this repository into the workspace used by the managed services.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
project_root=$PWD
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
# shellcheck source=/dev/null
source scripts/lib/build-common.sh
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
[[ $workspace == /* && -d $workspace/install ]] || die "installed workspace not found: $workspace"
build_compute=ON
# --dependencies builds only the patched drivers, for a shared dependency overlay.
dependencies_only=false
while [[ $# -gt 0 ]]; do
  case $1 in
    --device) build_compute=OFF ;;
    --dependencies) dependencies_only=true ;;
    *) die "usage: $0 [--device] [--dependencies]" ;;
  esac
  shift
done
[[ ! -L $workspace/current && ! -f $workspace/release.json ]] || die "stage a release instead of rebuilding a live or sealed workspace"

# shellcheck source=scripts/thirdparty-common.sh
source "$project_root/scripts/thirdparty-common.sh"

base_paths=()
packages=()
if ! $dependencies_only; then
  base_paths=("$project_root")
  packages=(lekiwi_rmf)
fi
lidar_source=$workspace/src/ldlidar_stl_ros2
if [[ -d $lidar_source/.git ]]; then
  apply_thirdparty_patches ldlidar_stl_ros2 "$lidar_source"
  base_paths+=("$lidar_source")
  packages+=(ldlidar_stl_ros2)
fi

# Rebuild the USB camera driver where it is installed, including device deploys.
astra_source=$workspace/src/ros2_astra_camera
if [[ -d $astra_source/.git ]]; then
  apply_thirdparty_patches ros2_astra_camera "$astra_source"
  base_paths+=("$astra_source")
  packages+=(astra_camera_msgs astra_camera)
fi

if (( ${#packages[@]} == 0 )); then
  exit 0
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
remove_foreign_build_cache "$workspace" "$project_root"

parallel_args=()
# Size the compile batch from available memory to avoid swapping ROS callbacks.
if low_available_memory; then
  parallel_args=(--parallel-workers 1)
  export MAKEFLAGS=-j1
fi

colcon --log-base "$workspace/log" build \
  --base-paths "${base_paths[@]}" \
  --packages-select "${packages[@]}" \
  --build-base "$workspace/build" \
  --install-base "$workspace/install" \
  "${parallel_args[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DLEKIWI_BUILD_COMPUTE="$build_compute" \
    -DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local" \
    -DPython3_EXECUTABLE=/usr/bin/python3

installed_driver=$workspace/install/lekiwi_rmf/lib/lekiwi_rmf/lekiwi_driver
if ! $dependencies_only && { [[ ! -x $installed_driver ]] || ! cmp -s lekiwi_rmf/driver.py "$installed_driver"; }; then
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
if ! $dependencies_only; then
  git -C "$project_root" rev-parse HEAD > "$workspace/install/lekiwi_rmf/.lekiwi-source-revision"
fi
