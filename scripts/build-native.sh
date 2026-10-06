#!/usr/bin/env bash
# Build the pinned fixes into the compute workspace, without changing /opt/ros.
set -Eeuo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
die() { echo "$0: $*" >&2; exit 1; }
[[ $workspace == /* && -d $workspace/install ]] || die "installed workspace not found: $workspace"
[[ ! -L $workspace/current ]] || die "stage a release instead of rebuilding the bootstrap workspace"
# shellcheck disable=SC1091 # Resolve the helper from this checkout at runtime.
source "$project_root/scripts/thirdparty-common.sh"

# Runtime ROS binaries do not install this CMake build tool. Build the pinned
# macro package in the same overlay, so deploy does not need privileged apt.
vendor_tools=$workspace/src/ament_cmake
vendor_tools_revision=5741cff5b9f83253bf3521bd8f44108fde3504ad
if [[ ! -d $vendor_tools/.git || $(git -C "$vendor_tools" rev-parse HEAD) != "$vendor_tools_revision" ]]; then
  checkout_pinned https://github.com/ament/ament_cmake.git "$vendor_tools" "$vendor_tools_revision"
fi

for dependency in class_loader rclcpp navigation2 rviz; do
  case $dependency in
    class_loader) revision=c404b82f04e6b0cce5c6205626b2d35fdf4dc882; repository=https://github.com/ros/class_loader.git ;;
    rclcpp) revision=3aa906a2c7ad13d1623b31a55731993f56538e72; repository=https://github.com/ros2/rclcpp.git ;;
    navigation2) revision=f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501; repository=https://github.com/ros-navigation/navigation2.git ;;
    rviz) revision=feb01669f1297df2af755ce9cd2ed18083e7a8b2; repository=https://github.com/ros2/rviz.git ;;
  esac
  source_dir=$workspace/src/$dependency
  patches=("$project_root/thirdparty/$dependency/"*.patch)
  if [[ ! -d $source_dir/.git || $(git -C "$source_dir" rev-parse HEAD) != "$revision" ]]; then
    checkout_pinned "$repository" "$source_dir" "$revision" "${patches[@]}"
  fi
  for patch in "${patches[@]}"; do
    apply_pinned_patch "$source_dir" "$patch" "$dependency native reliability fix"
  done
done

set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$workspace/install/setup.bash"
set -u
# One compiler at a time keeps the running robot stack responsive during deploy.
export MAKEFLAGS=-j1
export PATH=/usr/bin:/bin:$PATH
override_args=()
if [[ $(colcon build --help) == *--allow-overriding* ]]; then
  override_args=(--allow-overriding class_loader rclcpp nav2_util nav2_lifecycle_manager nav2_bringup rviz_ogre_vendor ament_cmake_vendor_package)
fi
colcon --log-base "$workspace/log" build \
  --base-paths "$vendor_tools/ament_cmake_vendor_package" "$workspace/src/class_loader" "$workspace/src/rclcpp/rclcpp" \
    "$workspace/src/navigation2/nav2_util" "$workspace/src/navigation2/nav2_lifecycle_manager" \
    "$workspace/src/navigation2/nav2_bringup" "$workspace/src/rviz/rviz_ogre_vendor" \
  --packages-select ament_cmake_vendor_package class_loader rclcpp nav2_util nav2_lifecycle_manager nav2_bringup rviz_ogre_vendor \
  --executor sequential "${override_args[@]}" \
  --build-base "$workspace/build" --install-base "$workspace/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local" -DPython3_EXECUTABLE=/usr/bin/python3

[[ -s $workspace/install/rclcpp/lib/librclcpp.so &&
   -s $workspace/install/class_loader/lib/libclass_loader.so &&
   -x $workspace/install/nav2_lifecycle_manager/lib/nav2_lifecycle_manager/lifecycle_manager &&
   -s $workspace/install/rviz_ogre_vendor/opt/rviz_ogre_vendor/lib/OGRE/RenderSystem_GL.so ]] || \
  die "native build did not install the required library and manager"
git -C "$project_root" rev-parse HEAD > "$workspace/install/.lekiwi-native-revision"
