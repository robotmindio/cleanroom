#!/usr/bin/env bash
# Print the content key of a role's patched third-party dependency overlay.
# The overlay changes only with these pinned inputs, the dependency sources in
# the bootstrap workspace and the installed ROS packages it links against.
set -Eeuo pipefail
[[ $# == 3 ]] || { echo "usage: $0 compute|device SOURCE WORKSPACE" >&2; exit 2; }
role=$1
source_root=$2
workspace=$3
{
  echo "role=$role"
  dpkg-query -W 'ros-jazzy-*'
  # The release source is a clean commit, so tracked blob IDs identify inputs.
  git -C "$source_root" ls-tree -r HEAD -- scripts/dependency-overlay-key.sh scripts/build-native.sh \
    scripts/build-lekiwi.sh scripts/thirdparty-common.sh scripts/lib/runtime-common.sh \
    scripts/lib/build-common.sh thirdparty/class_loader thirdparty/rclcpp thirdparty/navigation2 \
    thirdparty/rviz thirdparty/ldlidar_stl_ros2 thirdparty/ros2_astra_camera
  for dependency in ament_cmake class_loader rclcpp navigation2 rviz ldlidar_stl_ros2 ros2_astra_camera; do
    printf '%s=%s\n' "$dependency" "$(git -C "$workspace/src/$dependency" rev-parse HEAD 2>/dev/null || echo absent)"
  done
} | sha256sum | cut -c1-32
