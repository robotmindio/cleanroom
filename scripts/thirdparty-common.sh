#!/usr/bin/env bash

# Shared pinned third-party sources and binaries. Consumed by the installers
# that source this file, which ShellCheck cannot follow across entry points.
# Callers provide die().
# shellcheck disable=SC2034
ZENOH_VERSION=1.5.0
# LDROBOT LD06 lidar driver, tag v3.0.3. Not released into the ROS apt repos;
# thirdparty/ldlidar_stl_ros2/ carries a build fix applied after the clone.
# shellcheck disable=SC2034
LDLIDAR_STL_REV=cac5d3d4c15522c6126ef65cfa8a65b08531a66b
# shellcheck disable=SC2034
LDLIDAR_STL_REPOSITORY=https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

checkout_pinned() { # checkout_pinned <url> <destination> <revision> [known-patch]
  local url=$1 destination=$2 revision=$3 expected_patch=${4:-}
  if [[ ! -d $destination/.git ]]; then
    git clone --filter=blob:none "$url" "$destination"
  elif [[ -n $(git -C "$destination" status --porcelain) ]]; then
    # Permit only a prior application of the tracked build patch. Applying
    # its reverse preserves unrelated local work, which the checkout rejects.
    if [[ -n $expected_patch ]] && git -C "$destination" apply --reverse --check "$expected_patch" 2>/dev/null; then
      git -C "$destination" apply --reverse "$expected_patch"
    else
      die "$destination has local changes; preserve them before rerunning"
    fi
  fi
  git -C "$destination" fetch --depth 1 origin "$revision"
  git -C "$destination" checkout --detach FETCH_HEAD
}

apply_pinned_patch() { # apply_pinned_patch <destination> <patch> <description>
  local destination=$1 patch=$2 description=$3
  if git -C "$destination" apply --check "$patch" 2>/dev/null; then
    git -C "$destination" apply "$patch"
  else
    die "could not apply ${description}; upstream may have changed or the checkout has unexpected edits"
  fi
}

install_zenoh_bridge() { # install_zenoh_bridge <destination-directory>
  local destination=$1 arch archive tmp_dir bridge
  case $(uname -m) in
    x86_64) arch=x86_64-unknown-linux-gnu ;;
    aarch64|arm64) arch=aarch64-unknown-linux-gnu ;;
    *) die "unsupported CPU architecture: $(uname -m)" ;;
  esac
  archive="zenoh-plugin-ros2dds-${ZENOH_VERSION}-${arch}-standalone.zip"
  tmp_dir=$(mktemp -d)
  curl -fL -o "$tmp_dir/$archive" \
    "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/${archive}"
  unzip -q "$tmp_dir/$archive" -d "$tmp_dir/zenoh"
  bridge=$(find "$tmp_dir/zenoh" -type f -name zenoh-bridge-ros2dds -print -quit)
  [[ -n $bridge ]] || die "Zenoh archive did not contain zenoh-bridge-ros2dds"
  mkdir -p "$destination"
  install -m 0755 "$bridge" "$destination/zenoh-bridge-ros2dds"
  find "$tmp_dir" -type f -delete
  find "$tmp_dir" -depth -type d -empty -delete
}
