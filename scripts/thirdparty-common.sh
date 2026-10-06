#!/usr/bin/env bash

# Shared pinned third-party sources and binaries. Consumed by the installers
# that source this file, which ShellCheck cannot follow across entry points.
# Callers source scripts/lib/runtime-common.sh for die().
# shellcheck disable=SC2034
ZENOH_VERSION=1.5.0
THIRDPARTY_PATCH_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../thirdparty" && pwd)
# LDROBOT LD06 lidar driver, tag v3.0.3. Not released into the ROS apt repos;
# thirdparty/ldlidar_stl_ros2/ carries the fixes applied after the clone.
# shellcheck disable=SC2034
LDLIDAR_STL_REV=cac5d3d4c15522c6126ef65cfa8a65b08531a66b
# shellcheck disable=SC2034
LDLIDAR_STL_REPOSITORY=https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

download_verified() { # download_verified <url> <sha256> <destination>
  curl -fL -o "$3" "$1"
  printf '%s  %s\n' "$2" "$3" | sha256sum --check --quiet - || {
    rm -f -- "$3"
    die "checksum mismatch for $1"
  }
}

checkout_pinned() { # checkout_pinned <url> <destination> <revision> [known-patch ...]
  local url=$1 destination=$2 revision=$3 patch_index recognized_patch=false
  shift 3
  local -a expected_patches=("$@")
  if [[ ! -d $destination/.git ]]; then
    git clone --filter=blob:none "$url" "$destination"
  elif [[ -n $(git -C "$destination" status --porcelain) ]]; then
    # Undo recognized patches in reverse application order before updating the
    # pinned checkout. Leave other local edits for git checkout to preserve.
    for ((patch_index=${#expected_patches[@]} - 1; patch_index >= 0; patch_index--)); do
      if git -C "$destination" apply --reverse --check "${expected_patches[patch_index]}" 2>/dev/null; then
        git -C "$destination" apply --reverse "${expected_patches[patch_index]}"
        recognized_patch=true
      elif ! git -C "$destination" apply --check "${expected_patches[patch_index]}" 2>/dev/null; then
        die "$destination has local changes; preserve them before rerunning"
      fi
    done
    if [[ $recognized_patch != true ]]; then
      die "$destination has local changes; preserve them before rerunning"
    fi
  fi
  git -C "$destination" fetch --depth 1 origin "$revision"
  git -C "$destination" checkout --detach FETCH_HEAD
}

apply_thirdparty_patches() { # apply_thirdparty_patches <dependency> <destination>
  # Applies thirdparty/<dependency>/*.patch in name order; already applied ones are kept.
  local patch
  for patch in "$THIRDPARTY_PATCH_ROOT/$1/"*.patch; do
    [[ -e $patch ]] || continue
    apply_pinned_patch "$2" "$patch" "${patch#"$THIRDPARTY_PATCH_ROOT"/}"
  done
}

checkout_with_patches() { # checkout_with_patches <dependency> <url> <destination> <revision>
  local -a patches=("$THIRDPARTY_PATCH_ROOT/$1/"*.patch)
  [[ -e ${patches[0]} ]] || patches=()
  checkout_pinned "$2" "$3" "$4" "${patches[@]}"
  apply_thirdparty_patches "$1" "$3"
}

apply_pinned_patch() { # apply_pinned_patch <destination> <patch> <description>
  local destination=$1 patch=$2 description=$3
  if git -C "$destination" apply --reverse --check "$patch" 2>/dev/null; then
    return 0
  fi
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
