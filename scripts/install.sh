#!/usr/bin/env bash
set -Eeuo pipefail

ZENOH_VERSION=1.5.0
LEROBOT_VERSION=0.6.1
FREE_FLEET_REV=e178db662720e36116a5559e4c13847466d5be2d
RMF_DEMOS_REV=2.3.0
# LDROBOT LD06 lidar driver, tag v3.0.3. Not released into the ROS apt repos;
# thirdparty/ldlidar_stl_ros2/ carries a build fix applied after this clone.
LIDLIDAR_STL_REV=cac5d3d4c15522c6126ef65cfa8a65b08531a66b
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE=${LEKIWI_WS:-"$HOME/lekiwi_ws"}

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

if [[ ${1:-} == --help ]]; then
  printf 'Usage: LEKIWI_WS=/path/to/workspace %s [--simulation]\n' "$0"
  printf '  --simulation  install the Gazebo/ROS stack without the LeRobot hardware environment\n'
  exit 0
fi
install_mode=full
[[ $# -le 1 ]] || die "unknown argument: $2"
case ${1:-} in
  '') ;;
  --simulation) install_mode=simulation ;;
  *) die "unknown argument: $1" ;;
esac

# shellcheck disable=SC1091
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 24.04 ]] || die "Ubuntu 24.04 is required (found ${PRETTY_NAME:-unknown})"
ROS_DISTRO=jazzy

case $(uname -m) in
  x86_64) ZENOH_ARCH=x86_64-unknown-linux-gnu ;;
  aarch64|arm64) ZENOH_ARCH=aarch64-unknown-linux-gnu ;;
  *) die "unsupported CPU architecture: $(uname -m)" ;;
esac

command -v sudo >/dev/null || [[ $EUID -eq 0 ]] || die "sudo is required"
SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)

log "Installing Ubuntu and ROS repository prerequisites"
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y software-properties-common
"${SUDO[@]}" add-apt-repository -y universe
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y \
  curl \
  git \
  locales \
  python3-pip \
  python3-venv \
  shellcheck \
  unzip

legacy_ros_source=/etc/apt/sources.list.d/ros2.list
modern_ros_source=/etc/apt/sources.list.d/ros2.sources
# Older ROS instructions create ros2.list directly. The current ros-apt-source
# package creates ros2.sources with an inline key; apt refuses to read both for
# the same repository because their Signed-By values differ. Keep the legacy
# file beside it, disabled, so this migration is reversible.
if [[ -e $legacy_ros_source ]]; then
  log "Disabling the legacy ROS apt source"
  "${SUDO[@]}" mv -f "$legacy_ros_source" "$legacy_ros_source.disabled"
fi

if [[ ! -e $modern_ros_source ]]; then
  tmp_dir=$(mktemp -d)
  ros_apt_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p')
  [[ -n $ros_apt_version ]] || die "could not determine ros-apt-source release"
  curl -fL -o "$tmp_dir/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${VERSION_CODENAME}_all.deb"
  "${SUDO[@]}" dpkg -i "$tmp_dir/ros2-apt-source.deb"
  find "$tmp_dir" -type f -delete
  rmdir "$tmp_dir"
fi

log "Installing ROS 2 ${ROS_DISTRO}, Nav2, Gazebo, RTAB-Map, and Open-RMF"
"${SUDO[@]}" apt-get update
if dpkg-query -W -f='${Status}' python3-paraview 2>/dev/null | grep -q 'ok installed'; then
  die "Ubuntu ParaView conflicts with RTAB-Map's python3-vtk9; remove it first: sudo apt-get remove paraview python3-paraview"
fi
"${SUDO[@]}" apt-get install -y \
  ros-dev-tools \
  "ros-$ROS_DISTRO-camera-calibration" \
  "ros-$ROS_DISTRO-nav2-bringup" \
  "ros-$ROS_DISTRO-navigation2" \
  "ros-$ROS_DISTRO-moveit" \
  "ros-$ROS_DISTRO-rmf-dev" \
  "ros-$ROS_DISTRO-rmw-cyclonedds-cpp" \
  "ros-$ROS_DISTRO-rqt-image-view" \
  "ros-$ROS_DISTRO-topic-tools" \
  "ros-$ROS_DISTRO-ros-base" \
  "ros-$ROS_DISTRO-rosbridge-server" \
  "ros-$ROS_DISTRO-ros-gz" \
  "ros-$ROS_DISTRO-image-transport" \
  "ros-$ROS_DISTRO-image-transport-plugins" \
  "ros-$ROS_DISTRO-rtabmap-ros" \
  "ros-$ROS_DISTRO-rviz2" \
  "ros-$ROS_DISTRO-v4l2-camera" \
  python3-matplotlib \
  python3-opencv \
  python3-yaml \
  python3-zmq \
  psmisc \
  v4l-utils

# Qualification invokes these through the system interpreter and PATH.  Check
# that exact contract now so an incomplete deployment image fails during
# installation rather than silently omitting the three ZMQ-dependent tests or
# the shell validation step later.
command -v shellcheck >/dev/null || die "ShellCheck was not installed"
python3 -c 'import zmq' || die "python3-zmq was not installed for the system Python"

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${SUDO[@]}" rosdep init
fi
# ponytail: rosdep update has no retry flag and its index fetch times out on slow links
for attempt in 1 2 3; do
  rosdep update && break
  [[ $attempt -lt 3 ]] || die "rosdep update failed after 3 attempts"
  sleep $((attempt * 10))
done

mkdir -p "$WORKSPACE/src" "$HOME/.local/bin"

checkout() {
  local url=$1 destination=$2 revision=$3 expected_patch=${4:-}
  if [[ ! -d $destination/.git ]]; then
    git clone --filter=blob:none "$url" "$destination"
  elif [[ -n $(git -C "$destination" status --porcelain) ]]; then
    # Some vendored sources need a tracked build patch after checkout. Permit
    # only that exact, reversible diff; all other local edits remain protected.
    if [[ -n $expected_patch ]] && git -C "$destination" apply --reverse --check "$expected_patch" 2>/dev/null; then
      git -C "$destination" reset --hard HEAD >/dev/null
    else
      die "$destination has local changes; preserve them before rerunning"
    fi
  fi
  git -C "$destination" fetch --depth 1 origin "$revision"
  git -C "$destination" checkout --detach FETCH_HEAD
}

apply_pinned_patch() {
  local destination=$1 patch=$2 description=$3
  if git -C "$destination" apply --check "$patch" 2>/dev/null; then
    git -C "$destination" apply "$patch"
  else
    die "could not apply ${description}; upstream may have changed or the checkout has unexpected edits"
  fi
}

log "Fetching pinned Free Fleet and RMF task tools"
free_fleet_source="$WORKSPACE/src/free_fleet"
free_fleet_patch="$PROJECT_ROOT/thirdparty/free_fleet/0001-retry-nav2-goal-during-activation.patch"
checkout https://github.com/open-rmf/free_fleet.git "$free_fleet_source" "$FREE_FLEET_REV" "$free_fleet_patch"
apply_pinned_patch "$free_fleet_source" "$free_fleet_patch" "the Free Fleet Nav2 activation retry patch"
checkout https://github.com/open-rmf/rmf_demos.git "$WORKSPACE/src/rmf_demos" "$RMF_DEMOS_REV"

log "Fetching the pinned LDROBOT LD06 driver"
checkout https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git \
  "$WORKSPACE/src/ldlidar_stl_ros2" "$LIDLIDAR_STL_REV" \
  "$PROJECT_ROOT/thirdparty/ldlidar_stl_ros2/0001-linux-build-fixes.patch"
ldlidar_source="$WORKSPACE/src/ldlidar_stl_ros2"
ldlidar_patch="$PROJECT_ROOT/thirdparty/ldlidar_stl_ros2/0001-linux-build-fixes.patch"
apply_pinned_patch "$ldlidar_source" "$ldlidar_patch" "the LDROBOT Linux build fixes"

log "Installing the Zenoh ROS 2 bridge"
zenoh_zip="zenoh-plugin-ros2dds-${ZENOH_VERSION}-${ZENOH_ARCH}-standalone.zip"
tmp_dir=$(mktemp -d)
curl -fL -o "$tmp_dir/$zenoh_zip" \
  "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/${zenoh_zip}"
unzip -q "$tmp_dir/$zenoh_zip" -d "$tmp_dir/zenoh"
zenoh_bridge=$(find "$tmp_dir/zenoh" -type f -name zenoh-bridge-ros2dds -print -quit)
[[ -n $zenoh_bridge ]] || die "Zenoh archive did not contain zenoh-bridge-ros2dds"
install -m 0755 "$zenoh_bridge" "$HOME/.local/bin/zenoh-bridge-ros2dds"
find "$tmp_dir" -type f -delete
find "$tmp_dir" -depth -type d -empty -delete

log "Creating the ROS-compatible Python environment"
python3 -m venv --system-site-packages "$WORKSPACE/.venv"
# ponytail: ROS setup.bash and venv activate both read unset vars; -u must be off for them
set +u
# shellcheck disable=SC1090,SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1091
source "$WORKSPACE/.venv/bin/activate"
set -u
python -m pip install --upgrade pip
# ROS's compiled extensions (rmf_adapter, cv_bridge) are built against the system numpy,
# so this venv must match it or rmf_adapter segfaults mid-run: Jazzy/Noble ships numpy
# 1.26. setuptools<80 is colcon-core's ceiling.
# transforms3d 0.4.2 replaces the np.maximum_sctype call that Ubuntu's 0.4.1 makes, which
# numpy removed in 2.0.
python -m pip install \
  "setuptools<80" \
  "numpy<2" \
  "eclipse-zenoh==${ZENOH_VERSION}" \
  nudged pycdr2 rosbags \
  "transforms3d>=0.4.2"

if [[ $install_mode == full ]]; then
  log "Creating the LeRobot Python environment"
  # ponytail: LeRobot requires numpy>=2, which cannot coexist with ROS's numpy 1.26 in one
  # interpreter. Separate venv; only the hardware driver runs in it (see bringup.launch.py).
  python3 -m venv --system-site-packages "$WORKSPACE/.venv-lerobot"
  (
    set +u
    # shellcheck disable=SC1091
    source "$WORKSPACE/.venv-lerobot/bin/activate"
    set -u
    python -m pip install --upgrade pip
    python -m pip install "lerobot[lekiwi,hardware]==${LEROBOT_VERSION}"
  )
else
  log "Simulation-only installation: skipping the LeRobot hardware environment"
fi

log "Resolving package dependencies and building the workspace"
rosdep install --from-paths \
  "$PROJECT_ROOT" \
  "$WORKSPACE/src/free_fleet" \
  "$WORKSPACE/src/ldlidar_stl_ros2" \
  "$WORKSPACE/src/rmf_demos/rmf_demos_tasks" \
  --ignore-src --rosdistro "$ROS_DISTRO" -yr
# colcon reuses each package's CMake cache. If this repository was previously built from
# another worktree, CMake refuses the reused cache before it can regenerate anything.
package_build="$WORKSPACE/build/lekiwi_rmf"
cache="$package_build/CMakeCache.txt"
if [[ -f $cache ]]; then
  cached_source=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache")
  if [[ -n $cached_source && $cached_source != "$PROJECT_ROOT" ]]; then
    log "Removing stale lekiwi_rmf build cache from $cached_source"
    rm -rf -- "$package_build"
  fi
fi
# A Pi-sized machine can exhaust itself here: five packages building in parallel
# while everything else runs is how oomd ended up killing a whole session
# (2026-08-23). On small RAM, cap packages-in-flight and compiler jobs per
# package; CMAKE_BUILD_PARALLEL_LEVEL is what make/ninja actually read.
mem_total_mb=$(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo)
parallel_args=()
if (( mem_total_mb < 8000 )); then
  log "Low memory (${mem_total_mb} MB): capping build parallelism"
  parallel_args=(--parallel-workers 2)
  export CMAKE_BUILD_PARALLEL_LEVEL=2
fi
colcon --log-base "$WORKSPACE/log" build \
  --base-paths "$PROJECT_ROOT" "$WORKSPACE/src/free_fleet" \
    "$WORKSPACE/src/ldlidar_stl_ros2" "$WORKSPACE/src/rmf_demos/rmf_demos_tasks" \
  --packages-select lekiwi_rmf free_fleet free_fleet_adapter ldlidar_stl_ros2 rmf_demos_tasks \
  --build-base "$WORKSPACE/build" \
  --install-base "$WORKSPACE/install" \
  "${parallel_args[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# colcon prints "Finished" per package it reached, and an interrupted run can
# still leave a plausible-looking install tree with share/ metadata but no
# compiled node (an oomd kill mid-build did exactly that once). Trust binaries,
# not summaries.
log "Verifying the built executables"
missing=()
for exe in \
  "$WORKSPACE/install/lekiwi_rmf/lib/lekiwi_rmf/lekiwi_driver" \
  "$WORKSPACE/install/lekiwi_rmf/lib/lekiwi_rmf/camera_relay" \
  "$WORKSPACE/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node" \
  "$WORKSPACE/install/free_fleet_adapter/lib/free_fleet_adapter/fleet_adapter.py"
do
  [[ -x $exe ]] || missing+=("$exe")
done
if (( ${#missing[@]} )); then
  die "build finished but these executables are missing: ${missing[*]}
       (a killed or interrupted build leaves stub installs behind; delete the
       package's build/ and install/ dirs and re-run this script)"
fi

setup_file="$PROJECT_ROOT/scripts/setup.bash"
setup_marker='# >>> lekiwi setup >>>'
install_shell_setup() {
  local profile=$1
  if [[ -f $profile ]] && grep -Fq "$setup_file" "$profile"; then
    return
  fi
  touch "$profile"
  {
    printf '\n%s\n' "$setup_marker"
    printf '# Added by LeKiwi installer. Remove this block to disable automatic ROS setup.\n'
    printf 'if [ -z "${LEKIWI_WS:-}" ]; then export LEKIWI_WS=%q; fi\n' "$WORKSPACE"
    printf 'if [ -f %q ]; then\n  . %q\nfi\n' "$setup_file" "$setup_file"
    printf '# <<< lekiwi setup <<<\n'
  } >> "$profile"
  log "Added LeKiwi setup to $profile"
}

# setup.bash selects the matching ROS setup file, so the same source line is valid
# in bash and zsh. Do not create .zshrc for bash-only users.
install_shell_setup "$HOME/.bashrc"
if [[ ${SHELL:-} == */zsh || -f "$HOME/.zshrc" ]]; then
  install_shell_setup "$HOME/.zshrc"
fi

log "Installation complete"
printf 'Open a new shell to load the LeKiwi environment automatically.\n'
printf 'Then launch the managed simulation with renderer preflight:\n  scripts/sim-up.sh\n'
printf 'For the real robot, scripts/up.sh brings everything up by hand.\n'
printf 'At boot instead: install-device-services.sh where the motors/cameras\n'
printf 'plug in, install-compute-services.sh where the stack should run --\n'
printf 'the same machine or two different ones, in either direction.\n'
