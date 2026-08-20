#!/usr/bin/env bash
set -Eeuo pipefail

ZENOH_VERSION=1.5.0
LEROBOT_VERSION=0.6.1
FREE_FLEET_REV=e178db662720e36116a5559e4c13847466d5be2d
RMF_DEMOS_REV=2.3.0
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE=${LEKIWI_WS:-"$HOME/lekiwi_ws"}

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

if [[ ${1:-} == --help ]]; then
  printf 'Usage: LEKIWI_WS=/path/to/workspace %s\n' "$0"
  exit 0
fi
[[ $# -eq 0 ]] || die "unknown argument: $1"

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
"${SUDO[@]}" apt-get install -y curl git locales python3-pip python3-venv software-properties-common unzip
"${SUDO[@]}" add-apt-repository -y universe

if [[ ! -e /etc/apt/sources.list.d/ros2.sources ]]; then
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
  "ros-$ROS_DISTRO-rmf-dev" \
  "ros-$ROS_DISTRO-rmw-cyclonedds-cpp" \
  "ros-$ROS_DISTRO-ros-base" \
  "ros-$ROS_DISTRO-rosbridge-server" \
  "ros-$ROS_DISTRO-ros-gz" \
  "ros-$ROS_DISTRO-image-transport" \
  "ros-$ROS_DISTRO-image-transport-plugins" \
  "ros-$ROS_DISTRO-rtabmap-ros" \
  "ros-$ROS_DISTRO-rviz2" \
  "ros-$ROS_DISTRO-v4l2-camera" \
  python3-opencv

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
  local url=$1 destination=$2 revision=$3
  if [[ ! -d $destination/.git ]]; then
    git clone --filter=blob:none "$url" "$destination"
  elif [[ -n $(git -C "$destination" status --porcelain) ]]; then
    die "$destination has local changes; preserve them before rerunning"
  fi
  git -C "$destination" fetch --depth 1 origin "$revision"
  git -C "$destination" checkout --detach FETCH_HEAD
}

log "Fetching pinned Free Fleet and RMF task tools"
checkout https://github.com/open-rmf/free_fleet.git "$WORKSPACE/src/free_fleet" "$FREE_FLEET_REV"
checkout https://github.com/open-rmf/rmf_demos.git "$WORKSPACE/src/rmf_demos" "$RMF_DEMOS_REV"

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

log "Resolving package dependencies and building the workspace"
rosdep install --from-paths \
  "$PROJECT_ROOT" \
  "$WORKSPACE/src/free_fleet" \
  "$WORKSPACE/src/rmf_demos/rmf_demos_tasks" \
  --ignore-src --rosdistro "$ROS_DISTRO" -yr
colcon --log-base "$WORKSPACE/log" build \
  --base-paths "$PROJECT_ROOT" "$WORKSPACE/src/free_fleet" "$WORKSPACE/src/rmf_demos/rmf_demos_tasks" \
  --packages-select lekiwi_rmf free_fleet free_fleet_adapter rmf_demos_tasks \
  --build-base "$WORKSPACE/build" \
  --install-base "$WORKSPACE/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

log "Installation complete"
printf 'Open a new shell and run:\n  source %q/scripts/setup.bash\n' "$PROJECT_ROOT"
printf 'Then launch the simulation:\n  ros2 launch lekiwi_rmf bringup.launch.py mode:=sim\n'
