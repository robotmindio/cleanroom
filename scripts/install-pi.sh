#!/usr/bin/env bash
# Install the LeRobot host on the robot's Raspberry Pi.
# This machine runs LeRobot only -- ROS, Nav2, RTAB-Map and RMF stay on the workstation,
# which reaches this Pi over ZMQ 5555/5556. Do not run scripts/install.sh here.
set -Eeuo pipefail

LEROBOT_VERSION=0.6.1
# LDROBOT LD06 driver revision and local Jazzy build patch. Keep this aligned
# with scripts/install.sh so the device service has the same serial driver.
LIDLIDAR_STL_REV=cac5d3d4c15522c6126ef65cfa8a65b08531a66b
VENV=${LEKIWI_LEROBOT_VENV:-"$HOME/lerobot-venv"}
EXAMPLES=${LEKIWI_LEROBOT_SRC:-"$HOME/lerobot-src"}
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE=${LEKIWI_WS:-"$HOME/lekiwi_ws"}

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
trap 'printf "error: installer failed at line %s\n" "$LINENO" >&2' ERR

if [[ ${1:-} == --help ]]; then
  printf 'Usage: [LEKIWI_LEROBOT_VENV=/path] %s\n' "$0"
  exit 0
fi
[[ $# -eq 0 ]] || die "unknown argument: $1"

case $(uname -m) in
  aarch64|arm64) ;;
  *) die "expected a 64-bit Raspberry Pi OS or Ubuntu arm64 image (found $(uname -m)); \
32-bit images cannot install the PyTorch wheels LeRobot needs" ;;
esac

# LeRobot 0.6.1 declares requires-python >=3.12. Raspberry Pi OS Bookworm ships 3.11 and
# will fail here; Trixie (3.13) and Ubuntu 24.04 (3.12) both work.
python3 - <<'PY' || die "LeRobot $LEROBOT_VERSION needs Python 3.12+; this image has an older one"
import sys
raise SystemExit(0 if sys.version_info >= (3, 12) else 1)
PY

SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)
# Only escalate for the steps that still need it, so a Pi that is already provisioned
# installs over SSH without a sudo password.
need_root() {
  [[ $EUID -eq 0 ]] || command -v sudo >/dev/null || die "sudo is required to $1"
}

log "Checking system prerequisites"
installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'ok installed'; }
missing=()
for pkg in curl git python3-venv python3-pip; do
  installed "$pkg" || missing+=("$pkg")
done
if (( ${#missing[@]} )); then
  need_root "install ${missing[*]}"
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y "${missing[@]}"
else
  printf 'already installed\n'
fi

# brltty claims CH34x USB serial adapters and steals the Feetech bus from LeRobot.
if installed brltty; then
  log "Removing brltty (it grabs the CH34x motor bus adapter)"
  need_root "remove brltty"
  "${SUDO[@]}" apt-get remove -y brltty
fi

log "Granting serial and camera access"
for group in dialout video; do
  if id -nG "$USER" | tr ' ' '\n' | grep -qx "$group"; then
    printf 'already in %s\n' "$group"
  else
    need_root "add $USER to $group"
    "${SUDO[@]}" usermod -aG "$group" "$USER"
    printf 'added to %s (log out and back in to take effect)\n' "$group"
  fi
done

log "Creating the LeRobot environment in $VENV"
python3 -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --upgrade pip

# PyPI's aarch64 torch wheel now depends on the CUDA runtime, for Jetson and GH200 boards:
# over a gigabyte of nvidia-* wheels that a Pi has no GPU to use. Install the CPU build
# from PyTorch's own index first, so the lerobot resolve below sees torch as satisfied.
# The bounds are lerobot 0.6.1's own; they resolve to torch 2.11.0+cpu / torchvision 0.26.0+cpu.
# --resume-retries: these are ~100 MB wheels over the robot's wifi, and the default of one
# resume attempt is not enough to survive a dropout.
log "Installing CPU-only PyTorch"
python -m pip install --resume-retries 10 \
  --index-url https://download.pytorch.org/whl/cpu \
  --extra-index-url https://pypi.org/simple \
  "torch>=2.7,<2.12" "torchvision>=0.22,<0.27"

log "Installing LeRobot $LEROBOT_VERSION"
python -m pip install --resume-retries 10 "lerobot[lekiwi,hardware]==${LEROBOT_VERSION}"
python -c 'import torch; assert "+cpu" in torch.__version__, f"CUDA build leaked in: {torch.__version__}"'

log "Installing the ROS camera stack"
# The cameras are wired to this Pi, so the ROS nodes that read them run here. Only
# ros-base plus the camera packages -- Nav2, RTAB-Map and RMF stay on the workstation.
# ROS 2 ships binaries for Ubuntu noble/Jazzy only;
# Raspberry Pi OS has no ROS packages, so any other image gets the LeRobot host and
# nothing else. Must match the workstation's distro -- see scripts/install.sh.
codename=$(. /etc/os-release && echo "${VERSION_CODENAME:-}")
case $codename in
  noble) pi_ros_distro=jazzy ;;
  *) pi_ros_distro="" ;;
esac
if [[ -n $pi_ros_distro ]]; then
  legacy_ros_source=/etc/apt/sources.list.d/ros2.list
  modern_ros_source=/etc/apt/sources.list.d/ros2.sources
  # The workstation installer uses ros-apt-source (ros2.sources). Do not add
  # the legacy list beside it: apt rejects their different Signed-By settings.
  if [[ -e $legacy_ros_source && -e $modern_ros_source ]]; then
    log "Disabling the duplicate legacy ROS apt source"
    need_root "disable the duplicate ROS 2 apt source"
    "${SUDO[@]}" mv -f "$legacy_ros_source" "$legacy_ros_source.disabled"
  fi
  if [[ ! -e $legacy_ros_source && ! -e $modern_ros_source ]]; then
    need_root "add the ROS 2 apt repository"
    "${SUDO[@]}" apt-get install -y curl gnupg
    "${SUDO[@]}" curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $codename main" |
      "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
    "${SUDO[@]}" apt-get update
  fi
  need_root "install the ROS device packages"
  # image-transport-plugins provides the compressed transport: raw 640x480 at 30 Hz is
  # 27 MB/s, which the robot's wifi cannot carry. cyclonedds matches the workstation.
  "${SUDO[@]}" apt-get install -y \
    "ros-$pi_ros_distro-ros-base" \
    "ros-$pi_ros_distro-v4l2-camera" \
    "ros-$pi_ros_distro-image-transport-plugins" \
    "ros-$pi_ros_distro-rmw-cyclonedds-cpp" \
    ros-dev-tools \
    python3-yaml \
    psmisc \
    v4l-utils

  log "Installing the pinned LD06 ROS driver"
  lidar_source="$WORKSPACE/src/ldlidar_stl_ros2"
  lidar_patch="$PROJECT_ROOT/thirdparty/ldlidar_stl_ros2/0001-linux-build-fixes.patch"
  mkdir -p "$WORKSPACE/src"
  if [[ ! -d $lidar_source/.git ]]; then
    git clone --filter=blob:none https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$lidar_source"
  elif [[ -n $(git -C "$lidar_source" status --porcelain) ]]; then
    if git -C "$lidar_source" apply --reverse --check "$lidar_patch" 2>/dev/null; then
      git -C "$lidar_source" apply --reverse "$lidar_patch"
    else
      die "$lidar_source has local changes; preserve them before rerunning"
    fi
  fi
  git -C "$lidar_source" fetch --depth 1 origin "$LIDLIDAR_STL_REV"
  git -C "$lidar_source" checkout --detach FETCH_HEAD
  git -C "$lidar_source" apply --check "$lidar_patch" || \
    die "could not apply the LD06 Linux build fixes"
  git -C "$lidar_source" apply "$lidar_patch"
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  set -u
  colcon --log-base "$WORKSPACE/log" build \
    --base-paths "$lidar_source" --packages-select ldlidar_stl_ros2 \
    --build-base "$WORKSPACE/build" --install-base "$WORKSPACE/install"
  [[ -x $WORKSPACE/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node ]] || \
    die "LD06 ROS driver build did not install its node"
else
  printf 'not Ubuntu 24.04 -- skipping ROS; this Pi can run the LeRobot host but\n'
  printf 'not publish cameras to ROS. Reimage with Ubuntu Server 24.04 arm64 for that.\n'
fi

log "Fetching the LeKiwi example scripts"
# LeRobot packages only src/, so examples/ is absent from the wheel but the docs use it.
if [[ -d $EXAMPLES/.git ]]; then
  git -C "$EXAMPLES" fetch --depth 1 origin "v${LEROBOT_VERSION}"
  git -C "$EXAMPLES" checkout --detach FETCH_HEAD
else
  git clone -b "v${LEROBOT_VERSION}" --depth 1 --filter=blob:none \
    https://github.com/huggingface/lerobot.git "$EXAMPLES"
fi

python -c 'import lerobot; print("lerobot", lerobot.__version__)'

log "Installation complete"
cat <<EOF
Activate the environment in every new shell:
  source $VENV/bin/activate

One-time motor setup (see HARDWARE.md for the full procedure):
  lerobot-find-port
  lerobot-setup-motors --robot.type=lekiwi --robot.port=/dev/ttyACM0

Then start the Pi host, camera publisher, and LD06 publisher. On its first run it guides you through
motor calibration automatically:
  $PROJECT_ROOT/scripts/pi-up.sh

Once that works by hand, run $PROJECT_ROOT/scripts/install-device-services.sh
to have the host start by itself at boot. If this Pi should also run the ROS
stack, add $PROJECT_ROOT/scripts/install-compute-services.sh (no arguments).

This Pi's address (give it to the workstation as remote_ip):
  $(hostname -I 2>/dev/null | awk '{print $1}')
EOF
