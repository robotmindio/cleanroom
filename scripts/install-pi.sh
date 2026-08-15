#!/usr/bin/env bash
# Install the LeRobot host on the robot's Raspberry Pi.
# This machine runs LeRobot only -- ROS, Nav2, RTAB-Map and RMF stay on the workstation,
# which reaches this Pi over ZMQ 5555/5556. Do not run scripts/install.sh here.
set -Eeuo pipefail

LEROBOT_VERSION=0.6.1
VENV=${LEKIWI_LEROBOT_VENV:-"$HOME/lerobot-venv"}
EXAMPLES=${LEKIWI_LEROBOT_SRC:-"$HOME/lerobot-src"}

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

command -v sudo >/dev/null || [[ $EUID -eq 0 ]] || die "sudo is required"
SUDO=()
[[ $EUID -eq 0 ]] || SUDO=(sudo)

log "Installing system prerequisites"
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y curl git python3-venv python3-pip

# brltty claims CH34x USB serial adapters and steals the Feetech bus from LeRobot.
if dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q 'ok installed'; then
  log "Removing brltty (it grabs the CH34x motor bus adapter)"
  "${SUDO[@]}" apt-get remove -y brltty
fi

log "Granting serial and camera access"
for group in dialout video; do
  if id -nG "$USER" | tr ' ' '\n' | grep -qx "$group"; then
    printf 'already in %s\n' "$group"
  else
    "${SUDO[@]}" usermod -aG "$group" "$USER"
    printf 'added to %s (log out and back in to take effect)\n' "$group"
  fi
done

log "Creating the LeRobot environment in $VENV"
python3 -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --upgrade pip
# Torch pulls a large aarch64 wheel; on a 2 GB Pi this is the step that runs out of memory.
python -m pip install "lerobot[lekiwi,hardware]==${LEROBOT_VERSION}"

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
  lerobot-calibrate --robot.type=lekiwi --robot.id=lekiwi_1

Then start the host that the workstation connects to:
  python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=lekiwi_1 --host.connection_time_s=86400

This Pi's address (give it to the workstation as remote_ip):
  $(hostname -I 2>/dev/null | awk '{print $1}')
EOF
