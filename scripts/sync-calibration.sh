#!/usr/bin/env bash
# Copy this robot's saved calibration files from the machine that produced them
# (the camera/motor-bus Pi in a split deployment) to this machine's matching
# paths, so a compute-side stack (scripts/up.sh, scripts/rearm-robot.sh, ...)
# sees the same calibration the Pi calibrated.
#
#   scripts/sync-calibration.sh [[user@]source-host]   default: LEKIWI_ROBOT_HOST from .env
#
# Retries the files that failed, five attempts in all, then exits non-zero.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
load_lekiwi_env

host="${1:-${LEKIWI_ROBOT_HOST:-}}"
[[ -n $host ]] || {
  echo "usage: $0 [user@]source-host (or set LEKIWI_ROBOT_HOST in .env)" >&2
  exit 2
}

# Paths documented in scripts/calibrate.sh, relative to $HOME on both sides.
pending=(
  .ros/lekiwi_arm_calibration.json
  .ros/camera_info/lekiwi_front.yaml
  .ros/camera_info/lekiwi_wrist.yaml
  .ros/lekiwi_launch_calibration.conf
  .cache/huggingface/lerobot/calibration/robots/lekiwi/lekiwi_1.json
)

attempts=5
for ((attempt = 1; attempt <= attempts; attempt++)); do
  failed=()
  for f in "${pending[@]}"; do
    if rsync -a --mkpath -e "ssh -o ConnectTimeout=10" "$host:$f" "$HOME/$f" 2>&1; then
      echo "synced $f"
    else
      echo "$0: failed to sync $f (attempt $attempt of $attempts)" >&2
      failed+=("$f")
    fi
  done
  pending=("${failed[@]}")
  (( ${#pending[@]} )) || break
  (( attempt == attempts )) || { echo "retrying in 5s..." >&2; sleep 5; }
done

if (( ${#pending[@]} )); then
  echo "$0: not synced from $host: ${pending[*]}" >&2
  exit 1
fi
echo "All calibration files synced from $host."
