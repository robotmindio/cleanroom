#!/usr/bin/env bash
# Copy this robot's saved calibration files from the machine that produced them
# (the camera/motor-bus Pi in a split deployment) to this machine's matching
# paths, so a compute-side stack (scripts/up.sh, scripts/rearm-robot.sh, ...)
# sees the same calibration the Pi calibrated.
#
#   scripts/sync-calibration.sh [source-host]   default source-host: 192.168.100.115
#
# Retries until every file copies successfully (Ctrl-C to give up).
set -Eeuo pipefail

host="${1:-192.168.100.115}"

# Paths documented in scripts/calibrate.sh, relative to $HOME on both sides.
files=(
  .ros/lekiwi_arm_calibration.json
  .ros/camera_info/lekiwi_front.yaml
  .ros/camera_info/lekiwi_wrist.yaml
  .ros/lekiwi_launch_calibration.conf
  .cache/huggingface/lerobot/calibration/robots/lekiwi/lekiwi_1.json
)

attempt=0
while true; do
  attempt=$((attempt + 1))
  ok=1
  for f in "${files[@]}"; do
    if rsync -a --mkpath -e ssh "$host:$f" "$HOME/$f" 2>&1; then
      echo "synced $f"
    else
      echo "$0: failed to sync $f (attempt $attempt)" >&2
      ok=0
    fi
  done
  [ "$ok" = 1 ] && break
  echo "retrying in 5s..." >&2
  sleep 5
done

echo "All calibration files synced from $host."
