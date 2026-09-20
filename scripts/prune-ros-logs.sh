#!/usr/bin/env bash
# Delete ROS logs of finished launches. Logs untouched for four days are removed
# (loose logs and launch-directory files, depth 1-2), except any file a running
# process still holds open: unlinking that would silently discard everything the
# process logs afterwards. Launch directories left empty for an hour go too.
# find does not follow symlinks, so `latest` is never removed or traversed.
set -Eeuo pipefail

log_dir=${1:?usage: prune-ros-logs.sh LOG_DIR}
open_logs=$(mktemp)
trap 'rm -f "$open_logs"' EXIT

# One scan of /proc for every open log. A fuser call per candidate would cost a
# full /proc walk each: about 70 ms x the backlog of a first run, minutes of CPU.
# Other users' processes are unreadable and cannot be ours, so errors are ignored.
find /proc/[0-9]*/fd -lname "$log_dir/*" -printf '%l\n' 2>/dev/null | sort -u >"$open_logs" || true

find "$log_dir" -mindepth 1 -maxdepth 2 -type f -mtime +3 -print0 \
  | { grep -zvxFf "$open_logs" || true; } | xargs -0r rm -f --
find "$log_dir" -mindepth 1 -maxdepth 1 -type d -empty -mmin +60 -delete
