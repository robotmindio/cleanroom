#!/usr/bin/env bash
# Delete ROS logs of finished launches. Logs untouched for four days are removed
# (loose logs and launch-directory files, depth 1-2), except any file a running
# process still holds open: unlinking that would silently discard everything the
# process logs afterwards. Launch directories left empty for an hour go too.
# find does not follow symlinks, so `latest` is never removed or traversed.
#
# With the logrotate state file, converge that file as well. logrotate never
# forgets an entry and rotates hourly from it. ROS names one log per process, so
# the file grew to 940k entries and 67 MB on the robot, and entries a clock jump
# left in the future made logrotate force a rotation -- one error line per entry
# -- every five minutes. Entries whose log is gone are dropped, future ones set
# to now.
set -Eeuo pipefail

log_dir=${1:?usage: prune-ros-logs.sh LOG_DIR [LOGROTATE_STATE]}
state=${2:-}
open_logs=$(mktemp)
trap 'rm -f "$open_logs"' EXIT

# One scan of /proc for every open log. A fuser call per candidate would cost a
# full /proc walk each: about 70 ms x the backlog of a first run, minutes of CPU.
# Other users' processes are unreadable and cannot be ours, so errors are ignored.
find /proc/[0-9]*/fd -lname "$log_dir/*" -printf '%l\n' 2>/dev/null | sort -u >"$open_logs" || true

find "$log_dir" -mindepth 1 -maxdepth 2 -type f -mtime +3 -print0 \
  | { grep -zvxFf "$open_logs" || true; } | xargs -0r rm -f --
find "$log_dir" -mindepth 1 -maxdepth 1 -type d -empty -mmin +60 -delete

if [[ -n $state && -f $state ]]; then
  find "$log_dir" -mindepth 1 -maxdepth 2 -type f -print0 >"$open_logs"
  /usr/bin/python3 - "$state" "$open_logs" <<'PY'
import os, sys, time

state, listing = sys.argv[1], sys.argv[2]
with open(listing, 'rb') as stream:
    existing = set(stream.read().split(b'\0'))
existing.discard(b'')
now = time.time()
kept = dropped = clamped = 0
temporary = state + '.new'
with open(state, 'rb') as source, open(temporary, 'wb') as target:
    for line in source:
        path, _, stamp = line.rstrip(b'\n').rpartition(b' ')
        try:
            when = time.mktime(time.strptime(stamp.decode(), '%Y-%m-%d-%H:%M:%S'))
        except (ValueError, UnicodeDecodeError):
            target.write(line)  # the "logrotate state -- version 2" header
            continue
        if path.strip(b'"') not in existing:
            dropped += 1
            continue
        if when > now:
            line = path + b' ' + time.strftime('%Y-%m-%d-%H:%M:%S', time.localtime(now)).encode() + b'\n'
            clamped += 1
        target.write(line)
        kept += 1
os.chmod(temporary, 0o600)
os.replace(temporary, state)
print(f'prune-ros-logs.sh: logrotate state kept {kept}, dropped {dropped} stale, clamped {clamped} future entries')
PY
fi
