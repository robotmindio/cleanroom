#!/usr/bin/env bash
# Reclaim the disk these hosts share with everything else. Dry-run by default;
# lekiwi-disk-hygiene.timer runs it with --apply once a day. Run as root: the
# journal, the system logs and the package caches need it. The cleanup inside
# the service user's home (release workspaces, Trash, retired RTAB-Map databases)
# still runs as that user, so nothing lands there owned by root.
#
# usage: scripts/disk-hygiene.sh [--apply] [--user USER] [--workspace PATH]
set -Eeuo pipefail

apply=false
user=root
workspace=
while (($#)); do
  case $1 in
    --apply) apply=true; shift ;;
    --user) user=${2:?--user needs a value}; shift 2 ;;
    --workspace) workspace=${2:?--workspace needs a value}; shift 2 ;;
    --help) sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "usage: $0 [--apply] [--user USER] [--workspace PATH]" >&2; exit 2 ;;
  esac
done
[[ $(id -u) -eq 0 ]] || { echo "disk-hygiene: run as root; the timer already does" >&2; exit 1; }
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
home=$(getent passwd "$user" | cut -d: -f6)
[[ $user == root || (-n $home && -d $home) ]] || { echo "disk-hygiene: no home for $user" >&2; exit 1; }
# The service installers can name the active release (.../lekiwi_ws/current);
# the retention policy works on the workspace that holds releases/ and overlays/.
if [[ -n $workspace && ! -d $workspace/releases ]]; then
  workspace=$(dirname -- "$(dirname -- "$(realpath -e -- "$workspace")")")
fi

act() { # act <description> <command...>
  local description=$1
  shift
  if [[ $apply == true ]]; then
    printf 'disk-hygiene: %s\n' "$description"
    "$@"
  else
    printf 'disk-hygiene: would %s\n' "$description"
  fi
}

# The journal has its own hard cap since /etc/systemd/journald.conf.d/lekiwi.conf;
# this also gives back what that cap only enforces on the next write.
act 'vacuum the journal to 512M' /usr/bin/journalctl --vacuum-size=512M

# /var/log/syslog* is distro logrotate's (weekly, four archives) and was 400 MB
# for one week of a runaway driver. Truncating the rotated copies keeps the
# archive names logrotate expects; the active file is rotated by its own timer.
for file in /var/log/syslog.[0-9] /var/log/kern.log.[0-9] /var/log/auth.log.[0-9]; do
  [[ -f $file ]] || continue
  size=$(stat -c %s "$file")
  (( size > 100 << 20 )) && act "truncate $file ($((size >> 20)) MiB)" truncate -s 0 "$file"
done

# snapd keeps every downloaded snap revision and two extra installed ones: 4.1 GB
# of cache and superseded firefox/gnome/emacs revisions on the robot.
if command -v snap >/dev/null; then
  act 'keep the current and one previous snap revision' /usr/bin/snap set system refresh.retain=2
  act 'empty the snap download cache' find /var/lib/snapd/cache -mindepth 1 -maxdepth 1 -type f -delete
fi
[[ -d /var/cache/apt/archives ]] && act 'empty the apt package cache' /usr/bin/apt-get clean

# A release workspace keeps what it is running, one rollback, locked
# qualification worktrees and the three newest sealed builds. The tool is the
# one stage-release.sh uses, so deployment and this timer share the policy and
# its locks; a deployment in progress makes it refuse this run.
if [[ -n $workspace && -d $workspace ]]; then
  retention=(/usr/bin/python3 "$project_root/scripts/prune-release-builds.py" "$workspace" --retain)
  [[ $apply == true ]] && retention+=(--apply)
  printf 'disk-hygiene: %s\n' "release retention in $workspace"
  /usr/sbin/runuser -u "$user" -- env HOME="$home" "${retention[@]}" \
    || printf 'disk-hygiene: release retention failed; the line above says why\n'
fi

# Nothing below is a record: Trash is what the operator threw away, and a
# .stale-/.corrupt- RTAB-Map database is a copy of a database that was already
# replaced.
remove_older_than() { # remove_older_than <days> <path> <find expression...>
  local days=$1 path=$2
  shift 2
  [[ -d $path ]] || return 0
  if [[ $apply == true ]]; then
    find "$path" -mindepth 1 -maxdepth 1 -mtime +"$days" "$@" -exec rm -rf -- {} +
  else
    find "$path" -mindepth 1 -maxdepth 1 -mtime +"$days" "$@" -print | sed 's/^/disk-hygiene: would remove /'
  fi
}
remove_older_than 30 "$home/.local/share/Trash/files" -user "$user"
remove_older_than 14 "$home/.ros" -user "$user" -type f \( -name '*.stale-*' -o -name '*.corrupt-*' \)

read -r available used < <(df -P / | awk 'NR == 2 {print $4, $5}')
printf 'disk-hygiene: %s MiB free, %s used on /\n' "$((available / 1024))" "$used"
(( available > 1 << 20 )) || { echo "disk-hygiene: less than 1 GiB free after cleanup" >&2; exit 1; }
