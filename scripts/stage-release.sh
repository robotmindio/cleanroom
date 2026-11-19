#!/usr/bin/env bash
# Build and test a detached, persistent release without changing live paths.
# Parse all commands before building: Bash otherwise rereads an edited caller
# file at its old offset when a long-running child returns.
{
set -Eeuo pipefail
umask 077
export GIT_TERMINAL_PROMPT=0
project_root=${4:-$(cd -- "$(dirname -- "$0")/.." && pwd)}
die() { echo "$0: $*" >&2; exit 1; }
[[ $# == 3 || $# == 4 ]] || die "usage: $0 compute|device WORKSPACE REVISION [SOURCE_REPOSITORY]"
role=$1
workspace=$2
revision=$3
[[ $role == compute || $role == device ]] || die "unknown release role"
[[ $workspace =~ ^/[A-Za-z0-9._/-]+$ && $revision =~ ^[0-9a-f]{40}$ ]] || die "invalid workspace or revision"
workspace=$(realpath -e "$workspace")
[[ $workspace != /tmp && $workspace != /tmp/* ]] || die "releases require a persistent workspace outside /tmp"
[[ ! -f $workspace/release.json ]] || die "pass the bootstrap workspace, not a release"
[[ -f $workspace/install/setup.bash ]] || die "bootstrap workspace is not installed"
release=$workspace/releases/$revision
mkdir -p "$workspace/releases"
exec 9>"$workspace/releases/stage.lock"
flock -n 9 || die "another release is being staged"
if [[ -f $release/release.json ]]; then
  /usr/bin/python3 "$release/source/scripts/check-release.py" verify "$release" "$revision" "$role"
  exit
fi
[[ $(readlink -f "$workspace/current" 2>/dev/null || true) != "$release" ]] || die "cannot rebuild an active release"
# Bound what the workspace keeps before building, while the space still helps.
# The staged revision's own policy keeps the active, rollback, in-use and newest
# sealed releases; it removes older ones, unsealed leftovers of failed runs
# (this revision's included) and dependency overlays nothing kept is built on.
# Staging then needs room for a checkout, a build and, when the pinned
# dependencies changed, a new overlay: 95 + 130 + 316 MB measured on the device
# and 0.9 + 1.8 GB on compute, roughly doubled for test output.
need_free=$((4 << 30))
[[ $role == compute ]] || need_free=$((1 << 30))
git -C "$project_root" show "$revision:scripts/prune-release-builds.py" |
  /usr/bin/python3 - "$workspace" --retain --need-free "$need_free" --apply
# A release directory deleted by hand leaves a record that blocks its checkout.
git -C "$project_root" worktree prune
[[ ! -e $release ]] || die "an unsealed $release is still in use; stop what uses it and retry"
# A failed or interrupted run frees its partial release and overlay at once: a
# full disk endangers the running robot. A sealed release is complete and stays.
# shellcheck disable=SC2317 # Reached through the EXIT trap.
discard_unsealed_release() {
  [[ ! -e $release/release.json ]] || return 0
  cd /
  git -C "$project_root" worktree remove --force --force "$release/source" 2>/dev/null || true
  rm -rf -- "$release"
  git -C "$project_root" worktree prune || true
  [[ -z ${overlay:-} || -e $overlay/.complete ]] || rm -rf -- "$overlay"
}
trap discard_unsealed_release EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
mkdir -p "$release/install" "$release/src"
git -C "$project_root" worktree add --detach "$release/source" "$revision"

# Preserve the installed service's local settings without exposing their contents.
unit=lekiwi-stack.service
[[ $role != device ]] || unit=lekiwi-host.service
settings_root=$(systemctl show -P WorkingDirectory "$unit" 2>/dev/null || true)
[[ -f $settings_root/.env ]] || settings_root=$project_root
if [[ -f $settings_root/.env ]]; then
  install -m 0600 "$settings_root/.env" "$release/source/.env"
fi
for venv in .venv .venv-lerobot; do
  if [[ -d $workspace/$venv ]]; then
    ln -s "$workspace/$venv" "$release/$venv"
  fi
done
# Patched third-party dependencies (native ROS fixes, camera and lidar drivers)
# change rarely. Each role builds them once into a fixed, content-keyed overlay
# on the bootstrap workspace, and later releases build only this package on top.
# Installed files keep valid absolute paths because an overlay never moves.
key=$("$release/source/scripts/dependency-overlay-key.sh" "$role" "$release/source" "$workspace")
overlay=$workspace/overlays/$role-$key
build_args=()
[[ $role == compute ]] || build_args+=(--device)
if [[ $(cat "$overlay/.complete" 2>/dev/null || true) != "$key" ]]; then
  # An interrupted build leaves no marker; never reuse its partial install.
  rm -rf -- "$overlay"
  mkdir -p "$overlay/install" "$overlay/src"
  printf 'source %q\n' "$workspace/install/setup.bash" > "$overlay/install/setup.bash"
  dependencies=(ldlidar_stl_ros2 ros2_astra_camera)
  [[ $role != compute ]] || dependencies+=(class_loader rclcpp navigation2 rviz rtabmap_ros ament_cmake)
  for dependency in "${dependencies[@]}"; do
    if [[ -d $workspace/src/$dependency/.git ]]; then
      # Installer caches are partial clones; copying their full history can ask
      # for blobs they deliberately never downloaded. Only HEAD is materialized.
      git clone --depth 1 "file://$workspace/src/$dependency" "$overlay/src/$dependency"
      git -C "$overlay/src/$dependency" remote set-url origin \
        "$(git -C "$workspace/src/$dependency" remote get-url origin)"
    fi
  done
  if [[ $role == compute ]]; then
    LEKIWI_WS=$overlay "$release/source/scripts/build-native.sh"
  fi
  LEKIWI_WS=$overlay "$release/source/scripts/build-lekiwi.sh" --dependencies "${build_args[@]}"
  printf '%s\n' "$key" > "$overlay/.complete"
fi
printf 'source %q\n' "$overlay/install/setup.bash" > "$release/install/setup.bash"
printf '%s\n' "$overlay" > "$release/install/.lekiwi-overlay"
export LEKIWI_WS=$release
"$release/source/scripts/build-lekiwi.sh" "${build_args[@]}"
set +u
# shellcheck source=/dev/null
source "$release/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
export TMPDIR=$release/test-tmp
mkdir -p "$TMPDIR"
/usr/bin/python3 -c 'import zmq'
/usr/bin/python3 -m compileall -q "$release/source/lekiwi_rmf" "$release/source/scripts" "$release/source/test"
cd "$release/source"
mapfile -t shell_scripts < <(find "$release/source/scripts" -type f \( -name '*.sh' -o -name '*.bash' \) | sort)
shellcheck "${shell_scripts[@]}"
ctest --test-dir "$release/build/lekiwi_rmf" --output-on-failure --output-junit "$release/build/lekiwi_rmf/release-ctest.xml"
/usr/bin/python3 "$release/source/scripts/check-release.py" seal "$release" "$revision" "$role"
exit
}
