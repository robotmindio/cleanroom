#!/usr/bin/env bash
# Rebuild only this repository into the workspace used by the managed services.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
project_root=$PWD
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
[[ $workspace == /* && -d $workspace/install ]] || {
  echo "$0: installed workspace not found: $workspace" >&2
  exit 1
}

set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$workspace/install/setup.bash"
set -u

# Keep user-installed CMake/Protobuf copies from overriding the ROS packages.
PATH=/usr/bin:/bin:$PATH
export PATH
rm -rf -- "$workspace/build/lekiwi_rmf"

parallel_args=()
if (( $(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo) < 8000 )); then
  parallel_args=(--parallel-workers 1)
  export MAKEFLAGS=-j1
fi

colcon --log-base "$workspace/log" build \
  --base-paths "$project_root" \
  --packages-select lekiwi_rmf \
  --build-base "$workspace/build" \
  --install-base "$workspace/install" \
  "${parallel_args[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_IGNORE_PREFIX_PATH="$HOME/.local"

installed_driver=$workspace/install/lekiwi_rmf/lib/lekiwi_rmf/lekiwi_driver
if [[ ! -x $installed_driver ]] || ! cmp -s lekiwi_rmf/driver.py "$installed_driver"; then
  echo "$0: build completed without installing the current driver" >&2
  exit 1
fi

# The split deployer can skip an otherwise disruptive rebuild when both
# workspaces already contain this exact source revision.
git -C "$project_root" rev-parse HEAD > "$workspace/install/lekiwi_rmf/.lekiwi-source-revision"
