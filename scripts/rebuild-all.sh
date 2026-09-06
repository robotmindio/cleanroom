#!/usr/bin/env bash
# Regenerate the source model, vendor it, render it, build ROS, and run its tests.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
project_root=$PWD
lekiwi_source=${LEKIWI_SOURCE:-$project_root/../LeKiwi}
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}

git -C "$lekiwi_source" rev-parse --is-inside-work-tree >/dev/null 2>&1 || {
  echo "$0: LeKiwi source repository not found: $lekiwi_source" >&2
  exit 1
}

if [[ -z ${CADQUERY_PYTHON:-} && -x $HOME/.cache/lekiwi-cadquery-venv/bin/python ]]; then
  export CADQUERY_PYTHON=$HOME/.cache/lekiwi-cadquery-venv/bin/python
fi

model_paths=(URDF/LeKiwi.urdf.xacro URDF/model-manifest.json URDF/meshes cad scripts)
[[ -z $(git -C "$lekiwi_source" status --porcelain --untracked-files=no -- "${model_paths[@]}") ]] || {
  echo "$0: commit LeKiwi model changes before rebuilding" >&2
  exit 1
}
source_head=$(git -C "$lekiwi_source" rev-parse HEAD)
cleanup_source() {
  git -C "$lekiwi_source" restore --worktree -- "${model_paths[@]}"
}
trap cleanup_source EXIT
(cd "$lekiwi_source" && ./scripts/verify_robot.sh)
cleanup_source
trap - EXIT
[[ $(git -C "$lekiwi_source" rev-parse HEAD) == "$source_head" ]] || {
  echo "$0: LeKiwi HEAD changed during validation; run again" >&2
  exit 1
}
PYTHONNOUSERSITE=1 python3 scripts/vendor-lekiwi-model.py --source "$lekiwi_source"
PYTHONNOUSERSITE=1 python3 scripts/render-model.py
./scripts/build-lekiwi.sh
ctest --test-dir "$workspace/build/lekiwi_rmf" --output-on-failure
