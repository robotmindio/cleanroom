#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
set +u
# shellcheck source=/dev/null
source "$script_dir/setup.bash"
set -u
export PYTHONPATH="$script_dir/..${PYTHONPATH:+:$PYTHONPATH}"
if [[ ${1:-} == --describe ]]; then
  shift
  exec python -m lekiwi_rmf.object_recognition "$@"
fi
exec ros2 launch "$script_dir/../launch/object_perception.launch.py" "$@"
