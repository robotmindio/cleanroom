#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
set +u
# shellcheck source=/dev/null
source "$script_dir/setup.bash"
set -u
if [[ ${1:-} == --describe ]]; then
  export PYTHONPATH="$script_dir/..${PYTHONPATH:+:$PYTHONPATH}"
  exec python -m lekiwi_rmf.object_recognition
fi
exec ros2 launch lekiwi_rmf object_perception.launch.py "$@"
