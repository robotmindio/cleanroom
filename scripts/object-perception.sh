#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
set +u
# shellcheck source=/dev/null
source "$script_dir/setup.bash"
set -u
exec ros2 launch lekiwi_rmf object_perception.launch.py "$@"
