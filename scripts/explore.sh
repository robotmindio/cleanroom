#!/usr/bin/env bash
# Start attended exploration with the tracked server limits; Ctrl-C cancels it.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# ROS's setup.bash reads unset variables.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

exec /usr/bin/python3 -m lekiwi_rmf.explore_client
