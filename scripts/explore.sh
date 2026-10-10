#!/usr/bin/env bash
# Start attended exploration with the tracked server limits; Ctrl-C cancels it.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# Operator commands use the managed deployment when one is installed.
managed_workspace="${LEKIWI_WS:-$HOME/lekiwi_ws}/current"
if [[ -f $managed_workspace/install/setup.bash ]]; then
  export LEKIWI_WS="$managed_workspace"
fi
# ROS's setup.bash reads unset variables.
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

exec /usr/bin/python3 -P -m lekiwi_rmf.explore_client
