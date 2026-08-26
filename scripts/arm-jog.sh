#!/usr/bin/env bash
# One explicit, bounded physical arm-joint jog. See scripts/arm-jog.sh --help.
set -Eeuo pipefail
cd "$(dirname "$0")/.."
set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u
exec ros2 run lekiwi_rmf arm_jog.py "$@"
