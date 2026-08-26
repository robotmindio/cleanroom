#!/usr/bin/env bash
# Provision a headless Gazebo/ROS simulation server.  Hardware-only LeRobot
# packages are deliberately omitted; use install.sh without --simulation on a
# host that will talk to motors or cameras.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
exec scripts/install.sh --simulation
