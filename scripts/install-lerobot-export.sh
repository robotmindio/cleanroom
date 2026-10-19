#!/usr/bin/env bash
# Isolated CPU environment for scripts/export-lerobot.py; ROS and the motor
# host Python installations are not modified.
set -euo pipefail
venv=${LEKIWI_LEROBOT_VENV:-$HOME/.ros/lekiwi/lerobot-venv}
uv venv --python 3.12 "$venv"
VIRTUAL_ENV=$venv uv pip install --index-strategy unsafe-best-match \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  lerobot==0.4.1 mcap==1.5.0 mcap-ros2-support==0.5.7
echo "$venv/bin/python scripts/export-lerobot.py EPISODE... --root DATASET"
