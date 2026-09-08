#!/usr/bin/env bash
# Open the local, read-only Foxglove dashboard on the graphical workstation.
set -Eeuo pipefail

command -v foxglove-studio >/dev/null || {
  echo "Foxglove Desktop is not installed; run scripts/install.sh first" >&2
  exit 1
}

exec foxglove-studio \
  "foxglove://open?ds=foxglove-websocket&ds.url=ws://127.0.0.1:8765/"
