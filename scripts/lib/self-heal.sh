#!/usr/bin/env bash

# Source-only. A serial node keeps a dead handle after USB re-enumeration;
# kill its main process so systemd's Restart= policy opens the new device.
# DDS uses loopback and the network transports reconnect themselves. Restarting
# those services on Wi-Fi changes interrupts trajectories and map persistence.
#
# The main process dies from SIGKILL, which every Restart= policy counts as a failure.
# The units keep NoNewPrivileges, so nothing here asks sudo or systemd to restart anything.
# Manual runs (no INVOCATION_ID) are left alone: nothing would restart them.
#
# Usage, immediately before the final `exec`: self_heal [DEVICE_NODE]

_self_heal_kill() { # <main pid> <reason>
  echo "self-heal: $2; letting systemd restart the service" >&2
  # Let device enumeration finish so one restart covers it.
  sleep "${LEKIWI_SELF_HEAL_SETTLE:-3}"
  kill -KILL "$1" 2>/dev/null || true
}

_self_heal_watch_device() { # <main pid> <device node>
  local identity
  identity=$(stat -L -c %d:%i "$2") || return 0
  while [[ $(stat -L -c %d:%i "$2" 2>/dev/null) == "$identity" ]]; do sleep 2; done
  _self_heal_kill "$1" "$2 was re-enumerated or removed"
}

self_heal() { # [DEVICE_NODE]
  [[ -n ${INVOCATION_ID:-} && -n ${1:-} ]] || return 0
  _self_heal_watch_device "$$" "$1" &
}
