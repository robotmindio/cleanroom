#!/usr/bin/env bash

# Source-only. Under systemd, kill the service's main process when it can no longer work
# but would never exit by itself, so the unit's Restart= policy brings it back:
#
#   - Cyclone DDS binds its network interface once and never rebinds. When an IPv4 address
#     on a real interface is removed or added (Wi-Fi switch, link bounce, DHCP arriving
#     after boot), every DDS participant in the process goes silent while still looking
#     healthy.
#   - A serial node keeps a dead handle after its USB device re-enumerates.
#
# The main process dies from SIGKILL, which every Restart= policy counts as a failure.
# The units keep NoNewPrivileges, so nothing here asks sudo or systemd to restart anything.
# Manual runs (no INVOCATION_ID) are left alone: nothing would restart them.
#
# Usage, immediately before the final `exec`: self_heal [DEVICE_NODE]

_self_heal_real_interface() { # Docker, libvirt and VPN interfaces change with unrelated work
  case $1 in lo|docker*|br-*|veth*|virbr*|tailscale*|wg*) return 1 ;; esac
}

self_heal_addresses() { # "interface address" per line, real interfaces only
  local _ ifname addr
  ip -4 -o addr show scope global | while read -r _ ifname _ addr _; do
    if _self_heal_real_interface "$ifname"; then printf '%s %s\n' "$ifname" "$addr"; fi
  done | sort
}

_self_heal_events() { ip -4 -o monitor address; }

_self_heal_kill() { # <main pid> <reason>
  echo "self-heal: $2; letting systemd restart the service" >&2
  # Let a link bounce finish so one restart covers it.
  sleep "${LEKIWI_SELF_HEAL_SETTLE:-3}"
  kill -KILL "$1" 2>/dev/null || true
}

_self_heal_watch_network() { # <main pid>
  local baseline line ifname addr fields
  baseline=$(self_heal_addresses)
  _self_heal_events | while read -r line; do
    [[ $line == *"scope global"* ]] || continue
    read -r -a fields <<<"$line"
    if [[ ${fields[0]} == Deleted ]]; then
      ifname=${fields[2]} addr=${fields[4]}
    else
      ifname=${fields[1]} addr=${fields[3]}
    fi
    _self_heal_real_interface "${ifname%:}" || continue
    # A DHCP renewal re-announces an address that is already bound; only removal
    # or a new address breaks the DDS binding.
    if [[ ${fields[0]} == Deleted ]] || ! grep -qxF "${ifname%:} $addr" <<<"$baseline"; then
      _self_heal_kill "$1" "IPv4 address ${ifname%:} $addr changed"
      break
    fi
  done
}

_self_heal_watch_device() { # <main pid> <device node>
  local identity
  identity=$(stat -L -c %d:%i "$2") || return 0
  while [[ $(stat -L -c %d:%i "$2" 2>/dev/null) == "$identity" ]]; do sleep 2; done
  _self_heal_kill "$1" "$2 was re-enumerated or removed"
}

self_heal() { # [DEVICE_NODE]
  [[ -n ${INVOCATION_ID:-} ]] || return 0
  local main=$$
  _self_heal_watch_network "$main" &
  if [[ -n ${1:-} ]]; then _self_heal_watch_device "$main" "$1" & fi
}
