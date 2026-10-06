#!/usr/bin/env bash
# Source-only helpers for scripts/build-lekiwi.sh and scripts/build-native.sh.

remove_foreign_build_cache() { # remove_foreign_build_cache <workspace> <project_root>
  # CMake cannot reuse a build from another checkout; otherwise its normal
  # incremental build avoids unnecessary CPU contention with robot callbacks.
  local cache=$1/build/lekiwi_rmf/CMakeCache.txt
  if [[ -f $cache && $(awk -F= '$1 == "CMAKE_HOME_DIRECTORY:INTERNAL" {print $2}' "$cache") != "$2" ]]; then
    rm -rf -- "$1/build/lekiwi_rmf"
  fi
}

low_available_memory() { # low_available_memory [meminfo]: under 8000 MiB available now
  # Other applications can consume most RAM even on a large compute host, so
  # judge by available rather than total memory.
  (( $(awk '/^MemAvailable:/ {print int($2/1024)}' "${1:-/proc/meminfo}") < 8000 ))
}

colcon_supports_overriding() { # older colcon lacks the optional --allow-overriding extension
  [[ $(colcon build --help) == *--allow-overriding* ]]
}
