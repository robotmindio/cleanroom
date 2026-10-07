#!/usr/bin/env bash
# Source-only helpers for scripts/deploy-split.sh; die() comes from runtime-common.sh.

transfer_device_revision() ( # transfer_device_revision <source-repo> <device-repo> <revision> <ssh command...>
  # The compute revision is already verified against origin. The device can
  # receive those same Git objects over SSH when its DNS/Internet is unavailable.
  local source_repo=$1 device_repo=$2 revision=$3 previous bundle remote_bundle
  local ssh=("${@:4}") exclusions=()
  previous=$("${ssh[@]}" git -C "$device_repo" rev-parse HEAD)
  [[ $previous =~ ^[0-9a-f]{40}$ ]] || die "invalid device revision"
  [[ $previous != "$revision" ]] || return 0
  if git -C "$source_repo" cat-file -e "$previous^{commit}" 2>/dev/null; then
    exclusions=("^$previous")
  fi
  bundle=$(mktemp)
  remote_bundle=$("${ssh[@]}" mktemp /tmp/lekiwi-source.XXXXXXXX.bundle)
  [[ $remote_bundle =~ ^/tmp/lekiwi-source\.[A-Za-z0-9]+\.bundle$ ]] || die "invalid device bundle path"
  trap 'rm -f -- "$bundle"; "${ssh[@]}" rm -f -- "$remote_bundle"' EXIT
  git -C "$source_repo" bundle create "$bundle" HEAD "${exclusions[@]}"
  "${ssh[@]}" "cat > '$remote_bundle'" < "$bundle"
  "${ssh[@]}" git -C "$device_repo" fetch "$remote_bundle" HEAD
)
