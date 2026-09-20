#!/usr/bin/env bash
# Export this device machine's sensor topics to the compute stack over zenoh.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
# shellcheck source=/dev/null
source scripts/lib/self-heal.sh
source_device_ros_env
export PATH="$HOME/.local/bin:$PATH"

# The link is mutual TLS (config/zenoh_device.json5). Never fall back to plaintext.
tls_dir=/etc/lekiwi/zenoh-tls
for file in ca.crt device.crt device.key; do
  [[ -r $tls_dir/$file ]] || {
    echo "ros-zenoh: missing $tls_dir/$file; run scripts/setup-zenoh-tls.sh on the compute machine" >&2
    exit 1
  }
done

# The bridge's DDS side binds the network interface like any other participant.
self_heal
exec zenoh-bridge-ros2dds -c config/zenoh_device.json5
