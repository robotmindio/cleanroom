#!/usr/bin/env bash
# Export this device machine's sensor topics to the compute stack over zenoh.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/self-heal.sh
workspace=${LEKIWI_WS:-$HOME/lekiwi_ws}
if [ -f "$workspace/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  # shellcheck source=/dev/null
  source "$workspace/install/setup.bash"
  set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$PWD/config/cyclonedds.xml"
else
  # shellcheck source=/dev/null
  source scripts/setup-pi.bash
fi
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
