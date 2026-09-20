#!/usr/bin/env bash
# Mutual-TLS identities for the zenoh sensor bridge (config/zenoh_device.json5,
# config/zenoh_compute.json5). Run on the compute machine.
#
# A private CA signs one certificate for the device (server) and one for compute
# (client). Each side trusts only that CA, and the device refuses any connection
# that does not present the compute certificate. The CA key stays in --ca-dir on
# this machine; each machine gets only the CA certificate and its own key, under
# /etc/lekiwi/zenoh-tls, the fixed path both configs name.
#
# Re-running reinstalls the same identities; --renew replaces all of them.
#
# Usage: scripts/setup-zenoh-tls.sh [--ca-dir DIR] [--renew] [USER@]DEVICE
#        scripts/setup-zenoh-tls.sh --ca-dir DIR --generate-only
set -Eeuo pipefail

die() { printf 'error: %s\n' "$*" >&2; exit 1; }
log() { printf '\n==> %s\n' "$*"; }

INSTALL_DIR=/etc/lekiwi/zenoh-tls
ca_dir=${LEKIWI_ZENOH_CA_DIR:-$HOME/.ros/lekiwi/zenoh-ca}
renew=false generate_only=false device=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --ca-dir) [[ $# -ge 2 ]] || die "--ca-dir needs a directory"; ca_dir=$2; shift 2 ;;
    --renew) renew=true; shift ;;
    --generate-only) generate_only=true; shift ;;
    -*) die "unknown option: $1" ;;
    *) [[ -z $device ]] || die "only one device"; device=$1; shift ;;
  esac
done
[[ $generate_only == true || -n $device ]] || die "usage: $0 [--ca-dir DIR] [--renew] [USER@]DEVICE"
[[ $ca_dir == /* && $ca_dir != *[[:space:]]* ]] || die "--ca-dir must be an absolute path without whitespace"
if [[ -n $device && ! $device =~ ^([a-z_][a-z0-9_-]*@)?[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  die "device must be a hostname or address, optionally prefixed by USER@"
fi

days=1825
key() { openssl genpkey -algorithm EC -pkeyopt ec_paramgen_curve:P-256 -out "$1" 2>/dev/null && chmod 600 "$1"; }
# leaf NAME EKU: a certificate for NAME limited to one role (serverAuth or clientAuth).
leaf() {
  local name=$1 eku=$2
  key "$ca_dir/$name.key"
  openssl req -new -key "$ca_dir/$name.key" -subj "/CN=lekiwi-$name" -out "$ca_dir/$name.csr"
  openssl x509 -req -in "$ca_dir/$name.csr" -CA "$ca_dir/ca.crt" -CAkey "$ca_dir/ca.key" \
    -CAcreateserial -days "$days" -sha256 -out "$ca_dir/$name.crt" 2>/dev/null \
    -extfile <(printf 'basicConstraints=CA:FALSE\nkeyUsage=digitalSignature\nextendedKeyUsage=%s\nsubjectAltName=DNS:lekiwi-%s\n' "$eku" "$name")
  rm -f "$ca_dir/$name.csr"
}

[[ $renew == true ]] && rm -rf "$ca_dir"
install -d -m 0700 "$ca_dir"
if [[ ! -f $ca_dir/ca.key ]]; then
  log "Creating the zenoh CA in $ca_dir"
  key "$ca_dir/ca.key"
  openssl req -new -x509 -key "$ca_dir/ca.key" -subj "/CN=lekiwi-zenoh-ca" -days "$days" -sha256 \
    -addext "basicConstraints=critical,CA:TRUE,pathlen:0" -addext "keyUsage=critical,keyCertSign" \
    -out "$ca_dir/ca.crt"
fi
[[ -f $ca_dir/device.crt ]] || leaf device serverAuth
[[ -f $ca_dir/compute.crt ]] || leaf compute clientAuth
for name in device compute; do
  openssl verify -CAfile "$ca_dir/ca.crt" "$ca_dir/$name.crt" >/dev/null || die "$name certificate does not verify against the CA"
done
[[ $generate_only == true ]] && { printf 'generated %s\n' "$ca_dir"; exit 0; }

# The owner is the unprivileged account that runs the bridge.
log "Installing the compute identity in $INSTALL_DIR"
sudo install -d -o "$(id -un)" -m 0700 "$INSTALL_DIR"
for f in ca.crt compute.crt compute.key; do
  sudo install -o "$(id -un)" -m 0600 "$ca_dir/$f" "$INSTALL_DIR/$f"
done

log "Installing the device identity on $device"
# The device gets no CA key and no compute key. Files travel over ssh stdin, not argv.
tar -C "$ca_dir" -cf - ca.crt device.crt device.key |
  ssh -o BatchMode=yes "$device" 'set -e; d=$(mktemp -d); trap "rm -rf $d" EXIT; tar -C $d -xf -;
    sudo -n install -d -o "$(id -un)" -m 0700 /etc/lekiwi/zenoh-tls;
    for f in ca.crt device.crt device.key; do sudo -n install -o "$(id -un)" -m 0600 "$d/$f" /etc/lekiwi/zenoh-tls/$f; done'
printf 'Done. Restart the bridge on both machines (scripts/deploy-split.sh does both).\n'
