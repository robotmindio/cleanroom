#!/usr/bin/env bash
# Mutual-TLS identities for the zenoh sensor bridge (config/zenoh_device.json5,
# config/zenoh_compute.json5). install-compute-services.sh runs this whenever
# the stack takes sensors over the bridge; run it by hand to renew.
#
# A private CA signs one certificate for the device (server) and one for compute
# (client). Each side trusts only that CA, and the device refuses any connection
# that does not present the compute certificate. The CA key stays in --ca-dir on
# the compute machine; each machine gets only the CA certificate and its own
# key, under /etc/lekiwi/zenoh-tls, the fixed path both configs name.
#
# Idempotent: existing identities are kept and already-installed files are left
# alone (no sudo needed). --renew replaces every identity.
#
# DEVICE is [USER@]HOST reached over ssh, or "local" when the device services run
# on this machine too.
#
# Usage: scripts/setup-zenoh-tls.sh [--user USER] [--ca-dir DIR] [--renew] DEVICE
#        scripts/setup-zenoh-tls.sh --ca-dir DIR --generate-only
set -Eeuo pipefail

die() { printf 'error: %s\n' "$*" >&2; exit 1; }
log() { printf '\n==> %s\n' "$*"; }

INSTALL_DIR=/etc/lekiwi/zenoh-tls
service_user=${SUDO_USER:-$(id -un)}
ca_dir="" renew=false generate_only=false device=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --user) [[ $# -ge 2 ]] || die "--user needs a name"; service_user=$2; shift 2 ;;
    --ca-dir) [[ $# -ge 2 ]] || die "--ca-dir needs a directory"; ca_dir=$2; shift 2 ;;
    --renew) renew=true; shift ;;
    --generate-only) generate_only=true; shift ;;
    -*) die "unknown option: $1" ;;
    *) [[ -z $device ]] || die "only one device"; device=$1; shift ;;
  esac
done
[[ $generate_only == true || -n $device ]] || die "usage: $0 [--user USER] [--ca-dir DIR] [--renew] DEVICE"
service_home=$(getent passwd "$service_user" | cut -d: -f6) || die "unknown user: $service_user"
[[ -n $service_home ]] || die "unknown user: $service_user"
: "${ca_dir:=${LEKIWI_ZENOH_CA_DIR:-$service_home/.ros/lekiwi/zenoh-ca}}"
[[ $ca_dir == /* && $ca_dir != *[[:space:]]* ]] || die "--ca-dir must be an absolute path without whitespace"
if [[ -n $device && $device != local && ! $device =~ ^([a-z_][a-z0-9_-]*@)?[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  die "device must be local or a hostname or address, optionally prefixed by USER@"
fi

# Everything that touches the CA or ssh keys runs as the service user, never as root.
as_user() {
  if [[ $EUID -eq 0 && $service_user != root ]]; then runuser -u "$service_user" -- "$@"; else "$@"; fi
}
as_root() { if [[ $EUID -eq 0 ]]; then "$@"; else sudo "$@"; fi; }

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

generate() {
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
    openssl verify -CAfile "$ca_dir/ca.crt" "$ca_dir/$name.crt" >/dev/null || \
      die "$name certificate does not verify against the CA"
  done
}

if [[ $generate_only == true ]]; then
  generate
  printf 'generated %s\n' "$ca_dir"
  exit 0
fi
if [[ $EUID -eq 0 && $service_user != root ]]; then
  renew_arg=()
  [[ $renew == true ]] && renew_arg=(--renew)
  as_user "$0" --user "$service_user" --ca-dir "$ca_dir" "${renew_arg[@]}" --generate-only
else
  generate
fi

# The files one role needs, as checksummed from inside their directory.
identity_files() { printf '%s\n' ca.crt "$1.crt" "$1.key"; }

# install_local ROLE: install this machine's ROLE identity unless it already matches.
install_local() {
  local role=$1 files
  mapfile -t files < <(identity_files "$role")
  if [[ $(cd "$INSTALL_DIR" 2>/dev/null && sha256sum "${files[@]}" 2>/dev/null) == \
        $(cd "$ca_dir" && sha256sum "${files[@]}") ]]; then
    printf '%s identity already installed in %s\n' "$role" "$INSTALL_DIR"
    return
  fi
  log "Installing the $role identity in $INSTALL_DIR"
  as_root install -d -o "$service_user" -m 0700 "$INSTALL_DIR"
  for f in "${files[@]}"; do as_root install -o "$service_user" -m 0600 "$ca_dir/$f" "$INSTALL_DIR/$f"; done
}

install_local compute
if [[ $device == local ]]; then
  install_local device
  exit 0
fi

# The device gets no CA key and no compute key. Files travel over ssh stdin, not argv.
mapfile -t device_files < <(identity_files device)
ssh_device=(as_user ssh -o BatchMode=yes -o ConnectTimeout=10 "$device")
remote_sums=$("${ssh_device[@]}" "cd $INSTALL_DIR 2>/dev/null && sha256sum ${device_files[*]}" 2>/dev/null || true)
if [[ $remote_sums == $(cd "$ca_dir" && sha256sum "${device_files[@]}") ]]; then
  printf 'device identity already installed on %s\n' "$device"
  exit 0
fi
log "Installing the device identity on $device"
# shellcheck disable=SC2016 # $d and $(id -un) expand on the device.
tar -C "$ca_dir" -cf - "${device_files[@]}" |
  "${ssh_device[@]}" 'set -e; d=$(mktemp -d); trap "rm -rf $d" EXIT; tar -C $d -xf -;
    sudo -n install -d -o "$(id -un)" -m 0700 /etc/lekiwi/zenoh-tls;
    for f in ca.crt device.crt device.key; do sudo -n install -o "$(id -un)" -m 0600 "$d/$f" /etc/lekiwi/zenoh-tls/$f; done'
printf 'Restart the bridge on both machines to use it (scripts/deploy-split.sh does both).\n'
