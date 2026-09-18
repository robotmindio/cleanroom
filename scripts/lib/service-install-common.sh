#!/usr/bin/env bash
# Shared, deliberately small helpers for the systemd installers. This file is
# sourced by installers that already provide log() and die().

if ! declare -F as_root >/dev/null; then
  SUDO=()
  [[ $EUID -eq 0 ]] || SUDO=(sudo)
  as_root() { # as_root <command...>
    [[ $EUID -eq 0 ]] || command -v sudo >/dev/null || die "sudo is required to $1"
    "${SUDO[@]}" "$@"
  }
fi

service_escape_sed() {
  # Escape the replacement portion of our | delimited sed expressions.
  printf '%s' "$1" | sed 's/[\\&|]/\\&/g'
}

resolve_service_user() {
  # resolve_service_user [requested-user]
  # A root invocation has no safe implied human account.  sudo supplies the
  # invoking account, but direct root must choose explicitly.
  local requested=${1:-} candidate entry uid
  if [[ -n $requested ]]; then
    candidate=$requested
  elif [[ $EUID -eq 0 ]]; then
    [[ -n ${SUDO_USER:-} && ${SUDO_USER} != root ]] || \
      die "running as root requires --service-user USER"
    candidate=$SUDO_USER
  else
    candidate=$(id -un)
  fi

  [[ $candidate =~ ^[a-z_][a-z0-9_-]*$ ]] || die "invalid service user: $candidate"
  entry=$(getent passwd "$candidate") || die "service user does not exist: $candidate"
  uid=$(id -u "$candidate") || die "cannot determine UID for service user: $candidate"
  [[ $uid -ne 0 ]] || die "refusing to install robot services as root"

  IFS=: read -r _ _ _ _ _ LEKIWI_SERVICE_HOME _ <<<"$entry"
  [[ $LEKIWI_SERVICE_HOME == /* && -d $LEKIWI_SERVICE_HOME ]] || \
    die "service user $candidate has no usable home directory: $LEKIWI_SERVICE_HOME"
  LEKIWI_SERVICE_USER=$candidate
  export LEKIWI_SERVICE_USER LEKIWI_SERVICE_HOME
}

resolve_service_paths() {
  # resolve_service_paths [workspace] [lerobot-venv] [require-workspace] [require-lerobot]
  local workspace_arg=${1:-} venv_arg=${2:-} require_workspace=${3:-false}
  local require_lerobot=${4:-true}
  LEKIWI_SERVICE_WORKSPACE=${workspace_arg:-"$LEKIWI_SERVICE_HOME/lekiwi_ws"}
  [[ $LEKIWI_SERVICE_WORKSPACE == /* ]] || die "workspace must be an absolute path"
  if [[ -n $workspace_arg ]]; then
    [[ -d $LEKIWI_SERVICE_WORKSPACE ]] || die "workspace does not exist: $LEKIWI_SERVICE_WORKSPACE"
  fi

  if [[ $require_lerobot != true ]]; then
    LEKIWI_SERVICE_LEROBOT_VENV=""
  elif [[ -n $venv_arg ]]; then
    LEKIWI_SERVICE_LEROBOT_VENV=$venv_arg
  elif [[ -x $LEKIWI_SERVICE_WORKSPACE/.venv-lerobot/bin/python ]]; then
    LEKIWI_SERVICE_LEROBOT_VENV=$LEKIWI_SERVICE_WORKSPACE/.venv-lerobot
  else
    LEKIWI_SERVICE_LEROBOT_VENV=$LEKIWI_SERVICE_HOME/lerobot-venv
  fi
  if [[ $require_lerobot == true ]]; then
    [[ $LEKIWI_SERVICE_LEROBOT_VENV == /* ]] || die "LeRobot venv must be an absolute path"
    [[ -x $LEKIWI_SERVICE_LEROBOT_VENV/bin/python ]] || \
      die "LeRobot Python is missing: $LEKIWI_SERVICE_LEROBOT_VENV/bin/python"
  fi
  if [[ $require_workspace == true ]]; then
    [[ -d $LEKIWI_SERVICE_WORKSPACE/install ]] || \
      die "no installed ROS workspace at $LEKIWI_SERVICE_WORKSPACE -- run scripts/install.sh as $LEKIWI_SERVICE_USER first"
  fi
  export LEKIWI_SERVICE_WORKSPACE LEKIWI_SERVICE_LEROBOT_VENV
}

render_systemd_unit() {
  # render_systemd_unit <template> <destination>
  local template=$1 destination=$2 root user home workspace venv python bind_address
  local curve_server_secret curve_server_public curve_authorized curve_health_secret
  root=$(service_escape_sed "$PROJECT_ROOT")
  user=$(service_escape_sed "$LEKIWI_SERVICE_USER")
  home=$(service_escape_sed "$LEKIWI_SERVICE_HOME")
  workspace=$(service_escape_sed "$LEKIWI_SERVICE_WORKSPACE")
  venv=$(service_escape_sed "$LEKIWI_SERVICE_LEROBOT_VENV")
  python=$(service_escape_sed "$LEKIWI_SERVICE_LEROBOT_VENV/bin/python")
  bind_address=$(service_escape_sed "${LEKIWI_HOST_BIND_ADDRESS:-0.0.0.0}")
  curve_server_secret=$(service_escape_sed "${LEKIWI_CURVE_SERVER_SECRET:-}")
  curve_server_public=$(service_escape_sed "${LEKIWI_CURVE_SERVER_PUBLIC:-}")
  curve_authorized=$(service_escape_sed "${LEKIWI_CURVE_AUTHORIZED_CLIENTS:-}")
  curve_health_secret=$(service_escape_sed "${LEKIWI_CURVE_HEALTH_CLIENT_SECRET:-}")
  sed -e "s|@PROJECT_ROOT@|$root|g" \
      -e "s|@SERVICE_USER@|$user|g" \
      -e "s|@SERVICE_HOME@|$home|g" \
      -e "s|@WORKSPACE@|$workspace|g" \
      -e "s|@LEROBOT_VENV@|$venv|g" \
      -e "s|@LEROBOT_PYTHON@|$python|g" \
      -e "s|@HOST_BIND_ADDRESS@|$bind_address|g" \
      -e "s|@CURVE_SERVER_SECRET@|$curve_server_secret|g" \
      -e "s|@CURVE_SERVER_PUBLIC@|$curve_server_public|g" \
      -e "s|@CURVE_AUTHORIZED_CLIENTS@|$curve_authorized|g" \
      -e "s|@CURVE_HEALTH_CLIENT_SECRET@|$curve_health_secret|g" \
      "$template" | as_root tee "$destination" >/dev/null
}

verify_systemd_units() {
  # Validate rendered units before they are enabled.  This catches malformed
  # substitutions and missing executables without starting the robot.
  local unit
  command -v systemd-analyze >/dev/null || die "systemd-analyze is required to validate installed units"
  for unit in "$@"; do
    # Dependencies outside this installer can be absent by design (the remote
    # topology has no local host) or unrelated and broken. Validate this unit.
    as_root systemd-analyze verify --recursive-errors=no "$UNIT_DIR/$unit"
  done
}
