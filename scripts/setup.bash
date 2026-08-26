#!/usr/bin/env bash
# Source this file; do not execute it. Works in bash and zsh.

# Prefer the repository containing this script. This keeps `scripts/up.sh` and
# `colcon build` on the same checkout instead of silently launching an older
# ~/lekiwi_ws overlay. LEKIWI_WS remains an explicit deployment override.
if [ -n "${ZSH_VERSION:-}" ]; then
  _lekiwi_setup_file="${(%):-%x}"
else
  _lekiwi_setup_file="${BASH_SOURCE[0]}"
fi
_lekiwi_repo_workspace="$(cd "$(dirname "$_lekiwi_setup_file")/.." && pwd)"
_lekiwi_workspace=${LEKIWI_WS:-"$_lekiwi_repo_workspace"}
# A source checkout may intentionally share a separately installed LeRobot
# environment. In that case retain the established overlay until install.sh has
# created this checkout's own .venv; never select a half-installed workspace.
if [ -z "${LEKIWI_WS:-}" ] && [ ! -f "$_lekiwi_workspace/.venv/bin/activate" ] \
  && [ -f "$HOME/lekiwi_ws/.venv/bin/activate" ]; then
  _lekiwi_workspace="$HOME/lekiwi_ws"
fi
# ponytail: zsh must use the .zsh variants -- the .bash ones read ${BASH_SOURCE[0]},
# which is unset in zsh, and the .zsh ones use `builtin cd -q` so a chpwd hook that
# prints (eza, ls) cannot pollute the command substitution that resolves the prefix.
if [ -n "${ZSH_VERSION:-}" ]; then
  _lekiwi_shell=zsh
else
  _lekiwi_shell=bash
fi

if [ ! -f "/opt/ros/jazzy/setup.$_lekiwi_shell" ] ||
   [ ! -f "$_lekiwi_workspace/install/setup.$_lekiwi_shell" ]; then
  printf 'LeKiwi stack is not installed. Run scripts/install.sh first.\n' >&2
  unset _lekiwi_workspace _lekiwi_repo_workspace _lekiwi_setup_file _lekiwi_shell
  return 1 2>/dev/null
  exit 1
fi

# shellcheck disable=SC1090
. "/opt/ros/jazzy/setup.$_lekiwi_shell"
# shellcheck disable=SC1091
. "$_lekiwi_workspace/.venv/bin/activate"
# shellcheck disable=SC1090
. "$_lekiwi_workspace/install/setup.$_lekiwi_shell"
export PATH="$HOME/.local/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_lekiwi_workspace/install/lekiwi_rmf/share/lekiwi_rmf/config/cyclonedds.xml"
unset _lekiwi_workspace _lekiwi_repo_workspace _lekiwi_setup_file _lekiwi_shell
