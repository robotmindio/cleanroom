#!/usr/bin/env bash
# Source this file; do not execute it. Works in bash and zsh.

_lekiwi_workspace=${LEKIWI_WS:-"$HOME/lekiwi_ws"}
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
  unset _lekiwi_workspace _lekiwi_shell
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
unset _lekiwi_workspace _lekiwi_shell
