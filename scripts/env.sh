#!/usr/bin/env bash
# Source from Bash; changes affect only this terminal and its children.
_pai_nav_restore_nounset=0
if [[ $- == *u* ]]; then _pai_nav_restore_nounset=1; set +u; fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  printf '%s\n' 'ROS 2 Jazzy is missing; see docs/SETUP.md.' >&2
  if [[ $_pai_nav_restore_nounset == 1 ]]; then set -u; fi
  unset _pai_nav_restore_nounset
  return 1
fi
source /opt/ros/jazzy/setup.bash
NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NAV_ROOT
if [[ -f "$NAV_ROOT/.venv/bin/activate" ]]; then
  source "$NAV_ROOT/.venv/bin/activate"
fi
export ROS_DOMAIN_ID=42
export GZ_PARTITION=pai_navigation
export PYTHONPATH="$NAV_ROOT${PYTHONPATH:+:$PYTHONPATH}"
if [[ $_pai_nav_restore_nounset == 1 ]]; then set -u; fi
unset _pai_nav_restore_nounset
