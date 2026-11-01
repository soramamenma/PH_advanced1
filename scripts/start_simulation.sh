#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec ros2 launch "$NAV_ROOT/launch/sim.launch.py" "gui:=${NAV_GUI:-false}"
