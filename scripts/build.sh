#!/usr/bin/env bash
# Build tools live inside this project; do not upgrade the shared arm venv.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$NAV_ROOT"
if [[ ! -f .build_tools/bin/colcon ]]; then
  python3 -m pip install --target "$NAV_ROOT/.build_tools" colcon-core==0.21.3 colcon-ros==0.5.0 colcon-python-setup-py==0.2.9
fi
PYTHONPATH="$NAV_ROOT/.build_tools:$PYTHONPATH" python3 "$NAV_ROOT/.build_tools/bin/colcon" \
  --log-base .ros_log build --paths . --build-base .ros_build --install-base .ros_install
source "$NAV_ROOT/.ros_install/local_setup.sh"
ros2 pkg executables pai_ai_navigation