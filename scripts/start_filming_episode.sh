#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/env.sh"
session="${1:-}"
if [[ ! "$session" =~ ^capture_control_[0-9a-f]{32}$ ]]; then
  echo 'Invalid capture session' >&2
  exit 2
fi
scenario="${2:-standard}"
exec python3 -m pai_ai_navigation.episode --mode ai-shield --scenario "$scenario" --timeout 80 --gui --video-view --manual-control-dir "$NAV_ROOT/runtime/$session" --output "$NAV_ROOT/runtime/$session/episode"