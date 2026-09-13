#!/usr/bin/env bash
set -e
mode="${1:-sim}"
[[ $# -gt 0 ]] && shift
source "$(dirname "${BASH_SOURCE[0]}")/ten_environment.bash" "$mode"
if [[ "$mode" == sim ]]; then
  exec bash "$(dirname "${BASH_SOURCE[0]}")/start_vendor_omni_sim.sh" "$@"
else
  exec bash "$(dirname "${BASH_SOURCE[0]}")/start_vendor_omni.sh" "$@"
fi
