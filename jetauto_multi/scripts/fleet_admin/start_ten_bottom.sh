#!/usr/bin/env bash
set -e
[[ $# -eq 0 ]] || { echo 'Usage: start_ten_bottom.sh' >&2; exit 2; }
case "${FLEET_RESTART_EXISTING:-false}" in
  false) action=start ;;
  true) action=restart ;;
  *) echo 'FLEET_RESTART_EXISTING must be true or false.' >&2; exit 2 ;;
esac
# Unlike the six-car legacy starter, this never auto-starts a master on a car.
exec bash "$(dirname "${BASH_SOURCE[0]}")/manage_ten_bottom.sh" "$action" all
