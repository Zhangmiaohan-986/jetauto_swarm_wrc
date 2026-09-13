#!/usr/bin/env bash
set -e
source "$(dirname "${BASH_SOURCE[0]}")/ten_environment.bash" real
exec bash "$(dirname "${BASH_SOURCE[0]}")/check_fleet_bottom.sh" "$@"
