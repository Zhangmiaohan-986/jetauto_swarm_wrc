#!/usr/bin/env bash
# Compatibility entry; implementation uses the selected runtime roster.
exec bash "$(dirname "${BASH_SOURCE[0]}")/start_fleet_bottom.sh" "$@"
