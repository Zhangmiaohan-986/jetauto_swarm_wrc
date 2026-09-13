#!/usr/bin/env bash
# One shared implementation; selecting all never includes unlisted hosts.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-}"
case "$action" in start|stop|restart|app-stop|status|logs) ;; *)
  echo 'Usage: manage_ten_bottom.sh start|stop|restart|app-stop|status|logs all|1 2 ... 10' >&2
  exit 2 ;;
esac
shift
[[ $# -gt 0 ]] || { echo 'Specify all or explicit robot numbers.' >&2; exit 2; }
if [[ "$1" == all ]]; then
  [[ $# -eq 1 ]] || exit 2
  set -- {1..10}
fi
for robot in "$@"; do
  [[ "$robot" =~ ^(robot_)?([1-9]|10)$ ]] || { echo "Invalid robot: $robot" >&2; exit 2; }
done
source "$script_dir/ten_environment.bash" real
if [[ "$action" == start || "$action" == restart ]]; then
  bash "$script_dir/start_ten_master.sh" --check
fi
# Concurrent submission. The selected master must already be running.
# Ctrl+C of this dispatcher is NOT a remote stop: use the explicit stop action.
pids=()
names=()
for robot in "$@"; do
  bash "$script_dir/manage_robot.sh" "$robot" "$action" &
  pids+=("$!")
  names+=("$robot")
done
result=0
for i in "${!pids[@]}"; do
  if wait "${pids[$i]}"; then
    echo "OK ${names[$i]} $action"
  else
    echo "FAIL ${names[$i]} $action" >&2
    result=1
  fi
done
exit "$result"
