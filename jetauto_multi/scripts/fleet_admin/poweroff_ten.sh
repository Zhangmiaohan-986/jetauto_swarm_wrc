#!/usr/bin/env bash
# No ROS dependency: .111 is deliberately shut down first.
set -euo pipefail
[[ $# -eq 0 ]] || { echo 'Usage: bash poweroff_ten.sh' >&2; exit 2; }
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../../config/fleet_vendor_omni/profiles/fleet10_same_map/runtime.bash"
[[ ${#FLEET_RUNTIME_ROWS[@]} -eq 10 ]] || exit 2
# Validate the whole roster before any shutdown.
for n in {1..10}; do
  IFS='|' read -r robot ip rest <<< "${FLEET_RUNTIME_ROWS[$((n-1))]}"
  [[ "$robot" == "robot_$n" && "$ip" == "192.168.1.$((110+n))" ]] || {
    echo "ERROR: unexpected roster row $n; no shutdown performed." >&2; exit 2;
  }
done
for cmd in sshpass ssh timeout; do command -v "$cmd" >/dev/null; done
export SSHPASS="${SWARM_SSH_PASSWORD:-hiwonder}"
ssh_options=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5
  -o ServerAliveInterval=5 -o ServerAliveCountMax=2)
payload=$(cat <<'REMOTE'
set -eu
echo 'SYNC: flushing disk'
sync
pattern='^(/[^ ]*/)?(python[23]? +)?(/[^ ]*/)?roslaunch +(jetauto_slam +jetauto_robot[.]launch|jetauto_bringup +bringup[.]launch|/home/jetauto/swarm_logs/robot_[1-9][0-9]*_transport[.]launch)( |$)'
if pgrep -af "$pattern"; then
  echo 'BOTTOM: running; preparing graceful stop'
else
  echo 'BOTTOM: not running'
fi
# Prevent vendor APP from respawning its bottom layer.
timeout 25 systemctl stop start_app_node.service
pkill -INT -f "$pattern" 2>/dev/null || true
for ((i=0; i<100; i++)); do
  if ! pgrep -f "$pattern" >/dev/null; then break; fi
  sleep 0.2
done
if pgrep -af "$pattern"; then
  echo 'ERROR: bottom did not stop; skipping poweroff of this robot.' >&2
  exit 8
fi
sync
echo 'POWEROFF: requesting system shutdown'
systemctl poweroff
REMOTE
)
printf -v remote_command 'sudo -S -p "" bash -c %q' "$payload"
result=0
echo 'WARNING: stop the VM motion task before using this script.'
for n in {1..10}; do
  IFS='|' read -r robot ip rest <<< "${FLEET_RUNTIME_ROWS[$((n-1))]}"
  echo "=== $robot $ip ==="
  if ! timeout 12 sshpass -e ssh "${ssh_options[@]}" "${SSH_USER:-jetauto}@$ip" true; then
    echo "SKIP: $robot SSH unavailable; continuing."
    result=1
    continue
  fi
  # Password goes only to sudo's stdin, never into the remote command line.
  if printf '%s\n' "$SSHPASS" | timeout 75 sshpass -e ssh "${ssh_options[@]}" \
      "${SSH_USER:-jetauto}@$ip" "$remote_command"; then
    echo "SENT: $robot shutdown requested (not proof of physical power-off)."
  else
    echo "CHECK: $robot command failed or SSH disconnected; inspect its output. Continuing." >&2
    result=1
  fi
done
echo 'Finished .111 through .120. Check SKIP/CHECK entries; then switch off physical power.'
exit "$result"
