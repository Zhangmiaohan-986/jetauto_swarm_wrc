#!/usr/bin/env bash
# Usage: source .../ten_environment.bash sim|real
_fleet_mode="${1:-real}"
case "$_fleet_mode" in sim|real) ;; *) echo 'Use sim or real' >&2; return 2 ;; esac
export VM_WS="${VM_WS:-/home/ubuntu/jetauto_real_dev_ws}"
source "$(dirname "${BASH_SOURCE[0]}")/source_vm_ros.bash" || return $?
_fleet_package="$(rospack find jetauto_multi)"
export FLEET_PROFILE="$_fleet_package/config/fleet_vendor_omni/profiles/fleet10_same_map/profile.yaml"
export FLEET_RUNTIME_CONFIG="$_fleet_package/config/fleet_vendor_omni/profiles/fleet10_same_map/runtime.bash"
# Network defaults and the robot roster have the same single source.
# Explicit FLEET_MASTER_IP overrides remain supported; remove a stale old
# export (unset FLEET_MASTER_IP) before sourcing to adopt a changed default.
source "$FLEET_RUNTIME_CONFIG" || return $?
export FLEET_ROBOT_COUNT=10
export MACHINE_TYPE=JetAuto
unset ROS_HOSTNAME
if [[ "$_fleet_mode" == sim ]]; then
  export ROS_MASTER_URI=http://127.0.0.1:11321 ROS_IP=127.0.0.1
else
  export ROS_MASTER_URI="http://${FLEET_MASTER_IP}:11311" ROS_IP="$FLEET_VM_IP"
fi
fleetmaster() {
  bash "$(rospack find jetauto_multi)/scripts/fleet_admin/start_ten_master.sh" "$@"
}
unset _fleet_mode _fleet_package
