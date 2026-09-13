#!/usr/bin/env bash
# Catkin/Melodic setup scripts inspect unset variables on their first load.
# Preserve the caller's nounset setting instead of weakening the whole script.
_fleet_source_vm_ros() {
  local restore_nounset=false result=0
  [[ $- == *u* ]] && restore_nounset=true
  set +u
  source /opt/ros/melodic/setup.bash || result=$?
  if [[ "$result" -eq 0 ]]; then
    source "${VM_WS:-/home/ubuntu/jetauto_real_dev_ws}/devel/setup.bash" || result=$?
  fi
  if [[ "$restore_nounset" == true ]]; then set -u; fi
  return "$result"
}
_fleet_source_vm_ros || { unset -f _fleet_source_vm_ros; return 1; }
unset -f _fleet_source_vm_ros
