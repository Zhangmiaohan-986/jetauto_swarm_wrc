#!/usr/bin/env bash
set -euo pipefail
VM_WS="${VM_WS:-/home/ubuntu/jetauto_real_dev_ws}"
export ROS_MASTER_URI="http://${FLEET_MASTER_IP:-192.168.1.111}:11311"
export ROS_IP="${FLEET_VM_IP:-192.168.1.104}"
unset ROS_HOSTNAME
export MACHINE_TYPE=JetAuto
source "$(dirname "${BASH_SOURCE[0]}")/source_vm_ros.bash"
profile="${FLEET_PROFILE:-$(rospack find jetauto_multi)/config/fleet_vendor_omni/six_robot_profile.yaml}"
exec roslaunch jetauto_multi real.launch profile:="$profile" "$@"
