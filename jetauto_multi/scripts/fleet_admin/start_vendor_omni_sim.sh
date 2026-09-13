#!/usr/bin/env bash
set -euo pipefail
VM_WS="${VM_WS:-/home/ubuntu/jetauto_real_dev_ws}"
# Simulation is local to this VM, separate from every physical robot master.
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export MACHINE_TYPE=JetAuto
source "$(dirname "${BASH_SOURCE[0]}")/source_vm_ros.bash"
profile="${FLEET_PROFILE:-$(rospack find jetauto_multi)/config/fleet_vendor_omni/six_robot_profile.yaml}"
exec roslaunch -p 11321 jetauto_multi sim.launch profile:="$profile" "$@"
