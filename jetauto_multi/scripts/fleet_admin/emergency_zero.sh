#!/usr/bin/env bash
set -u

# Emergency software stop while the shared ROS master is reachable.  It first
# asks the active fleet supervisor to stop, then sends zero to every chassis
# command topic for three seconds.  This script never publishes non-zero data.

MASTER_IP="${FLEET_MASTER_IP:-192.168.1.111}"
VM_IP="${FLEET_VM_IP:-192.168.1.104}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${MASTER_IP}:11311}"
export ROS_IP="${ROS_IP:-${VM_IP}}"
unset ROS_HOSTNAME
source /opt/ros/melodic/setup.bash >/dev/null 2>&1 || true
source "$HOME/jetauto_real_dev_ws/devel/setup.bash" >/dev/null 2>&1 || true

if ! rosnode list >/dev/null 2>&1; then
  echo "ERROR: ROS Master is unreachable; use the physical emergency stop instead."
  exit 2
fi

rosservice call /fleet_vendor_omni/stop >/dev/null 2>&1 || true
rosservice call /mission/stop >/dev/null 2>&1 || true

pids=()
for ((id=1; id<=${FLEET_ROBOT_COUNT:-6}; id++)); do
  topic="/robot_${id}/jetauto_controller/cmd_vel"
  timeout 3 rostopic pub -r 20 "$topic" geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 &
  pids+=("$!")
done

for pid in "${pids[@]}"; do
  wait "$pid" || true
done

echo "Emergency zero was sent to ${FLEET_ROBOT_COUNT:-6} chassis topics for three seconds."
