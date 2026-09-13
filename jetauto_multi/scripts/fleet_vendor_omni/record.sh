#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.1.111:11311}"
export ROS_IP="${ROS_IP:-192.168.1.104}"
unset ROS_HOSTNAME

record_dir="${FLEET_RECORD_DIR:-$HOME/fleet_vendor_omni_records}"
mkdir -p "$record_dir"
stamp=$(date '+%Y%m%d_%H%M%S')
output="$record_dir/fleet_vendor_omni_${FLEET_ROBOT_COUNT:-6}_$stamp"

topics=(
  /tf /tf_static "${FLEET_MAP_TOPIC:-/robot_1/map}"
  /fleet_vendor_omni/state_json /fleet_vendor_omni/event_json
  /fleet_vendor_omni_sim_adapter/truth_json
)
for ((robot_id=1; robot_id<=${FLEET_ROBOT_COUNT:-6}; robot_id++)); do
  robot="robot_${robot_id}"
  topics+=(
    "/${robot}/amcl_pose"
    "/${robot}/odom"
    "/${robot}/imu"
    "/${robot}/jetauto_controller/cmd_vel"
    "/${robot}/move_base/status"
    "/${robot}/move_base/result"
    "/${robot}/move_base/TebLocalPlannerROS/via_points"
  )
  # Ten raw scans plus local/global plans can starve the VM ROS graph while
  # recording. Keep them available for diagnosis on demand.
  if [[ "${FLEET_RECORD_COMPACT:-false}" != true ]]; then
    topics+=(
      "/${robot}/scan"
      "/${robot}/amcl_scan"
      "/${robot}/amcl_teammate_scan_filter/health_json"
    )
  fi
  if [[ "${FLEET_RECORD_BOTTOM_RAW:-true}" == true ]]; then
    topics+=(
      "/${robot}/particlecloud"
      "/${robot}/odom_raw"
      "/${robot}/ros_robot_controller/imu_raw"
      "/${robot}/ros_robot_controller/set_motor"
    )
  fi
  if [[ "${FLEET_RECORD_COMPACT:-false}" != true ||
        "${FLEET_RECORD_COSTMAPS:-false}" == true ]]; then
    topics+=(
      "/${robot}/move_base/GlobalPlanner/plan"
      "/${robot}/move_base/TebLocalPlannerROS/local_plan"
    )
  fi
  if [[ "${FLEET_RECORD_COSTMAPS:-false}" == true ]]; then
    topics+=("/${robot}/move_base/local_costmap/costmap"
             "/${robot}/move_base/local_costmap/costmap_updates"
             "/${robot}/move_base/goal")
  fi
done

echo "Recording ${FLEET_ROBOT_COUNT:-6}-robot vendor-omni evidence to ${output}.bag"
exec rosbag record --lz4 --buffsize=256 --chunksize=768 \
  -O "$output" "${topics[@]}" "$@"
