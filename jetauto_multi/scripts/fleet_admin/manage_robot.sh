#!/usr/bin/env bash
set -euo pipefail

# Manage one physical JetAuto bottom-layer bringup from the VM.
#
# Usage:
#   manage_robot.sh robot_6 restart
#   manage_robot.sh 6 status
#
# This script never stops roscore.  In particular, stopping robot_1 only stops
# robot_1's jetauto_robot.launch; the ROS Master on 192.168.1.111 is preserved.

usage() {
  cat <<'EOF'
Usage: manage_robot.sh <robot_N|N> <start|stop|restart|app-stop|status|logs>

Examples:
  manage_robot.sh robot_6 restart
  manage_robot.sh 4 logs

Environment overrides:
  FLEET_MASTER_IP       default: 192.168.1.111
  FLEET_RUNTIME_CONFIG  default: shared six_robot_runtime.bash table
  FLEET_FUSE_IMU_YAW    optional one-run override of the selected row
  FLEET_IMU_BIAS_ESTIMATION optional one-run override of the selected row
  FLEET_WIFI_POWER_SAVE on|off|unchanged; six default unchanged, ten runtime off
  FLEET_LOCAL_TF        true|false; isolate each robot's TF transport only
  SSH_USER              default: jetauto
  SWARM_SSH_PASSWORD    default: hiwonder
  ROBOT_WS              default: /home/jetauto/jetauto_ws
EOF
}

[[ $# -eq 2 ]] || { usage >&2; exit 2; }

robot_arg="$1"
action="$2"
robot_arg="${robot_arg#robot_}"

[[ "$robot_arg" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: invalid robot '$1'." >&2; exit 2; }

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runtime_config="${FLEET_RUNTIME_CONFIG:-$script_dir/../../config/fleet_vendor_omni/six_robot_runtime.bash}"
[[ -f "$runtime_config" ]] || { echo "ERROR: missing runtime table $runtime_config" >&2; exit 2; }
# shellcheck disable=SC1090
source "$runtime_config"
[[ "$robot_arg" -le "${#FLEET_RUNTIME_ROWS[@]}" ]] || { echo "ERROR: robot is not in runtime roster." >&2; exit 2; }
IFS='|' read -r robot ip factory_host profile_fuse_imu_yaw profile_imu_bias \
  _max_vel_x _max_vel_y _max_vel_x_backwards _max_vel_theta \
  _acc_lim_x _acc_lim_y _acc_lim_theta _amcl_min_d _amcl_min_a \
  _recovery_slow _recovery_fast <<<"${FLEET_RUNTIME_ROWS[$((robot_arg - 1))]}"
[[ "$robot" == "robot_${robot_arg}" ]] || { echo "ERROR: runtime row order is invalid." >&2; exit 2; }

case "$action" in
  start|stop|restart|app-stop|status|logs) ;;
  *) echo "ERROR: unknown action '$action'." >&2; usage >&2; exit 2 ;;
esac

master_ip="${FLEET_MASTER_IP:-192.168.1.111}"
ssh_user="${SSH_USER:-jetauto}"
ssh_password="${SWARM_SSH_PASSWORD:-hiwonder}"
robot_ws="${ROBOT_WS:-/home/jetauto/jetauto_ws}"
log_dir="${LOG_DIR:-/home/jetauto/swarm_logs}"
fuse_imu_yaw="${FLEET_FUSE_IMU_YAW:-$profile_fuse_imu_yaw}"
imu_bias_estimation="${FLEET_IMU_BIAS_ESTIMATION:-$profile_imu_bias}"
wifi_power_save="${FLEET_WIFI_POWER_SAVE:-unchanged}"
local_tf="${FLEET_LOCAL_TF:-false}"
[[ "$log_dir" =~ ^/[a-zA-Z0-9_./-]+$ ]] || { echo 'ERROR: unsupported LOG_DIR.' >&2; exit 2; }
transport_file="$log_dir/${robot}_transport.launch"
launch_prefix='^(/[^ ]*/)?(python[23]? +)?(/[^ ]*/)?roslaunch +'
legacy_pattern="${launch_prefix}jetauto_slam +jetauto_robot[.]launch( |$)"
transport_pattern="${launch_prefix}${transport_file//./[.]}( |$)"
launch_pattern="${launch_prefix}(jetauto_slam +jetauto_robot[.]launch|${transport_file//./[.]})( |$)"

case "$fuse_imu_yaw" in
  true|false) ;;
  *) echo "ERROR: FLEET_FUSE_IMU_YAW must be true or false." >&2; exit 2 ;;
esac

case "$imu_bias_estimation" in
  true|false) ;;
  *) echo "ERROR: FLEET_IMU_BIAS_ESTIMATION must be true or false." >&2; exit 2 ;;
esac

case "$wifi_power_save" in
  on|off|unchanged) ;;
  *) echo 'ERROR: FLEET_WIFI_POWER_SAVE must be on, off or unchanged.' >&2; exit 2 ;;
esac
case "$local_tf" in true|false) ;; *) echo 'ERROR: FLEET_LOCAL_TF must be true or false.' >&2; exit 2 ;; esac

for command in sshpass ssh ping; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "ERROR: missing command '$command'." >&2
    exit 3
  }
done

ssh_options=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o ConnectTimeout=5
  -o ServerAliveInterval=5
  -o ServerAliveCountMax=2
)

remote() {
  sshpass -p "$ssh_password" ssh "${ssh_options[@]}" \
    "${ssh_user}@${ip}" "$@"
}

require_robot() {
  if ! ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
    echo "ERROR: $robot ($ip) is unreachable." >&2
    exit 4
  fi
  if ! remote "true" >/dev/null 2>&1; then
    echo "ERROR: SSH failed for $robot ($ip)." >&2
    exit 5
  fi
}

stop_robot() {
  echo "Stopping $robot bottom layer on $ip ..."
  remote "bash -s" <<REMOTE
set -eu
echo '${ssh_password}' | sudo -S -p '' systemctl stop start_app_node.service
pkill -INT -f '${launch_pattern}' 2>/dev/null || true
for _ in \$(seq 1 100); do
  pgrep -af '${launch_pattern}' >/dev/null 2>&1 || exit 0
  sleep 0.2
done
echo 'ERROR: roslaunch did not stop within 20 seconds; inspect before restarting.' >&2
exit 8
REMOTE
  echo "DONE: $robot bottom layer stopped; ROS Master was not touched."
}

start_robot() {
  remote "bash -lc 'source /opt/ros/melodic/setup.bash; export ROS_MASTER_URI=http://${master_ip}:11311 ROS_IP=${ip}; unset ROS_HOSTNAME; timeout 5 rosnode list >/dev/null'" || {
    echo "ERROR: ROS Master $master_ip is unavailable; start roscore on the master first." >&2
    return 7
  }
  if [[ "$wifi_power_save" != unchanged ]]; then
    remote "echo '$ssh_password' | sudo -S -p '' iw dev wlan0 set power_save '$wifi_power_save' && iw dev wlan0 get power_save" || {
      echo "ERROR: $robot Wi-Fi power-save setting failed; inspect wlan0 before retrying." >&2
      return 10
    }
  fi
  if remote "pgrep -af '$launch_pattern' >/dev/null 2>&1"; then
    opposite_pattern="$transport_pattern"
    [[ "$local_tf" == true ]] && opposite_pattern="$legacy_pattern"
    if remote "pgrep -af '$opposite_pattern' >/dev/null 2>&1"; then
      echo "ERROR: $robot is running the old TF transport; use restart to apply FLEET_LOCAL_TF=$local_tf." >&2
      return 6
    fi
    if ! remote "for node_pid in \$(pgrep -f '$launch_pattern'); do
      grep -z -Fxq 'ROS_MASTER_URI=http://${master_ip}:11311' /proc/\$node_pid/environ || exit 6
    done"; then
      echo "ERROR: $robot is already running with a different ROS Master; use restart to apply $master_ip." >&2
      return 6
    fi
    echo "SKIP: $robot bottom layer is already running on $ip."
    return 0
  fi

  launch_command="roslaunch jetauto_slam jetauto_robot.launch sim:=false app:=false use_joy:=false set_pose:=false fuse_imu_yaw:=${fuse_imu_yaw}"
  if [[ "$local_tf" == true ]]; then
    command -v scp >/dev/null || { echo 'ERROR: scp is required for local TF transport.' >&2; return 3; }
    wrapper="$script_dir/../../launch/fleet_vendor_omni/include/robot_bottom_transport.launch"
    [[ -f "$wrapper" ]] || { echo "ERROR: missing transport wrapper $wrapper" >&2; return 3; }
    remote "set -e; mkdir -p '$log_dir'; if [ -f '$transport_file' ] && [ ! -e '${transport_file}.before_fleet${robot_arg}' ]; then cp -p '$transport_file' '${transport_file}.before_fleet${robot_arg}'; fi"
    sshpass -p "$ssh_password" scp "${ssh_options[@]}" "$wrapper" "${ssh_user}@${ip}:${transport_file}"
    launch_command="roslaunch $transport_file robot_name:=$robot fuse_imu_yaw:=$fuse_imu_yaw"
  fi
  echo "Starting $robot bottom layer on $ip (factory=$factory_host yaw=$fuse_imu_yaw bias=$imu_bias_estimation) ..."
  remote "mkdir -p '$log_dir'; \
    echo '$ssh_password' | sudo -S -p '' systemctl stop start_app_node.service || exit 8; \
    nohup bash -lc '
      cd \"$robot_ws\" || exit 20
      source /opt/ros/melodic/setup.bash >/dev/null 2>&1 || exit 21
      source devel/setup.bash >/dev/null 2>&1 || exit 22
      export ROS_MASTER_URI=http://${master_ip}:11311
      export ROS_IP=${ip}
      unset ROS_HOSTNAME
      export ROBOT_HOST=${robot}
      export ROBOT_MASTER=${FLEET_MASTER_ROBOT:-robot_1}
      export MACHINE_TYPE=JetAuto
      export LIDAR_TYPE=A1
      export DEPTH_CAMERA_TYPE=AstraProPlus
      export FLEET_IMU_BIAS_ESTIMATION=${imu_bias_estimation}
      export FLEET_LOCAL_TF=${local_tf}
      exec ${launch_command}
    ' >'$log_dir/${robot}_bringup.log' 2>&1 </dev/null & \
    echo \$! >'$log_dir/${robot}_bringup.pid'"

  sleep 3
  if remote "pgrep -af '$launch_pattern' >/dev/null 2>&1"; then
    echo "DONE: $robot bottom layer started; log=$log_dir/${robot}_bringup.log"
  else
    echo "ERROR: $robot bringup exited. Last log lines:" >&2
    remote "tail -60 '$log_dir/${robot}_bringup.log' 2>/dev/null || true" >&2
    exit 9
  fi
}

status_robot() {
  echo "Robot : $robot"
  echo "IP    : $ip"
  echo "Factory: $factory_host"
  echo "Master: $master_ip"
  echo "Expected fuse_imu_yaw=$fuse_imu_yaw dynamic_bias=$imu_bias_estimation"
  echo "Expected local TF transport=$local_tf"
  if remote "pgrep -af '$launch_pattern'"; then
    echo "STATUS: RUNNING"
  else
    echo "STATUS: STOPPED"
    return 1
  fi
}

logs_robot() {
  remote "tail -100 '$log_dir/${robot}_bringup.log' 2>/dev/null || { echo 'No bringup log found.'; exit 1; }"
}

require_robot

case "$action" in
  start) start_robot ;;
  stop) stop_robot ;;
  restart) stop_robot; sleep 1; start_robot ;;
  app-stop) remote "echo '$ssh_password' | sudo -S -p '' systemctl stop start_app_node.service" ;;
  status) status_robot ;;
  logs) logs_robot ;;
esac
