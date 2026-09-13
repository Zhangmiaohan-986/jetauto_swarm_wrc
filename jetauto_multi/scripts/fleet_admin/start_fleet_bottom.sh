#!/usr/bin/env bash
set -u

# Roster-driven JetAuto bottom-layer launcher for the vendor-omni mainline.
# Run this script on the VM (192.168.1.104).
#
# It will:
#   1) check the configured six physical IPs;
#   2) stop start_app_node.service on each robot;
#   3) ensure roscore is running on 192.168.1.111;
#   4) start jetauto_slam/jetauto_robot.launch on robot_1 ... robot_6;
#   5) keep each roslaunch in the robot background and write logs to ~/swarm_logs.
#
# NOTE: This script does NOT start the VM formation/mission layer and does NOT start vehicle motion.

# Use fleet-specific names so stale generic MASTER_IP/VM_IP variables from an
# earlier local simulation cannot silently redirect the real robot launcher.
MASTER_IP="${FLEET_MASTER_IP:-192.168.1.111}"
MASTER_ROBOT="${FLEET_MASTER_ROBOT:-robot_1}"
VM_IP="${FLEET_VM_IP:-192.168.1.104}"
SSH_USER="${SSH_USER:-jetauto}"
SSH_PASSWORD="${SWARM_SSH_PASSWORD:-hiwonder}"
ROBOT_WS="${ROBOT_WS:-/home/jetauto/jetauto_ws}"
LOG_DIR="${LOG_DIR:-/home/jetauto/swarm_logs}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_CONFIG="${FLEET_RUNTIME_CONFIG:-$SCRIPT_DIR/../../config/fleet_vendor_omni/six_robot_runtime.bash}"
# Delay between newly submitted robot bringups so USB drivers, ROS XMLRPC and
# the shared master are not hit by six bottom-layer startups at once.
START_STAGGER_SECONDS="${FLEET_START_STAGGER_SECONDS:-1.5}"
# In laboratory bringup, a powered-off spare must not prevent the remaining
# logical fleet from coming online.  robot_1 remains mandatory because it owns
# the ROS master.  The strict default is preserved for older deployments.
SKIP_UNREACHABLE="${FLEET_SKIP_UNREACHABLE:-false}"
# A clean restart is required after changing either IMU switch because an
# already-running complementary filter cannot inherit a new environment.
RESTART_EXISTING="${FLEET_RESTART_EXISTING:-false}"

SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o ConnectTimeout=5
  -o ServerAliveInterval=5
  -o ServerAliveCountMax=2
)

red=$'\033[31m'
green=$'\033[32m'
yellow=$'\033[33m'
bold=$'\033[1m'
reset=$'\033[0m'

die() {
  echo "${red}ERROR:${reset} $*" >&2
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

need_cmd sshpass
need_cmd ssh
need_cmd ping

[[ -f "$RUNTIME_CONFIG" ]] || die "Missing runtime table: $RUNTIME_CONFIG"
# shellcheck disable=SC1090
source "$RUNTIME_CONFIG"
[[ "${#FLEET_RUNTIME_ROWS[@]}" -gt 0 ]] || die "Runtime table is empty."

IPS=()
ROBOTS=()
FACTORY_HOSTS=()
FUSE_IMU_YAWS=()
IMU_BIASES=()
for idx in "${!FLEET_RUNTIME_ROWS[@]}"; do
  IFS='|' read -r robot ip factory_host fuse_imu_yaw imu_bias \
    max_vel_x max_vel_y max_vel_x_backwards max_vel_theta \
    acc_lim_x acc_lim_y acc_lim_theta amcl_min_d amcl_min_a \
    recovery_slow recovery_fast <<<"${FLEET_RUNTIME_ROWS[$idx]}"
  [[ "$robot" == "robot_$((idx + 1))" ]] || die "Runtime row $((idx + 1)) must describe robot_$((idx + 1))."
  [[ -n "$ip" && -n "$factory_host" ]] || die "Runtime row for $robot has an empty IP or factory hostname."
  case "$fuse_imu_yaw" in true|false) ;; *) die "$robot fuse_imu_yaw must be true or false." ;; esac
  case "$imu_bias" in true|false) ;; *) die "$robot dynamic IMU bias must be true or false." ;; esac
  for value in "$max_vel_x" "$max_vel_y" "$max_vel_x_backwards" \
      "$max_vel_theta" "$acc_lim_x" "$acc_lim_y" "$acc_lim_theta" \
      "$amcl_min_d" "$amcl_min_a" "$recovery_slow" "$recovery_fast"; do
    [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "$robot has invalid numeric value: $value"
  done
  IPS+=("$ip")
  ROBOTS+=("$robot")
  FACTORY_HOSTS+=("$factory_host")
  FUSE_IMU_YAWS+=("$fuse_imu_yaw")
  IMU_BIASES+=("$imu_bias")
done

MASTER_INDEX=-1
for idx in "${!ROBOTS[@]}"; do
  if [[ "${ROBOTS[$idx]}" == "$MASTER_ROBOT" && "${IPS[$idx]}" == "$MASTER_IP" ]]; then MASTER_INDEX="$idx"; fi
done
[[ "$MASTER_INDEX" -ge 0 ]] || die "Master IP/name do not match a runtime row."

case "$SKIP_UNREACHABLE" in
  true|false) ;;
  *) die "FLEET_SKIP_UNREACHABLE must be true or false (got: $SKIP_UNREACHABLE)." ;;
esac
case "$RESTART_EXISTING" in
  true|false) ;;
  *) die "FLEET_RESTART_EXISTING must be true or false (got: $RESTART_EXISTING)." ;;
esac
if [[ ! "$START_STAGGER_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  die "FLEET_START_STAGGER_SECONDS must be a non-negative number (got: $START_STAGGER_SECONDS)."
fi

remote() {
  local ip="$1"
  shift
  sshpass -p "$SSH_PASSWORD" ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "$@"
}

echo "${bold}=== ${#ROBOTS[@]} Robot Bottom-Layer Launcher ===${reset}"
echo "ROS Master : ${MASTER_IP}:11311"
echo "VM         : ${VM_IP}"
echo "Config     : ${RUNTIME_CONFIG}"
echo "Start gap  : ${START_STAGGER_SECONDS}s"
echo "Skip offline: ${SKIP_UNREACHABLE}"
echo "Restart old: ${RESTART_EXISTING}"
for idx in "${!ROBOTS[@]}"; do
  printf "  %-7s %-15s factory=%-7s yaw=%-5s bias=%s\n" \
    "${ROBOTS[$idx]}" "${IPS[$idx]}" "${FACTORY_HOSTS[$idx]}" \
    "${FUSE_IMU_YAWS[$idx]}" "${IMU_BIASES[$idx]}"
done
echo

echo "${bold}[1/5] Ping check${reset}"
ACTIVE_INDICES=()
ping_failed=0
for idx in "${!IPS[@]}"; do
  ip="${IPS[$idx]}"
  robot="${ROBOTS[$idx]}"
  if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
    echo "${green}PASS${reset} $robot $ip"
    ACTIVE_INDICES+=("$idx")
  else
    echo "${red}FAIL${reset} $robot $ip"
    ping_failed=1
    if [[ "$robot" == "$MASTER_ROBOT" || "$SKIP_UNREACHABLE" != "true" ]]; then
      continue
    fi
    echo "${yellow}SKIP${reset} $robot is offline; it will not be launched."
  fi
done
if [[ "$ping_failed" -ne 0 && "$SKIP_UNREACHABLE" != "true" ]]; then
  die "At least one robot is unreachable. Fix network before ROS startup."
fi
[[ " ${ACTIVE_INDICES[*]} " == *" ${MASTER_INDEX} "* ]] || die "${MASTER_ROBOT} (${MASTER_IP}) is unreachable."

echo
echo "${bold}[2/5] SSH check + stop autostart service${reset}"
SSH_READY_INDICES=()
for idx in "${ACTIVE_INDICES[@]}"; do
  ip="${IPS[$idx]}"
  robot="${ROBOTS[$idx]}"
  printf "%-8s %-15s " "$robot" "$ip"
  if remote "$ip" "echo '$SSH_PASSWORD' | sudo -S -p '' systemctl stop start_app_node.service >/dev/null 2>&1 && echo SSH_OK" 2>/dev/null | grep -q SSH_OK; then
    echo "${green}PASS${reset}"
    SSH_READY_INDICES+=("$idx")
  else
    echo "${red}FAIL${reset}"
    if [[ "$robot" == "$MASTER_ROBOT" || "$SKIP_UNREACHABLE" != "true" ]]; then
      die "SSH failed for $robot ($ip)."
    fi
    echo "${yellow}SKIP${reset} $robot SSH unavailable; it will not be launched."
  fi
done

echo
echo "${bold}[3/5] Ensure ROS Master on ${MASTER_ROBOT} (${MASTER_IP})${reset}"
if remote "$MASTER_IP" "bash -lc 'source /opt/ros/melodic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI=http://${MASTER_IP}:11311; export ROS_IP=${MASTER_IP}; unset ROS_HOSTNAME; rosnode list >/dev/null 2>&1'"; then
  echo "${green}PASS${reset} roscore already reachable."
else
  echo "${yellow}INFO${reset} roscore not reachable; starting it on ${MASTER_IP} ..."
  remote "$MASTER_IP" "mkdir -p '$LOG_DIR'; \
    nohup bash -lc 'source /opt/ros/melodic/setup.bash >/dev/null 2>&1 || true; \
    export ROS_MASTER_URI=http://${MASTER_IP}:11311; \
    export ROS_IP=${MASTER_IP}; unset ROS_HOSTNAME; \
    exec roscore' >'$LOG_DIR/roscore.log' 2>&1 </dev/null & echo \$! >'$LOG_DIR/roscore.pid'"
  ok=0
  for _ in $(seq 1 15); do
    sleep 1
    if remote "$MASTER_IP" "bash -lc 'source /opt/ros/melodic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI=http://${MASTER_IP}:11311; export ROS_IP=${MASTER_IP}; unset ROS_HOSTNAME; rosnode list >/dev/null 2>&1'"; then
      ok=1
      break
    fi
  done
  [[ "$ok" -eq 1 ]] || die "roscore failed to start. Check ${MASTER_IP}:${LOG_DIR}/roscore.log"
  echo "${green}PASS${reset} roscore started."
fi

echo
echo "${bold}[4/5] Start ${#ROBOTS[@]} robot bringups${reset}"
for idx in "${SSH_READY_INDICES[@]}"; do
  ip="${IPS[$idx]}"
  robot="${ROBOTS[$idx]}"
  fuse_imu_yaw="${FUSE_IMU_YAWS[$idx]}"
  imu_bias="${IMU_BIASES[$idx]}"
  log="$LOG_DIR/${robot}_bringup.log"
  pid="$LOG_DIR/${robot}_bringup.pid"

  echo "--- $robot @ $ip ---"

  # A changed environment only takes effect in a new bottom-layer process.
  if remote "$ip" "pgrep -af '[r]oslaunch.*jetauto_slam.*jetauto_robot.launch' >/dev/null 2>&1"; then
    if [[ "$RESTART_EXISTING" != "true" ]]; then
      echo "${yellow}SKIP${reset} existing jetauto_robot.launch found; runtime switches were not reapplied."
      continue
    fi
    echo "${yellow}INFO${reset} stopping existing bottom layer so the runtime table takes effect ..."
    if ! remote "$ip" "pkill -INT -f '[r]oslaunch.*jetauto_slam.*jetauto_robot.launch' 2>/dev/null || true; for _ in \$(seq 1 30); do pgrep -af '[r]oslaunch.*jetauto_slam.*jetauto_robot.launch' >/dev/null 2>&1 || exit 0; sleep 0.2; done; exit 1"; then
      die "$robot old bottom layer did not stop cleanly."
    fi
  fi

  remote "$ip" "mkdir -p '$LOG_DIR'; \
    nohup bash -lc '
      cd \"$ROBOT_WS\" || exit 20
      source /opt/ros/melodic/setup.bash >/dev/null 2>&1 || exit 21
      source devel/setup.bash >/dev/null 2>&1 || exit 22
      export ROS_MASTER_URI=http://${MASTER_IP}:11311
      export ROS_IP=${ip}
      unset ROS_HOSTNAME
      export ROBOT_HOST=${robot}
      export ROBOT_MASTER=${MASTER_ROBOT}
      export MACHINE_TYPE=JetAuto
      export LIDAR_TYPE=A1
      export DEPTH_CAMERA_TYPE=AstraProPlus
      export FLEET_IMU_BIAS_ESTIMATION=${imu_bias}
      exec roslaunch jetauto_slam jetauto_robot.launch sim:=false app:=false use_joy:=false set_pose:=false fuse_imu_yaw:=${fuse_imu_yaw}
    ' >'$log' 2>&1 </dev/null & echo \$! >'$pid'"

  echo "Waiting ${START_STAGGER_SECONDS}s before checking this startup and submitting the next robot ..."
  sleep "$START_STAGGER_SECONDS"
  if remote "$ip" "pgrep -af '[r]oslaunch.*jetauto_slam.*jetauto_robot.launch' >/dev/null 2>&1"; then
    echo "${green}PASS${reset} started; log=$log"
  else
    echo "${red}FAIL${reset} bringup exited; last log lines:"
    remote "$ip" "tail -30 '$log' 2>/dev/null || true"
    die "$robot bringup failed."
  fi
done

echo
echo "${bold}[5/5] Startup submitted${reset}"
echo "${green}DONE${reset} Bottom-layer launch commands are running on all reachable robots."
echo
echo "Next, on the VM run:"
echo "  bash check_fleet_bottom.sh"
echo
echo "Robot logs:"
echo "  /home/jetauto/swarm_logs/robot_N_bringup.log"
