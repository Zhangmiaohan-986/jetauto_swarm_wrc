#!/usr/bin/env bash
set -u

# Read-only roster-driven bottom-layer checker.
# Run on the VM after start_six_bottom.sh.
# It does NOT publish motion commands.

SSH_USER="${SSH_USER:-jetauto}"
SSH_PASSWORD="${SWARM_SSH_PASSWORD:-hiwonder}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_CONFIG="${FLEET_RUNTIME_CONFIG:-$SCRIPT_DIR/../../config/fleet_vendor_omni/six_robot_runtime.bash}"

SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o ConnectTimeout=4
)

red=$'\033[31m'
green=$'\033[32m'
yellow=$'\033[33m'
bold=$'\033[1m'
reset=$'\033[0m'

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

pass() { echo "${green}PASS${reset} $*"; PASS_COUNT=$((PASS_COUNT+1)); }
fail() { echo "${red}FAIL${reset} $*"; FAIL_COUNT=$((FAIL_COUNT+1)); }
warn() { echo "${yellow}WARN${reset} $*"; WARN_COUNT=$((WARN_COUNT+1)); }

if [[ ! -f "$RUNTIME_CONFIG" ]]; then
  echo "ERROR: missing runtime table $RUNTIME_CONFIG" >&2
  exit 2
fi
# shellcheck disable=SC1090
source "$RUNTIME_CONFIG"
MASTER_IP="${FLEET_MASTER_IP:-192.168.1.111}"
VM_IP="${FLEET_VM_IP:-192.168.1.104}"
if [[ "${#FLEET_RUNTIME_ROWS[@]}" -lt 1 ]]; then
  echo "ERROR: runtime table is empty." >&2
  exit 2
fi
IPS=()
ROBOTS=()
FACTORY_HOSTS=()
FUSE_IMU_YAWS=()
IMU_BIASES=()
for idx in "${!FLEET_RUNTIME_ROWS[@]}"; do
  IFS='|' read -r robot ip factory_host fuse_imu_yaw imu_bias \
    _max_vel_x _max_vel_y _max_vel_x_backwards _max_vel_theta \
    _acc_lim_x _acc_lim_y _acc_lim_theta _amcl_min_d _amcl_min_a \
    _recovery_slow _recovery_fast <<<"${FLEET_RUNTIME_ROWS[$idx]}"
  IPS+=("$ip")
  ROBOTS+=("$robot")
  FACTORY_HOSTS+=("$factory_host")
  FUSE_IMU_YAWS+=("$fuse_imu_yaw")
  IMU_BIASES+=("$imu_bias")
done

export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="${VM_IP}"
unset ROS_HOSTNAME

source "$SCRIPT_DIR/source_vm_ros.bash" || exit $?

remote() {
  local ip="$1"
  shift
  sshpass -p "$SSH_PASSWORD" ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "$@"
}

node_exists() {
  local node="$1"
  printf '%s\n' "$NODE_LIST" | grep -Fxq "$node"
}

echo "${bold}=== ${#ROBOTS[@]} Robot Bottom-Layer Health Check ===${reset}"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "Runtime table=$RUNTIME_CONFIG"
echo

echo "${bold}[MASTER]${reset}"
if NODE_LIST="$(timeout 6 rosnode list 2>/dev/null)"; then
  pass "ROS Master reachable at ${MASTER_IP}:11311"
else
  fail "ROS Master unreachable at ${MASTER_IP}:11311"
  echo
  echo "Stop here: fix roscore/network first."
  exit 2
fi

# Sample all twelve streams in parallel and warm one persistent TF buffer.
# Repeated short-lived rostopic/tf_echo processes caused intermittent false
# failures on an otherwise healthy six-robot network.
PROBE_FILE="$(mktemp /tmp/six_robot_health.XXXXXX)"
trap 'rm -f "$PROBE_FILE"' EXIT
if ! timeout 25 python "$SCRIPT_DIR/probe_fleet_runtime_health.py" "${ROBOTS[@]}" >"$PROBE_FILE" 2>/dev/null; then
  fail "shared ROS runtime probe failed"
fi
declare -A SCAN_RATE ODOM_RATE TF_STATE
while IFS='|' read -r robot scan_rate odom_rate tf_state; do
  [[ "$robot" == robot_* ]] || continue
  SCAN_RATE["$robot"]="$scan_rate"
  ODOM_RATE["$robot"]="$odom_rate"
  TF_STATE["$robot"]="$tf_state"
done <"$PROBE_FILE"

# Bringup may still be registering when the first Master reachability query
# runs. Use one fresh graph snapshot after the shared sensor warmup.
if ! NODE_LIST="$(timeout 6 rosnode list 2>/dev/null)"; then
  fail "ROS graph refresh failed after sensor warmup"
  exit 2
fi

echo
for idx in "${!IPS[@]}"; do
  ip="${IPS[$idx]}"
  robot="${ROBOTS[$idx]}"
  factory_host="${FACTORY_HOSTS[$idx]}"
  expected_fuse="${FUSE_IMU_YAWS[$idx]}"
  expected_bias="${IMU_BIASES[$idx]}"
  ns="/${robot}"

  echo "${bold}===== ${robot} @ ${ip} (factory ${factory_host}) =====${reset}"

  if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
    pass "ping"
  else
    fail "ping"
  fi

  if remote "$ip" "true" >/dev/null 2>&1; then
    pass "SSH"

    if remote "$ip" "test -L /dev/rrc && test -c /dev/rrc && test -L /dev/lidar && test -c /dev/lidar" >/dev/null 2>&1; then
      pass "controller/lidar serial aliases"
    else
      fail "missing or invalid /dev/rrc or /dev/lidar"
    fi

    recent_usb_events="$(remote "$ip" "journalctl -k --since '-15 minutes' --no-pager 2>/dev/null | grep -E 'USB disconnect|ttyACM.*disconnected|ttyUSB.*disconnected' | tail -n 6" 2>/dev/null || true)"
    if [[ -n "$recent_usb_events" ]]; then
      fail "recent USB serial disconnect detected; inspect/reboot before motion"
      printf '  %s\n' "$recent_usb_events"
    else
      pass "no USB serial disconnect in last 15 minutes"
    fi
  else
    fail "SSH"
  fi

  # Critical bottom-layer nodes only.
  critical_nodes=(
    "${ns}/rplidarNode"
    "${ns}/ros_robot_controller"
    "${ns}/jetauto_odom_publisher"
    "${ns}/imu_filter"
    "${ns}/ekf_localization"
    "${ns}/robot_state_publisher"
  )

  for node in "${critical_nodes[@]}"; do
    if node_exists "$node"; then
      pass "node $node"
    else
      fail "node $node missing"
    fi
  done

  actual_bias="$(rosparam get "${ns}/imu_filter/do_bias_estimation" 2>/dev/null || true)"
  # Melodic has no `rosnode get-uri`; list -a prints URI then node name.
  controller_uri="$(timeout 6 rosnode list -a "${ns}/ros_robot_controller" 2>/dev/null |
    awk -v node="${ns}/ros_robot_controller" '$2 == node {print $1}')"
  if [[ -z "$controller_uri" ]]; then
    fail "controller URI lookup unavailable: ${ns}/ros_robot_controller"
  elif [[ "$controller_uri" == "http://${ip}:"* ]]; then
    pass "logical namespace belongs to expected IP $ip"
  else
    fail "namespace/IP mismatch: ${ns} controller URI=$controller_uri expected=$ip"
  fi
  if [[ "$actual_bias" == "$expected_bias" ]]; then
    pass "dynamic IMU bias=$actual_bias"
  elif [[ -z "$actual_bias" ]]; then
    fail "dynamic IMU bias parameter missing"
  else
    fail "dynamic IMU bias=$actual_bias expected $expected_bias; clean restart required"
  fi

  imu_values=()
  while read -r value; do
    [[ -n "$value" ]] && imu_values+=("$value")
  done < <(rosparam get "${ns}/ekf_localization/imu0_config" 2>/dev/null | awk '/^- / {print $2}')
  expected_yaw="$expected_fuse"
  if [[ "${#imu_values[@]}" -ge 12 && "${imu_values[5]}" == "$expected_yaw" && "${imu_values[11]}" == "true" ]]; then
    pass "EKF IMU contract yaw=${imu_values[5]} yaw_rate=${imu_values[11]}"
  else
    fail "EKF IMU contract mismatch; expected yaw=$expected_yaw yaw_rate=true"
  fi

  rate="${SCAN_RATE[$robot]:-NA}"
  if [[ "$rate" != "NA" ]]; then
    # Some healthy A1 units run just below 8 Hz.  Fresh data at 5-8 Hz is a
    # warning; a missing stream or sustained rate below 5 Hz blocks motion.
    if awk -v r="$rate" 'BEGIN {exit !(r >= 8.0)}'; then
      pass "scan_raw ${rate} Hz"
    elif awk -v r="$rate" 'BEGIN {exit !(r >= 5.0)}'; then
      warn "scan_raw below nominal but usable: ${rate} Hz"
    else
      fail "scan_raw too slow: ${rate} Hz"
    fi
  else
    fail "scan_raw no messages"
  fi

  rate="${ODOM_RATE[$robot]:-NA}"
  if [[ "$rate" != "NA" ]]; then
    if awk -v r="$rate" 'BEGIN {exit !(r >= 5.0)}'; then
      pass "odom ${rate} Hz"
    else
      fail "odom too slow: ${rate} Hz"
    fi
  else
    fail "odom no messages"
  fi

  if [[ "${TF_STATE[$robot]:-FAIL}" == "PASS" ]]; then
    pass "TF ${robot}/odom -> ${robot}/base_footprint"
  else
    fail "TF ${robot}/odom -> ${robot}/base_footprint"
  fi

  cmd_topic="${ns}/jetauto_controller/cmd_vel"
  info="$(rostopic info "$cmd_topic" 2>/dev/null || true)"
  if [[ -n "$info" ]]; then
    pass "topic $cmd_topic exists"
    if printf '%s\n' "$info" | grep -q 'Subscribers:'; then
      pass "cmd_vel control chain has subscriber metadata"
    else
      warn "could not confirm cmd_vel subscriber"
    fi
  else
    fail "topic $cmd_topic missing"
  fi

  # Remote process check provides the most useful log path if ROS node checks fail.
  transport_file="${LOG_DIR:-/home/jetauto/swarm_logs}/${robot}_transport.launch"
  bottom_pattern="^(/[^ ]*/)?(python[23]? +)?(/[^ ]*/)?roslaunch +(jetauto_slam +jetauto_robot[.]launch|${transport_file//./[.]})( |$)"
  if remote "$ip" "pgrep -af '$bottom_pattern'" >/dev/null 2>&1; then
    pass "remote jetauto_robot.launch alive"
  else
    fail "remote jetauto_robot.launch not running"
    echo "  Check: ssh ${SSH_USER}@${ip} 'tail -80 ~/swarm_logs/${robot}_bringup.log'"
  fi

  echo
done

echo "${bold}=== RESULT ===${reset}"
echo "PASS=${PASS_COUNT} WARN=${WARN_COUNT} FAIL=${FAIL_COUNT}"

if [[ "$FAIL_COUNT" -eq 0 ]]; then
  echo "${green}READY:${reset} ${#ROBOTS[@]} bottom layers passed the critical checks."
  echo "Next step: start the intended VM upper layer with motion disabled."
  exit 0
else
  echo "${red}NOT READY:${reset} do not start motion_enabled:=true."
  echo "Fix the FAIL items first."
  exit 1
fi
