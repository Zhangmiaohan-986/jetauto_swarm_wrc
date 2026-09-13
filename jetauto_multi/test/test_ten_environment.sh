#!/usr/bin/env bash
# Bounded offline shell contract. Master/IP probes are mocked; no nodes start.
set -euo pipefail
admin="$(cd "$(dirname "$0")/../scripts/fleet_admin" && pwd)"
for file in source_vm_ros.bash ten_environment.bash start_ten_master.sh \
    manage_ten_bottom.sh start_ten_bottom.sh check_ten_bottom.sh \
    start_ten_vendor_omni.sh start_vendor_omni.sh start_vendor_omni_sim.sh \
    manage_robot.sh check_fleet_bottom.sh; do
  bash -n "$admin/$file"
done
env -u ROS_DISTRO -u FLEET_MASTER_IP -u FLEET_VM_IP -u FLEET_MASTER_ROBOT \
  bash --noprofile --norc -eu -s -- "$admin" <<'CHECK'
admin="$1"
source "$admin/ten_environment.bash" real
[[ $- == *u* && "$ROS_DISTRO" == melodic ]]
[[ "$FLEET_MASTER_IP" == "$FLEET_VM_IP" && "$FLEET_VM_IP" == 192.168.1.104 ]]
[[ "$ROS_MASTER_URI" == http://192.168.1.104:11311 && "$ROS_IP" == 192.168.1.104 ]]
[[ "$FLEET_MASTER_ROBOT" == robot_1 && "$FLEET_ROBOT_COUNT" == 10 ]]
[[ -z "${ROS_HOSTNAME:-}" ]]
declare -F fleetmaster >/dev/null
export FLEET_MASTER_IP=192.168.1.111
source "$admin/ten_environment.bash" real
[[ "$ROS_MASTER_URI" == http://192.168.1.111:11311 ]]
source "$admin/ten_environment.bash" sim
[[ "$ROS_MASTER_URI" == http://127.0.0.1:11321 && "$ROS_IP" == 127.0.0.1 ]]
set +u
source "$admin/source_vm_ros.bash"
[[ $- != *u* ]]
set -u
# No API calls or roscore: the online probe exits successfully without launch;
# offline --check and an offline explicitly remote master must reject startup.
python() { return "${FLEET_TEST_MASTER_RESULT:-0}"; }
ip() { printf '2: ens33 inet 192.168.1.104/24 scope global ens33\n'; }
export -f python ip
export FLEET_MASTER_IP=192.168.1.104 FLEET_TEST_MASTER_RESULT=0
bash "$admin/start_ten_master.sh" --check
bash "$admin/start_ten_master.sh"
export FLEET_TEST_MASTER_RESULT=1
if bash "$admin/start_ten_master.sh" --check; then exit 90; fi
export FLEET_MASTER_IP=192.168.1.111
if bash "$admin/start_ten_master.sh"; then exit 91; fi
export FLEET_TEST_MASTER_RESULT=2
if bash "$admin/start_ten_master.sh" --check; then exit 92; fi
CHECK
echo 'PASS: shell syntax; clean nounset environment; VM default; explicit master override; isolated sim; master check/no duplicate/no remote launch'
