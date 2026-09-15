#!/usr/bin/env bash
# Ten-car discovery/parameters stay on the VM; robot_1 remains the map frame.
export FLEET_VM_IP="${FLEET_VM_IP:-192.168.1.104}"
export FLEET_MASTER_IP="${FLEET_MASTER_IP:-$FLEET_VM_IP}"
export FLEET_MASTER_ROBOT="${FLEET_MASTER_ROBOT:-robot_1}"
# Apply at bottom start without changing Wi-Fi credentials, IP or vendor files.
export FLEET_WIFI_POWER_SAVE="${FLEET_WIFI_POWER_SAVE:-off}"
export FLEET_LOCAL_TF="${FLEET_LOCAL_TF:-true}"
# Keep the evidence recorder from subscribing to ten high-rate scan streams.
export FLEET_RECORD_COMPACT="${FLEET_RECORD_COMPACT:-true}"
# This real-trial profile records TEB evidence without re-enabling ten raw scans.
export FLEET_RECORD_COSTMAPS="${FLEET_RECORD_COSTMAPS:-true}"
# IP and logical namespace are matched here, independent of factory hostname.
# Fields: namespace|IP|expected label|fuse_imu_yaw|dynamic_bias|
# vx|vy|vx_backwards|wz|acc_x|acc_y|acc_wz|
# amcl_update_min_d|amcl_update_min_a|recovery_slow|recovery_fast
# The third field is informational. ROBOT_HOST always uses the first field.
# Previous baseline kept for rollback: per-robot max_vel_theta=0.08;
# acc_lim_theta=0.15 remains unchanged.
# Yaw-fusion fix (2026-09-15): fuse_imu_yaw/dynamic_bias false->true,
# amcl_update_min_a 0.10->0.02, recovery_slow/fast 0.0/0.0->0.001/0.05.
# Rollback = restore each row tail to:
#   "false|false|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
FLEET_RUNTIME_ROWS=(
  "robot_1|192.168.1.111|robot_1|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_2|192.168.1.112|robot_2|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_3|192.168.1.113|robot_3|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_4|192.168.1.114|robot_4|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_5|192.168.1.115|robot_5|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_6|192.168.1.116|robot_6|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_7|192.168.1.117|robot_7|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_8|192.168.1.118|robot_8|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_9|192.168.1.119|robot_9|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
  "robot_10|192.168.1.120|robot_10|true|true|0.10|0.10|0.10|0.10|0.10|0.10|0.15|0.01|0.02|0.001|0.05"
)
# Previous formation_line baseline:
# FLEET_FORMATION_SPEED=0.08
# FLEET_FORMATION_MIN_SPEED=0.025
# FLEET_FORMATION_MAX_LATERAL=0.020
# FLEET_FORMATION_MAX_ANGULAR=0.08
# Previous: FLEET_FORMATION_SPEED=0.10
FLEET_FORMATION_SPEED=0.15
# Previous: FLEET_FORMATION_MIN_SPEED=0.05
# Previous: FLEET_FORMATION_MIN_SPEED=0.08
# Previous: FLEET_FORMATION_MIN_SPEED=0.10
FLEET_FORMATION_MIN_SPEED=0.03
# Previous: FLEET_FORMATION_MAX_LATERAL=0.05
# Previous: FLEET_FORMATION_MAX_LATERAL=0.08
FLEET_FORMATION_MAX_LATERAL=0.10
FLEET_FORMATION_MAX_ANGULAR=0.10
# Gated TEB follows the reserved prefix; six-robot default remains 10.
FLEET_TRANSITION_VIA_WEIGHT=10
# Optimization penalties, NOT velocity/acceleration limits. All x/y/yaw axes.
FLEET_TRANSITION_MOTION_WEIGHT=20
