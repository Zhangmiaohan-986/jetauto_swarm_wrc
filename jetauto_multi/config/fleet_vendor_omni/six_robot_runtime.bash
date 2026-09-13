#!/usr/bin/env bash

# One editable table for the physical roster, bottom IMU/EKF switches, and
# upper AMCL/TEB limits.  Fields are separated by "|":
# logical | IP | factory hostname | fuse IMU yaw | dynamic IMU bias |
# max vx | max vy | max reverse vx | max wz | acc x | acc y | acc wz |
# AMCL min d | AMCL min a | recovery slow | recovery fast
#
# robot_2 at .112 is the physically validated baseline.  The other five use
# the same speed-trial limits for this six-car run; tune one row only after
# that chassis has its own bag-backed test.
FLEET_RUNTIME_ROWS=(
  "robot_1|192.168.1.111|robot_2|false|false|0.10|0.10|0.10|0.08|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
  "robot_2|192.168.1.112|robot_3|false|false|0.10|0.10|0.10|0.08|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
  "robot_3|192.168.1.113|robot_4|false|false|0.10|0.10|0.10|0.08|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
  "robot_4|192.168.1.117|robot_8|false|false|0.10|0.10|0.10|0.08|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
  "robot_5|192.168.1.115|robot_6|false|false|0.10|0.10|0.10|0.08|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
  "robot_6|192.168.1.116|robot_7|false|false|0.10|0.10|0.10|0.08|0.10|0.10|0.15|0.01|0.10|0.0|0.0"
)

# Direct formation_line stages intentionally stay fleet-wide so synchronized
# rows do not give one chassis a different nominal progress rate.
FLEET_FORMATION_SPEED=0.08
FLEET_FORMATION_MIN_SPEED=0.025
FLEET_FORMATION_MAX_LATERAL=0.020
FLEET_FORMATION_MAX_ANGULAR=0.08
