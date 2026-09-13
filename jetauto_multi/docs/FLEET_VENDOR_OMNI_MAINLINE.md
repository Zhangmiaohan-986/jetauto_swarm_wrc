# Six-Robot Vendor-Omni Mainline

This page describes the retained six-robot profile only. The shared implementation
also supports the ten-robot candidate; its different IP roster, scheduling and
validation evidence are in [FLEET10_RUNBOOK.md](FLEET10_RUNBOOK.md).

This is the only active six-robot implementation line. It deliberately keeps
the vendor bottom chain and does not include the historical safety guard,
measured-anchor TF publisher, role router, or dynamic group manager.

## Runtime contract

The physical roster is defined once in
`config/fleet_vendor_omni/six_robot_runtime.bash`:

```text
.111 robot_2 -> logical robot_1 (ROS Master)
.112 robot_3 -> logical robot_2
.113 robot_4 -> logical robot_3
.117 robot_8 -> logical robot_4
.115 robot_6 -> logical robot_5
.116 robot_7 -> logical robot_6
```

The validated six-car candidate uses `fuse_imu_yaw=false`, dynamic IMU bias
`false`, TEB `vx/vy/backwards=0.10 m/s`, TEB `wz=0.08 rad/s`, and
`formation_line/speed=0.08 m/s`. The initial poses and mission geometry are in
`config/fleet_vendor_omni/six_robot_demo.yaml`.

## Real and simulated execution

Both public launch files use the same mission YAML, supervisor, navigation
include, TEB defaults, topic names, TF names, stage barriers, and recorder:

```text
launch/real.launch
launch/sim.launch
```

The only intentional difference is the bottom adapter:

```text
real: vendor jetauto_slam/jetauto_robot.launch on each physical robot
sim:  scripts/fleet_vendor_omni/sim_adapter.py
```

The simulator publishes the same `/robot_i/odom`, `/robot_i/imu`,
`/robot_i/scan`, `/robot_i/scan_raw`, `/robot_i/amcl_pose`, no-motion service,
and `map -> odom -> base_footprint` TF contract. It then feeds the real
vendor `move_base + TEB` and the real `fleet_vendor_omni/supervisor.py`.
This is a controller/mission parity simulation, not the old measured-anchor
simulation.

## Actual control chain

```text
physical vendor bottom or sim_adapter
  ├─ odom / IMU / scan
  └─ map -> odom -> base_footprint TF

scan -> amcl_teammate_scan_filter -> amcl_scan
amcl (real only) -> map -> odom
move_base + vendor TEB -> /robot_i/jetauto_controller/cmd_vel

fleet_vendor_omni/supervisor.py
  ├─ preflight and runtime parameter checks
  ├─ stage barrier and checkpoint verification
  ├─ move_base goals for transition stages
  └─ formation_line direct vector control for straight stages
```

Stages without `mode` use `move_base + TEB`. `ROW_FORWARD`,
`COLUMN_FORWARD`, and `TRIANGLE_FORWARD` use `formation_line`. The current
formation controller publishes directly to the vendor command topic; there is
no independent safety guard in this release candidate. The supervisor still
stops on stale sensors, teammate clearance violations, timeout, or failed
checkpoint, and `scripts/fleet_admin/emergency_zero.sh` sends software zero
commands.

## Localization policy

The lower chain remains vendor-owned. The current EKF policy is to reject IMU
absolute yaw and retain yaw-rate use. Dynamic bias is disabled in the validated
run. AMCL remains a map localization source; it is not a velocity controller.
The vendor odometry implementation is command-integrated rather than a full
wheel-encoder measurement, so this remains a known hardware limitation.

## Operations

On the VM, source the workspace and run the bottom launcher first:

```bash
bash scripts/fleet_admin/start_six_bottom.sh
bash scripts/fleet_admin/check_six_bottom.sh
bash scripts/fleet_admin/start_vendor_omni.sh \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true
```

Use `launch/sim.launch` for the same mission without the
physical bottom layer, or run `bash scripts/fleet_admin/start_vendor_omni_sim.sh`.
Keep `motion_enabled:=false` until the supervisor reports `READY`.

## Source/deployment boundary

The F: drive is authoritative. `/home/ubuntu/jetauto_real_dev_ws` is a build
and run copy, and `/home/jetauto/jetauto_ws/src` on each robot is the vendor
hardware workspace. Neither VM nor robot workspaces are source archives.

The frozen snapshot under `vendor/jetauto_navigation` is not edited as part of
fleet behavior; current overrides are under `config/fleet_vendor_omni`.
