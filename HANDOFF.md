# JetAuto 十车项目交接（2026-09-13）

本文写给完全没有历史上下文的新对话。先读本文件，再读
`jetauto_multi/AGENTS.md`，然后才允许分析或修改代码。

## 1. 项目在做什么

这是 ROS1 Melodic + JetAuto 全向麦轮底盘的十车集中式编队系统。VM 负责 ROS Master、
每车 AMCL/move_base/TEB、统一 supervisor、RViz 和录包；每台车只运行厂商底层控制、
雷达、IMU、里程计和 TF。

当前同地图任务共五段：

```text
ROW_FORWARD       两排五车同步直行（formation_line）
TO_COLUMNS        两排转换为双列（move_base + TEB + progress gates）
COLUMN_FORWARD    双列同步直行（formation_line）
TO_V_PAIRS        双列转换为上下两组五车 V 形（move_base + TEB + progress gates）
TRIANGLE_FORWARD  两组 V 形同步直行后停止（formation_line）
```

AMCL只回答“车在哪里”，不替代控制器。直行段由固定车头全向向量控制；需要避障和绕行的
转换段使用厂商 move_base + TEB。当前没有独立 safety guard 或唯一 cmd_vel mux。

## 2. 权威源码和运行副本

- 唯一源码真相：`F:\swarm_src\src\swarm_wrc\jetauto_swarm`
- 活动上层包：`F:\swarm_src\src\swarm_wrc\jetauto_swarm\jetauto_multi`
- VM 运行副本：`/home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi`
- 实车厂商工作空间：每车 `/home/jetauto/jetauto_ws`
- VM 当前 SSH：`ubuntu@192.168.1.104`，使用
  `C:\Users\10196\.ssh\id_ed25519_jetauto_vm`

2026-09-13 本仓库从 VM 工作树直接恢复。VM 的 `jetauto_multi` 基线提交为
`27a55c235dc240ef87feb8f8bf3167d4f79eee98`，恢复时仍有未提交的真实试验改动；这些改动
已原样带回，不能用旧 F 盘目录覆盖。

同步时逐文件校验 VM 六个工作空间包，共243个文件，哈希不一致数为0。一次性同步了：

```text
jetauto_multi
jetauto_navigation
jetauto_simulations
jetauto_sdk
rf2o_laser_odometry
virtual_wall
```

仓库原有厂商包（`jetauto_driver`、`jetauto_slam`、`jetauto_bringup` 等）继续保留。这里是
厂商源码/构建依赖快照，不代表已经镜像十台车磁盘上的每个现场文件；车上的固定IP、
`.jetautorc` 和临时 transport launch 由运维脚本和现场配置负责。

## 3. 当前完成情况与卡点

已经完成：

- 十车 IP/ROS 名称一一对应的 runtime 表；ROS Master 移至 VM `.104`。
- 十车同地图五阶段 mission、两组五车 V 形路径和进度门控并发转换。
- VM 传感器/TF 汇聚，减少十车无线重复订阅。
- TEB 空 global plan、短 connector、方向拒绝、route deviation、传感器短暂停顿、
  AMCL槽位确认等局部等待/恢复逻辑。
- 十车控制同源仿真曾完成五阶段，但理想仿真不证明轮滑、死区、网络或USB可靠性。
- `slot_amcl_verify_enabled=true` 的实车槽位确认入口，最多并行3辆。

当前不能宣称“十车实车已稳定跑通”。最新整体记录是九车运行，因为 Robot10 曾出现
ext4/文件损坏（`Structure needs cleaning`）并临时下线。最新九车已知失败链为：

```text
TO_V_PAIRS 中 Robot9 短 connector 被判有障碍
→ 调度仍向更远节点19派发目标
→ global plan为空
→ 回退到节点16后仍被同一connector障碍阻塞
→ recovery_owner使其他车辆全部等待
→ direction_candidates_exhausted
→ LOCAL_GOAL_HOLD
```

该包缺少 Robot9 的原始 scan/amcl_scan，无法仅凭 RViz 证明雷达物理偏移或 AMCL 错误。
上述失败链来自已完整分析的 `fleet_vendor_omni_9_20260913_001754.bag`；目录中修改时间
更新的 `003546.bag` 只代表最新文件，尚未在本交接中声称已经完成因果分析。
下一步应优先解决“connector等待期间不得派发更远目标”和“恢复只阻塞路线冲突车辆”，
再做一轮有界仿真和实车验证；不要继续无条件增加超时或反复调用 `start_all` 掩盖状态机问题。

## 4. 代码结构

```text
jetauto_multi/
├─ AGENTS.md                         项目级开发约束，必须先读
├─ config/fleet_vendor_omni/
│  ├─ profiles/fleet10_same_map/
│  │  ├─ profile.yaml                地图、mission、runtime、RViz入口
│  │  ├─ runtime.bash                IP、IMU/EKF、速度、通信参数唯一来源
│  │  ├─ planning.yaml               初始点、阶段、槽位和规划参数可编辑源
│  │  └─ mission.yaml                生成物；不要和planning双向手改
│  ├─ amcl_initialization.yaml        AMCL初始协方差与缓存策略
│  ├─ amcl_scan_filter.yaml           队友激光遮挡过滤
│  └─ teb_defaults.yaml               公共TEB默认参数
├─ maps/realmap_obstacle/...          当前实车地图PGM/YAML
├─ launch/
│  ├─ ten_real.launch                 十车实车入口
│  ├─ ten_sim.launch                  十车同源仿真入口
│  ├─ ten_sp.launch                   可关闭首速度方向保护的消融入口
│  └─ fleet_vendor_omni/include/      每车导航和底层transport薄封装
├─ scripts/fleet_vendor_omni/
│  ├─ launch_fleet.py                 加载profile并生成统一launch
│  ├─ fleet_profile.py                roster、参数、active fleet装配
│  ├─ plan_profile.py                 planning生成mission
│  ├─ fleet_geometry.py               地图、路线和冲突几何
│  ├─ transition_gates.py             路段许可/依赖/候选逻辑
│  ├─ supervisor.py                   五阶段状态机和全部恢复主逻辑
│  ├─ sim_adapter.py                  理想全向仿真底层
│  ├─ tf_ingress.py                   VM静态TF汇聚
│  └─ record.sh                       精炼rosbag话题表
├─ scripts/fleet_admin/
│  ├─ sync_vm.ps1                     F盘源码推送VM、自动备份并编译
│  ├─ start_ten_master.sh              VM ROS Master
│  ├─ start_ten_bottom.sh              启动十车底层
│  ├─ check_ten_bottom.sh              十车底层健康检查
│  ├─ manage_ten_bottom.sh             批量start/stop/restart/status/logs
│  ├─ manage_robot.sh                  单车底层管理
│  ├─ poweroff_ten.sh                  顺序sync并关机，离线车跳过
│  └─ start_ten_vendor_omni.sh         sim/real统一上层入口
├─ rviz/fleet_vendor_omni_ten.rviz
└─ test/                               Python/契约/转换定向测试
```

顶层其他目录是厂商包或 VM 构建依赖。上层日常修改原则上只改 `jetauto_multi`；不得顺手
修改真实车辆的 `/home/jetauto/jetauto_ws`。

## 5. 十车网络和初始化约定

| IP | ROS namespace | 系统hostname建议 |
|---|---|---|
| 192.168.1.111 | robot_1 | robot1 |
| 192.168.1.112 | robot_2 | robot2 |
| 192.168.1.113 | robot_3 | robot3 |
| 192.168.1.114 | robot_4 | robot4 |
| 192.168.1.115 | robot_5 | robot5 |
| 192.168.1.116 | robot_6 | robot6 |
| 192.168.1.117 | robot_7 | robot7 |
| 192.168.1.118 | robot_8 | robot8 |
| 192.168.1.119 | robot_9 | robot9 |
| 192.168.1.120 | robot_10 | robot10 |

所有车和 VM 位于同一 `192.168.1.0/24` 网络。Wi-Fi SSID 是 `Swarm`；密码属于现场凭据，
不要提交进 Git。每车运行时必须满足：

```bash
export ROS_MASTER_URI=http://192.168.1.104:11311
export ROS_IP=192.168.1.11N
unset ROS_HOSTNAME
export ROBOT_HOST=robot_N
export ROBOT_MASTER=robot_1
export FLEET_IMU_BIAS_ESTIMATION=false
```

底层启动同时传 `fuse_imu_yaw:=false`，即 EKF 不融合漂移的 IMU 绝对 yaw，只融合 yaw rate；
动态 IMU bias 关闭。十车 runtime 还启用 `FLEET_LOCAL_TF=true` 和 Wi-Fi powersave off。

固定 IP 是操作系统网络配置，不等于 `ROS_IP`。新刷机车辆还需确认：

- `start_app_node.service` 已停止，避免厂商自动进程抢占；
- `/dev/rrc` 与 `/dev/lidar` 是有效字符设备软链接；
- `/home/jetauto/jetauto_ws` 可正常 source；
- `jetauto_robot.launch` 接受 `fuse_imu_yaw`，动态bias环境变量能到达IMU filter；
- `robot_N_transport.launch` 由 VM 管理，不在十台车上手改十份。

## 6. 当前关键参数

唯一入口：`jetauto_multi/config/fleet_vendor_omni/profiles/fleet10_same_map/runtime.bash`。

```text
TEB max_vel_x/y/backwards = 0.10 m/s
TEB max_vel_theta         = 0.10 rad/s
TEB acc_lim_x/y           = 0.10 m/s²
TEB acc_lim_theta         = 0.15 rad/s²
formation_line max        = 0.15 m/s
formation_line min        = 0.10 m/s
formation lateral max     = 0.10 m/s
formation angular max     = 0.10 rad/s
short connector max/min   = 0.10 m/s
AMCL update_min_d/a       = 0.01 m / 0.10 rad
AMCL recovery alpha       = 0.0 / 0.0
```

改 runtime 的上层速度要重启上层；改 IMU/EKF/TF transport 必须重启对应底层并重新跑
`check_ten_bottom.sh`。提高速度不会自动提高 AMCL 精度，也不能修复USB、轮滑或错误odom。

## 7. F盘与VM双向同步

先确认 VM 上层已 Ctrl+C 停止。

F盘修改后推送到 VM（自动备份 VM 旧包并 `catkin_make --pkg jetauto_multi`）：

```powershell
powershell -ExecutionPolicy Bypass -File F:\swarm_src\src\swarm_wrc\jetauto_swarm\jetauto_multi\scripts\fleet_admin\sync_vm.ps1 -VmAddress 192.168.1.104
```

如果直接在 VM 修改，先提交/暂存本地 F 盘改动，再拉回并查看 diff：

```powershell
powershell -ExecutionPolicy Bypass -File F:\swarm_src\src\swarm_wrc\jetauto_swarm\tools\pull_jetauto_multi_from_vm.ps1 -VmAddress 192.168.1.104
git -C F:\swarm_src\src\swarm_wrc\jetauto_swarm status --short
git -C F:\swarm_src\src\swarm_wrc\jetauto_swarm diff -- jetauto_multi
```

拉取脚本默认拒绝覆盖本地未提交的 `jetauto_multi`。只有明确要以 VM 覆盖本地时才加
`-Force`。不要把旧目录 `F:\swarm_src\src\src\jetauto_multi` 再当作源码源头。

## 8. Git保存

此仓库已配置 Git LFS 保存 `.bag`。新电脑首次执行：

```powershell
git lfs install
git status --short
git add .gitattributes .gitignore README.md HANDOFF.md tools jetauto_multi `
  jetauto_navigation jetauto_simulations jetauto_sdk rf2o_laser_odometry virtual_wall `
  experiment_records
git commit -m "restore current ten-robot VM source and handoff"
git push
```

提交前必须检查 `git diff --cached --stat`；不要提交SSH私钥、Wi-Fi密码、ROS日志、build/devel。

## 9. VM仿真命令

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi
source scripts/fleet_admin/ten_environment.bash sim
bash scripts/fleet_admin/start_ten_vendor_omni.sh sim \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true
```

另一个已 source 同一环境的终端：

```bash
rostopic echo -n 1 /fleet_vendor_omni/state_json
rosservice call /fleet_vendor_omni/start_all '{}'
```

仿真不是 Gazebo 物理仿真，而是同一 move_base/TEB/supervisor 配理想全向底层和模拟雷达。

## 10. 十车实车标准启动

全部在 VM 执行。先启动/确认 Master：

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi
source scripts/fleet_admin/ten_environment.bash real
bash scripts/fleet_admin/start_ten_master.sh
```

另一个终端启动并检查底层：

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi
source scripts/fleet_admin/ten_environment.bash real
FLEET_RESTART_EXISTING=true bash scripts/fleet_admin/start_ten_bottom.sh
bash scripts/fleet_admin/check_ten_bottom.sh
```

先静止验收上层：

```bash
bash scripts/fleet_admin/start_ten_vendor_omni.sh real \
  motion_enabled:=false allow_full_run:=false record_data:=true gui:=true \
  slot_amcl_verify_enabled:=true slot_amcl_max_parallel:=3
```

状态必须持续 `READY`。Ctrl+C结束静止上层；人员、净空和实体急停确认后，才运行：

```bash
bash scripts/fleet_admin/start_ten_vendor_omni.sh real \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true \
  slot_amcl_verify_enabled:=true slot_amcl_max_parallel:=3
```

另一个终端一次性启动任务：

```bash
rostopic echo -n 1 /fleet_vendor_omni/state_json
rosservice call /fleet_vendor_omni/start_all '{}'
```

停车：

```bash
rosservice call /fleet_vendor_omni/stop '{}'
```

不要在 `a stage is already active` 时重复 `start_all`。那会隐藏原始失败链，并可能清空或重置
局部状态；应先保存 bag/state/event 再诊断。

单车和批量管理：

```bash
bash scripts/fleet_admin/manage_ten_bottom.sh status all
bash scripts/fleet_admin/manage_ten_bottom.sh restart 9
bash scripts/fleet_admin/manage_ten_bottom.sh logs 9
bash scripts/fleet_admin/manage_ten_bottom.sh stop all
bash scripts/fleet_admin/poweroff_ten.sh
```

Robot10临时下线时，不改十车profile源文件；上层可临时传：

```bash
excluded_robots:="robot_10"
```

恢复十车时去掉该参数并重启上层、重新底层健康检查。

## 11. 快速抓包和分析失败

正式启动使用 `record_data:=true`，bag默认写到：

```text
/home/ubuntu/fleet_vendor_omni_records/
```

先只选最新一份，不要反复扫全部历史包：

```bash
latest_bag=$(find /home/ubuntu/fleet_vendor_omni_records -maxdepth 1 -type f -name '*.bag' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)
echo "$latest_bag"
rosbag info "$latest_bag"
```

诊断原则：用一个临时 Python `rosbag.Bag(...).read_messages(topics=[...])` 一次遍历，先定位
`/fleet_vendor_omni/event_json` 中首次进入 `ERROR/GOAL_HOLD/LOCAL_GOAL_HOLD/cancel_all`
的时刻，只分析前后10～15秒；再对齐触发车的目标、global/local plan、move_base result、
最终 cmd_vel、odom、AMCL、scan 和 TF。缺失topic必须写明“bag无法证明”，不能靠RViz猜。

最小核心topic：

```text
/fleet_vendor_omni/state_json
/fleet_vendor_omni/event_json
/rosout
/tf  /tf_static
/robot_N/amcl_pose
/robot_N/odom  /robot_N/odom_raw
/robot_N/scan_raw  /robot_N/amcl_scan
/robot_N/move_base/status  /robot_N/move_base/result
/robot_N/move_base/GlobalPlanner/plan
/robot_N/move_base/TebLocalPlannerROS/local_plan
/robot_N/jetauto_controller/cmd_vel
/robot_N/ros_robot_controller/set_motor
```

精炼运行默认不录所有底层原始诊断。若要证明电机少走、USB断线、轮速或IMU原始链，单独将
`record_bottom_raw: true` 做一次诊断实验；不能用缺失数据推定硬件正常。

本次复制的最新九车和十车bag及SHA256见 `experiment_records/README.md`。

## 12. 绝对不要再踩的坑

- 不把 `.111` 当十车 ROS Master；当前唯一 Master 是 VM `.104`。
- 不混用 `ROS_HOSTNAME` 和 `ROS_IP`；十车使用 `ROS_IP` 并 `unset ROS_HOSTNAME`。
- 不让厂商 `start_app_node.service` 和 VM 管理的底层同时运行。
- 不通过反复 `start_all`、增大timeout或放宽安全门槛伪造“恢复”。
- 不把 TEB `SUCCEEDED`、odom自洽或RViz位置正确当成实体已到位。
- 不让 AMCL 负责运动控制；AMCL不更新时保留预留并等待，不能用旧位姿直接放行。
- 不同时手改 `planning.yaml` 和生成的 `mission.yaml`。
- 不复制 `v2/final/new/copy` 并行算法包；十车/九车用profile参数切换。
- 不顺手修改厂商底层、AMCL/EKF/TF数学链或加入新guard。
- `/dev/rrc`/USB掉线先查线缆、Hub、供电和内核日志，不能靠上层补丁修硬件。
- Robot10 的 `Structure needs cleaning` 是文件系统问题；必须离线fsck/重刷或可靠修复，
  不能继续从其他车复制随机库文件掩盖。
- VM源码曾包含未提交实车补丁；每轮实车前先提交或打tag，确保bag能对应唯一代码版本。

## 13. 下一步最小计划

1. 在本仓库提交本次恢复快照并打标签，例如 `fleet10-vm-snapshot-20260913`。
2. 修复/确认 Robot10 文件系统和底层健康；十车 checker 必须0 FAIL。
3. 针对 Robot9 connector 等待冲突，只改 `supervisor.py` 和定向测试：等待期间锁住更远目标，
   恢复只阻塞路线冲突车辆。
4. 运行Python语法、相关transition测试和一次注入该失败条件的有界仿真。
5. 实车先 `motion_enabled=false` 持续READY，再完整十车一次启动；不人工补按 `start_all`。
6. 保存源码commit、启动参数、bag路径和首次失败事件，交给证据分析对话。
