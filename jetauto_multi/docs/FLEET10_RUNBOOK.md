# 十车候选主线（2026-09-07）

验收状态：地图几何预检、Python2/XML/bash 语法和 26 项契约测试通过；
新局部路段调度完整 ROS 仿真 `transition_gates/sim_07` 已 SUCCEEDED，五阶段全部通过。
两次转换均实测7车同时运动，实车尚未验证。新算法及回退见
[PROGRESS_GATED_TRANSITIONS.md](PROGRESS_GATED_TRANSITIONS.md)。
用户已确认接受保留原路线的窄 V，两翼中心宽0.56/0.60m；宽 V 需另选区域，不能直接放大。
换地图、8种备用编队及6套顺序参考见 [NEW_MAP_FORMATION_WORKFLOW.md](NEW_MAP_FORMATION_WORKFLOW.md)。
不覆盖 `deployment20260906六车部署(勿删)`，不修改实车厂商底层，不新增独立 guard。

## 网络与编号

- Windows 当前开发 SSH：`ssh -i C:\Users\10196\.ssh\id_ed25519_jetauto_vm ubuntu@192.168.1.104`。
- VM 当前通过桥接车队网访问；地址为 `192.168.1.104`。若网络调整，先用 `ip -4 addr` 核对，再显式传 `-VmAddress`。
- VM ens33=192.168.1.104/24 保留给车队。实车时 Windows/VM 桥接网卡须连接车队同一局域网。
- 十车 ROS Master=VM 192.168.1.104:11311；六车旧入口仍用 .111。
  十车所有底层、上层和检查终端必须连接同一个 VM Master。
  .111 旧 roscore 即使还在，也不再接入十车实验；开发 NAT IP 不能代替 ROS_IP。
- .111… .120 一一对应 robot_1…robot_10。底层脚本显式设置 ROBOT_HOST/ROBOT_MASTER，
  不修改操作系统 hostname；health check 核对底盘 ROS 节点实际 IP，避免发错车。
- 六个继承槽位：112←旧2、113←旧1、114←旧3、117←旧5、118←旧4、119←旧6。
  新增两侧车保持20cm车体净距；两排纵向间距沿用原配置。

Windows PowerShell 同步命令（先停止 VM 上层；同步前自动备份 VM 旧包）：

```powershell
powershell -ExecutionPolicy Bypass -File F:\swarm_src\src\swarm_wrc\jetauto_swarm\jetauto_multi\scripts\fleet_admin\sync_vm.ps1 -VmAddress 192.168.1.104
```

SSH 私钥沿用当前电脑的认证文件，不把私钥放进部署包。新电脑配置自己的 SSH 认证，
通过 `-IdentityFile` 指定。同步只覆盖 VM 的确切 jetauto_multi 包，旧 VM 包归档在
`/home/ubuntu/experiment_records/source_backups`，不会触碰厂商底层。

## 文件与参数唯一入口

```text
config/fleet_vendor_omni/profiles/fleet10_same_map/
  profile.yaml        地图、任务、参数表、RViz 的引用
  runtime.bash        10车 IP、底层开关、TEB/AMCL 参数和编队速度
  planning.yaml       初始点、编队次序/槽位尺度/换队区域、规划净距
  mission.yaml        由 planning.yaml 生成；不要直接编辑
scripts/fleet_vendor_omni/
  launch_fleet.py     ROS 入口，装配 profile
  fleet_profile.py    六/十共用参数加载与 launch 生成
  fleet_geometry.py   地图膨胀、A*路线、槽位分配、全路径冲突判断
  transition_gates.py 有界自由槽位候选、局部冲突依赖、实际进度放行
  plan_profile.py     从地图+编队意图生成阶段、路线和依赖
  supervisor.py      六车原阶段 + 十车依赖调度；同一个控制实现
  sim_adapter.py     理想全向底层、地图雷达、接触检测、truth_json
  record.sh          按车数录制话题
scripts/fleet_admin/
  ten_environment.bash     source 环境与十车 profile
  start_ten_master.sh      VM 前台 ROS Master；已有服务不重复启动
  start_ten_bottom.sh      十车入口 → manage_ten_bottom.sh
  check_ten_bottom.sh      十车入口 → check_fleet_bottom.sh
  probe_fleet_runtime_health.py  按名单检查雷达/odom/TF
  manage_robot.sh          单车 start/stop/restart/status/logs
  start_ten_vendor_omni.sh  sim/real → 共用上层入口
  emergency_zero.sh        软件停车，不代替实体急停
launch/{real,sim,ten_real,ten_sim}.launch  薄入口
launch/fleet_vendor_omni/include/robot_navigation.launch  每车相同导航链
```

当前十车 runtime：TEB vx/vy/backwards/wz 上限均为0.10；formation_line 最大速度0.15m/s、
最低速度0.10m/s、横向限幅0.10m/s、角速度上限0.10rad/s；短连接速度上限和最低速度
均限制到0.10m/s。动态 IMU bias=false，EKF yaw=false/yaw-rate=true。
runtime 修改速度需重启上层；修改 IMU/EKF 开关必须重启相应底层并重新 check。
不是将历史 Robot2 原地实验的0.10rad/s补偿自动推广至所有车。
新调度专用TEB优化权重也来自runtime：VIA_WEIGHT=10、MOTION_WEIGHT=20；
它们不是速度限幅，只对progress_gates生效，六车基线不变。

## 十车通信减负（2026-09-07 夜）

- Master 移到 VM `.104`：VM 上参数查询、节点注册不再逐次跨 Wi-Fi 访问 `.111`。
  这不是搬迁底层 EKF：底层仍在各车，TF frame 与 map→odom→base 所有权不变。
- `src/fleet_sensor_ingress.cpp` 每车只订阅一次 scan/odom/imu，原样发布到
  `/fleet/robot_N/{scan,odom,imu}`。上层导航、supervisor、RViz、录包都共享 VM 内数据。
  使用 queue=1/TCP_NODELAY，不改时间戳、不掩盖 NaN、不放宽过期门槛。
  底层 EKF 在本车订阅 IMU 不受影响。
- 十车 `runtime.bash` 的 `FLEET_LOCAL_TF=true` 由 VM 运维脚本传递给薄底层启动 wrapper。
  wrapper 只把每车 `/tf`、`/tf_static` 映射到 `/robot_N/tf`、`/robot_N/tf_static`，
  各车 EKF 不再接收其他车辆 TF；VM 的原生入口汇总动态 TF，静态入口保留所有固定边。
  不修改厂商源码、TF frame、TF 所有权、EKF配置或数学计算。wrapper副本位于各车
  `~/swarm_logs/robot_N_transport.launch`，由 VM 包统一管理，不手改十份。
  该开关改变后必须重启全部底层和上层，不可混用全局/分车TF两种通信域。
  底层检查会按同一 runtime 开关订阅各车的TF话题；裸 `tf_echo` 则要显式重映射。
- 十车 profile 默认 `record_bottom_raw: false`：126个录制目标，而非166个。
  不再默认录 particlecloud、odom_raw、imu_raw、set_motor。
  在线订阅的是 `/fleet/robot_N/...` 与 `/fleet/tf[_static]`；实测 rosbag 内仍保存
  原请求名称 `/robot_N/{scan,odom,imu}`、`/tf`、`/tf_static`。分析 bag 时按 bag info
  中的名字读取，不能把在线 remap 名直接当成 bag 的存储名称。
  若要诊断电机/IMU原始链，在专门诊断实验中将该 profile 参数改为 true；
  精简运行包不能替代原始底层诊断包。
- 十车 `runtime.bash` 默认 `FLEET_WIFI_POWER_SAVE=off`。
  `fleet start/restart` 用系统现有 iw 关闭 wlan0 省电，不改 IP、SSID、密码或厂商文件。
  `start` 对已运行且 Master 一致的底层只应用该无线选项，不重启节点；失败则明确报错。
  `on` 可恢复省电，`unchanged` 不操作；六车未显式配置时保持 unchanged。
- VM `.bashrc` 不再硬编码旧 Master；启动新终端或 `source ~/.bashrc` 后应看到
  `ROS_MASTER_URI=http://192.168.1.104:11311`。旧 `.111` Master 没被脚本停止，
  但不要再把十车终端或手动启动的底层接到它。

一次完整组合的静止验收需同时开启 GUI 和录包，检查连续 READY、扫描年龄和实际话题连接；
禁止仅靠增加 timeout、丢掉 NOT_READY 样本或不断重试直到偶然通过来宣称已修好。
现场无线链路仍可能变化，短时静止通过不等于十车运动、长时间可靠性或断线恢复已验收。

### 本轮实际结果（22:56–23:03，完整 GUI + 录包组合）

- 底层健康检查：189 PASS、2 WARN、0 FAIL；两项 WARN 为 Robot1/4 雷达约7.6–7.9Hz。
- 由上层启动到首个 READY：20.55秒。VM 参数查询5次为0.44–1.16ms；此前远程
  Master .111 的历史抽测为140–918ms。不是严格同负载性能基准。
- 从首个 READY 到测试结束：424条 READY、1条 NOT_READY。启动第48秒出现十车
  scan/odom/imu 共同停顿，数据接收间隔约1.7–2.1秒；未发现同期时钟跳变、VMware
  暂停或网卡重连证据，根因尚未完全确认。VM有CPU调度压力，不能据此认定已定位。
- 恢复后连续395.30秒、396条状态均 READY。原始3分钟检查的 `passed:false`
  保留，不删除失败帧、不增加超时来改成通过；这不是“全程零抖动”的验收。
- 检查290103条动态/静态TF：非有限数和非法四元数均为0。十车各请求一次
  `request_nomotion_update` 后均收到更新且有限的位姿，10条 map→base 查询均可用。
  AMCL保留输入扫描时间，不能要求新位姿stamp必然晚于服务请求时间。
- RViz显示十车模型/扫描；RViz和recorder的直连底层 scan/odom/imu/TF连接均为0。
  本地汇聚节点有30路传感器及40路TF发布者连接；各车EKF只订阅本车TF。
- 本轮不改变速度、路线、IMU/EKF滤波参数，不新增guard，未派发任务或非零速度。
  bag中无cmd_vel消息不等于收到零速确认。测试上层已Ctrl+C结束；Master和底层留在线。

证据目录：`F:/swarm_src/experiment_records/20260907/vm_transport_ready/`，VM同名目录
位于 `/home/ubuntu/experiment_records/20260907/`。以 `local_tf_full_bag_summary.json`、
`local_tf_nomotion_validation.json` 和 `fleet_vendor_omni_10_20260907_225616.bag` 为本轮记录；
前两轮global-TF尝试及失败日志一并保留。正式运动前重新启动上层并确认持续READY，
若再次出现全车同时stale，先检查VM负载与公共接收链，不重复修改AMCL或放宽门槛。

## 当前仿真证据：局部路段放行

| 阶段 | 原整路段调度 | 当前含验收耗时 | 当前实际运动峰值 |
|---|---:|---:|---:|
| ROW_FORWARD | 20.068s | 18.378s | 10 |
| TO_COLUMNS | 84.585s | 29.440s | 7 |
| COLUMN_FORWARD | 27.927s | 25.376s | 10 |
| TO_V_PAIRS | 135.096s | 51.466s | 7 |
| TRIANGLE_FORWARD | 19.907s | 17.171s | 10 |

五阶段合计287.583→141.831秒，约缩短50.7%；两次转换的活动动作峰值2→7。
“实际运动”按truth相邻位姿速度>0.01m/s计数，不只统计已发目标数。
双列/V转换平均运动数3.184/4.423，最小车心距离0.5657/0.5291m；
全程最小0.4714m来自初始摆放，接触0次。两次转换最大路线偏差0.0488/0.0497m，
均低于0.08m退出门槛；十车终点误差0.0483～0.0499m，沿用5cm终点容差。

这是同地图、同初始点、同速度上限的两次软件仿真对比，不是多次统计均值，
也不是只改变单一因素的消融：新版本同时改变槽位分配、局部调度和TEB路线适配。
不能把50.7%当作实车提速承诺。当前仿真仍是理想底层，不是Gazebo物理验收。

当前证据在F盘和VM各自的 `experiment_records/20260907/transition_gates/sim_07/`：
`evidence.json`、`summary.json`、`launch.log`、
`fleet_vendor_omni_10_20260907_135005.bag`。本轮修改前源码保存在
F盘 `experiment_records/20260907/transition_gates/source_before.tar.gz`。
仿真结束后已停止本次测试进程；没有操作任何实车。

## 历史对照：原整路段调度（不代表当前算法）

| 阶段 | 含验收耗时 | TEB 最大并发 |
|---|---:|---:|
| ROW_FORWARD | 20.068s | —（10车直行） |
| TO_COLUMNS | 84.585s | 2 |
| COLUMN_FORWARD | 27.927s | —（10车直行） |
| TO_V_PAIRS | 135.096s | 2 |
| TRIANGLE_FORWARD | 19.907s | —（10车直行） |

合计约288秒，终点误差4.42～5.00cm（沿用5cm直行到点容差），最小车心距离
0.4714m（初始摆放），0次接触。18次依赖任务释放延迟平均10.98ms、最大37.93ms。
并发不是十车同时换队；有交叉/狭窄段保留依赖。未测量六车同条件耗时，不虚报提升百分比。

证据目录（包外）：

- F盘：`F:/swarm_src/experiment_records/20260907/ten_robot_design/sim_05/`
- VM：`/home/ubuntu/experiment_records/20260907/ten_robot_design/sim_05/`
- `evidence.json`：bag离线提取的位姿、速度、间距、阶段和释放时间。
- `summary.json`、`launch.log`、`fleet_vendor_omni_10_20260907_012309.bag`。
- `trajectory_and_final.png`：F盘的轨迹与终点图。

调试过程：前三次仅排除启动权限/缺失环境和模拟参数问题；sim_04 在下侧V转折点
触发 TEB 振荡退出，无接触。原因是离线规划未包含厂商 TEB 0.13m 障碍距离偏好。
修正为0.34m车心规划留距、换队前端x=4.20m、V横向半宽0.28/0.30m后，sim_05全程通过。
没有靠修改 IMU/EKF、增大速度或反复重发失败目标绕过问题。

六车 `six_robot_demo.yaml` 与 `six_robot_runtime.bash` 的 SHA256 已和同步前 VM 包比对一致；
共享 launch/调度有更新，六车没有重新进行实车验收。底层批量重启脚本仅做静态检查，
本次没有登录/启动/移动任何实车。

## VM 上的仿真命令

可视化修复：仿真分支由 `fleet_profile.py` 为每车加载厂家 `jetauto_description`
URDF，并启动带 robot_N TF前缀的 robot_state_publisher、无GUI joint_state_publisher。
sim_adapter仍唯一负责 map→odom→base_footprint，实车分支不重复发布模型内部TF。
VM静止启动检查通过：10车、110个可视部件TF、13个mesh资源，RViz实际截图确认车体显示。
证据在 `F:/swarm_src/experiment_records/20260907/rviz_model_fix/`；此修复未重跑运动任务。

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd "$(rospack find jetauto_multi)"
source scripts/fleet_admin/ten_environment.bash sim
bash scripts/fleet_admin/start_ten_vendor_omni.sh sim \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true
```

另一个终端 source 同一 `ten_environment.bash sim`，确认状态 READY 后：

```bash
rostopic echo -n 1 /fleet_vendor_omni/state_json
rosservice call /fleet_vendor_omni/start_all '{}'
```

仿真只使用 localhost:11321；真实 move_base+TEB+supervisor，不是 Gazebo。
底层为理想速度积分、地图模拟雷达及0.20m外接圆接触阻挡；AMCL/IMU/EKF输出
由适配器模拟，不能证明真实 IMU、轮滑、死区或网络延迟。bag 默认在
`/home/ubuntu/fleet_vendor_omni_records`，可用 FLEET_RECORD_DIR 指定包外目录。

## 实车操作顺序（完整仿真通过后再执行）

所有命令在 VM 执行。先接通实验网络、确认小车IP和实体急停，不让实车使用 localhost。

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd "$(rospack find jetauto_multi)"
source scripts/fleet_admin/ten_environment.bash real
# 单独一个 VM 终端执行并保留：bash scripts/fleet_admin/start_ten_master.sh
FLEET_RESTART_EXISTING=true bash scripts/fleet_admin/start_ten_bottom.sh
bash scripts/fleet_admin/check_ten_bottom.sh
# 单独重启第10辆；需已 source 十车环境，否则默认六车表
bash scripts/fleet_admin/manage_robot.sh 10 restart
# 首先只启动导航检查，不允许运动
bash scripts/fleet_admin/start_ten_vendor_omni.sh real \
  motion_enabled:=false allow_full_run:=false record_data:=true gui:=true
```

确认上层 READY 后 Ctrl+C 结束上层；人员就位后改用：

```bash
bash scripts/fleet_admin/start_ten_vendor_omni.sh real \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true
# 另一个终端也 source ten_environment.bash real，READY 后一次启动全程
rosservice call /fleet_vendor_omni/start_all '{}'
# 停车
rosservice call /fleet_vendor_omni/stop '{}'
```

底层批量启动会停厂商 start_app 服务、显式设置每车命名空间和 VM Master。
必须先在 VM 运行 `fleetmaster`（或 start_ten_master.sh），底层脚本不再 SSH 启动 .111 的 roscore。
它不是给十车发布行驶指令。若 checker FAIL，不得启动运动；单看RViz有车不等于READY。

## 十车 VM TF 分发（2026-09-07）

十车 profile 启用 `vm_tf_relay: true`。当前 runtime 的 `FLEET_LOCAL_TF=true` 使底层
按车发布 `/robot_N/tf`、`/robot_N/tf_static`；`fleet_sensor_ingress` 除传感器外还
汇总这些动态TF到 `/fleet/tf`，`tf_ingress.py` 合并并锁存静态TF到 `/fleet/tf_static`。
只有显式回退 `FLEET_LOCAL_TF=false` 并重启全部底层/上层，才恢复旧公共TF输入与
C++ `topic_tools/relay` 的兼容链路；不能只改一侧。
不要改回 Python 动态转发：十车约600条/秒、31个上层订阅者的实测中，
Python 转发曾占满一个核；C++ 转发约5% CPU（单次静止启动观察，非性能承诺）。
上层 AMCL、TEB、supervisor、扫描过滤、RViz 和录包通过 launch 统一重映射。
AMCL 的 map→odom 在 VM TF 话题中发布，底层 EKF 不依赖该全局定位变换。
不修改坐标名称、时间戳、外参、odom→base 所有权或 IMU/EKF 参数。
静态转发缓存所有机器人固定边，供后启动的订阅者获取；不是仅转发最后一辆车。
六车 profile 没启用此选项，原话题保持不变。十车仿真同样使用 VM TF 话题，
但不启动实车 ingress；仿真 adapter/model 直接发布到 VM TF 话题。

手工单独启动十车 RViz 时也要共享 VM 传感器，避免重新订阅十路无线 scan：

```bash
scan_remaps=()
for n in {1..10}; do scan_remaps+=("/robot_${n}/scan:=/fleet/robot_${n}/scan"); done
rviz -d /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi/rviz/fleet_vendor_omni_ten.rviz \
  /tf:=/fleet/tf /tf_static:=/fleet/tf_static "${scan_remaps[@]}"
rosrun tf tf_echo robot_1/map robot_10/base_footprint \
  /tf:=/fleet/tf /tf_static:=/fleet/tf_static
```

不要同时开启两份上层。launcher 在注册 ROS 节点之前取得同一 master 的本地锁，
第二份启动会拒绝执行；旧上层应 Ctrl+C 并等退出后再启动。
原始 `/tf` 中看不到上层 AMCL 的 map→odom，并不等于 VM TF 链断开。
正式静止验收必须包含 `motion_enabled:=false allow_full_run:=false record_data:=true gui:=true`。
只在无 GUI、无录包时 READY，不代表正式组合已通过；通信修复也不代表运动已验收。

## 历史：AMCL NaN 启动修复与静止验收（2026-09-07 21时段）

以下保留当时使用 .111 Master 的诊断链，不能当成当前启动耗时；最新通信配置与
完整组合结果以本页“十车通信减负”为准。

- 原故障链：旧 AMCL 将初始协方差保存到仍运行的 ROS Master；其中出现
  极小负值或 NaN。旧 launch 只重设 x/y/yaw，没有重设三个初始方差。
  原生 AMCL 高斯采样接受负方差后可生成 NaN，污染 map→odom。
- `amcl_initialization.yaml` 是三个初始方差的唯一入口，每次启动显式加载
  `0.25 / 0.25 / 0.06853891945200942`（AMCL 默认不确定度）；初始位姿仍用任务配置。
  配置要求有限且严格为正，不可用零/极小值来伪造定位精度。
- 同一文件设置 `save_pose_rate: -1.0`，关闭周期位姿缓存。GDB 实测 AMCL 的
  `laserReceived → savePoseToServer → setParam` 阻塞于远程 XMLRPC；该改动消除
  每车每2秒六次同步参数写入，不改变滤波/重采样算法。不能填0，原生代码计算1/rate。
  AMCL 退出时仍可能保存一次，故每次启动重设初始协方差仍必需。
- supervisor 拒绝非有限位姿/协方差、不合法四元数，废弃该车旧有效缓存；
  既影响 READY，也影响定位验收和运动过程的位姿读取。不新增独立 guard。
- 已执行原生 AMCL 的参数污染回归、消息有效性、C++/静态 TF 转发测试。
  静止实车验收35秒收到36条 READY，错误列表均为空；每车一次无运动更新后，
  十车均收到新时间戳且有限的位姿/协方差。此轮日志 NaN/非法四元数计数为0。
- 测试过程中 `.119` 的 USB Hub 于21:24:43断连，控制板/雷达串口重新枚举；
  用户重启后已重新启动 Robot9 正确底层并恢复数据。若再次发生，应查 USB/供电，
  不能只看进程存在，也不应更改 AMCL 来遮掩硬件掉线。
- 随后21:38:35，Robot9同一Hub再次断连，该轮复验中断。用户再次重启后，
  最终配置（包括 `save_pose_rate=-1`）已重新完成静止验收：181.48秒达到READY，
  随后35秒34条状态全部READY，十车无运动更新后均收到新且有限的AMCL输出。
  记录为 `ready_final_evidence.json`；USB反复断连的物理原因尚未证明根除，
  启动运动前仍需确认该车USB接头/供电稳定、现场急停就位。
- 本轮上层一直 `motion_enabled=false, allow_full_run=false`，无行驶指令/目标。
  静止无 cmd_vel 消息不等于收到了零速消息；本轮未观测到非零命令。
- 启动仍可能耗时数分钟：ROS Master `.111` 的10次只读 RPC 抽测总计5.46秒，
  单次0.140～0.918秒；日志显示 TEB/代价地图参数初始化分段耗时。
  `.111` 本机同类查询仅0.0027～0.0064秒；RViz GDB也显示订阅初始化阻塞于参数查询。
  READY 已验证，不代表启动延迟已彻底解决，也不是十车实车全程验收。

记录：F盘 `F:/swarm_src/experiment_records/20260907/amcl_ready_fix/`；
VM `/home/ubuntu/experiment_records/20260907/amcl_ready_fix/`。
`ready_evidence.json` 保存前一轮结果；`ready_final_evidence.json` 保存最终配置结果。
静止时 AMCL pose 的时间戳不持续变化可以正常；应结合新扫描和 map→odom，
必要时用无运动更新核实，不要单凭旧 pose 时间戳判断断流。

## 调度及换地图

十车五个大阶段：排队直行→双列转换→双列直行→两组V转换→V直行停止。
每辆车有完整过渡路径，槽位按几何候选自由分配，不固定leader/编号优先级。
冲突只锁对应局部路段；前车实际清空该段即可放行后车，不必等前车到最终槽位。
无冲突的已许可前缀并发，原拐点与必要冲突等待点仍可能停车。
依赖在执行前固定为无环顺序，不在行驶中任意换位或反转顺序；不是全局最优求解器。
普通落位不重复全队0.8s停车+AMCL采样，最终点需位置/速度符合门槛。
大阶段结束仍保留六车的定位验收。TEB 短暂超过8cm走廊时只暂停并局部重发该车目标；
超过10cm持续2秒、车间距不足、进度回退或超时才停止全队；
没有加入独立 safety guard，也不把 formation_line 当雷达避障器。

换地图：新增 maps/<map_id> 的 YAML/PGM及 profile，不能覆盖旧图。提供目的地、初始点、
允许的编队顺序和换队区锚点，编辑 planning.yaml 后执行：

```bash
python3 scripts/fleet_vendor_omni/plan_profile.py \
  config/fleet_vendor_omni/profiles/fleet10_same_map/planning.yaml \
  --output config/fleet_vendor_omni/profiles/fleet10_same_map/mission.yaml
python3 -m unittest discover -s test -v
```

新地图改为相应 profile 路径。生成器检查已知空闲区、槽位间距、路线与停放车辆、
路径冲突；不可行时报告具体阶段，不持续重发。当前模板限定十车两组，可配置先V后双列；
不从地图自动猜目的地，也不承诺任意狭窄地图可容纳五车V。应先调整换队锚点/间距，
再进行一次有界完整仿真，最后实车。地图更换要同时更新 profile 与 planning 的 map_yaml。
当前直行模板沿 map+x、车头0°；其他航向的地图需先设计路径方向/车头方向独立参数，
不能仅旋转目标点就声称已支持。当前已验证的编队顺序为本页五阶段，其余顺序需另行仿真。

## 历史：短许可段修复验收（2026-09-07）

源码和VM已同步。共享转换逻辑增加2.5cm入轨门槛、25cm最短TEB目标、
全队空闲时用认证短段向量控制解锁；详见 `PROGRESS_GATED_TRANSITIONS.md`。
底层、AMCL/EKF、TEB速度及普通直行速度未改。重新启动上层加载，无须因本次代码重启底层。

最终 `transition_entry_fix/sim_final` 在独立 localhost:11321 验证：
直接从双列转换入口开始，十车沿x均落后6cm，随后连续执行四个剩余阶段，SUCCEEDED。
这不是重跑名义五阶段，也不是实车验收；原五阶段 mission 初始点保持不变。

| 指标 | 双列转换 | V形转换 |
|---|---:|---:|
| 含入轨耗时 | 44.606s | 100.456s |
| 最大路径偏差（含入口） | 0.0600m | 0.0496m |
| 最小车心间距 | 0.5753m | 0.5198m |
| 入轨后同时运动峰值 | 6 | 6 |
| 最短TEB目标距离 | 0.2564m | 0.2533m |
| 入轨后路径进度最大回退 | 0.0000m | 0.0000m |

碰撞记录为空、无死锁、十车最终位置误差4.46～4.88cm。局部costmap、更新、局部路径、
目标、速度与仿真真值均已录制；10项相关契约测试通过。
当前策略偏保守，不能把耗时改善作为本次结论，也不能保证任意新地图无死锁。
短段控制未验证真实低速轮死区/制动与网络延迟；实车仍需先READY，再由用户启动完整任务。

## 最新：逐车短段/TEB并发验收（2026-09-08）

共享supervisor已去掉“短段等全队TEB空闲”和逐个阻塞执行：两侧有许可且路径不冲突
即可并发。短段只控制本车；继续保留2.5cm入轨、25cm最短TEB目标、0.05m/s短段限速、
8cm走廊和0.42m实际车心距门槛。运行参数/地图/槽位分配/原路径均未变。
仿真与实车共用同一mission和代码，不需要生成第二份实车路径。

证据目录：包外 `experiment_records/20260908/mixed_transition/`。

| 指标 | 用户14:41完整仿真 | 本次完整五阶段仿真 |
|---|---:|---:|
| 双列转换 | 44.881s | 40.784s |
| V形转换 | 121.254s | 69.930s |
| V阶段Robot10首次出发 | 94.886s | 39.289s |
| 两组都未完成时，单侧连续不动最长时间 | 25.911s | 2.243s |
| V阶段入轨后运动峰值 | 6 | 7 |
| V阶段最小车心距 | 0.5285m | 0.5320m |

`sim_full_01`：五阶段SUCCEEDED，碰撞0，转换最大路线误差4.97cm，终点全部<5cm。
`sim_offset_pause_02`：6cm入口偏差，从双列转换开始的四阶段；V阶段Robot5实际停住
7.006s，其他车在该停顿期间继续运动；SUCCEEDED，转换最小车心距0.5271m，
最大路线误差6cm（含注入入口偏差），无碰撞/越界/死锁。两轮入轨后进度无>1cm回退。
9项转换/控制测试及4项实车仿真同源契约通过。两轮均录制十车局部costmap/路径/速度。

这是两次指定工况的理想底层仿真，不是实车保证或统计性能测试。运动间距实际低于
0.42m、路线越界、TF失效或同车控制权冲突仍应停止，不能屏蔽故障来“跑通”。
仿真停顿工具只在包外测试launch中注入，正式sim/real入口不含该功能。

原十车启动命令不变；重新启动上层加载新代码。底层IMU/EKF配置未修改，不必因本次
改动重启底层参数。实车先READY、再由用户确认环境/急停并启动完整任务。
