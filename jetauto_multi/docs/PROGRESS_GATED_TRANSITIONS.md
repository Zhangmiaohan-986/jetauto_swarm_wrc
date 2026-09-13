# 十车局部路段放行（2026-09-07）

只替换变换段调度；仿真/实车共用同一个 supervisor。实车须用户验收，
仿真成功不证明真实电机、制动、定位误差和通信延迟满足运动包络。

## 代码与数据

- `transition_gates.py`：无 ROS 的槽位候选、路段依赖编译、进度投影和契约校验。
- `plan_profile.py`：生成十车自由槽位分配，输出路径、里程、依赖和候选评分。
- `supervisor.py::run_transition`：唯一切换入口；新分支为 `run_gated_transition`。
- `fleet_geometry.py`：继续复用地图膨胀/A*/旧路线生成；分配并列值固定精度，保证跨平台可复现。
- `planning.yaml`：唯一任务源；`mission.yaml` 是生成物，不能双向手改。

## 算法边界

1. 比较直线距离分配、地图瓶颈距离分配及原可行种子，共至多三种分配。
2. 每种分配最多尝试 shortest/front_first/rear_first 三种几何种子顺序。
   没有固定 robot 编号/leader 优先；最终选择依赖关键路径较短的可行候选。
   这是有界启发式（最多九个候选），不是完整 CBS，也不承诺全局时间最优。
3. 原整条路线依赖被编译成动作前缀依赖。每个冲突动作必须等相关前车
   清空冲突路段及 release_margin；涉及前车终点的依赖必须等前车确认停车。
4. 中间 gate_step 采样只用于保守几何检测，连续可行前缀合并成目标，
   不按每个采样点停车。原路线拐点仍由 TEB 执行，必要等待点可以停车。
5. 放行依赖实际 map 位姿进度，不依赖预计到点秒数。发目标前还检查
   其他车辆已承诺路线及实际停车位置。延迟不会自动释放被占路段。
6. 首次出发前检查起点；执行时检查新鲜 odom、可用 map TF、路径偏差、
   进度回退、航向偏离及原车间距。8cm内正常；8–10cm只暂停该车、保留其路线预留，
   等待1秒并在冲突检查通过后重发本车TEB目标；超过10cm持续2秒才按路线偏离停止全队。
   实际车心距不足、进度回退及传感器/TF失效仍是硬故障，不进入该容错分支。
7. 不在行驶中交换槽位或翻转已执行的依赖；分配在该阶段执行前生成。

TEB 只接收已许可的当前前缀目标。它仍是独立单车局部规划器，不能单凭
预先生成的折线或时刻表证明严格轨迹跟踪。车队 costmap 的队友过滤保持原样，
不能取消本调度的预留检查后依靠 TEB 自行解决车队冲突。

新分支另外发布已许可前缀的局部 `nav_msgs/Path` 到每车 TEB `via_points`，
不只发送终点。只取车前0.05～0.80m已许可的点，随着实际进度裁剪。
Melodic接口忽略消息frame字段，因此代码显式将map坐标变换为各车odom坐标后发布；
没有重定义或修改map→odom→base TF。
该profile启动时设置global_plan_viapoint_sep=-0.1和via_points_ordered=false，
使用Melodic动态配置允许的负值下限，避免节点把-1钳位成-0.1后READY不匹配。
每个许可前缀只跨当前直线段，用最近点匹配；不使用ordered模式的index+2关联，
避免低速短TEB上的密集引导点挤到少数轨迹节点。
weight_viapoint由十车runtime.bash的FLEET_TRANSITION_VIA_WEIGHT=10提供；
FLEET_TRANSITION_MOTION_WEIGHT=20统一提供x/y/yaw速度和加速度的优化权重。
这是优化惩罚，不是速度/加速度限幅；原速度与加速度限幅未改变。
READY检查接口参数及权重，缺失订阅/TF时停止。
六车/whole_route分支仍使用原全局路径via-points。到达/停止时清空自定义引导。
Via-points是软优化约束；8cm是单车恢复入口，10cm/2秒是持续偏差退出条件，
不能声称严格硬约束保证。

VM安装TEB为0.8.4，逐轴限幅，不支持新版本的use_proportional_saturation。
厂商配置weight_max_vel_y/weight_acc_lim_y=0，不能只提高引导点权重。
sim_06记录Robot9路线约(-3,+1)，指令却连续(-0.10,+0.10)，随后偏差超限；
因此新调度补齐双轴优化约束，而非修改厂商快照、安装另一版TEB或增加cmd_vel节点。
该证据解释本轮新调度适配问题，不推翻历史Robot2动态IMU bias消融结论。

对应已安装版本源码：[速度限幅/自定义引导点](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.8.4/src/teb_local_planner_ros.cpp)、
[引导点关联与优化边](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.8.4/src/optimal_planner.cpp)。

## 参数与回退

`config/fleet_vendor_omni/profiles/fleet10_same_map/planning.yaml`：

```yaml
planning:
  transition_scheduler: progress_gates  # whole_route 为旧行为
  gate_step: 0.10                       # 依赖几何采样上限，m
  release_margin: 0.10                  # 前车离开冲突边界后的余量，m
  wall_radius: 0.34
  parked_clearance: 0.55
  route_reservation: 0.65
  tracking_error: 0.08
  tracking_error_hard: 0.10
  tracking_error_hold: 2.0
  tracking_recovery_wait: 1.0
  transition_sensor_recovery_timeout: 8.0
  teb_abort_max_retries: 3
  teb_abort_retry_delay: 1.5
  teb_abort_timeout: 18.0
  connector_recheck_period: 0.20
  connector_clear_stable: 0.50
  connector_block_timeout: 20.0
  connector_motion_timeout: 20.0
  dependency_deadlock_timeout: 8.0
```

速度仍来自 runtime.bash：TEB vx/vy=0.10m/s，wz=0.08rad/s；
formation_line=0.08m/s。IMU bias=false、EKF绝对yaw=false保持不变。
不得单独调低净距/提高速度并假定旧预留模型仍然成立。
名义停车点的规划留距仍为0.55m；实际停车中心已由TF测得时，在线检查使用
`runtime_teammate_min_distance + tracking_error = 0.42 + 0.08 = 0.50m`。
这保留正常移动路径误差包络，不重复加名义停车位置误差。8–10cm只作为有时限的
单车恢复区；恢复路线继续参与0.65m预留，其他冲突车辆等待，无冲突车辆继续。

停止上层后生成并验证（新算法或回退都需要）：

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd "$(rospack find jetauto_multi)"
python3 scripts/fleet_vendor_omni/plan_profile.py \
  config/fleet_vendor_omni/profiles/fleet10_same_map/planning.yaml \
  --output config/fleet_vendor_omni/profiles/fleet10_same_map/mission.yaml
python3 -m unittest discover -s test -v
```

正式修改以 F 盘为准，再同步 VM；上述 VM 生成命令只用于验证/明确回退，
不要留下未回写 F 盘的另一份权威配置。旧整路径分支保留共享函数，不复制旧包。

## 使用

仿真（独立 localhost:11321；不启动实车底层）：

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd "$(rospack find jetauto_multi)"
bash scripts/fleet_admin/start_ten_vendor_omni.sh sim \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true
```

另一个 VM 终端在 READY 后开始完整任务：

```bash
source /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi/scripts/fleet_admin/ten_environment.bash sim
rostopic echo -n 1 /fleet_vendor_omni/state_json
rosservice call /fleet_vendor_omni/start_all '{}'
```

实车仍使用已有十车脚本，不改变 .111… .120 与 robot_1…robot_10 映射。
先停止仿真；十车 Master 地址以 runtime.bash 为准（当前 VM .1.104），
实车链路使用 .1.104 而非 NAT 地址；不要另起 .111 Master。
底层启动、READY 检查和完整运行见 `FLEET10_RUNBOOK.md`。
先 motion_enabled:=false 检查，用户确认环境/急停后再重启为 true 并 start_all。

## 记录与验证

- `TRANSITION_RELEASED`：车辆第一次获许可。
- `TRANSITION_GATE_GRANTED`：执行前缀延长，不代表必须在每个里程点停车。
- `TRANSITION_PROGRESS`：实际进度、依赖/已承诺路径/停车位置等待原因、并发数。
- `TRANSITION_ARRIVED`：最终槽位已达到且 cmd/odom 停车门槛通过。
- 每个大阶段仍做原有定位与停车 checkpoint，未逐槽位增加 checkpoint。

`active_count` 是活动动作数，不等同实际运动数；应结合 bag 中 cmd/位姿检查。
`critical_path_distance` 是候选排序的距离等效估计，不是实测秒数。
`test_transition_gates.py` 覆盖交叉、首车意外停顿7秒、依赖完整性和无环校验；
该测试是无 ROS 的理想执行，不替代完整 TEB/实车试验。

本轮证据放在包外 `experiment_records/20260907/transition_gates/`。

最终验收 `sim_07`：SUCCEEDED，五阶段141.831s；双列转换29.440s、V转换51.466s，
两段活动动作峰值/实际运动峰值均7。对应最小车心距离0.5657/0.5291m，
最大路线偏差0.0488/0.0497m，接触0次。终点误差4.83～4.99cm，满足原5cm容差。
26项契约测试通过（含首车延迟的理想进度依赖测试），没有实车测试。
对照历史整路段版本及完整启动顺序见 `FLEET10_RUNBOOK.md`。

调试记录保留，不将失败尝试计作验收：sim_01修正ROS XML-RPC不能序列化null；
sim_02双列通过后因重复计入停车误差而死锁；sim_03/04在V转换偏离预留路线；
sim_05发现动态配置负值钳位造成NOT_READY；sim_06发现仅提高via权重仍导致双轴饱和。
所有试验仅运行localhost:11321，异常取消目标并结束本次roslaunch，没有自动重发失败目标。

实车前必须确认真实路线偏差、定位跳变和停止距离满足包络。0.65m预留并不代表
任意速度/制动/延迟下的无碰撞证明；新障碍导致TEB绕出路线时本版停止，需要重新规划。
不自动翻转已执行依赖、不在线任意绕车，以免破坏已放行车辆的占用约定。

## 2026-09-07 实车短许可段修复

实车失败记录为 `to_columns_failure_232811`：Robot10 入段约6cm误差，
首个许可目标约9.2cm；全局路径方向正确，但TEB局部输出先向反方向，
路线偏差达到8.99cm触发原8cm门槛。不是以 move_base 取消结果反推规划失败。

双列和V转换统一使用以下交接，不修改 AMCL/EKF、底层或TEB速度参数：

1. `planning.yaml: planning.entry_tolerance=0.025`：检查实际位置到下一段起点，
   超过2.5cm先入轨；超过原允许的13cm直接停止，不自动做长距离纠偏。
   入轨后首次TEB派发再检查，等待期间发生漂移则停止。
2. `planning.min_teb_distance=0.25`：短于25cm的许可目标不交给TEB，等待
   依赖清空后合并为较长前缀；不能越过未许可的冲突门或原路径拐点。
3. 当时版本在全队TEB空闲时执行一个短段解锁（此全队屏障已被下面9月8日版本替换）。
   复用 `formation_line` 固定车头向量控制，短段速度上限0.05m/s、径向到点2.5cm，
   不让它与活动TEB同时发布速度。保守串行短段是有意限制，不宣称最优调度。
4. 短段每轮检查静态地图空闲（膨胀半径0.34m）、队友间距、新鲜雷达/TF以及
   原始短段8cm走廊；受阻、盲区、数据过期即停车报告，不用向量控制绕障。
   这是短段执行条件，不是新增独立 safety guard；普通直行段未增加此分支。

`mission.yaml` 是生成物；修改上述参数须重新运行 `plan_profile.py`。
新事件 `TRANSITION_CONNECTOR_START/END` 区分 `entry_alignment` 与 `short_gate`。
`TRANSITION_CONNECTOR_BLOCKED`、`TRANSITION_ENTRY_DRIFT` 均需排查后再运行，不能自动重试。

需要诊断 costmap 时，启动前设置 `export FLEET_RECORD_COSTMAPS=true`，并使用
`record_data:=true`；额外录制十车局部costmap及更新、move_base目标，原局部路径与速度
仍保留。常规默认不增加这部分带宽，诊断后 `unset FLEET_RECORD_COSTMAPS`。

`fleet10_same_map/runtime.bash` 当前为本轮实车消融默认打开该开关。它仍保持
`FLEET_RECORD_COMPACT=true`，因此不会重新录制十路原始 scan；记录内容包含每车
`local_costmap/costmap[_updates]`、`GlobalPlanner/plan`、`TebLocalPlannerROS/local_plan`、
move_base goal、via_points 与最终 cmd_vel。完成根因判定后可显式设置
`FLEET_RECORD_COSTMAPS=false` 恢复普通精简录包。

每次派发新 TEB 目标后，supervisor 将首个有效平移命令从 base 坐标旋转到 map 坐标，
与当前已许可路线切向量计算余弦。余弦小于0表示命令含反向分量：只取消该车，保留
路线预留，等待停稳并通过原有地图/scan/TF/队友检查后，用固定车头 connector 前进到
下一条约0.12m的安全路线节点，再尝试TEB。8cm软越界采用同一回归链；回归阻塞时只
暂停冲突车辆，真实间距、TF/定位、回归超时及重复错误方向仍按硬故障停止。
本次仿真与指标在包外 `experiment_records/20260907/transition_entry_fix/`。

## 2026-09-08 短段/TEB混合并发（当前实现）

同一个 `run_gated_transition` 通过 `execute_gated_transition` 统一推进两种控制器，
没有新节点、线程调度器、独立guard或第二份实车任务。槽位/折线路径/原依赖不变。

- 小于25cm的已许可前缀最多等0.3s尝试合并；本车动作结束、odom/cmd停稳后，
  注册本车direct publisher，预留明确的直线短段并等待连接。其他TEB不受取消。
- `tick_connector` 每轮只更新一个控制步，复用 `vector_command` 的固定车头向量公式；
  限速0.05m/s、原加速度/航向限幅、2.5cm径向到点继续保留。
- 达点输出零速度且停稳0.2s，注销本车direct publisher后，才允许派发下一TEB目标。
  同车若意外出现活动TEB，报告 `TRANSITION_COMMAND_OWNER_CONFLICT` 停止。
- 静止队友用0.42+0.08=0.50m，不再次加名义停车误差；运动队友（含其他短段）
  用0.65m剩余路径预留。参数校验要求预留至少覆盖0.42+双方跟踪误差+0.04m。
  这是一致性下界，不是对未知实车延迟/制动的证明。
- 未放行的冲突不作为运动故障；已放行短段被新障碍/预留/扫描检查挡住时，
  只向本车输出零、保留其预留、继续检查其他车辆。单短段仍有15s总时限，
  无法恢复时停止报告；真实0.42m间距不足/8cm越界等硬故障仍全队停止。
- `TRANSITION_PROGRESS.controllers` 区分idle/teb/connector；短段START记录本车、
  目标及active_teb；WAIT/RESUMED明确阻挡原因。不以action数量替代实测运动并发。

本次代码/验收记录：包外 `experiment_records/20260908/mixed_transition/`。
实车与仿真共享同一source/profile；下一次重启上层生效，底层和速度表无需因本次重启。

## 2026-09-10 单车局部自动恢复（当前实现）

恢复逻辑继续放在同一个 `execute_gated_transition` 状态机内，没有增加节点、guard、
后台任务或第二套调度器。双列与V形转换共用以下行为：

- 单车 TEB 返回 `ABORTED` 时，只停止该车并保留同一目标路线。确认 scan、map/odom TF、
  odom/scan TF 与车辆停止连续稳定0.5s后，延迟1.5s重发当前目标；最多3次且总计不超过18s。
  无冲突车辆继续执行。恢复成功发布 `TRANSITION_TEB_RECOVERED`；耗尽预算后才进入
  `GOAL_HOLD`，不会静默跳过目标。
- 短连接段每0.2s检查一次。尚未获准执行的候选段不占预留，避免多个入口互相等待；
  连续清空0.5s后才准入。控制器一旦开始，临时障碍、路线冲突或扫描条件不足只停车本车，
  并继续保留该段预留；持续20s才报告失败。运动计时不包含受阻等待时间。
- 短连接在线预留为 `0.42 + 2*0.08 = 0.58m`，不超过普通TEB路线预留0.65m。
  该值覆盖两车各8cm跟踪包络，并允许名义0.60m并行车道，不再把合法并行短段误判为冲突。
- 8cm以内正常，8–10cm进入本车路线恢复，超过10cm持续2s才是路线持续越界。
  TF/定位长期丢失、真实车心距不足、控制权冲突、进度回退及重试耗尽仍停止全队。
- 诊断事件新增 `TRANSITION_TEB_RETRY_WAIT/TRANSITION_TEB_RETRY/TRANSITION_TEB_RECOVERED`
  与 `TRANSITION_CONNECTOR_WAIT/TRANSITION_CONNECTOR_RESUMED`，用于区分暂态恢复和硬故障。

验证：13项过渡状态机测试和13项项目契约测试通过。十车无界面完整仿真只调用一次
`start_all`，五阶段最终 `SUCCEEDED`，总任务约165.6s；2056个truth采样的最小车心距
0.4714m，高于0.42m硬门槛，接触0次。仿真名义路径没有自然产生TEB `ABORTED`，
所以ABORTED分支由有界状态机单元测试覆盖，仍需实车包验证实际恢复次数和停止距离。
