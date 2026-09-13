# 换地图与备用编队：快速使用约定

当前十车同地图窄 V 几何方案已获用户确认；最新并行变换调度见
[PROGRESS_GATED_TRANSITIONS.md](PROGRESS_GATED_TRANSITIONS.md)。
这里是**备用几何模板和任务顺序参考**，不是八套已跑通的实车程序。
不复制控制器、不增加 guard、不改变厂商 IMU/EKF/AMCL 链。

## 1. 下次交给模型什么

```text
地图ID：例如 lab_b
地图：lab_b.yaml + lab_b.pgm（保留 resolution、origin、原始来源）
车辆：10辆，IP/命名是否继续 .111… .120 → robot_1…robot_10
初始摆放：每车在新 map 坐标中的 x/y/yaw；不能照搬旧地图绝对坐标
任务：终点、必经点、禁行区/单向通行要求；没有目的地就不能确定任务
编队偏好：可选形状、允许拆成两组吗、是否指定 leader/分组
现场：地图中没体现的障碍、人行区域、可用换队空地
要求：先给路径/编队/分配预览，通过后只做一次有界完整仿真，再实车
```

只有地图时，模型可以先筛选可用通道/区域，不能自行猜测任务终点。
PGM须为当前工具支持的8位P5格式，地图origin旋转角须为0；不支持时先转换并核对坐标，
不能直接删改元数据假装已经对齐。优先沿用 `robot_1/map`，否则还要核对RViz固定坐标系。

## 2. 八种备用编队

唯一编队库：`config/fleet_vendor_omni/formation_catalog.yaml`。
所有点都是**车心相对坐标（米）**，默认行进轴为+x，锚点是前排中心。

| 名称 | 结构 | 适用场景 | 名义横向筛选宽度 |
|---|---|---|---:|
| single_file | 10车单列，纵距0.60m | 狭窄直通道 | 0.68m |
| double_column | 两列各5车，列距0.60m | 常规走廊 | 1.28m |
| staggered_columns | 两列错开0.30m，列距0.70m | 较宽走廊，错列布置 | 1.38m |
| two_rows | 两排各5车，横距0.60m | 宽阔起步/停靠区域 | 3.08m |
| narrow_v_pairs | 两组1+2+2，翼宽0.56/0.60m | 两条较窄分流路线 | 每组1.28m |
| wide_v_pairs | 两组1+2+2，翼宽0.90/1.80m | 两条宽路线、明显V形 | 每组2.48m |
| diamond_pairs | 每组四顶点＋中心车 | 两处开阔终点 | 每组1.88m |
| triangle_10 | 一组1+2+3+4 | 单个宽阔终点 | 2.48m |

这里的宽度仅是 `车心横向跨度 + 2×0.34m`，用于早期筛选，**不是安全通行保证**。
栅格、定位/控制误差、转弯扫掠及动态障碍需要额外考虑，完整路线必须重新检查。
双组队形要逐组检查各自通道和分流/汇合区，不要求两组之间的区域全部空闲。
队形旋转后应重新计算其世界坐标包围范围，不能继续套用表中宽度。

还要检查长度：单列车心纵跨度5.40m，加规划留距后约6.08m；通道足够窄不代表入口空地
能完成十车收拢。菱形中心槽位可能被先到车辆包围，也不能只验证最终点不碰撞。
窄双V库的两组中心默认相距3m，**不等于当前地图已验证的两组绝对位置**；原任务保留原配置。

## 3. 六套任务顺序参考

| 方案ID | 编队顺序 | 用途 |
|---|---|---|
| standard_split | 两排 → 双列 → 窄双V | 接近当前流程 |
| narrow_then_open | 双列 → 单列 → 双列 → 十车三角 | 中途窄口、终点开阔 |
| open_then_narrow | 十车三角 → 双列 → 单列 | 先展示/展开，再进入窄通道 |
| wide_split | 两排 → 错列 → 宽双V | 两条宽通道分流 |
| dual_station | 双列 → 双菱形 | 两处开阔停靠点 |
| compact_arrival | 单列 → 双列 → 两排 | 通过狭窄区域后整齐停车 |

顺序也存放在编队库 `schemes` 中，**仅为人工/模型选型参考，不会自动变成可执行任务**。
每次变换都要指定地图锚点，在变换之间按实际距离插入前进阶段。无需机械地使用整套方案；
地图不需要三角形就不必硬加三角形，减少无用变换通常更省时间。

## 4. 新地图只增加一个 profile

```text
maps/lab_b/
  lab_b.yaml
  lab_b.pgm
config/fleet_vendor_omni/profiles/lab_b_delivery/
  profile.yaml       引用新地图、生成任务、运行参数及RViz
  planning.yaml      唯一可编辑的初始点、锚点、阶段顺序
  mission.yaml       生成物，不手改
  runtime.bash       仅当名单/参数不同才增加；否则引用已有十车表
```

`config/fleet_vendor_omni/templates/new_map_planning.yaml` 是可复制的起步模板。
它有意保留空初始点和null锚点，未填写时不能运行；不要拿它覆盖当前跑通 profile。

新 profile.yaml 示例（lab_b及任务名是示例，必须换成真实路径）：

```yaml
map_yaml: maps/lab_b/lab_b.yaml
mission: config/fleet_vendor_omni/profiles/lab_b_delivery/mission.yaml
runtime: config/fleet_vendor_omni/profiles/fleet10_same_map/runtime.bash
rviz: rviz/fleet_vendor_omni_ten.rviz
```

复用运行参数意味着修改该表会同时影响所有引用它的任务；若新实验要独立调参，则单独
建立本profile的runtime。不要复制 supervisor、launch、TEB或整个厂家包。

## 5. 一段配置如何生成任务分配

在 planning.yaml 的 phases 中选择编队、指定锚点：

```yaml
- name: FORM_COLUMNS_BEFORE_CORRIDOR
  mode: transition
  layout: template
  formation: double_column
  anchor: [4.0, 0.0]       # 仅演示格式，必须在新地图重新标定
  rotation_deg: 0.0        # 旋转槽位，不旋转车头
  scale: 1.0              # 同比缩放槽位；不能缩小车体/安全留距
  timeout: 300.0
- name: CROSS_CORRIDOR
  mode: formation_line
  layout: translate
  delta: [1.5, 0.0]
  timeout: 100.0
```

默认按**车到槽位的直线距离平方和最小**分配，再用已有A*检查地图/停放车辆并生成路径。
这是初始分配，不是综合障碍路径、时间和拥堵的全局最优调度，也不会自动穷举所有分配。
路径不通时，检查分组/槽位分配，再调整锚点或编队；禁止无限重发失败目标。

需要固定leader或固定分流组时，在模板阶段加 `slot_robots`，按编队库slots顺序填满十车：

```yaml
slot_robots: [robot_1, robot_2, robot_3, robot_4, robot_5, robot_6, robot_7, robot_8, robot_9, robot_10]
```

例如双V前五个slots属于第一组，后五个属于第二组。此配置会完全指定分配；不能重复/漏车。
已经明确分组的分流路线优先使用它，避免仅按直线距离把车辆分到不可达的另一侧。
相交路径保留先后依赖，无冲突路径可并发；现有执行器仍在到位、速度符合要求后马上释放后继。

## 6. 换队点怎样选

1. 在膨胀后的地图上确定主路线、窄口、转角、分流/汇合点和终点。
2. 窄口之前选择可容纳**当前队形＋目标队形＋变换路径**的空地，不能到门口才开始收队。
3. 分流前明确两组名单；汇合时不要让先到者堵住后到者必经路径。
4. 排布新队形时先检查全车目标、停放车辆及通路，再确定先后/并发。
5. 完整检查前进扫掠，不能只有变换终点是空的；变宽必须等整队尾车离开窄口。
6. 几何不通过先移动换队区，其次选择更窄模板，再检查人工分配。
   不通过降低0.34m规划留距、缩小车体或随意放宽误差来伪造通过。

不同方向：当前车头仍保持map yaw=0。非+x平移可用 `layout: translate, mode: transition`，
由TEB逐车跟踪并按路径依赖调度；不保证保持刚性队形/同步进度。不能把非+x轨迹交给
当前formation_line，也不能直接要求车辆原地大幅转头而忽略现有15°航向偏离限制。
真正需要“其他方向仍同步固定车头直行”时，再单独扩展路径方向参数并验证控制逻辑。

## 7. 最短操作流程

在源码目录准备地图与profile，规划器不需要ROS Master：

```bash
# 在VM的包目录中演示；若生成文件在VM试算，确认后必须回写F盘权威源码
cd /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi
python3 scripts/fleet_vendor_omni/plan_profile.py --list-formations
python3 scripts/fleet_vendor_omni/plan_profile.py \
  config/fleet_vendor_omni/profiles/lab_b_delivery/planning.yaml \
  --output config/fleet_vendor_omni/profiles/lab_b_delivery/mission.yaml
```

Windows开发时也可用有PyYAML/numpy的Python3运行同一命令。编队库/模板使用Python3生成，
ROS节点继续用现有Python2；不要为此更换ROS、Gazebo或厂家底层环境。
任务生成成功只证明离线几何检查通过，不是仿真通过。查看全部生成目标、路径和after依赖。

预览无误后，用公共启动脚本选择新profile（不要用会重新选回旧profile的十车快捷上层入口）：

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/jetauto_real_dev_ws/devel/setup.bash
cd "$(rospack find jetauto_multi)"
export FLEET_PROFILE="$PWD/config/fleet_vendor_omni/profiles/lab_b_delivery/profile.yaml"
unset FLEET_RUNTIME_CONFIG   # 由profile选参数，不遗留前次试验的覆盖表
bash scripts/fleet_admin/start_vendor_omni_sim.sh \
  motion_enabled:=true allow_full_run:=true record_data:=true gui:=true
```

另一个终端加载ROS环境、指定本机仿真Master，确认READY后开始：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
rostopic echo -n 1 /fleet_vendor_omni/state_json
rosservice call /fleet_vendor_omni/start_all '{}'
```

一次有界完整仿真通过后，用同一profile做实车；名单不变时沿用现有底层十车命令。
如runtime也变更，先显式指定新 `FLEET_RUNTIME_CONFIG`，使用公共
`start_fleet_bottom.sh` / `check_fleet_bottom.sh`，避免十车快捷入口选回原表。
实车上层使用 `start_vendor_omni.sh` 和新 FLEET_PROFILE，先禁用运动检查READY。
详细实车启动和停车流程见 `FLEET10_RUNBOOK.md`。

每次保存地图版本、planning/mission/runtime快照、bag、轨迹、阶段耗时、最小间距、
终点误差和失败原因。失败先找一条证据支持的原因，修改后才重跑；不把所有备用方案
后台反复跑一遍。测试图不得当作新地图验收记录。

## 本次交付边界

新增8个槽位模板、6种顺序参考和一个未填坐标的起步模板；5项离线检查通过：
包括模板间距、旋转/指定分配、无效输入、与既有路径规划器的接口及当前任务回归一致。
没有执行八套完整ROS仿真，没有新的实车实验。除当前同地图窄V任务外，
备用模板均未获地图级/动态仿真验收，不自动接入默认launch。
