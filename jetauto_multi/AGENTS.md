# jetauto_multi 开发约束

本文件是本包内代码模型和开发者的最小工作协议。目标是保持六车主线可调试，
避免复制包、混用旧版本和无意义的后台测试。

## 1. 权威边界

- F 盘 `F:\\swarm_src\\src\\swarm_wrc\\jetauto_swarm\\jetauto_multi` 是唯一源码真相。
- VM `/home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi` 只是构建/运行副本。
- 实车 `/home/jetauto/jetauto_ws` 是厂商底层；除非用户明确授权，不修改它。
- `archive/`、`historical_not_for_deployment/` 和包外旧目录只读，不重新接入主线。
- 当前主线不加入独立 safety guard、role router、measured-anchor 或旧 mission manager。

## 2. 目录规则

活动包只保留一套实现：

```text
config/fleet_vendor_omni/       参数表、任务、TEB 覆盖
launch/                         real.launch、sim.launch 公共入口
launch/fleet_vendor_omni/       共享导航 include；薄入口由 launch_fleet.py 装配
scripts/fleet_admin/            名单驱动的底层/上层运维，六/十车只有薄入口
scripts/fleet_vendor_omni/      supervisor、sim_adapter、record
maps/<map_id>/                  地图 yaml、pgm 及同一地图的元数据
vendor/jetauto_navigation/      不直接修改的厂商导航快照
test/                           最小契约测试
docs/                           设计和运行说明
```

- 不创建 `v2_final`、`new2`、`copy`、`backup` 等并行活动包。
- 实验 bag、日志、截图和分析结果放在包外 `experiment_records/<日期>/<实验名>/`。
- 新地图必须使用唯一 `map_id`；不要覆盖旧 PGM/YAML。
- 新任务用 profile，不复制整个包。profile 至少包含 map、初始点、任务阶段和运行参数。
- 备用槽位只放 `formation_catalog.yaml`；顺序参考不是已验证任务。新地图流程见
  `docs/NEW_MAP_FORMATION_WORKFLOW.md`。不得覆盖当前已确认的窄V profile 来试备用方案。

## 3. 参数唯一归属

每个参数只能有一个可编辑来源：

| 参数类别 | 唯一来源 |
|---|---|
| IP、逻辑编号、IMU/EKF、TEB/AMCL、formation_line 限幅、十车路线跟踪权重 | 当前 profile 引用的 `runtime.bash`（六车为 `six_robot_runtime.bash`） |
| 十车初始位姿、阶段顺序、槽位和路径依赖 | profile 的 `planning.yaml`；`mission.yaml` 为生成物，不能双向手改 |
| 六车初始位姿、阶段顺序、目标点 | `six_robot_demo.yaml` |
| 共用 TEB 默认值 | `teb_defaults.yaml` |
| 队友扫描过滤频率、扫描年龄上限、遮挡几何 | `amcl_scan_filter.yaml`；TF 查询固定非阻塞 |
| 每次实验的 AMCL 初始协方差、周期缓存开关 | `amcl_initialization.yaml`；方差必须有限且为正，每次启动覆盖旧缓存；`save_pose_rate=-1` 禁用周期缓存，不可为0；初始位姿仍来自 mission |
| TF 通信域 | 十车 `runtime.bash` 的 `FLEET_LOCAL_TF=true`：底层按 `/robot_N/tf[_static]` 隔离；profile 的 `vm_tf_relay` 使 VM 汇总到 `/fleet/tf[_static]`；frame/所有权不变 |
| 十车 Master/VM 地址 | 十车 `runtime.bash`；默认 Master 在 VM .104，显式环境变量可覆盖；六车仍 .111，不改 robot_1/map |
| VM 传感器接收与录包 | profile 的 `vm_sensor_relay`、`record_bottom_raw`；十车 scan/odom/imu 单路汇聚，默认不录底层诊断原始流 |
| 车体 footprint | `fleet_footprint_overlay.yaml` |
| 厂商底层默认值 | `vendor/`，只读 |

- launch 只做参数传递，不再复制一套隐藏默认值。
- 十车开发说明/验收结果见 `docs/FLEET10_RUNBOOK.md`。不能把几何预检、理想底层仿真或启动 READY 等同实车验收。
- 修改底层 IMU/EKF 环境变量后必须重新启动对应底层进程；不假设运行中的节点会热更新。
- 修改地图时必须同步检查 map frame、resolution、origin、初始点和所有目标点。

## 4. 10 车或新地图的扩展原则

### 10 车同地图

不要把六车 launch 复制成第二个包。正确顺序是：

1. 新建 `config/fleet_vendor_omni/profiles/fleet10_same_map/`。
2. 为 10 辆车提供完整 runtime 表和 mission 文件。
3. 将 launch/supervisor 中当前六车硬编码改成由 `robot_order`/`robot_count` 驱动。
4. 扩展 RViz、record、health probe 和契约测试到 10 车。
5. 先静态检查和仿真；未完成第 3 步前，不得只增加两行参数就声称支持 10 车。

### 新地图

1. 建 `maps/<map_id>/<map_id>.yaml` 和 `<map_id>.pgm`，保留原始来源信息。
2. 建对应 profile，显式写 `map_yaml`、初始位姿和阶段目标。
3. 运行地图解析、目标占用、车体扫掠和 TF/frame 检查。
4. 在仿真中跑完整任务并保存 bag；仿真通过后再做实车。
5. 新地图只改变地图和任务 profile，不复制 supervisor、TEB 或底层。

仿真不能证明真实电机死区、轮胎打滑、IMU bias 或网络延迟；它只能证明话题、TF、
任务几何和上层控制契约。

## 5. 固定验证流程

### 代码/配置变更

1. 先说明变更文件、直接消费者和可能的参数类型/范围错误。
2. 运行一次 XML/YAML/Python 静态检查。
3. 运行与变更直接相关的最小契约测试。
4. 需要控制逻辑时，运行一次有界仿真；不自动启动实车。

### 实车变更

1. `start_six_bottom.sh` 启动底层。
2. `check_six_bottom.sh` 必须无 FAIL。
3. 上层先 `motion_enabled:=false`，确认 supervisor 为 `READY`。
4. 用户确认人员、障碍物和实体急停就位后，才运行运动任务。
5. 记录 bag、state_json、event_json 和启动日志。

### 参数只改一个值时

只回答并检查：

- 谁读取它；
- 是否需要重启哪个进程；
- 是否与其他参数冲突；
- 是否会造成 launch、类型、范围或 preflight 错误；
- 最小的一次静态验证命令。

不自动做以下事情：

- 不后台循环 roslaunch、rosbag、unittest 或反复思考同一参数；
- 不因为改了 YAML 就重复完整六车实车实验；
- 不在用户未要求时修改相关代码、底层、AMCL/EKF/TF 或安全栈；
- 不把“测试尚未运行”描述成“已经验证”。

## 6. 当前控制边界

- 过渡段：厂商 `move_base + TEB`，使用 costmap 和障碍物扫描。
- 十车 `progress_gates` 在同一 supervisor 内按实际路段进度放行，向 TEB 发布
  已许可前缀的 via_points。必须先将 map 点变换到各车 odom，不能只改消息 frame。
  说明见 `docs/PROGRESS_GATED_TRANSITIONS.md`；六车/whole_route 保留原调度行为。
- 直线编队段：supervisor 的 `formation_line` 直接发布 `vx/vy/wz`。
- 十车转换先按 `planning.yaml` 的 `entry_tolerance` 入轨；不足
  `min_teb_distance` 的许可前缀最多等待0.3s合并，本车停稳后可用非阻塞
  固定车头向量控制，与无冲突的其他车TEB并发。不得恢复全队空闲屏障，
  不得为切换一辆车取消其他车目标。短段必须通过地图、所有已承诺路径、
  队友和新鲜雷达检查；禁止绕过失败检查。尚未获得动态净空的候选短段不占用
  路线预留，避免等待互锁；控制器开始执行后若临时受阻，停车并继续保留其路线预留。
  双列和 V 形转换共用此逻辑，不复制第二套调度器。
- 单车 TEB `ABORTED` 先进入本车有界恢复：保留当前目标路线，等待 scan/TF/停车
  连续稳定后延时重发；不得取消无冲突车辆。达到重试次数或总时限后才进入
  `GOAL_HOLD`。定位/TF长期失效、真实间距不足、持续越界和控制权冲突仍是硬故障。
- 每次新 TEB 目标的首个有效平移命令必须在 map 坐标中具有沿已许可路线向前的
  分量；反向命令立即取消，仅该车经现有短连接器回到下一安全路线节点后再试 TEB。
  路线偏差也复用该连接器回归，保留冲突预留；不得盲目重发同一 TEB 目标。
- 安全距离不重复相加：在线静止车采用实际中心距门槛+一车跟踪误差，
  运动双方采用route_reservation并校验覆盖双方跟踪误差。正常未放行冲突是等待，
  真实间距不足、越界、TF失效或控制权冲突仍按故障停止，不能屏蔽错误来通过验收。
- AMCL 只提供 map 定位，不负责电机控制。
- EKF 当前不融合 IMU 绝对 yaw，只保留 yaw rate；验证版本关闭动态 bias。
- 当前没有唯一 cmd_vel mux，也没有独立 safety guard；formation_line 不等同于雷达避障器。
- 任何新增安全策略必须用户明确授权，并单独设计、测试、记录，不能顺手混入参数修改。

## 7. 变更报告格式

每次工作结束只报告：

```text
变更：文件 + 参数/逻辑
影响：直接消费者、重启要求、潜在冲突
验证：实际执行的一次最小检查及结果
未执行：明确说明没有做重复测试或实车运动
下一步：只有用户需要决定的动作
```

完成条件不是“文件更多”，而是：单一来源、可追溯 profile、仿真通过、实车门槛
明确、失败时能定位到一条链路。
