# SimEnv 两层自主探索整改实施规格（Codex 版本）

> **用途**：本文件面向 Codex / 编程代理，作为直接修改仓库时的工程任务规格。  
> **工作目标**：从官方 SimEnv 仓库干净基线开始，先完成稳定的**单层房间搜索 + 红色球形危险源检测**，再在不重写单层核心逻辑的前提下扩展为**前两层连续自主探索**。  
> **优先级**：比赛合规 > 稳定完成 > 危险源召回率 > 实时性能 > 极限速度。  
> **重要原则**：不要把本任务重新实现成传统“全局最大 Frontier 覆盖探索”。房间内部允许使用受拓扑、净空和死胡同约束的**信息增益优先安全前沿策略**；走廊、房间外和楼层全局仍不启用普通 Frontier 目标选择。比赛目标仍是尽快访问可能含危险源的房间并可靠检测危险源。
> **Stage B 调整基线**：本版已吸收 `SimEnv_Stage_B_Adjustment_Plan_Anti_Overfitting.md` 和当前收尾优先级。当前先证明四个房间能够完整访问且危险物不漏检；时间优化后置。本次行为调整是将房内固定观察位和最近前沿优先替换为房间局部的拓扑约束信息增益优先探索，并保留走廊门洞发现、候选融合、进门/退出状态机、危险源检测和楼层完成条件。若调整方案与本文件旧表述冲突，以本文件当前版本为准；后续执行只维护本总方案，避免形成两套相互漂移的实施依据。
> **方案变更锁**：本次用户已明确授权同步修改本文件；本次调整完成后，除非用户再次明确要求，否则后续实现、测试、修复、重构或阶段推进仍不得擅自修改本文件。

---

## 0. 官方仓库、分支与工作边界

### 0.1 官方仓库

官方 Gitee：

```text
https://gitee.com/guoyulun/SimEnv
```

克隆：

```bash
git clone https://gitee.com/guoyulun/SimEnv.git
cd SimEnv
git checkout -b two-floor-navigation
```

### 0.2 开始编码前必须先读

先检查官方仓库当前版本，至少阅读：

```text
docs/competition-rules.md
docs/algorithm-interfaces.md
docs/sensors-and-topics.md
docs/evaluation.md
```

还需要检查：

```text
src/unitree_guide/
generated_building/team_scene_info.json
```

**如果代码、topic、service、文件名与本文档描述不一致，以当前官方仓库为准，不要凭记忆硬编码。**

### 0.3 Codex 工作协议

执行本方案时遵守以下约束：

1. **先检查再修改。** 修改节点、topic、frame、launch 或参数前，先在仓库中确认真实名称。
2. **一次只完成当前阶段。** 当前阶段验收未通过，不要提前堆叠下一阶段复杂功能。
3. **优先新增独立 `simnav_*` 包。** 除 locomotion policy 切换等确实需要的部分外，尽量不要侵入式重写官方环境代码。
4. **不要修改建筑生成逻辑来降低任务难度。** 不允许通过扩大门、删除电梯、固定危险源位置等方式“解决”导航问题。
5. **不要把两层硬编码进核心算法。** 测试目标先设两层，但 `FloorManager`、地图管理和任务状态必须可扩展到 N 层。
6. **所有任务超时、扫描时长、卡死判断优先使用 ROS simulated time**，不要用 wall-clock 时间替代 Gazebo 仿真时间。
7. **GUI 性能测试默认关闭 Gazebo GUI。** GUI 仅用于必要的可视化调试。
8. **性能问题先降高层频率、减少不必要点云处理，不降低 Unitree 官方底层控制周期。**
9. 每个阶段完成后输出：
   - 修改文件清单；
   - 启动命令；
   - 关键参数；
   - 测试结果；
   - 尚未解决的问题。
10. 每个阶段建议单独 Git commit，避免一次提交混入多个阶段。
11. **先分类失败，再修改代码。** 每次失败必须先归入定位、局部扫描、门洞证据、候选融合、状态机、运动执行或危险源检测中的一类。
12. **一次只验证一个核心假设。** 不在同一轮同时更换定位、重写 Detector、改状态机并增加 Recovery；否则无法判断改善来源。
13. **禁止单 seed 特化。** 一个 seed 的一次失败不足以增加新分支、扩大阈值或新增 Detector；同类失效必须有重复证据和日志支持。
14. **相同职责只有一个所有者。** Scan 与 Map 可以提供异构证据，但候选生命周期、走廊判断、恢复预算和结果写出分别只能有一个统一管理模块。

### 0.4 第一次交给 Codex 时的推荐起始指令

将本文件放在仓库根目录后，第一次可以直接要求：

```text
阅读本文件，并同时检查当前官方仓库 docs 与源码。
现在只执行“阶段 A：恢复官方基线并建立可测系统”。
不要实现阶段 B/C/D。
先检查真实 topic、service、frame、launch 和 policy 文件，再做最小必要修改。
阶段 A 验收结束后，按本文件“Codex 每阶段执行模板”汇报并停止，等待下一步指令。
```

后续阶段同理：**一次只授权一个阶段。**

### 0.5 总方案变更授权机制

本文件是工程范围、阶段边界和验收口径的受控基线，默认处于冻结状态。

修改本文件必须同时满足：

1. 用户在当前请求中明确点名 `SimEnv_two_floor_exploration_codex_spec.md` 或“方案总文件”；
2. 用户使用“修改、更新、合并、增删”等明确授权措辞；
3. 修改范围不超过用户本次明确授权的内容。

以下请求均**不构成**修改本文件的授权：

```text
执行某个阶段
按方案实现代码
修复测试或仿真问题
继续推进
同步一般文档
更新进度记录
重构实现
```

执行代理在获得授权时必须先复述本次允许修改的范围；修改完成后立即恢复冻结。若代码现状与本方案冲突，只能在实现汇报中列出差异，不得自行回写或“顺手修订”本文件。

---

# 1. 比赛合规红线

正式算法允许依赖官方公开接口和公开场景信息，例如：

```text
/cmd_vel
/scan
/livox/Pointcloud2
/livox/imu
/trunk_imu
/real_sense/rgb/image_raw
/real_sense/rgb/camera_info
/real_sense/depth/image_raw
/real_sense/depth/camera_info
/real_sense/depth/points
/set_door_state
/call_elevator
generated_building/team_scene_info.json
```

具体名称必须以当前官方 `docs/` 和实际 `rostopic/rosservice` 输出为准。

## 1.1 禁止依赖

正式算法数据链中禁止使用：

```text
/Odometry_gazebo
/ground_truth/*
results/danger_truth.json
generated_building/danger_truth.json
generated_building/layout_metadata.json
generated_building/building_config.json
generated_building/competition_scene.world
generated_building/scene_manifest.json
```

如果这些数据在调试时被人工查看，只能作为离线人工诊断，**不得进入节点输入、规划逻辑、目标生成或最终结果计算。**

## 1.2 任务目标

比赛危险源是红色球形目标，官方场景还存在红色方块和绿色球体干扰。

正式系统优化目标：

```text
优先访问房间
    ↓
快速完成房间观察
    ↓
检测红色球形危险源
    ↓
估计 world [x, y, z]
    ↓
多帧确认和去重
```

**不要以地图覆盖率作为完成条件。**

---

# 2. 目标总体架构

最终双层目标架构如下；Stage B 按 B-1、B-2 顺序逐步达到该结构，不要求在一轮修改中同时替换定位和行为层：

```text
Livox + IMU ──> LocalizationBackend ──> /simnav/odom + TF
                       │                         │
                       │                         ├──> LocalizationHealthMonitor
                       │                         │        GOOD/DEGRADED/BAD/STALE
                       │                         │
                       │                         └──> 当前楼层 2D Mapping
                       │                                  │
                       │                      ┌───────────┴───────────┐
                       │                      ▼                       ▼
                       │               Navigation Map        Exploration Map
                       │                                              │
                       │                                  MapDoorDetector
                       │                                       │ Evidence
Livox /scan_2d ──> CorridorEstimator ──> ScanDoorDetector     │
       │                    │                    │ Evidence     │
       │                    │                    └──────┬───────┘
       │                    │                           ▼
       │                    └────────────────────> DoorFusion
       │                                                │
       │                                                ▼
       │                                    DoorCandidateManager
       │                                                │
       └────────────────────────────────────> SingleFloor FSM
                                                        │
                                      CORRIDOR/ENTER/SCAN/EXIT
                                                        │
                                                      /cmd_vel

RealSense RGB-D ──> DangerDetector ──> world XYZ ──> ResultWriter

MissionManager
   │
   ├── ENTER_BUILDING
   ├── SINGLE_FLOOR_SEARCH
   ├── FLOOR_TRANSITION
   ├── SINGLE_FLOOR_SEARCH
   └── MISSION_COMPLETE
```

## 2.1 关键架构约束

- 定位后端对上层固定输出 `/simnav/odom`、`/simnav/world_pose` 和约定 TF；Stage B 的 B-1 可暂用 Hector 收敛行为架构，B-2 达到触发条件后再替换为 Livox + IMU LIO。
- 最终 LIO 从机器人出生后开始，跨楼层持续运行，不因切层重置。
- 二维地图按楼层管理；上楼梯期间当前楼层 2D mapper 暂停。
- Navigation Map 与 Exploration Map 分离。
- Exploration Map 是门洞与任务语义辅助地图，可以屏蔽入口外部、Gazebo 小缝、电梯内部和暂缓楼梯区域；Stage B 仅允许房间确认后在当前房间 ROI 内使用它生成受约束的局部前沿，不得用它进行走廊或楼层全局 Frontier 目标选择。
- 导航模块只负责“如何到某个 pose”；Mission/Explorer 决定“为什么去那里”。
- Stage B 的走廊、进门、扫描和退出由 `SingleFloor FSM` 直接控制 `/cmd_vel`；`move_base`/DWA 保留代码与配置，但降级到 Stage C 的楼梯 pre-pose、较长距离目标和必要辅助导航。
- `ScanDoorDetector` 与 `MapDoorDetector` 只产生 `DoorEvidence`，只有 `DoorCandidateManager` 有权创建和维护 `RoomCandidate`。
- `CorridorEstimator` 是所有走廊几何判断的唯一来源；`RecoveryManager` 是所有门/退出/反扫重试预算的唯一来源。
- 定位 `STALE` 必须停车，`BAD` 时禁止 Map 独立创建新 hypothesis；不得用 Explorer Recovery 掩盖定位异常。

---

# 3. 建议新增的软件结构

优先在 `src/` 下新增独立包，不要把所有逻辑塞进一个脚本。

建议结构：

```text
src/
├── unitree_guide/                 # 官方包；只做必要的 policy 管理改动
│
├── simnav_localization/
│   ├── launch/
│   ├── config/
│   └── src/
│       ├── localization_backend_adapter.*
│       ├── localization_health_monitor.*
│       └── livox_lio_adapter.*
│
├── simnav_mapping/
│   ├── launch/
│   ├── config/
│   └── src/
│       ├── floor_cloud_filter.*
│       ├── floor_grid_mapper.*
│       └── exploration_map_filter.*
│
├── simnav_navigation/
│   ├── launch/
│   └── config/
│       ├── global_costmap.yaml
│       ├── local_costmap.yaml
│       └── dwa.yaml
│
├── simnav_exploration/
│   └── src/
│       ├── corridor_estimator.*
│       ├── scan_door_detector.*
│       ├── map_door_detector.*
│       ├── door_fusion.*
│       ├── door_candidate_manager.*
│       ├── recovery_manager.*
│       └── single_floor_explorer.*
│
├── simnav_perception/
│   └── src/
│       ├── red_candidate_detector.*
│       ├── danger_3d_localizer.*
│       ├── danger_tracker.*
│       └── result_writer.*
│
└── simnav_mission/
    └── src/
        ├── mission_manager.*
        ├── floor_manager.*
        ├── door_manager.*
        ├── stair_manager.*
        └── locomotion_manager.*
```

语言建议：

- 高频点云、地图投影：C++ 优先；
- Mission FSM、房间队列、结果管理：Python 或 C++ 均可；
- 第一版不要为了统一语言而重写能正常工作的模块。

---

# 4. 阶段 A：恢复官方基线并建立可测系统

> **阶段目标**：得到一个“零探索算法修改”的、可重复运行的前两层测试基线，并确认后续所有算法使用的接口真实存在。

## A.1 必做事项

### A.1.1 干净启动

- 从官方仓库新分支开始。
- 不合并旧 Frontier 项目。
- 测试场景先配置为两层，但不要改成只能生成两层的代码。
- 关闭 GUI 运行一次完整环境。

### A.1.2 建立性能基线

记录至少：

```text
Gazebo real-time factor
CPU 占用
内存占用
启动时间
主要传感器实际频率
```

建议至少测：

```text
静止 60 仿真秒
平地运动 60 仿真秒
```

### A.1.3 验证公开接口

确认实际存在并记录真实名称：

```text
Livox 点云
Livox IMU
trunk IMU
RealSense RGB
RealSense depth
/cmd_vel
门控制 service
电梯 service
/clock
TF tree
```

不要仅根据本文档名称假设接口存在。

### A.1.4 检查官方 locomotion policy

确认 `unitree_guide` 中：

- 平地 policy 文件；
- 楼梯 policy 文件；
- 当前默认加载的是哪一个；
- policy 切换是否需要重新初始化节点。

为后续 `LocomotionManager` 做准备，但本阶段先不实现自动切换。

## A.2 本阶段禁止做

不要开始：

```text
自动探索
红球识别
楼梯自动切层
OctoMap
3D 全局规划
YOLO 门识别
RL exploration
```

## A.3 验收指标

阶段 A 只有在全部满足后才能结束：

- [ ] 官方两层场景能稳定启动；
- [ ] A1 能通过 `/cmd_vel` 正常平地运动；
- [ ] 所有计划使用的公开传感器可以正常接收；
- [ ] `/clock` 正常；
- [ ] 已记录基线 RTF / CPU / 内存；
- [ ] 没有使用 `/Odometry_gazebo` 或 `/ground_truth/*`；
- [ ] 已确认平地/楼梯 policy 实际文件和加载方式；
- [ ] 已建立新分支，旧探索代码未混入。

## A.4 阶段输出

Codex 完成阶段 A 后必须给出：

```text
1. 当前官方 commit / 分支
2. 两层场景启动命令
3. rostopic list 中实际使用的 topic
4. rosservice list 中实际使用的 service
5. baseline RTF / CPU / RAM
6. 平地/楼梯 policy 文件路径
7. 当前发现的官方代码问题
```

---

# 5. 阶段 B：第一层四房功能闭环

> **当前阶段目标**：机器人进入第一层后，完整访问四个真实小房间，使用房间局部安全前沿完成有效探索，完成房内危险源扫描并做到真实危险物零漏检，最后发布 `FLOOR_COMPLETE` 和正式结果文件。当前不以单层耗时或 RTF 作为功能收尾门槛；房内固定观察位已替换为本文件 B.9 定义的局部策略，其他行为接口保持冻结。

> **2026-08-31 验收状态**：冻结 seed `20260902` 的真实无界面 Gazebo 测试已在 `523.186 s` 仿真时间内完成 4/4 房间进入、局部覆盖和退出，发布 `FLOOR_COMPLETE`；第一层危险物 3/3 正确、漏检 0、误报 0，最大位姿步长 `0.0622 m`。该证据标记 B-1 单 seed 功能闭环通过，不替代 B-3 连续 5 seed 稳定验收。正式监视器不得再用全局地图覆盖率或全局 `task_extent_confident` 否决四房局部闭环；这些字段只作诊断。

这是整个工程的核心阶段。双层版本不得绕过本阶段重新实现另一套探索逻辑。

Stage B 不负责完整地图覆盖、全局或走廊普通 Frontier Exploration、其他走廊分支、电梯内部、楼梯大厅、自动上楼或多楼层调度。仅在已确认进入的当前小房间内运行受拓扑和机身净空约束的局部前沿规划；当前验收目标是四房，但必须通过参数 `expected_rooms_per_floor` 表达，禁止把数字 `4` 写死在状态机分支中。

## B.0 实施顺序与行为架构

Stage B 必须按顺序执行，禁止并行堆叠三轮核心整改：

```text
B-1 行为架构收敛
    DoorFusion + DoorCandidateManager
    LocalizationHealthMonitor
    CorridorEstimator
    事件式进门/退出
    RecoveryManager
    四房完成条件
    ResultWriter
        ↓ 冻结上层接口
B-2 定位后端整改（仅在触发条件成立时）
    Hector -> Livox + IMU LIO
        ↓
B-3 至少连续 5 个 seed 稳定验收
```

B-1 可以继续使用现有 Hector，以隔离行为层改动的效果。如果仍反复出现长走廊纵向漂移、超过 1 m 的非物理 pose jump、Hector crash 或房内旋转后位置异常，才进入 B-2。B-2 只替换定位后端，不重写 Explorer。

### B.0.1 本轮 Stage B 收尾基线

本轮收尾以 `docs/SimEnv_Stage_B_Closeout_Adjustment_Plan.md` 为历史基线，并采用本次已批准的房内策略调整：

```text
DOOR_CROSSING
    -> ROOM_SCAN / ROOM_FRONTIER_EXPLORE（当前房间局部探索）
    -> EXIT_ROOM
    -> DOOR_LOCAL_LOOP
    -> VISITED
```

同时执行以下边界：

- `room_entry` 只保存 FAST-LIO pose、入口附近局部点云、门法向、危险源基线和 Room Entry Frame，不承担固定观察位或导航目标职责。
- 房间确认后，`RoomFrontierPlanner` 只在当前房间拓扑 ROI 内选择信息增益最大的安全可达前沿；门内半平面、最大深度/横向范围、机身膨胀净空和逃生连通性共同约束候选，路径长度仅作次级代价。
- 未知栅格只能作为前沿边界，不能作为路径穿越单元；路径使用四连通栅格并保留水平/垂直转弯，窄墙缝、贴墙路径和长死胡同候选直接拒绝。
- 危险源检测在房间前沿运动产生的 RGB-D 帧中持续运行；检测只更新目标生命周期，不创建独立导航目标或把目标位置传给探索规划。
- 危险源在 Room Entry Frame 内关联，退出后的同门局部回环只做有界 ICP 和 Room Entry Frame 到 world 的修正，不直接大幅修改 FAST-LIO 内部状态。
- 跌倒、控制器进入 passive/down、定位持续 `BAD`、LIO 有效点连续过低或非物理 pose jump 进入 `MISSION_FAULT`，立即零速度并终止本轮正式统计；不得继续长时间等待。
- `B-CORRIDOR`、`B-FAULT`、`B-DET`、`B-DOOR` 分开归因。没有多轮证据时冻结 FAST-LIO 主参数、局部回环阈值、Door Verifier 基本结构、门宽、defer zone 和 entrance gate。

### B.0.2 当前功能优先级与冻结边界

当前整改只回答两个问题：

```text
是否完成 4/4 个真实房间的进门、房内扫描和退出？
真实危险物是否全部检出（missed == 0）？
```

当前版本中房间固定观察位机制已删除，房内只允许使用 `RoomFrontierPlanner` 的局部策略。不能因为一个 seed 的漏检或超时临时添加第二套房间规划器、全局/走廊 frontier 或新的检测分支；先通过检测生命周期和走廊/定位证据确认漏检阶段，再提出下一轮单点调整。
单层时间、RTF、CPU/RAM 和误报数量继续完整记录，但在本轮功能收尾中属于次级诊断项，
不替代四房完成和零漏检门槛。

本次调整范围明确如下：

- **调整**：删除房内固定观察位、固定扫描方向和观察位导航目标；在已确认房间内启用 `RoomFrontierPlanner`，按门内拓扑、机身净空、墙缝拒绝、四连通路径和死胡同/逃生约束选择信息增益优先安全前沿。
- **保持**：走廊 `CorridorEstimator`、Scan/Map `DoorEvidence`、`DoorFusion`、`DoorCandidateManager`、门洞候选优先级、进门/退出传感器事件、楼梯/电梯 defer、危险源检测生命周期、故障终止、四房完成条件和正式结果 schema 均沿用旧方案。
- **隔离**：房间前沿规划只在 `ROOM_SCAN` 的 `ROOM_FRONTIER_EXPLORE` phase 运行；走廊、房间外和全局地图不调用它，也不把红球位置传给规划器。

Stage B 主行为链为：

```text
/scan_2d ──> CorridorEstimator ──> ScanDoorDetector ──┐
                                                     ├──> DoorFusion
Exploration Map ───────────────> MapDoorDetector ────┘        │
                                                              ▼
                                                   DoorCandidateManager
                                                              │
                                                              ▼
                                                    SingleFloor FSM
                                        CORRIDOR_PROGRESSION
                                        -> GO_TO_PRE_DOOR
                                        -> ALIGN_TO_DOOR_NORMAL
                                        -> DOOR_CROSSING
                                        -> ROOM_SCAN
                                           room_scan_phase=ROOM_FRONTIER_EXPLORE
                                        -> EXIT_ROOM
                                        -> DOOR_LOCAL_LOOP -> VISITED
                                                              │
                                                           /cmd_vel
```

Scan 与 Map 是互补证据源，不是两套候选系统。不得新增第三套门洞 Detector。

`DoorFusion`、`DoorCandidateManager` 和 `CorridorEstimator` 只负责走廊中的门洞证据与候选生命周期；房间前沿规划不得参与走廊推进、门洞发现、候选融合或其他房间的搜索。确认的未访问房间候选始终优先于走廊推进，房间外不存在 `RoomFrontierPlanner` fallback。

---

## B.1 连续定位与定位健康

### 目标

建立比赛合规、可监测且对上层接口稳定的 pose。定位后端必须统一输出：

```text
/simnav/odom
/simnav/world_pose
TF: simnav_map -> simnav_odom -> base
```

B-1 可以使用 Hector；最终跨层第一选择是 Livox + IMU 的轻量 LIO，例如 FAST-LIO 类方案。

要求：

```text
robot birth
    ↓
first floor
    ↓
stair
    ↓
second floor
```

进入 Stage C 后，同一套最终定位必须持续运行且不得按楼层重置。

### 实现要求

- 不读取 Gazebo GT odometry。
- 先验证传感器时间戳和 frame。
- 必须配置真实外参，不允许用“看起来差不多”的 frame 假设。
- 定位 adapter 必须保持上层 topic/TF 契约不变。
- 如果接入 FAST-LIO 需要 adapter，写薄 adapter，不要引入重复点云处理链。
- 输出 frame 名称避免与 Gazebo 真值 frame 混淆，例如：

```text
simnav_map
simnav_odom
base
```

### world 对齐

从公开 `team_scene_info.json` 获取允许的机器人初始信息，建立：

```text
T_world_simnav
```

最终危险源位置通过合法的 TF 链变换到 `world`。

### LocalizationHealthMonitor

必须在“是否持续发布”之外监测：

```text
单帧平移
单帧角度变化
/simnav/odom 时间连续性
短时间累计位移
相对命令速度的物理合理性
异常 pose jump
```

统一输出四级状态，并严格影响门洞链：

| 状态 | Scan 行为 | Map 行为 | FSM 行为 |
|---|---|---|---|
| `GOOD` | 正常 | 正常，可独立形成强证据 | 正常 |
| `DEGRADED` | 正常 | 降权，只优先确认已有 hypothesis | 谨慎继续并记录 |
| `BAD` | 正常 | 禁止独立新建 hypothesis | 暂停依赖累计地图的新决策 |
| `STALE` | 不再用于运动决策 | 禁止 | 立即发布零速度，不进入普通 Recovery |

定位异常必须在定位层诊断；禁止通过扩大门宽、增加重试或新增 Explorer 分支进行补偿。

### Stage B 收尾健康侧车

Stage B 收尾增加三个诊断侧车，但不替代现有控制器、FAST-LIO 或
`LocalizationHealthMonitor` 的所有权：

```text
junior_ctrl -> /simnav/controller_health
/cloud_registered -> /simnav/lio_health
explorer     -> /simnav/mission_fault
```

`junior_ctrl` 仍是唯一的控制与步态执行节点；`controller_health` 只报告
`RESET/WAITING/ACTIVE/RL/PASSIVE/FALL` 等状态。`lio_health` 只做有界的有效点数和
更新时间监视，不在回调中逐点处理点云。行为节点把控制器跌倒/被动、定位持续
`BAD`、LIO 连续无有效点或非物理 pose jump 统一收敛为 `MISSION_FAULT`：发布原因、
置零速度并停止本轮正式统计。不得通过删除 `junior_ctrl` 或跳过健康消息来规避故障。

### 定位验收

- [ ] 人工控制走长走廊，轨迹连续；
- [ ] 原地旋转后没有明显位置爆炸；
- [ ] 进入/退出房间轨迹连续；
- [ ] `GOOD/DEGRADED/BAD/STALE` 状态转换和原因可记录；
- [ ] `BAD` 时 Map 不会独立创建新 hypothesis；
- [ ] `/simnav/odom` 超过 1 s 未更新时进入 `STALE` 并停车；
- [ ] 不出现超过 1 m 的非物理 pose jump；
- [ ] 不使用 GT 数据；
- [ ] world 对齐方式有独立代码和注释；
- [ ] 静止时 pose 不持续严重漂移。

本阶段暂不要求厘米级精度，以能够支撑门候选融合和最终 1 m 量级危险源匹配为工程目标。

---

## B.2 当前楼层轻量 2D 地图

### 目标

不做完整 3D navigation。使用当前楼层的点云高度切片生成轻量 2D OccupancyGrid。

### 推荐初始配置

```text
map_resolution: 0.05 m
mapping_frequency: 2 Hz
obstacle_projection: 5~10 Hz
```

不要让 10 Hz 全量 Livox 点云每帧都执行昂贵完整 ray-tracing。

允许：

```text
voxel downsample
range clipping
height filtering
angular binning
```

### 楼层概念

记录当前楼层参考高度：

```text
current_floor_z
```

仅将与当前楼层导航相关的点投影到 2D。

Map 门洞检测不得围绕当前机器人位姿使用固定窄走廊带。调整为：

```text
robot pose 只用于截取较大局部 ROI
        ↓
在 ROI 内重新估计左右主墙
        ↓
建立 local corridor frame
        ↓
在真实墙面序列中寻找 wall-gap-wall
```

目标是允许约 `0.3~0.5 m` 的横向定位漂移而不立即漏门。Map Detector 只输出 `DoorEvidence`，不得直接创建 `RoomCandidate`。

### 验收

- [ ] 长走廊墙体稳定出现；
- [ ] 房间门洞不会因地图算法自身被完全封死；
- [ ] 家具可作为障碍；
- [ ] Map Detector 使用局部主墙重估，而不是固定窄 corridor band；
- [ ] Map Detector 不拥有候选生命周期；
- [ ] 地图运行后 RTF 相比阶段 A 没有不可接受的明显恶化；
- [ ] 地图更新频率可配置。

---

## B.3 入口处理：定位可以早开，探索不能早开

机器狗出生后：

```text
BOOT
 ↓
START_LOCALIZATION
 ↓
ENTER_BUILDING
 ↓
CLOSE_MAIN_ENTRANCE
 ↓
INITIALIZE_FLOOR_MAP
 ↓
ENABLE_EXPLORATION
```

核心要求：

- 当前定位后端可以从门外开始，切换到最终 LIO 后保持同样行为；
- 第一层 Exploration Map 在进入建筑后建立/解锁；
- 主入口通过后调用公开门 service 关闭；
- 在 Exploration Map 中加入入口虚拟 gate；
- 即使 Gazebo 门模型仍有小缝，也不允许走廊或房间外 Frontier 再成为任务目标；房间内的前沿规划必须先通过机身净空和拓扑边界过滤。

### 验收

- [ ] 机器人通过主入口后不会因为门外未知区域重新规划出去；
- [ ] 关闭物理门后，即使地图上有模型缝也不会产生有效外部探索目标；
- [ ] 虚拟 gate 只影响探索决策，不破坏定位。

---

## B.4 Navigation Map 与 Exploration Map 分离

必须实现两个逻辑用途：

### Navigation Map

用于碰撞检查和后续长距离路径规划：

```text
collision
A*/Navfn
DWA
local obstacle avoidance
```

Stage B 的主走廊与门口动作不依赖 Navfn/DWA 决策，但不得因此破坏 Navigation Map 的真实性。

### Exploration Map

用于门洞检测和任务语义辅助，可以额外进行：

```text
entrance mask
small-gap closing
elevator defer
stair defer
visited-room marking
local corridor ROI
```

Stage B 明确禁止使用 Exploration Map 执行“哪里 unknown 最大就去哪里”的全局或走廊 Frontier 目标选择。已确认进入房间并处于 `ROOM_SCAN` 时，允许把当前房间的 Exploration Map ROI 提供给 `RoomFrontierPlanner`；规划器必须同时应用门内拓扑边界、机身净空、四连通可达性和死胡同过滤，且不能跨越门洞或离开当前房间。

### 小缝处理

如果 resolution = 0.05 m，初始可测试：

```text
morphology closing kernel: 3~5 cells
```

仅作用于 Exploration Map。

绝对不要为了去除假 Frontier 而把真实 Navigation Map 大量膨胀或封闭。

---

## B.5 运动控制权：Stage B 由显式 FSM 主导

`move_base`、Navfn/A* 和 DWA 的代码与配置继续保留，但在 Stage B 房间搜索中降级，不作为以下行为的主控制器：

```text
CORRIDOR
GO_TO_PRE_DOOR
ALIGN_TO_DOOR_NORMAL
DOOR_CROSSING
ROOM_SCAN（room_scan_phase=ROOM_FRONTIER_EXPLORE）
EXIT_ROOM
REVERSE_SWEEP
```

这些状态由 `SingleFloor FSM` 直接控制 `/cmd_vel`。`move_base` 后续主要用于 Stage C 楼层完成后前往楼梯 pre-pose、必要的较长距离目标和辅助导航；暂不默认引入 TEB。

### Footprint 规则

“能否通过”与“偏好离墙距离”分开：

```text
robot footprint + small padding = hard collision constraint
inflation cost                  = soft preference
```

不要用巨大 `inflation_radius` 表示机器人真实尺寸。

### 初始速度建议

以下是调试起点，不是官方限速：

```text
宽走廊:      0.30~0.40 m/s
接近门洞:    0.15~0.20 m/s
穿门:        0.15~0.20 m/s
房间内部:    0.15~0.25 m/s；按当前前沿路径最小净空动态限速
```

### 必须先做运动原语验收

在开启完整自主闭环前，通过测试节点逐项验证：

```text
长走廊
90/180 度掉头
房门进入
房门退出
房间内部移动
窄通道
```

### 验收

- [ ] 约 1.2 m 房门可重复稳定通过；
- [ ] 正常宽走廊不持续贴墙；
- [ ] 机器人不会因为 inflation 直接判门洞不可达；
- [ ] 失败 recovery 不产生长时间死循环；
- [ ] 运动原语稳定前，不启用完整自主 Explorer；
- [ ] 同一时刻只有一个模块拥有 `/cmd_vel` 控制权；
- [ ] Stage B 的进门和退出不由 DWA 随机角度完成。

---

## B.6 CorridorEstimator、门洞证据与统一候选

### 统一走廊判断与任务优先级

`CorridorEstimator` 是走廊判断的唯一实现，至少输出 `valid`、`confidence`、`axis_yaw`、左右墙距离、`corridor_width`、`center_error` 和 `front_clearance`。`ScanDoorDetector`、`MapDoorDetector`、`CorridorController`、`ExitDetector`、`EndOfCorridorDetector` 必须消费同一输出，禁止各自实现一套“像不像走廊”的判断。

Stage B 的任务优先级固定为：

```text
confirmed unvisited room candidate
        >
corridor progression
        >
one reverse sweep after corridor end
        >>>
stair/elevator/outside/other corridor branches
```

只有在候选已经完成进门并进入 `ROOM_SCAN` 后，房间局部前沿规划才获得控制权；它不能与确认的未访问房间候选或走廊推进竞争。不启用全局或走廊普通 Frontier fallback，也不进入侧向宽开放区域。红球检测在 `ROOM_SCAN` 的房间前沿运动期间进行，不创建一套独立的全层导航目标系统。

### 不做“语义门识别”

第一版不要求识别“这是一扇门”。大厅和侧向大开口应排除。

检测的是：

```text
走廊墙面中的可通行开口
+
开口后存在新的自由/未知区域
```

Scan 与 Map 不直接创建 `OpeningCandidate` 或 `RoomCandidate`，只输出 `DoorEvidence`：

```text
DoorEvidence
{
    source
    timestamp
    side
    center_estimate
    width_estimate
    normal_estimate
    corridor_confidence
    source_confidence
    opening_complete
    localization_health
}
```

Scan 门洞检测保留左右侧窄扇区、有限回波、`OPEN_START -> OPEN -> OPEN_END` 和晚闭合机制；`inf` 只表示 unknown。Map 门洞检测保留 local ROI 主墙重估、wall-gap-wall、完整双边缘、门前/门后自由栅格检查、small-gap closing、entrance gate 和 defer zone。

### DoorFusion 与 DoorCandidateManager

`DoorFusion` 维护唯一的融合假设：

```text
DoorHypothesis
{
    id
    side
    center
    width
    normal
    scan_support
    map_support
    first_seen
    last_seen
    confidence
    status
}
```

融合不得简单要求 `SCAN == YES AND MAP == YES`：

- `SCAN_STRONG`：走廊上下文、完整 open-close、合法门宽和可靠双边缘成立时，可单独确认；Map 显示实墙不能直接否决。
- `MAP_STRONG`：完整 wall-gap-wall、合法门宽、局部走廊可信且定位为 `GOOD` 时，可单独确认；Scan 为 `UNKNOWN` 不能直接否决。
- `MEDIUM + MEDIUM`：两种中等且空间/法向一致的证据可联合确认。
- 定位为 `DEGRADED` 时 Map 只降权确认已有 hypothesis；`BAD` 时 Map 禁止独立新建。

只有 `DoorCandidateManager` 有权把 hypothesis 转为 `RoomCandidate`，并统一管理：

```text
PENDING_AHEAD
PENDING_BEHIND
APPROACHING
ENTERING
SCANNING
EXITING
VISITED
UNREACHABLE
```

Scan/Map Detector 不得维护自己的重试次数、访问状态或空间去重表。

### 晚闭合与回访

门洞可以在机器人经过门中心以后才获得完整闭合边，此时创建 `PENDING_BEHIND`。回访采用“180 度转向 + 前进步态”，禁止重新引入倒车回访。

### 开口初始宽度范围

可从：

```text
0.8~1.6 m
```

开始测试，但必须参数化，不要因单个 seed 失败逐轮扩大范围。

---

## B.7 门洞必须有独立 Crossing 状态

不要直接给房间深处一个 goal 让 DWA 随机角度钻门。时间只限制动作持续范围，不作为进门或退出成功的唯一判据。

执行：

```text
GO_TO_PRE_DOOR
    ↓
ALIGN_TO_DOOR_NORMAL
    ↓
DOOR_CROSSING
    ↓
INSIDE_ROOM_CONFIRMED
    ↓
ROOM_SCAN（内部 phase = ROOM_FRONTIER_EXPLORE）
    ↓
EXIT_ROOM
    ↓
CORRIDOR_REACQUIRED
```

要求：

- pre-door 位于走廊侧；
- 使用平地 locomotion 完成原地对正；
- yaw 对齐后低速近似直行穿门；
- post-door 建议距门内约 0.8~1.2 m；
- 执行 `minimum_cross_time` 后，只有局部走廊结构连续若干 scan frame 消失，才确认 `INSIDE_ROOM`；
- `front_clearance < emergency_threshold` 时立即停车；
- 到达 `max_cross_time` 仍未确认时产生 `CROSSING_FAILED`；
- `EXIT_ROOM` 成功要求合理走廊宽度、左右双墙和稳定轴向持续 `0.5~1.0 sim s`；
- 固定退出时间只保留为 `max_exit_time`，不得直接标记成功。

### 验收

至少对多个随机房门进行重复测试：

- [ ] 不因门框 inflation 判定无路；
- [ ] 不持续斜着撞门；
- [ ] 穿门失败能够退出并重试/标记失败；
- [ ] 单个门洞不会无限重试；
- [ ] 进门成功来自 `INSIDE_ROOM` 传感器事件；
- [ ] 退出成功来自 `CORRIDOR_REACQUIRED` 传感器事件。

---

## B.8 电梯和楼梯在单层阶段必须 DEFER

除楼梯和电梯外，Stage B 同样不探索其他走廊分支。遇到侧向宽开放区域只记录或忽略，不进入新的分支搜索状态。

### 电梯

如果公开 service 允许且任务策略不使用电梯：

- 进入楼层后可关闭当前层电梯门；
- Exploration Map 中电梯内部仍设为低价值/禁止目标区域；
- 模型缝通过 small-gap filter 处理。

### 楼梯

楼梯不能永久设为 obstacle。

记录为：

```text
VerticalTransitionCandidate
status = DEFERRED
```

单层搜索阶段不得与房间候选竞争。

同时提前保存楼梯入口候选，避免楼层结束后重新从零寻找楼梯。

---

## B.9 房间内探索：入口拓扑与信息增益优先安全前沿

Stage B 不做楼层全局覆盖，也不把入口近端设置为固定观察位。单房流程固定为：

```text
DOOR_CROSSING
    ↓
ROOM_SCAN / ROOM_FRONTIER_EXPLORE
    │  入口参考只用于拓扑、局部回环和坐标关联
    │  过滤死胡同/窄缝后，优先选择信息增益最大的安全前沿
    │  目标和整条路径保持承诺，只有完成或路径失效才重规划
    │  RGB-D 检测随房内运动帧运行
    ↓
没有新的安全房间前沿（quiet window）？
    ├── 否 -> 重新规划并继续
    └── 是 -> EXIT_ROOM
```

`room_entry` 的职责只有：

- 保存当前 FAST-LIO pose 和房间入口局部点云关键帧；
- 保存 corridor / door normal，定义门内方向；
- 保存危险源 tracker 基线；
- 建立 `RoomEntryFrame`，为同门局部回环和危险源坐标修正提供参考。

`room_entry` 不执行完整 360 度视觉扫描、危险源覆盖判断、长时间原地旋转或前沿搜索；它只是局部参考，不是观察位、目标点或必须到达的停驻点。

#### 房间前沿约束

```text
doorway center + inward normal -> room half-plane
active-room depth/lateral limits -> topology ROI
robot radius + safety margin -> hard clearance
observed free cells -> traversable component
unknown cells -> frontier boundary only
```

规划器只从当前房间已观测且满足机身净空的自由连通分量中选择前沿。当前默认按 A1 footprint 的约 `0.38 m` 外接半径加 `0.04 m` 小余量计算硬净空（合计约 `0.42 m`）；该值不是走廊门洞宽度，也不替代真实碰撞检查。未知单元不得被当作路径穿越单元；前沿路径使用四连通栅格并保留转弯，避免对角线切角。候选必须通过整条路径的最小净空检查，并满足最小逃生连通性；窄于机身的墙缝、墙体 seam、贴墙点、过长低信息死胡同和没有安全退路的分支直接拒绝。

每次选择先执行硬安全过滤：整条四连通路径必须满足机身净空，且候选周围的逃生单元数达到 `room_minimum_escape_cells`；超过 `room_dead_end_path_threshold` 的单分支低逃生路径直接拒绝。通过过滤后，以候选前沿附近有界未知区域和前沿簇大小估算信息增益，信息增益为主排序，路径长度只作为次级代价和同等收益时的 tie-breaker。`room_frontier_revisit_radius` 仍用于抑制重复目标。

选定前沿后，控制器提交整条四连通路径并逐个经过路径点，不因普通地图增长或固定周期到期而替换目标。只有当前路径被新占据栅格/净空检查判定失效、定位回环修正，或目标完成时才停止并重新规划；`room_frontier_replan_period` 仅用于无活动目标时的重试。这样避免目标点每秒跳变导致机器狗原地徘徊或尚未到达就换分支。

房间覆盖率以机器狗实际可进入的安全自由空间为分母：

```text
safe_coverage = reachable_safe_cells / safe_room_cells
raw_coverage  = reachable_safe_cells / observed_room_free_cells  # 诊断项
```

默认 `room_frontier_coverage_target=0.80`。`raw_coverage` 继续输出用于诊断，但不再把墙体、家具和窄缝附近因机身保护而主动排除的单元计入必须覆盖的面积。连续 `room_frontier_quiet_period` 没有新的安全前沿后结束当前房间；若此时安全覆盖率低于目标，记录告警并保留未达标证据，不让物理不可达区域造成无限等待。房间探索不要求所有未知区域变为已知，也不追求几何中心或无意义全覆盖。

该规划器的调用边界严格限定为 `ROOM_SCAN`/`ROOM_FRONTIER_EXPLORE`。走廊推进、门洞发现、DoorFusion、楼梯/电梯 defer、其他开放分支和下一房间候选仍沿用旧方案；确认的未访问房间候选始终优先，房间外不得使用本规划器。

---

## B.10 红色球形危险源检测

第一版优先轻量传统视觉，不默认使用神经网络。

流程：

```text
RGB
 ↓
HSV red mask
 ↓
contour candidate
 ↓
2D geometry filter
 ↓
depth ROI
 ↓
3D target points
 ↓
sphere / size validation
 ↓
multi-frame confirmation
 ↓
world transform
 ↓
spatial clustering / dedup
```

### 性能原则

移动时：

```text
RGB detector: 3~5 Hz
```

房间扫描时：

```text
RGB detector: up to sensor rate
```

Depth 三维处理和 sphere fitting 必须 event-driven：只有出现红色候选才执行。

不要持续处理完整 `/real_sense/depth/points` 做全场景语义分割。

### 房间内 Room Entry Frame 与检测生命周期

房间内首先在 `RoomEntryFrame` 中记录目标，不把每一帧的漂移后 world 坐标直接当作最终轨迹：

```text
Camera -> Base -> RoomEntryFrame -> P_room
```

房间前沿运动期间第一次确认目标时保存 `target_id`、`P_room`、bearing、depth、shape evidence 和 observation count。后续有限复核（如确有必要）必须优先关联已有 `target_id`，不得因为 LIO 局部漂移按 world 距离重新创建同一目标。危险源检测不反向创建前沿目标，也不改变走廊候选优先级。

检测复核不是第二次全房搜索，只允许对当前新目标做有限定向观察：

```text
translation <= 0.5~0.8 m（无必要时原地完成）
目标方位 ±45 度或 90~180 度局部转动
```

完成后立即退出。已有目标不因进入新房间而重复触发验证。

每个候选必须记录完整生命周期，至少包括：

```text
RED_MASK_FOUND
CONTOUR_PASS / REJECT
DEPTH_PASS / REJECT
TF_PASS / REJECT
TRACK_CREATED / TRACK_ASSOCIATED
OBSERVATION_COUNT
CONFIRMED / UNCONFIRMED
LOCAL_LOOP_CORRECTION
FINAL_OUTPUT
```

这样才能区分“危险源房间尚未访问”和 detector 在颜色、形状、深度、TF、轨迹或确认阶段的真正漏检。没有证明 `RED_MASK_FOUND` 失败前，不先扩大 HSV 阈值。

### 必须处理的干扰

专门测试：

```text
红色球体   -> 应检出
红色方块   -> 应排除
绿色球体   -> 应排除
同一红球多角度 -> 只能输出一个最终危险源
```

### world 输出

最终保存：

```text
DangerTrack
{
    id
    position_world[3]
    observations
    confidence
    position_variance
    localization_health_history
}
```

相近 world 坐标观测进行聚类，禁止同一球重复报告。记录每条轨迹的坐标方差，并把定位异常与轨迹分裂关联记录；定位后端稳定后再重新评估 `danger_cluster_distance`。

`ROOM_SCAN` 直接消费实际可获得的 RGB-D 帧，不追求超过传感器真实频率的处理频率，也不在 Stage B 重写整套视觉算法。

### ResultWriter 与正式结果闭环

必须由独立 `ResultWriter` 区分调试输出和评分输出：

```text
results/detected_danger_debug.json  # 内部轨迹、置信度、方差、诊断信息
results/detected_danger.json        # 仅正式评分结构
```

正式文件结构：

```json
{
  "exploration_time": 0.0,
  "detected_danger_sources": [
    {
      "position": [0.0, 0.0, 0.0]
    }
  ]
}
```

`exploration_time` 必须使用 ROS `/clock` 仿真时间。正式 JSON 不得夹带调试字段，并应在 Stage B 验收脚本中验证 schema、数值类型和可读取性。

---

## B.11 RecoveryManager

所有行为恢复预算统一由 `RecoveryManager` 管理，第一版默认上限：

```text
door_attempt <= 2
exit_attempt <= 2
reverse_sweep <= 1
```

门洞失败先退回 pre-door 并重新对齐；退出失败先检查退出方向、局部障碍和 `CorridorEstimator` 是否重新成立。达到预算后才允许标记 `UNREACHABLE`，并必须记录原因。

定位 `BAD` 只暂停 Map 新候选，`STALE` 立即停车；二者不消耗普通门/退出重试预算。禁止用延长固定动作时间或增加 Recovery 次数掩盖 pose jump。

### B.11.1 故障路径与普通恢复隔离

普通门/退出失败继续使用 `RecoveryManager` 的有限预算；物理或定位故障走独立
终止路径：

```text
controller FALL/PASSIVE（已进入任务后）
Localization BAD 持续超过阈值
LIO NO_EFFECTIVE_POINTS 持续超过阈值或 STALE
非物理 pose jump
        ↓
MISSION_FAULT
        ↓
零速度、冻结 DoorEvidence/正式 danger 更新/走廊等待
        ↓
ABORT_RUN（记录首个根因）
```

Stage B 不因上述故障自动追加长时间等待或重复动作；监控器遇到
`MISSION_FAULT` 应立即结束本轮并把该轮标为无效。`RECOVERY` 仅保留给未来已经定义、
可证明安全的恢复动作，不能作为当前阶段的隐式兜底分支。

### B.11.2 恢复记录

每次普通恢复和故障终止至少记录：触发状态、触发时间、当前候选、控制器/LIO/
定位快照、最近命令、最近 pose 增量、剩余预算和最终动作。故障发生后不再增加
候选、不再推进 `VISITED`，避免把半轮结果混入正式统计。

---

## B.12 单层完成条件

绝对禁止用楼层全局的 `frontier_count` 简单使用：

```text
frontier_count == 0
```

作为楼层完成条件。房间内的 `NO_FRONTIER` 只表示当前已确认房间在 quiet window 内没有新的安全局部前沿，用于结束该房间并执行 `EXIT_ROOM`；楼层完成仍必须由已访问房间数量、危险源确认和走廊候选生命周期共同决定。

Stage B 测试目标使用参数：

```text
expected_rooms_per_floor: 4
```

当到达走廊末端时：

```text
VISITED == expected_rooms_per_floor ?
        ├── YES -> 等待 danger confirmation quiet window
        └── NO  -> TURN_180 -> REVERSE_SWEEP（最多一次）
```

正式完成逻辑：

```text
unique_VISITED == expected_rooms_per_floor
AND
当前没有进行中的 danger confirmation
AND
2~3 s sim-time quiet window
→ FLOOR_COMPLETE
```

普通未知区域、已排除的大开口和历史假 hypothesis 可以继续存在。不得要求所有历史候选都进入终态，也不得在四房完成后继续寻找“第五个房间”。

### B.12.1 走廊停滞诊断与开口生命周期

走廊推进每个控制周期发布有界 `/simnav/corridor_diagnostic`，至少包含：

```text
sim_time / FSM state / cmd_vel
LIO pose 与平移、旋转增量
corridor progress
front / left / right clearance
finite scan count / corridor valid / confidence
opening_state = CLOSED | OPEN_START | OPEN | OPEN_END
DoorEvidence 与候选数量
```

对未完成房间的走廊停滞先按证据归类：

| 分类 | 证据组合 | 后续动作 |
|---|---|---|
| A：运动/障碍 | 命令有效且 pose 有进展，但净空或障碍阻挡 | 查控制、碰撞和走廊安全距离 |
| B：定位/观测 | 机器人有运动命令，但 LIO progress 低或有效点退化 | 查 LIO、时间戳、点云健康和资源竞争 |
| C：FSM/记账 | LIO 和走廊证据正常，但状态、候选或完成计数不变 | 查状态转移、候选生命周期和完成条件 |

开口证据必须保留 `OPEN_START -> OPEN -> OPEN_END` 生命周期；证据尚未闭合时
不得直接产生正式 `RoomCandidate`。诊断只用于归因和验收，不新增第四套门洞检测器。

---

## B.13 单层阶段验收指标

Stage B 只有在同一冻结版本上至少连续 5 个 seed 全部通过后才结束：

多 seed 验收时固定 `ROBOT_X/Y/Z/YAW` 和除 `SEED` 外的测试参数，避免把出生位姿变化混入场景随机性比较；如需单独测试出生位姿鲁棒性，建立独立测试矩阵。

| 指标 | 必须达到 |
|---|---:|
| 第一层真实房间 | 4/4 |
| `floor_complete` | `true` |
| `UNREACHABLE` | 0 |
| 额外假门 | 0 |
| 单房进入尝试 | <= 2 |
| 退出尝试 | <= 2 |
| 反向回扫 | <= 1 |
| 定位节点 crash | 0 |
| 超过 1 m 非物理 pose jump | 0 |
| 人工干预 | 0 |
| 红方块误报 | 0 |
| 正式 JSON | schema 正确且可直接评分 |
| 多 seed | 至少连续 5 个通过 |

当前功能收尾的硬门槛是：

| 功能门槛 | 必须达到 |
|---|---:|
| 四房完整访问 | `room_entries = 4`、`room_exits = 4`、`floor_complete = true` |
| 真实危险物召回 | `danger_missed = 0`；测试真值中的每个危险物至少有一个确认轨迹 |
| 故障与中断 | 无 `MISSION_FAULT`、无人工干预、无未解释的 `UNREACHABLE` |

时间与性能只作为记录项：

```text
elapsed_sim_time / RTF / CPU / RAM / timeout_reason
```

它们用于后续性能阶段和方案讨论；本轮不得因为超过 300/360 sim s 就把“四房且零漏检”
判为功能失败，也不得为了赶时间减少必要的危险物确认。误报、单房耗时和房间探索质量
必须记录，等功能门槛稳定后再单独优化。

此外必须记录相对阶段 A 的 RTF/CPU/RAM；高层算法不得无意义地以 10~30 Hz 处理全图。
当前功能收尾中，任一 seed 只要未满足四房完整访问或危险物零漏检就不能通过；单纯
超时不再作为功能失败依据，但必须记录超时原因并留给后续性能阶段处理。产生误候选、
定位异常或人工干预仍需单独记录并按对应硬门槛处理。

收尾版本还必须满足：

| 收尾约束 | 验收要求 |
|---|---|
| 房间入口参考 | `room_entry` 只保存 pose、入口局部点云、门法向和 `RoomEntryFrame`；不是观察位或导航目标 |
| 房内探索 | 仅在当前 `ROOM_SCAN` 中运行拓扑约束信息增益优先安全前沿；使用机身膨胀、四连通路径、墙缝拒绝和死胡同/逃生过滤 |
| 走廊发现隔离 | 门洞发现、DoorFusion、候选生命周期、进门和退出事件沿用旧方案，不由房间前沿规划器参与 |
| 危险源复核 | 房间前沿运动帧执行检测；必要复核必须有界，不做第二次全房搜索且不创建导航目标 |
| 故障终止 | `MISSION_FAULT` 后零速度且不继续等待/记账；监控结果必须标记无效 |
| 诊断闭环 | 每轮存在走廊诊断、危险物生命周期和故障原因记录 |
| 时间/性能 | 只记录，不作为当前功能收尾失败依据 |

每轮必须保存：

```text
commit / seed / 完整启动参数
仿真开始与结束时间
VISITED / UNREACHABLE / floor_complete
DoorHypothesis 数量、来源及 scan/map 支持
进入次数 / 退出次数 / reverse sweep 次数
最大 pose jump / 定位状态变化
danger_count / 假阳性 / 轨迹坐标方差
RTF / CPU / RAM
正式结果 JSON
失败分类与首个根因证据
```

**在 Stage B 未满足上述全部门槛前，不开始双层自主任务。**

---

## B.14 失败分类与防过拟合纪律

任何失败先分类，再决定修改位置：

| 类别 | 典型表现 | 首查内容 | 禁止的第一反应 |
|---|---|---|---|
| 定位失败 | pose jump、Hector crash、长走廊不累计、房内旋转后乱跳 | 定位日志、时间戳、外参、健康状态 | 改门宽阈值 |
| Scan 漏门 | Map 有清晰门，Scan 无完整 open-close | 有限回波比例、OPEN 生命周期、扇区 | 扩大门宽范围 |
| Map 漏门 | Scan 能看到，Map 连续墙或检测带错位 | 定位、local ROI、主墙重估 | 增加第三套 Detector |
| 重复候选 | 同一房门出现多个 RoomCandidate | DoorFusion 与 hypothesis 生命周期 | 单纯扩大空间去重半径 |
| 进门失败 | 对不准、撞门、无法确认 inside | 法向、对准、front clearance、实际速度、inside 事件 | 先调地图 |
| 退出失败 | 房内徘徊或过早结束 | 退出方向、局部障碍、走廊重捕获 | 单纯延长退出时间 |

### B.14.1 本轮收尾的四类专项归因

| 代码 | 触发信号 | 首个证据 | 允许的处理边界 |
|---|---|---|---|
| `B-CORRIDOR` | 走廊进度停滞、开口未闭合或候选长期 pending | `corridor_diagnostic`、开口生命周期、cmd/pose/净空 | 先分 A/B/C；不得先改门宽或增加观察分支 |
| `B-FAULT` | FALL/PASSIVE、定位持续 BAD、LIO 无有效点、非物理跳变 | `controller_health`、`lio_health`、`localization_health`、`mission_fault` | 立即零速度并 `ABORT_RUN`；不计为完成，不拉长等待 |
| `B-DET` | 红球漏检、红方块误报、同球重复轨迹 | `RED_MASK_FOUND` 到 `FINAL_OUTPUT` 生命周期 | 先定位颜色/轮廓/深度/TF/轨迹阶段，再做单点改动 |
| `B-DOOR` | 真实门漏检、额外候选或进出失败 | Scan/Map evidence、DoorFusion、Verifier、inside/exit 事件 | 保持双证据和短时 verifier；不增加第三套 detector |

没有跨多个 seed 的同类证据时，冻结 FAST-LIO 主参数、局部回环 ICP 阈值、Scan/Map
基本结构、Door Verifier 基本结构、门宽、defer zone 和 entrance gate。任何专项修改
都必须能在下一轮日志中验证一个明确假设。

Stage B 期间明确禁止：

1. 因一个 seed 的一次失败增加特殊分支。
2. 没有多轮数据就连续扩大门宽、距离或去重阈值。
3. 读取 layout metadata、world、danger truth 或 Gazebo model states 修正算法。
4. 在同一轮同时修改多个核心模块而无法说明验证假设。
5. 增加第三套门洞 Detector。
6. 用新 Recovery 掩盖定位错误。
7. 把“最终能完成”当成“稳定、限时、无人通过”。

Stage B 通过后冻结 `SingleFloorExplorer` 的行为接口、DoorFusion 契约和完成语义；Stage C 只新增楼层过渡编排。

---

# 6. 阶段 C：在单层 Explorer 不变的前提下扩展双层

> **阶段目标**：第一层完成后自主切换到楼梯，通过官方楼梯 locomotion 到达第二层，然后重新调用同一个 `SingleFloorExplorer` 完成第二层。

进入本阶段的前置条件是 B-3 已在同一冻结版本上连续通过至少 5 个 seed。Stage C 只新增 `FloorTransitionManager`/Mission 编排和楼层地图生命周期，不得借双层需求重新修改门洞阈值、候选融合、进出房状态或单层完成语义。

核心约束：

```text
不要复制 second_floor_explorer
不要为二楼写另一套策略
```

正确结构：

```text
SingleFloorExplorer(current_floor)
      ↓
FloorTransitionManager
      ↓
ACQUIRE_CORRIDOR
      ↓
SingleFloorExplorer(next_floor)
```

---

## C.1 LocomotionManager

检查官方仓库实际存在的平地与楼梯 policy。

建立统一接口：

```text
set_mode(PLANE)
set_mode(STAIR)
```

推荐启动时预加载两个模型（如果内存和官方实现允许），每个时刻只执行当前 policy inference。

不要在楼梯入口临时做昂贵模型加载，除非实测预加载不可行。

---

## C.2 楼梯策略职责必须很小

官方楼梯 policy 如果旋转能力弱，则设计为：

```text
PLANE mode
    ↓
NAVIGATE_TO_STAIR_PREPOSE
    ↓
ALIGN_WITH_STAIR
    ↓
STOP
    ↓
SWITCH_TO_STAIR
    ↓
approx. straight climb
    ↓
LANDING_DETECTED
    ↓
STOP
    ↓
SWITCH_TO_PLANE
```

**不要让楼梯 policy 承担楼梯口复杂转向和全局导航。**

---

## C.3 楼梯候选

第一层搜索过程中可以持续维护：

```text
VerticalTransitionCandidate
```

但状态为：

```text
DEFERRED
```

只有：

```text
floor_complete == true
```

以后才变成：

```text
ACTIVE
```

激活楼梯后，较长距离前往 stair pre-pose 可以使用保留的 `move_base`/Navigation Map；楼梯口对正、policy 切换和爬升仍由显式状态机控制。

楼梯候选可以结合：

```text
3D point z variation
continuous elevation
local ground slope
2D free-space connectivity
```

第一版不需要识别每一级台阶。

---

## C.4 上楼梯时地图行为

上楼梯开始：

```text
Floor0 2D mapper = PAUSE
LIO = RUNNING
```

原因：禁止把楼梯斜面、第一层和第二层墙体一起压入同一 2D map。

LIO 在整个楼梯阶段保持连续，用于：

```text
x/y continuity
z increase
landing detection
world transform
```

---

## C.5 Landing 检测

不要仅使用固定时间判断“已经上楼”。

至少结合：

```text
z 相对 floor0 明显增加
+
dz/dt 接近 0
+
roll/pitch 恢复稳定
+
局部地面重新接近水平
```

持续一小段 **sim time** 后：

```text
floor_transition_complete = true
```

记录：

```text
current_floor = 1
current_floor_z = current_z
```

然后：

```text
create Floor1 2D map
switch PLANE
run SingleFloorExplorer(Floor1)
```

---

## C.6 二楼地图

二楼只新建当前层二维地图，不重置 LIO。

关系：

```text
continuous simnav_map
     │
     ├── floor0 occupancy
     │
     ├── stair transition trajectory
     │
     └── floor1 occupancy
```

两层危险源都使用同一个 world 坐标转换链。

到达二楼后必须重新执行 `ACQUIRE_CORRIDOR`，然后向同一个 `SingleFloorExplorer` 注入新的 `floor_id`、`current_floor_z`、本层地图句柄和参数化 `expected_rooms_per_floor`；不得复用一楼的房间 candidate ID 空间。

---

## C.7 双层 MissionManager

第一版明确实现以下状态：

```text
BOOT
INITIALIZE_LOCALIZATION
ENTER_BUILDING
INITIALIZE_FLOOR
SEARCH_FLOOR
FLOOR_COMPLETE
GO_TO_STAIR
ALIGN_STAIR
SWITCH_STAIR_MODE
CLIMB_STAIR
DETECT_LANDING
SWITCH_PLANE_MODE
INITIALIZE_NEXT_FLOOR
SEARCH_FLOOR
MISSION_COMPLETE
```

但代码不能使用：

```text
if floor == 1: mission_complete
```

这种硬编码作为架构基础。

应使用类似：

```text
max_test_floors = 2
current_floor_index
```

以后把配置改成 3 或 N 时，不需要重写 Explorer。

---

## C.8 双层阶段验收指标

### 楼层切换

- [ ] 一楼未完成时楼梯不会抢占任务；
- [ ] 一楼完成后能激活已记录楼梯候选；
- [ ] 能稳定到达楼梯 pre-pose；
- [ ] 使用 PLANE policy 完成楼梯朝向调整；
- [ ] 能切换 STAIR policy；
- [ ] 上楼期间 LIO 连续；
- [ ] Floor0 mapper 上楼期间暂停；
- [ ] 到达平台后能可靠判断 landing；
- [ ] 能切回 PLANE policy；
- [ ] 能建立 Floor1 map。

### 第二层

- [ ] 二楼直接复用同一个 SingleFloorExplorer；
- [ ] 二楼能够重新发现房间而不会和一楼 room ID 冲突；
- [ ] 两层地图互不污染；
- [ ] 两层危险源均输出统一 world XYZ；
- [ ] 二楼失败不会破坏一楼已经确认的危险源列表。

---

# 7. 阶段 D：随机两层场景完整验收与性能收敛

> **阶段目标**：不再增加新的主要算法模块，只修稳定性、参数、恢复逻辑和实时性能。

## D.1 测试方式

至少使用 5 个不同随机 seed 的两层建筑；如资源允许，应扩大测试矩阵并保留独立复验 seed。

不要只在一个固定地图上调通。

每次测试记录：

```text
commit
seed
完整启动参数
RTF
CPU
RAM
floor0/floor1 VISITED 与 UNREACHABLE
floor0/floor1 floor_complete
DoorHypothesis 数量、来源、scan/map 支持
进入/退出/reverse sweep 次数
最大 pose jump 与定位状态变化
danger detections
confirmed danger count
false positives
door-crossing failures
stair-transition failures
mission sim-time
正式结果 JSON
失败分类与首个根因证据
```

## D.2 第一轮工程目标

不要追求一次达到最终最优。

优先达到：

```text
1. 多个随机 seed 可无人干预完成两层
2. Stage B 单层行为保持冻结，不被门外、电梯、楼梯或其他走廊分支吸引
3. 房门通过稳定
4. 危险源召回率明显高于最低有效水平
5. False Alarm 受控
6. 任务尽量保持在 600 仿真秒内
7. RTF 没有因高层算法出现灾难性下降
```

## D.3 性能优化顺序

若 RTF 太低，按以下顺序优化，不要首先削弱 Unitree locomotion：

```text
1. 关闭 Gazebo GUI
2. 降低 SingleFloor FSM 决策与调试发布频率
3. 降低 MapDoorDetector 频率；ScanDoorDetector 只处理必要扇区
4. 降低 2D mapper 更新频率
5. 点云 voxel/downsample/range clipping
6. RGB 移动检测频率降到 3~5 Hz
7. depth/sphere fitting 保持 event-driven
8. 降低 global planner 无意义重规划频率
9. profile 后再优化具体热点
```

不要未经 profile 就引入复杂多线程或重写全部模块。

---

# 8. 建议频率预算

以下仅作为第一版起点，全部配置化：

| 模块 | 初始目标频率 |
|---|---:|
| Unitree 官方 locomotion | 保持官方控制周期 |
| Livox -> LIO | 传感器原始点云频率 |
| IMU -> LIO | 原始频率 |
| 2D obstacle projection | 5~10 Hz |
| Floor Occupancy mapping | 2 Hz |
| Local costmap | 5 Hz |
| CorridorEstimator / CorridorController | 5~10 Hz，按实测控制需求 |
| ScanDoorDetector | 跟随必要 scan 帧，可降采样 |
| MapDoorDetector | 1~2 Hz |
| DoorFusion / CandidateManager | 事件驱动 |
| SingleFloor FSM decision | 1~5 Hz，运动输出按控制需要 |
| DWA controller（Stage C 长距离辅助） | 5~8 Hz |
| Global planner（Stage C 长距离辅助） | 约 1 Hz / 按需 |
| RGB red detector（移动） | 3~5 Hz |
| RGB red detector（房间扫描） | 最高约传感器频率 |
| Depth target localization | event-driven |
| Sphere validation | event-driven |

所有频率放入 YAML / ROS param，不写死在算法中。

---

# 9. 错误恢复要求

所有状态机必须有失败出口，禁止无限循环。

至少处理：

```text
NAV_GOAL_TIMEOUT
DOOR_CROSSING_FAILED
ROOM_UNREACHABLE
NO_NEW_ROOM_FOUND
STAIR_APPROACH_FAILED
STAIR_CLIMB_TIMEOUT
LANDING_NOT_CONFIRMED
LOCALIZATION_UNSTABLE
LOCALIZATION_STALE
```

基本原则：

- Stage B 门和退出重试统一由 `RecoveryManager` 管理，默认均不超过 2 次；
- Stage B 反向扫掠最多 1 次，不存在普通 Frontier blacklist；
- 门洞失败：退回 pre-door，重新对齐，达到预算后记录明确失败原因；
- 定位 `BAD`：停止 Map 独立新建候选，不通过普通 Recovery 修复；
- 定位 `STALE`：立即零速度停车，不消耗普通 Recovery 预算；
- 楼梯失败：停止并进入安全恢复，不继续盲目前进；
- perception 失败不能阻塞整个导航线程。

所有 retry count 和 timeout 配置化。

---

# 10. 数据记录与调试接口

每个新模块都应发布足够的调试信息，但默认不做高频大数据 logging。

建议至少提供：

```text
/current_floor
/mission_state
/locomotion_mode
/localization_health
/corridor_estimate
/exploration_map
/door_evidence
/door_hypotheses
/room_candidates
/current_room_goal
/simnav/room_frontier_markers
/recovery_state
/stair_candidate
/danger_candidates
/confirmed_dangers
/result_writer_status
```

消息类型可按现有依赖选择，不强制自定义 msg；如果使用 Marker/MarkerArray 能满足调试需求，优先复用标准消息。

日志必须能回答：

```text
为什么选择了这个 goal？
为什么一条 DoorEvidence 被拒绝或降权？
为什么一个 DoorHypothesis 被确认、合并或删除？
为什么认为一个房间已访问？
为什么确认已进房或已重新进入走廊？
当前房间前沿规划的目标点、完整路径和净空为何被选择？
为什么一个红色候选被判为方块/无效？
为什么楼层完成？
为什么进入楼梯状态？
```

---

# 11. 明确的非目标（第一版不要做）

除非前述方案实测无法满足比赛需求，不要主动加入：

```text
端到端 RL exploration
RL high-level navigation
OctoMap navigation
3D global planner
semantic SLAM
YOLO doorway detector
大型视觉模型
全时 RGB-D pointcloud segmentation
完整房间 coverage planner
复杂 NBV graph optimizer
电梯跨层主方案
为二楼复制一套 Explorer
每层重新启动独立 SLAM
第三套门洞 Detector
全局/走廊普通 Frontier 或其他走廊分支探索（房间内仅允许 B.9 定义的受限局部前沿）
Scan/Map 各自维护完整候选生命周期
用更多 Recovery 掩盖定位异常
```

如果 Codex 判断必须引入其中某项，先说明：

```text
现有方案具体失败在哪里
新增模块解决什么问题
预计实时性能代价
有没有更简单替代方案
```

再实施。

---

# 12. 最终代码应具备的配置入口

至少将以下内容参数化：

```text
max_test_floors
expected_rooms_per_floor
map_resolution
mapping_frequency
projection_frequency
height_filter_min
height_filter_max
small_gap_kernel
opening_min_width
opening_max_width
scan_strong_threshold
map_strong_threshold
fusion_spatial_gate
fusion_normal_gate
corridor_confidence_threshold
corridor_width_min
corridor_width_max
localization_stale_timeout
localization_pose_jump_threshold
localization_degraded_thresholds
pre_door_distance
post_door_distance
door_cross_speed
minimum_cross_time
max_cross_time
max_exit_time
inside_room_confirm_frames
corridor_reacquire_duration
emergency_front_clearance
normal_max_speed
room_robot_radius
room_safety_margin
room_frontier_replan_period
room_frontier_coverage_target
room_information_gain_weight
room_information_radius
room_dead_end_weight
room_frontier_target_tolerance
room_frontier_quiet_period
room_frontier_speed
room_frontier_min_speed
room_frontier_heading_tolerance
room_frontier_cluster_radius
room_frontier_min_cluster_cells
room_frontier_revisit_radius
room_topology_entry_margin
room_topology_max_depth
room_topology_lateral_limit
room_dead_end_path_threshold
room_minimum_escape_cells
max_door_attempt
max_exit_attempt
max_reverse_sweep
completion_quiet_window
stair_defer_enable
red_hsv_thresholds
sphere_radius_range
danger_cluster_distance
result_output_path
debug_result_output_path
navigation_timeout
stair_climb_timeout
landing_stable_duration
```

不要把随机场景某一次测得的位置、走廊长度、门坐标、楼梯坐标写入配置。

`expected_rooms_per_floor` 是任务配置，不得通过读取禁止文件推导；Stage B 固定测试值为 `4`，核心状态机仍按参数工作。`frontier_fallback_enable` 不属于 Stage B 配置入口；上述 `room_*frontier*` 参数只属于已确认房间的 `RoomFrontierPlanner`，不得作为走廊或楼层全局目标开关。

---

# 13. Codex 每阶段执行模板

每次只执行当前阶段，并在修改结束后按此格式汇报：

````markdown
## Stage X Result

### Files changed
- ...

### What was implemented
- ...

### Hypothesis and failure class
- Failure class: localization / scan / map / fusion / FSM / motion / perception
- Single hypothesis tested:
- Evidence collected before modification:

### Commands to build
```bash
...
```

### Commands to run
```bash
...
```

### Parameters added/changed
- ...

### Acceptance tests performed
- Test 1: PASS/FAIL
- Test 2: PASS/FAIL
- Seed matrix and frozen parameters:

### Performance
- RTF:
- CPU:
- RAM:

### Known issues
- ...

### Recommended next action
- ...
````

如果验收失败：

**不要把失败标成完成，不要直接跳到下一阶段。**

---

# 14. 最终两层任务状态机

期望最终逻辑：

```text
BOOT
  │
  ▼
START_LOCALIZATION
  │
  ▼
ENTER_BUILDING
  │
  ▼
CLOSE_MAIN_ENTRANCE
  │
  ▼
INIT_FLOOR_0_MAP
  │
  ▼
ACQUIRE_CORRIDOR
  │
  ▼
SINGLE_FLOOR_SEARCH
  │
  ├── CorridorEstimator
  ├── Scan/Map DoorEvidence -> DoorFusion
  ├── pre-door / align / sensor-confirmed crossing
  ├── room scan / danger detection
  ├── sensor-confirmed corridor reacquisition
  └── at most one reverse sweep when VISITED < expected
  │
  ▼
FLOOR_0_COMPLETE
  │
  ▼
ACTIVATE_STAIR_CANDIDATE
  │
  ▼
GO_TO_STAIR_PREPOSE
  │
  ▼
ALIGN_WITH_STAIR   [PLANE]
  │
  ▼
SWITCH_TO_STAIR
  │
  ▼
CLIMB
  │
  ▼
LANDING_DETECTED
  │
  ▼
SWITCH_TO_PLANE
  │
  ▼
INIT_FLOOR_1_MAP
  │
  ▼
ACQUIRE_CORRIDOR
  │
  ▼
SINGLE_FLOOR_SEARCH
  │
  ▼
FLOOR_1_COMPLETE
  │
  ▼
RESULT_WRITER: detected_danger.json
  │
  ▼
MISSION_COMPLETE
```

---

# 15. 最终验收定义

本两层整改版本可以认为“完成”，需要同时满足：

## 单层能力

- [ ] Stage B 同一冻结版本连续至少 5 个 seed 达到 4/4、零人工、零 `UNREACHABLE`；
- [ ] 进入建筑后不再被门外未知区域引导出去；
- [ ] Gazebo 小缝不会成为高价值探索通道；
- [ ] 电梯内部不会抢占任务；
- [ ] 楼梯在房间搜索完成前保持 deferred；
- [ ] 不探索其他走廊分支，不启用全局/走廊普通 Frontier fallback；房间内仅在 `ROOM_SCAN` 使用受约束的局部前沿；
- [ ] Scan/Map 只产 Evidence，候选生命周期只有一个管理者；
- [ ] 定位健康为 `BAD/STALE` 时执行规定的降权/停车行为；
- [ ] 主走廊和房间搜索逻辑稳定；
- [ ] 约 1.2 m 门洞具备较高通过成功率；
- [ ] 单房进门和退出尝试均不超过 2 次，反向扫掠不超过 1 次；
- [ ] 进门/退出成功由传感器事件而非固定时间确认；
- [ ] 房间扫描不追求无意义全覆盖；
- [ ] 红球可检测；
- [ ] 红方块有明确排除机制；
- [ ] 危险源可以输出 world XYZ；
- [ ] 重复观测不会重复计数；
- [ ] `results/detected_danger.json` schema 正确、时间来自 `/clock` 且可直接评分。

## 双层能力

- [ ] 第一层完成后自主前往楼梯；
- [ ] 能在平地 policy 下完成楼梯对正；
- [ ] 能切换 stair policy 并完成上楼；
- [ ] 上楼期间连续定位不中断；
- [ ] 楼梯期间不会污染 Floor0 2D map；
- [ ] 到达平台后能够检测 landing；
- [ ] 能切回 plane policy；
- [ ] 能创建独立 Floor1 2D map；
- [ ] 第二层复用同一个 SingleFloorExplorer；
- [ ] 两层危险源坐标统一在 world frame；
- [ ] 多个随机两层场景中能够无人干预完成任务。

## 工程质量

- [ ] 无正式算法 GT 依赖；
- [ ] 两层不是核心逻辑硬编码；
- [ ] 参数集中配置；
- [ ] 状态机有明确失败出口；
- [ ] 关键选择有调试日志；
- [ ] 每次失败先分类并记录首个根因证据；
- [ ] 没有第三套门洞 Detector 或重复候选管理器；
- [ ] Stage B 通过后未因双层需求改写 SingleFloorExplorer；
- [ ] 已记录与官方 baseline 的 RTF / CPU / RAM 对比；
- [ ] 没有为了当前 seed 修改官方建筑生成逻辑。

---

# 16. Codex 的最高优先级决策规则

当实现过程中存在多个可选方案时，按以下规则选择：

```text
1. 是否比赛合规？
   否 -> 禁止。

2. 是否能提高“访问房间 + 找到红球”的可靠性？
   是 -> 优先。

3. 是否只是提高地图覆盖率，但不提高危险源搜索能力？
   是 -> 降低优先级。

4. 是否显著增加 Gazebo/CPU 负担？
   是 -> 先寻找更轻方案。

5. 是否会让单层逻辑和双层逻辑变成两套代码？
   是 -> 重构为复用 SingleFloorExplorer。

6. 是否为了一个随机 seed 写了位置/尺寸特例？
   是 -> 禁止。

7. 失败是否已经归类，并有日志证明根因属于准备修改的模块？
   否 -> 先补诊断，不改无关阈值。

8. 是否新增了已有模块正在承担的候选、走廊或恢复职责？
   是 -> 合并到唯一职责所有者，不增加并行机制。

9. 是否准备在同一轮同时改多个核心模块？
   是 -> 拆成可独立验证的 B-1/B-2 或单一假设改动。
```

最终目标不是生成漂亮地图，而是：

> **在官方随机建筑中，以可接受实时负担稳定搜索前两层房间，可靠识别红色球形危险源，并输出统一 world 坐标。**
