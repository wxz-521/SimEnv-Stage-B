# Stage B 三层任务：当前工作介绍与全部机制详情

> 更新于 2026-09-12（run65 运行中）。本文档描述**当前代码线**的真实状态：目标判据、历史最好成绩、运行环境、全部机制、参数与诊断、以及接下来的工作。

---

## 1. 目标与达标判据

**任务**：Unitree A1 在仿真建筑内完成三层探索，随后自主下降、开大门、返回出生点。

**达标判据（不允许降低）**，以 `result.json` 为准：

| 字段 | 要求 |
|---|---|
| `completed_floor_indices` | `[0, 1, 2]` |
| 每层 `floor_results[n].completed_topologies` | 4 间小房间全部完成（各房间自身 combined 覆盖率 ≥ 0.55） |
| `returned_to_spawn` | `true` |
| `main_entrance_opened` | `true` |
| `mission_fault` | `null` |

探索策略约束（用户指定）：**纯几何 / 信息增益方法，禁止强化学习**。

判据实现要点：
- 每层完成 = `len(completed_topologies) >= expected_rooms_per_floor(4)` 且稳定 3 s（`topology_completion_ready`）。
- 房间完成 = 该房间掩膜下的 combined 覆盖率 ≥ `0.55`（`room_combined_coverage_target`），**逐房间记账**，不因分区而改变。
- 覆盖率为 `weighted_linear_coverage(laser, camera, camera_weight=0.95)`。

---

## 2. 当前最好成绩（有据可查）

| 轮次 | 达到的最远状态 | 证据 | 失败点 |
|---|---|---|---|
| **run49**（旧策略，分层图之前） | **0 层 4/4 + 1 层 4/4**，电梯正在上 2 层 | `result.json`: `completed_floor_indices=[0,1]`，`floor_results` 两层各 4 间，`elapsed_sim_time=545` | `MISSION_FAULT: ELEVATOR_PATH_BLOCKED_AFTER_0.93M`（1 层轿门未开，**已修**） |
| **run64**（当前代码线） | **0 层 4/4 完成并锁层 → 干净换层到 1 层 → 1 层 3/4** | 日志：`TASK_REGION_COMPLETE rooms=4/4`；`elevator_progress.log`：`EXIT_ELEVATOR 7.5 sim s`、`ESTABLISH_FLOOR_1_TOPOLOGY 12.9 sim s`；探针：`completed_topologies=[L_15,R_15,R_43]` 且 `ROOM_L_43` 未完成 | 1 层 3/4 时**全图零候选**（`generated_kind_counts={}`）→ 停滞（**已修**，见 §4.4） |
| run59 | 0 层完成 218.9 仿真秒 → 干净换层 → 1 层 3/4 | `elevator_progress.log` 全序列均在预算内 | 脚下 0.28 m 的退化前沿被判"已到达"→ 原地空转（**已修**：末段前蹭） |
| run65（进行中） | 0 层 3/4 | 见运行日志 | — |

**结论**：三层探索**尚未**端到端达成；**已端到端验证过的**是"0 层完整探索 + 锁层 + 电梯上行到 1 层"这一整段（run64 全程零故障）。1 层剩余房间、2 层、以及下降/开门/返回出生点**仍未验证**。

---

## 3. 运行环境与硬约束

| 项 | 值 |
|---|---|
| 仿真 | ROS1 Noetic + Gazebo Classic（headless），`max_step_size=0.002`、`real_time_update_rate=500` |
| 容器 | `simenv-noetic`，宿主网络；ROS master 端口 **11320**，Gazebo **11345** |
| 机器人 | Unitree A1，FAST-LIO 定位，`/cmd_vel` 控制 |
| **实时率 (RTF)** | **≈0.09–0.15**（实测：sim 2 s / wall 21 s = 0.095） |
| **单轮成本** | 三层全流程 ≈ 600–800 仿真秒 ≈ **2 小时真实时间** |
| 瓶颈定位 | `gzserver` 约 146% CPU（约 1.5 核跑满）+ `junior_ctrl` 约 73%；监控与所有 python 节点合计约 35%；跨轮残留进程约 5% —— **低 RTF 是物理配置固有，不是代码或残留造成的**（用户决定不改物理参数） |

**由此产生的设计铁律**：
1. 一切**会终止运行**的看门狗必须用**仿真时钟**判定（否则会把正常等待误杀）。
2. 每次改动必须有**离线验证**（单元测试 / 静态检查 / 现场地图回放），因为一次运行时错误 = 2 小时。

---

## 4. 机制详解

### 4.1 建图：按高度分开的多张 2D 图（`lio_occupancy_node.py` + `map_floors_core.py`）

- 旧行为：**单张 2D 图**，切层时 `grid.fill(-1)` **清空重画** → 拓扑必须"复用首层"、id 反复漂移、返回 0 层时无图可用。
- 现行为：`grids = {楼层: 2D 图}` **同时保留**，切层只**切换活动图**；
  - 层归属：以该层**实测最低机器人高度**为基准，点云按机器人相对高度带 `[-0.10, 1.30] m` 判归属（层高 2.6 m 时上层墙体落在带外，天然隔离）；带宽若触及相邻层高会告警。
  - 发布：`/map` = **当前层**（兼容原消费者）；每层 `/map/floor_{n}`；`/simnav/map_floors` 给出每层**高度与来源（observed/nominal）**、活动标志、已知/自由/占据格数。
- 切层时机：**电梯一落地就切**（`_publish_floor_context("arrival")`），不再等"拓扑建立完成"——否则会拿着上一层的图规划（run52 曾因此 A* 失败 654 次、5 分钟只挪 2 m）。
- 下降时同样发布 `floor_index=0`，使 0 层图**重新激活**，为"返回出生点"提供真实地图。

### 4.2 走廊分区：近区 / 远区，区内自由进出（用户指定策略）

- 边界：`zone_split_along()` = **已探测门洞 along 的中点**（无门洞时取任务纵深一半），并带 **±1.5 m 缓冲带**，避免跨中点的对房被拆到两区。
- 排序：`active_zone()` —— 近区只要还有未完成门洞就一直在近区；**近区全部完成后才允许去远区**。
- 区内自由：**锁住某房间时，本区半截走廊的候选依然可派发**（`zone_corridor_targets`），所以"进门后无前沿候选"时机器人可以退回/转去区内其它目标，而不是干等。
- 不变项：房间掩膜、逐房间覆盖率、`completed_topologies` 记账**完全未改**。

### 4.3 门 / 门户身份与穿越

问题背景：门户 id 是 **0.5 m 分箱标签**，随 SLAM 精化漂移（同一物理门可能是 `ROOM_L_10 / L_15 / L_43 / L_47 …`）。

- `doorways_match(a, b, tol=2.5)`：**同侧且 |Δalong| ≤ 2.5 m** 即视为同一物理门；`front_stations_complete()` 用它判断首对房间是否完成（此前只按 id 成员判断，导致 `front_rooms_complete` 永远 False → 走廊把所有新前沿按 `wrong_topology` 拒掉，run46 2 层死锁）。
- `door_crossing_along_offsets()`：门内窄搜索（±0.30 m，原逻辑）**之后**追加**有界宽域搜索（±2.5 m，步长 0.5 m）**，去程（`navigation_path_through_portal`）与**返程**（`navigation_path_from_room_through_portal` 的调用点）都已统一。
- `path_is_safe(..., despeckle=True)`：**在岗路径**改在**路由器同一张去斑图**上校验（此前路由去斑、校验不去斑，门框一个斑点就让路径恒判"不安全"→ 每周期丢目标、门口自旋）。
- `verified_door_band()`：对**已确认门洞**画门法向连通带（已知自由 + 净空 ≥0.12），把被斑点膨胀掐断的房间重新接回可达集。

### 4.4 防死锁兜底链（按优先级）

**房间锁定期间**：
1. 房间内候选（相机优先，其次激光）；
2. **本区半截走廊候选**（"允许出门"）；
3. 全空时 → `_recover_idle_lock()`：
   - 已进房 → 走**已验证的返廊路径**（房间保持未完成，稍后可再进）；
   - 未进房 → **释放锁**让区内继续（有界：`max_lock_releases=3`）；
   - 超限 → 该门标 `BLOCKED` 并 retired，不再反复锁。

**走廊（无锁）**：
1. 本区激光前沿（限活动区）；
2. **相机兜底**（有未完成门洞时放开所有权限制）；
3. **走廊巡游**（`corridor_wander_target`）：本区中线上的可达点（采样 ±2…±14 m，已访问半径减半）——走廊是自由空间，必定可派发；
4. **门口接近**（`_door_approach_target`）：全图零候选但仍有未完成房间时，直接派发"去该房间门口"的目标（复用同一 helper）；
5. **路由失败后**再走一遍 3/4（此前兜底都在路由之前，候选在路由阶段才清空时无人接手）。

所有兜底都在诊断里留痕：`corridor_camera_fallback / corridor_wander / door_approach_dispatched / door_approach_portal / lock_release_counts`。

### 4.5 活性护栏（防止"静默死亡 / 静默停车"）

| 机制 | 作用 | 触发时的行为 |
|---|---|---|
| `_control` 异常护栏 | rospy.Timer 回调抛异常会**永久杀死线程** | 记 `CONTROL_FAULT #n` + traceback，**循环继续** |
| `_plan` 异常护栏 | 同上（run57 静默停在 sim 219 就是这个） | 记 `PLAN_FAULT #n`，**循环继续** |
| 停滞目标兜底 | 目标在岗但**真实位移** <0.15 m 且转向 <0.20 rad 达 5 s | 丢弃该目标并冷却，让规划器换一个 |
| 不安全路径去抖 | 单次"路径不安全"读数立即重派发会造成 A/B 抖动 | 连续 3 个规划周期才重派发，并把旧目标冷却 45 s |
| 空路径可见化 | "有目标但无可执行路径"过去**静默停车** | 限流告警 + 2 周期后清理目标 |
| 末段前蹭 | 目标落在 `target_tolerance` 内但仍在 0.20 m 外时会被判"已到达"→ 永不移动 | 以 0.25 m/s 直线前蹭，直到 0.20 m 内才算到达 |

### 4.6 电梯链路（`elevator_transition_node.py`）

状态链：
`WAIT_FLOOR_COMPLETE → RETURN_TO_ELEVATOR → ALIGN_ELEVATOR → ENTER_ELEVATOR → RIDE_TO_FLOOR_1 → ALIGN_FLOOR_1_EXIT → EXIT_ELEVATOR → ESTABLISH_FLOOR_1_TOPOLOGY → FLOOR_1_READY → RETURN_TO_FLOOR_1_GATE → ENTER_FLOOR_1_LOBBY → SEARCH_FLOOR_1_ELEVATOR → ALIGN_FLOOR_1_ELEVATOR_RETURN → ENTER_FLOOR_1_ELEVATOR_RETURN → (RIDE_TO_NEXT_FLOOR | RIDE_TO_GROUND_FLOOR) → ALIGN_GROUND_FLOOR_EXIT → EXIT_GROUND_FLOOR → OPEN_MAIN_ENTRANCE → RETURN_TO_SPAWN → RETURNED_TO_SPAWN`
（`FLOOR_1_*` 是**楼层无关**的沿用命名，实际按 `current_floor` 工作。）

关键机制：
- **直行暴力进入（`direct_entry`）**：≤4 m 的短距视线目标**不走 A***，先**原地对准**（误差 >0.25 rad 只转向、绝不前进），再直线行驶，横向用左右净空做**门洞居中**；被堵超 25 s 才回退 A*。长距离（返回出生点）仍用 A*。
- **自动开轿门**：进轿前按 `elevator_floor_{current_floor}` 调 `/set_door_state(True)`（场景里 1 层轿门 `initial_open=false`，必须显式开），每次落层后重置。
- **进轿早接触重试**：行程未达 `minimum_entry_progress` 就撞停时，**退让 2.5 s**（让门动画开完）再试，最多 4 次，超限才失败。
- **倾倒判据改用机体 IMU**：`tilt_fault_state()` 用 `/trunk_imu` 的姿态，并要求**持续 0.4 s** 才判 `ROBOT_ROLLED`（metric 姿态在急转会单点跳变到 0.45 rad 而 IMU 仅 0.12 rad，曾误杀 run63）。
- **建立拓扑有界**：`establish_budget_exceeded()` 超过 150 s 实时预算就带警告继续（该步只是软提示）。
- **参考层采集解耦 + 有界等待**：参考层拓扑只作**提示**；采集不再因缺 `virtual_isolation_door` 被跳过，且 30 仿真秒拿不到就继续。
- **开门判据严格化**：`OPEN_MAIN_ENTRANCE` 要求 `accepted && state=="open"`（原先无条件置 `main_entrance_opened=True`，会假通过并撞关门）。

### 4.7 监控与看门狗（`team_scripts/`，共 6 个）

| 脚本 | 判定 | 时间基准 | 终止运行 |
|---|---|---|---|
| `watch_stage_b_status.py` | 逐周期打印状态/房间覆盖 | 仿真 | 否 |
| `watch_stage_b_freeze.py` | 覆盖停滞 forensics + 诊断转储 | 仿真 | 否 |
| `watch_position_stall.py` | **位移**停滞（转向不算进度） | **仿真秒**（180） | 是 |
| `watch_floor_deadline.py` | 单层预算（900 仿真秒） | **仿真秒** | 是 |
| `watch_elevator_progress.py` | **每状态耗时预算 + 动作质量**（位移/转角/非零指令占比），另有真实时钟兜底（1800 s） | **仿真秒** | 是 |
| `watch_run_periodic.py` | 每 5 分钟真实时间汇报 | 真实 | 否 |

辅助工具：
- `capture_stall_scene.py`：抓取**现场** `/navigation_map` + 门户几何，离线**复算**（把"猜"换成"测"，run50 就是这样定位到门内未知）。
- `probe_explorer_status.py`：解析式读取探索状态（避免 `rostopic echo` 的转义地狱）。

### 4.8 离线质量门禁

| 门禁 | 内容 |
|---|---|
| `python3 -m unittest`（容器内） | **173 条**测试全绿（core 层纯函数：分区、门户匹配、穿越偏移、路径去斑、兜底判定、电梯判据、分层图） |
| `check_read_before_assign.py` | 抓"先读后赋值"（run44 控制线程致死同型缺陷） |
| `check_undefined_attrs.py` | 抓 `self.x` **从未定义**（run60 的 `self.stop_distance` 笔误，护栏下仍浪费一轮） |

---

## 5. 参数与诊断速查

**探索**：`room_combined_coverage_target=0.55`、`camera_weight=0.95`、`target_rank_mode=nearest`、`portal_merge_radius=0.0`、`room_exit_requires_gain=false`、`max_lock_releases=3`、`creep_stop_distance=0.20`、`creep_speed=0.25`、`unsafe_path_replan_cycles=3`、`unsafe_path_block_seconds=45.0`、`stuck_target_seconds=5.0`、`front_station_match_tolerance=2.5`。

**电梯**：`direct_entry=true`、`direct_entry_max_distance=4.0`、`direct_align_tolerance=0.25`、`direct_fallback_seconds=25.0`、`entry_retry_limit=4`、`entry_retreat_seconds=2.5`、`minimum_entry_progress=1.0`、`establish_timeout=150`、`fall_tilt_persist=0.4`、`imu_fresh_seconds=0.5`、`reference_floor_wait=30.0`、`main_entrance_max_attempts=6`。

**关键诊断字段**：`last_plan_reason`、`planner_diagnostics{active_zone, zone_split_along, zone_topologies, zone_corridor_targets, corridor_wander, door_approach_*, candidate_reject_counts, generated_kind_counts}`、`control_faults / plan_faults / empty_path_cycles / stuck_target_drops / unsafe_path_replans / lock_release_counts`。

**失败取证顺序**（务必按此序，勿猜）：
1. `result.json`（`mission_fault` / `completed_floor_indices` / `returned_to_spawn`）；
2. 看门狗判定 JSON（`*_verdict.json`）+ `elevator_progress.log`；
3. `telemetry.csv`（`cmd_vx/cmd_wz` 与实际位移、`imu_roll` vs `wroll`）；
4. 探索/电梯日志的**最后若干行**（注意日志目录跨轮共用，需按时间过滤）；
5. 必要时 `capture_stall_scene.py` + 离线复算。

---

## 6. 接下来的工作

**短期（把三层跑通）**
1. 观察 run65：重点验证 §4.4 新增的"**全图零候选 → 门口接近**"兜底能否让 1 层 3/4 → 4/4。
2. 首次跑通 1 层完整后，验证 **1→2 层换层**（会用到 `elevator_floor_2` 轿门自动开启）与 **2 层探索**。
3. 首次验证**下降尾段**：`RIDE_TO_GROUND_FLOOR`（含 `floor_index=0` 上下文）→ `EXIT_GROUND_FLOOR` → `OPEN_MAIN_ENTRANCE`（严格 accepted 判据）→ `RETURN_TO_SPAWN`（A*，依赖保留的 0 层图）→ `returned_to_spawn=true`。
4. 每个失败按 §5 的取证顺序定位根因，改代码 + 加离线测试后再复测。

**中期（降低迭代成本，提升稳定性）**
5. 把"反复出现"的问题做**结构性收敛**（详见 `stage_b_recurring_difficulties.md`）：门户 id 的稳定身份、栅格/姿态的**单一事实来源**、兜底链从"逐个补"变为**统一的可派发目标生成器**、看门狗与任务的**共享状态机契约**。
6. 建立**尾段回归**手段：不依赖每次 2 小时的完整跑，用离线/脚本化方式验证 1 层、2 层与下降状态链（例如按层注入模拟状态、或从已完成楼层的存档地图直接跑规划器）。
7. 危险源漏检是**独立指标**（当前 recall 约 1/5）：在探索完成与返回稳定后，再评估"红球定向近距离扫视"方案（此前测得约 +170 m / +283 仿真秒每层）。
