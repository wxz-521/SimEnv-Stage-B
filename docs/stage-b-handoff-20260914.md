# Stage-B 会话交接（2026-09-14 交接点）

> **本文件用途**：跨会话交接。新会话请**先读这一份**，再读 `docs/功能模块设计.md`（活文档，含完整设计/变更记录）。
> **维护规则**：每次交接前更新本文件；大改仍写进 `功能模块设计.md` 的「变更记录」。

---

## 0. 当前目标（唯一）

**seed 20260902 / 55% 覆盖 / 三层 / 回出生点，一次跑完。** 跑通后再切 **84%**，并从那时起把**红球检测效果**纳入验收。

**验收判据（用户明确）**：
1. 不出现「进房 → 对准出门 → 再进房」；
2. 不出现目标点在**左右房之间反复横跳**；
3. 三层各间房覆盖达标 → 电梯上下三层 → 回到出生点；
4. 84% 阶段另加：红球召回 / 误报 / **坐标误差**（匹配容差 **1.0 m**，见 `docs/evaluation.md:48`）。

---

## 1. 代码状态（本会话结束时）

### 生效中

| 项 | 位置 | 说明 |
|---|---|---|
| **RTF 修复（7 倍提速）** | `src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro` 第 305/311 行 | 传感器自带网格 `<samples>100/360</samples>` → **2/2**。插件块（328 起，含 332/338/356）**一行未动**。RTF 0.124 → **0.866**，点云宽度不变（2354~2372，中位 2361），`lio_health=GOOD` |
| **三门槛统一** | `stage_b_behavior.launch`(68/83/84) + `stage_b_floor_explorer.launch`(8/9) + `two_floor_explorer_supervisor.py` | `room/camera/combined` 三个覆盖率门槛**同源于 `room_combined_coverage_target`**。曾因"房间按 0.55 出门、相机按 0.85 未达标"造成"出门后又进房" |
| **上层楼入场直行 9 m** | `two_floor_explorer_supervisor.py:110` | 环境变量 `STAGE_B_UPPER_FLOOR_FORWARD`，默认 **9.00**（原 0.0，后试 3.50 太短）。理由：走廊口→前门 7.0 m，需越过前门且不越过 `zone_split`(~17.5) |
| **⑥ 目标可换（自失效判据）** | core `target_switch_allowed()` + node `_switch_active_target()` / `_active_remaining_information()` | **用户认可保留**。触发条件 = **当前视点自己的剩余信息塌缩**（`active_remaining ≤ 0.25`），**不是**两目标比增益。视点从池中消失 → 视为已观测（返回 0.0）；判断不了 → 拒绝换 |
| 视点朝向 | core `_camera_look_at()` + `_room_interior_yaw()` | 由"未观测区**质心**"改为"**按方位 16 箱取未观测面积最大的方向**"（质心会退化成视点自身 ⇒ `atan2(0,0)` ⇒ 朝向塌成固定世界轴，即"门口横着"）。并列时用"房间内部方向"打破 |
| 走廊/大厅/身后禁区 | core 相机兜底的内联条件（`>= -0.5` 两处） | 走廊不接受相机前沿；大厅（闸门以内 <−0.5 m）不接受前沿；机器人身后不选 |
| 受护启动 + 开局健康闸 + 自动采样器 | `team_scripts/testonly_launch_guarded.sh` + `testonly_startup_health.py` | **测试专用，最终版删除**（主线 `run_stage_b_seed.sh` 无引用）。静默 15 s + 等进程真清空 + 端口检查 + 15 仿真秒健康闸（`wz∈[0.15,0.55]`、`|roll|≤0.35`、`|src|≤100 m`）+ 重试 3 次 + 自动挂采样器 |
| 闸门伪门户过滤 | — | ❌ **已回退**（见下） |

### 已回退（都验证过是副作用源）

| 项 | 回退原因 |
|---|---|
| ④ 相机兜底触发条件（"列表空"→"无房间目标"） | **本会话所有目标选择类退步的共同源头**：去大厅、往回走、走廊打转、门口选远处目标 |
| T1(a) 路径前插机器人位姿 / T1(b) 清脚下小圈 | 用户判定"更绕路了"（0.4 m 半径会误伤门框墙；后改 0.15 m 受 `navigation_clearance` 封顶，仍被判定无效，一并回退） |
| T2a-② 相机兜底接 `admit_candidate` | 建在 ④ 之上；随 ④ 一起回退 |
| **闸门伪门户过滤**（`along < 2.5 m` 拒收） | **用户判断正确**：那两个"小门"（`ROOM_L_0/ROOM_R_2`）一直在**补偿闸门不准**，删掉后闸门误差裸露 ⇒ 问题集中爆发。**该修的是闸门本身** |

### 未接线（在库中，备下一步使用）

- core `admit_candidate()`：**单一准入闸**（LOBBY / KIND / CORRIDOR_CAMERA / BEHIND / ZONE / ROOM_COVERED），返回 `(admitted, reason)`，含 **7 项契约测试** `CandidateAdmissionTest`。**未接入主干**。
- 相关可测不变量（建议接线时一并落地）：大厅候选=0、身后候选=0、走廊相机候选=0、**未探索房间目标数 ≥1**。

**离线回归：223 项全绿**（`docker exec` 内 `python3 -m unittest discover -s src/simnav/test -p "test_*_core.py"`）。

---

## 2. 已证实的测量（不要重新推演）

| 测量 | 数值 | 来源 |
|---|---|---|
| RTF | 改前 **0.124** → 改后 **0.866** | `/clock` 与墙钟对比；时间线核对 104 仿真秒/121 墙钟秒 |
| 电梯全循环（独立脚本） | **PASS**：两段上楼 + 两次出梯；真值 z 0.307→5.530，轿厢 1.030→6.230 | `team_scripts/elevator_only_test.sh` + `elevator_only_driver.py` |
| 6 cm 门槛通过条件 | **台阶步态 + 0.45 m/s**：stair@0.45 **3/3**、plane@0.45 **1/3**、stair@0.35 FAIL | 独立脚本 A/B 矩阵 |
| **下降**门槛必须**倒车** | 正着出梯卡满 35 s 预算超时；倒车出梯 **8~9 s 成功**。主线已改 `_drive_distance(..., reverse=True)` + 删除骑乘后 π 转身 | 同上 |
| 真实前门识别精度 | 识别 along **7.50 m** vs 真值 **7.57 m**（差 0.07 m）✅；左右 PAIRED 正确 | 门户 id 为 0.5 m 分箱 |
| **地图/坐标系的真实缺陷** | 墙体方向 **93.03°** ⇒ **恒定约 3° 旋转**；与"位姿对 vs 闸门对"算出的 Δyaw 差 **3.4°** 是同一件事；另有平移差约 **3 m** | 占据图结构张量拟合 + 逐帧遥测 |
| 该 3° 的后果 | 半径 r 处位置误差 ≈ `r × 0.0523` ⇒ **r > 19.1 m 时超过 1.0 m 匹配容差**（建筑纵向 36 m）⇒ **红球坐标会配错/配不上** | `docs/evaluation.md:48`（阈值 1.0 m）+ 几何推算 |
| 规划器成本 | 真实地图 600×800：`clearance` 场 15 ms；128 相机候选 × 600 点路径 23 ms（最坏 1200 点 49 ms） | 离线基准 |
| CPU 分布 | 探索器 41% 单核；rviz 146% + gzserver 127% + junior_ctrl 67%（24 核、load≈8） | `top` |

---

## 3. 待办（按优先级）

### T-A · **P3b「去远区固定步长前进」**（最高优先，用户反复要求）

**现状**：`coverage_explorer_core.py:3380-3396` 有明确 **TODO(P3b)**：
```python
# TODO(P3b): replace this ranking with a *fixed-step* transit target
# (corridor centreline a fixed number of metres ahead in the positive
# corridor direction, shortened until navigable).
targets.sort(key=lambda item: (-projected_travel(robot_pose, item.target, forward_yaw), item.path_length))
```
即 P3 只做了"排序取最远可达前沿"，**"固定步长前进"从未实现** ⇒ 目标随候选池变化 ⇒ 走走停停/来回。

**零件已存在**：`corridor_wander_target()`（core:3089）已实现"沿走廊中线按固定步长（2/4/6/9/14 m 阶梯）取点 + `zone_admits` 分区准入 + 可通行检查"，但**仅作为最后手段**（core:3322 / 3624 两处调用）。

**做法**：把"固定步长前进"从最后手段提升为**远区通行目标**——`active_zone == B`（或前站完成）时，走廊阶段直接派发该合成点（**只取向前阶梯**），不再排序前沿池。**必须同步更新走廊契约测试**（TODO 已提醒）。

**支持证据（run106，交接时仍在跑）**：前站两间已退休后，时间线显示长时间停在
`CORRIDOR`、`target_topology=CORRIDOR`、`stuck_drops` 从 21 涨到 24，而后区房覆盖几乎为 0
（`L_43=0.026; L_49=0.055; L_56=0.007; R_43=0.236`）——**"前站做完却没能有效推进到远区"**，
正是 P3b 缺失的直接症状（排序挑前沿 ⇒ 目标随池变化 ⇒ 反复丢弃）。

### T-B · 切 84% 并建立红球基线

一条环境变量即可：`STAGE_B_ROOM_COMBINED_COVERAGE_TARGET=0.84`（或不传，launch 默认即 0.84）。三门槛已同源。
每轮报：`result.json` 的 `danger_evaluation{recall,false_alarm_rate,correct,missed}` + **每个检出点到真值的三维距离**（阈值 1.0 m）；`team_scripts/evaluate_stage_b_danger.py` 已存在。

### T-C · 闸门结构拟合（红球坐标基准）

用**走廊入口横墙的墙线 + 该墙上的开口中心**拟合闸门（而非靠左右墙距之差微调），顺带得到墙线方向 ⇒ **一并修掉那 3° 旋转**。现有 `maximum_gate_recenter=0.80` 的重定心是**单帧、判据宽、直接改全局基准**，是闸门不准的主嫌；但它**不能简单降权限**——用户明确说明它的目的是**红球坐标基准**。

### T-D · 「进房后又出来再进」的剩余排查

已核：**A0**（进房路径航点仅 `(door_centre, room_entry, target)`，无走廊中线航点）**完好**；**P0.1**（`_mark_room_entered_locked` 用位姿 `along/lateral` 判定，非路径进度事件）**完好**。
剩查：**三门槛统一 0.55 与"出门/退休"的时序**——房间到 0.55 触发 `RETURN_TO_CORRIDOR`，而退休在其后 ⇒ 中间那一拍规划器可能仍能派房内目标。**用 `entries` 计数 + 锁定切换序列判定，不看截图。**

### T-E · 可观测性补齐（零行为风险，建议先做）

把 `room_entry_sequence = [(sim, room_id, 第几次), ...]` 与 `path_start_offset` 写进诊断。当前"进门次数"只在 `entries` 里、采样器 120 s 一次且常为 `None`，导致**用户与我反复互相误判**（用户看 RViz 说退步、我看日志说正常）。

### T-F · 命名债

`elevator_transition_node.py` 的 `floor1_gate/floor1_complete/floor1_topology_isolated/floor1_context_published` **实际承载所有上层楼**。改名 `upper_floor_*` **同时是状态 JSON 的 key**，而 `sample_run_status.py` / `verify_three_floor_run.py` / `monitor_stage_b_coverage.py` 在读 ⇒ **必须内部改名 + payload 保留旧 key 一版**，不能裸改。

---

## 3.5 ⚠️ 交接时的运行状态（**已确认：仿真已全部停止**）

交接时复核（`pgrep -af "run_stage_b_seed|coverage_explorer_node|gzserver"`）：**无任何仿真空跑**，
无需清理。两个最近轮次都在中途停止，最后观测点如下，**都只作参考、不是完整样本**：

| 轮次 | 代码状态 | 最后时间线 | 现象 |
|---|---|---|---|
| `run106_fwd9_20260913` | 旧代码（带闸门过滤 + 旧 ⑥；仅"9m 直行"与当前源码一致） | **sim 547.9, floor 0, CORRIDOR** | 前站两间已退休却长时间停在走廊，`stuck_drops` 21→24，后区房覆盖≈0（`L_43=0.026; L_49=0.055; L_56=0.007; R_43=0.236`） |
| `run105_rtf_20260913` | 旧代码（带闸门过滤 + 旧 ⑥；含 RTF 修复） | **sim 773.5, floor 1, CORRIDOR** | 已上到二楼（电梯链路正常），RTF 实测 0.866 |

⇒ **两轮都停在走廊未完成**，与 T-A（P3b 缺失）症状一致。新会话从干净状态起跑即可。
**注意**：当前源码与这两轮的进程代码**不一致**，不要拿它们的结果去评价当前源码。

```
run106_fwd9_20260913   仍在运行（受护启动，采样器在挂）
  它的进程里是【改动前】的代码：
    · 带闸门伪门户过滤（已回退）
    · 旧版 ⑥（Pareto/增益比，已改为自失效判据）
    · 上层楼直行 9.00 m（这一项仍然有效）
  ⇒ 它只能用来参考"9 m 直行"这一项；其余结论不适用
  ⇒ 新会话要么先 `kill_sim_processes.sh` 干净停掉它，要么明确按"旧代码结果"看待
```

## 4. 怎么跑（命令）

```bash
# 受护启动（推荐；自动清干净 + 健康闸 + 挂采样器）
docker exec -d simenv-noetic bash -lc 'cd /workspace/SimEnv && \
  STAGE_B_GUI=false STAGE_B_ROOM_COMBINED_COVERAGE_TARGET=0.55 \
  bash team_scripts/testonly_launch_guarded.sh 20260902 2400 \
       logs/runXXX/seed_20260902 three_floor 3 > /workspace/SimEnv/logs/runXXX.outer.log 2>&1'

# 看进度（时间线 / 电梯 / 探索器）
tail -4 logs/runXXX/seed_20260902/verify_timeline.log
# 电梯状态、探索器诊断、RTF 等见本文件第 2 节的量法

# 停干净
docker exec simenv-noetic bash /workspace/SimEnv/team_scripts/kill_sim_processes.sh

# 离线回归
docker exec simenv-noetic bash -lc 'cd /workspace/SimEnv && source /opt/ros/noetic/setup.bash && \
  source .simenv_build/devel/setup.bash 2>/dev/null && \
  export PYTHONPATH=/workspace/SimEnv/src/simnav/scripts:$PYTHONPATH && \
  python3 -m unittest discover -s src/simnav/test -p "test_*_core.py"'
```

**关键路径**
- 场景真值：`generated_building/layout_metadata.json`（12 房=4×3 层、闸门/前门/后门坐标、电梯门 y=2.6、`floor_height=2.6`）
- 地图/位姿：`/navigation_map`（源图）、`/exploration_map`、`/simnav/odom`（源图）、`/simnav/world_pose_metric`（世界系，**不可靠**：z 曾报 −2.37 m）
- 电梯：`/simnav/elevator_status`、`/call_elevator`、`/set_door_state`
- 危险源：`/simnav/danger_candidates` / `danger_tracks`，真值 `generated_building/danger_truth.json`，**5 红球 + 4 红色干扰箱**，`danger_guidance_level=1`

## 5. 资产

| 文件 | 用途 |
|---|---|
| `logs/snapshot_before_rtf_20260914_0322.tgz` | **RTF 改动前的源码快照**（src/simnav + team_scripts + docs） |
| `gazebo.xacro.bak_prertf` | RTF 改动前的 xacro 备份 |
| `team_scripts/elevator_only_test.sh` / `elevator_only_driver.py` | **独立电梯全循环测试**（不加载探索器/电梯节点，自控 `/cmd_vel`，真值判定）。改法：`TARGET_FLOORS="1,2"`、`SKIP_ENTRANCE=1`、`CROSSING_SPEED`、`GAIT_POLICY` |
| `team_scripts/elevator_only_matrix.sh` | 独立测试矩阵（速度/步态/进出时限 A/B） |
| `team_scripts/testonly_launch_guarded.sh` / `testonly_startup_health.py` | 受护启动 + 开局健康闸（**测试专用，最终删除**） |
| `team_scripts/kill_sim_processes.sh` | 安全清进程（模式表在文件内，避免 `pkill -f` 误杀调用者） |
| `team_scripts/record_elevator_phase.py` | 电梯阶段 0.5 仿真秒高频记录（只读） |

---

## 6. 流程铁律（本会话用血换来的，请遵守）

1. **只报仿真时间**。RTF 0.87 ⇒ 1 仿真秒 ≈ 1.15 墙钟秒；改前是 0.124（≈8 秒）。**用墙钟读数判断"是否卡住"会得出完全错误的结论**（我犯过两次：把 ~3 仿真秒的机动当成 18 秒卡死）。
2. **改一处、验一处，一次只加一件**。本会话连续三次副作用（去大厅、往回走、走廊打转）都源于"在没有验证连带影响的情况下加新池子"。**加新候选池时，必须同时套上既有的区域/方向/完成约束。**
3. **先量后改**。半径、阈值、距离都从几何或数据算出来（例：清圈半径受 `navigation_clearance` 封顶；上层楼直行 9 m 由"走廊口→前门 7.0 m"推出）。**不要拍数字。**
4. **不要靠截图判断逻辑**。用 `entries` 计数、锁定切换序列、`stuck_drops`、真值坐标。用户与我本会话多次因"看图 vs 看日志"互相误判。
5. **测试工具与主线分离**，测试专用文件用 `testonly_*` 前缀，最终版删除；主线 `run_stage_b_seed.sh` 保持无引用。
6. **启动是易错环节**。`kill -9` 异步，3 秒后拉新世界会让模型被弹射（world z 0.32→7.43→−11.1 m，sim 32 摔倒、LIO 发散到 −51 m、整轮报废）。必须静默 15 s + 复核进程为空再启动（受护脚本已内建）。
7. **用户判断优先**。本会话用户三次判断正确（启动问题、门户过滤是罪魁、3.5 m 太短），我三次判断偏差。**先按用户方向查证，再谈自己的假设。**

---

## 7. 续会话：改动与实测（2026-09-14 下午）

> 本节改动全部落在主线；离线回归
> `python3 -m unittest discover -s src/simnav/test -p "test_*_core.py"` **232 项全绿**
> （交接时 223，本续会话新增 9）。

### 7.1 已落地改动

| # | 改动 | 文件 | 触发证据 |
|---|---|---|---|
| 1 | T-E 可观测性：诊断新增 `room_entry_sequence`（`[sim, floor, room_id, nth]`，跨层不重置）与 `path_start_offset`；采样器加 `entry_seq`/`path_offset` 两列；遥测 CSV 加 `path_start_offset`/`entry_seq_len` | `coverage_explorer_node.py`、`team_scripts/sample_run_status.py`、`team_scripts/record_stage_b_telemetry.py` | "进门次数"只有 120 s 采样且常为 None，导致看图/看日志互相误判 |
| 2 | ⑥ 换目标统一上房间锁：core 新增纯函数 `room_lock_for_target()`，节点新增 `_arm_room_approach()`，普通派发与 `_switch_active_target` 都调用 | core + node + 测试 | run107 floor 0：196 仿真秒内 13 次换目标、`lock=None`、0 进门；run108 同窗口进 L_15/R_15 并退休 |
| 3 | A\* 起点改为**机器人自身格 + 3×3 物理占位窗口**（`_robot_local_safe`），**删除生产路径上的"最近安全格"**（`_nearest_seed` 仅留给 door-band 测试） | core + 测试 | run107 `path_start_offset` 1.737 / 2.455 m |
| 4 | 采用路径时**从当前位姿重新规划剩余路线**（`_anchor_path_to_current_pose(path, target)`），无法重规划才回退剪接；`_publish_path` 从 `path_index-1` 发布，画线从机器人脚下起 | node | `last_plan_ms` 3664~8235 ms，路径起点是数秒前的位姿 |
| 5 | 楼层入场一致性：`initial_forward_complete` 取代 `gate is None` 作为入场直行闸，**每层都跑**；上层保留电梯给的 gate、并恢复 plane 步态 | node | run112 floor 1：只前进 1.8 m 就开始探索，`stuck_drops=12`、各房 ~0.05 |
| 6 | 电梯节点每次过渡**重新确认 stair 步态**（两处 `FLOOR_1_READY` 重置 `gait_policy_selected`） | `elevator_transition_node.py` | 上层恢复 plane 后，一次性标志会让第二次乘梯不再切 stair |
| 7 | 远区通行：**前沿优先**；固定位移降为**备用触发**——当排序后的**首选目标落在"已探索过的房间"**（`targets[0].topology_id in completed`）时，改派走廊中线上的向前固定步，落点 = 下一个未完成房间门的 along（`next_doorway_along()`）；committed 模式放开 novelty 检查；阶梯 `(2,4,6,9,14,20,26,32)`；空池分支同样用它兜底 | core + 测试 | 实测"前沿优先"能到远区（用户认可），但首选目标有时落回已探索房间导致回退；纯固定位移预抢占又会让优先级压过前沿，故定为"仅目标落在已探索房间时启用" |
| 8 | 房间完成日志加 `combined / task_cells / best_remaining_gain` | node | 需要量化"提前退休" |
| 9 | **世界系 gate 竞态修复**：入场直行中 `lobby_entry_world` 一旦拿到世界位姿就锁存（不再只在锚点第一拍、`world_pose` 恰好在时才锁） | node | run117 floor 0：`virtual_isolation_door=None` ⇒ 电梯节点 `_explorer_status_callback` 在 524–528 行提前 `return` ⇒ 连 `virtual_isolation_door_source` 都没读到 ⇒ `gate_source=None` ⇒ `_navigation_map_callback` 的电梯候选检测整段不跑 ⇒ `elevator_portal=None` ⇒ 4/4 完成后 `RETURN_TO_ELEVATOR` 永不开始（sim 510.7 起一直 `Waiting for map-confirmed elevator portal before transition`） |
| 10 | **回电梯改为三段"固定位移 + A\*"**（只用于回程）：① `lateral` 超容差时先直行到走廊中线；② 对准走廊反向（`gate_yaw+π`）后用 `_drive_distance` 固定位移到"电梯门前 stand-off"（独立脚本 `elevator_only_driver.py` 的 `staging_world` 机制，`~staging_distance` 默认 2.20 m）；③ 从 stand-off 交给 `_drive_planned_to`（A\*） | node | 用户设计：先固定位移到入口节点，再从该节点走 A\*；run118 显示 `TASK_REGION_COMPLETE` 时 `lateral=-1.06 m`，必须先回走廊中线 |
| 11 | **电梯对中与出门卡死**：① `entry_lateral_tolerance` 0.35 → **0.15**（并解除被 `target_tolerance=0.35` 当上限的隐式钳制）；② `EXIT_ELEVATOR` 增加**卡滞看门狗**：6 s 内位移 < 0.15 m 就退回轿厢 `entry_retreat_seconds` 后重试，`exit_retry_limit=3` 次后 `_fail("ELEVATOR_EXIT_NO_PROGRESS")` | node | run124 floor 1：进门直线段横漂 -0.04 → **-0.36 m**，出门在 6 cm 门槛上以 `vx=-0.45` **死推 108 仿真秒**（位置钉在 (6.20,-1.37)、横向偏 +0.50 m、`fault=None`），因为 `_drive_distance` 的失速处理只覆盖 ENTER 两个状态 |
| 12 | **固定前行改成"committed 机动"**：core 在诊断里给出 `committed_transit_along`（落点=**活跃 zone 内**最近的未完成门 along），node 新增 `_control_committed_transit()`——① 不在中线就先直行到中线；② 对准走廊轴；③ 走**固定距离**到该 along。全程开环，地图更新无法中途改目标（这正是前沿追点走不到远区的原因）。落点不再用最小 along：门洞 id 后缀=along/0.5，漂移 bin `ROOM_L_35`(≈17.5，正好分区线) 会盖过真后门 `ROOM_R_43`(21.45)，导致"距离太近" | core + node | 用户指出：固定前行应与回电梯同款（先对准中心再走固定距离），而非"合成点让 A\* 追"；且落点要接近后门门口 |
| 13 | **上层楼门洞"种入 + 实时刷新"**：`_floor_context_callback` 恢复种入一层门洞（`_reused_floor_portals(payload)`，evidence=confirm，ROOM 目标首周期即可用）；`_plan_impl` 里本层实测门洞按 `doorways_match(..., doorway_merge_tolerance=1.5)` **合并进种入 id、几何以本层为准**（`replace(portal, topology_id=key)`） | node | run126 floor 1：`reused 0 topology portals`（代码把复用硬编码为空），本层门洞未在窗口内确认 ⇒ 目标池只有 `LASER_FRONTIER/CORRIDOR`（path 0.70/0.90/1.60），**110+ 秒无任何 ROOM 目标、只在走廊徘徊**。禁用复用的历史原因是 run80 旧几何被当成真值（evidence=1）导致 CANDIDATE_PATH_UNREACHABLE，故改为"种入但刷新" |
| 14 | **上层入场直行距离 9.00 → 6.00 m**（`STAGE_B_UPPER_FLOOR_FORWARD`） | `two_floor_explorer_supervisor.py` | run126 floor 1 实测：探索器锚点在 along≈1.4，前门在 7.4，9.09 m 落到 along≈10.4（**越过前门约 3 m**），即用户看到的"一直前进、太远" |
| 15 | **远区通行落点不必是门**：`next_doorway_along()` 在活跃 zone 内无已知门洞时，回退到**该 zone 走廊的中间点**（`0.5*(zone_split+forward_limit)`，保持在机器人前方并受任务范围限制）；并新增"**毫无目标可派时的总兜底**"——`not targets and not topology_lock and assignment_portal_count>0` 即派 committed 走廊段（不再依赖 `active_station_topologies`）。该兜底有前提：全部房间已完成时不派，保持 `test_completed_rooms_are_not_re_dispatched` 契约 | core | 用户要求"远区目标点不必是门，任何远区点即可"；run129 floor 0 在前两间房退休后 `NO_FRONTIER` 冻结 50+ 秒（目标 `ROOM_L_31` gains=0 且无法路由），需要一条不依赖站点条件的兜底 |
| 16 | **固定前进 = 远区第一优先 + 落点standoff 调参**：`zone_now=="B"` 且落点在机器人前方时，**直接替换整个目标池**派 committed 走廊段（不再先排序前沿）；落点 = 目标门 along − `COMMITTED_LANDING_SHORTEN`（先 2.0 m 太短，**用户定 1.0 m**） | core | 用户命令"必须以固定前进为第一目标"；此前"前沿优先"会先派零增益走廊目标、`Dropping stuck target`、再 `NO_FRONTIER` 卡几十秒（run129 冻结 50+ s） |
| 17 | **committed 段免疫于新规划 + 门控脱离 zone 标签**：① node 在 committed 字典里存 `target_along`，一旦开始就用自己的落点继续，**后续派发的目标点不能打断**（用户："一开始明明要去走廊，结果被规划目标点打断了"）；② core 的远区第一优先门控从 `zone_now=="B"` 改为**"没有明显属近区的未完成门洞"**（`portal.along < zone_split - 1.0`），`next_doorway_along()` 内部同样改为"非明显近区"过滤——因为 1.5 m 带宽把 along≈18 的漂移 bin `ROOM_L_36` 判成近区，`active_zone` 永远停在 A，固定前进永不触发（run134 实测） | core + node | run134：`zone A / committed None / fixed_step None`，派 `LASER_FRONTIER/ROOM_L_36 gains=0` 后 5 s 被丢；同轮 `test_corridor_generates_only_lidar_frontiers` 因"合成点不可派发就清空池"失败，已改为**保留排序池** |
| 18 | **⑥ 驻留时间 8 → 15 s**：`target_switch_dwell` 在两个 launch 里显式设为 15.0（原来只在节点默认 8.0） | `stage_b_floor_explorer.launch`、`stage_b_behavior.launch` | 用户："重规划太快、触发太容易"；run136 中 ⑥ 频繁换目标（`Switching target before arrival`）导致目标抖动 |
| 19 | **根因修复：`lifecycle_changed` 不再吞掉可派发目标**。`_plan_impl` 原来 `if lifecycle_changed: ... return`——只要房间状态机报一次变化（含它自己的 miss/block 计数）就提前返回、**不采纳 `plan.target`** | node | run138 floor 0：`Control stop: no active target (plan_reason=TARGET, plan_target=CAMERA_FRONTIER/ROOM_R_15, active_path_len=0, committed=False)` 反复出现；控制器无目标 → 停车；已接目标 6 s 无位移又被 backstop 丢弃 → 循环。**这一条解释了 run107/113/129/136/138 反复出现的"目标在却不动"**，与重规划频率无关。新诊断为 `Control stop: no active target (...)` |
| 20 | **固定前进升级为节点侧强制保证**（用户："不能靠运气选到后门目标点"）：`_committed_transit_along()` 不再只在 core 请求时才返回落点——**只要没有房间锁**（走廊阶段），优先级为 ① core 的 `committed_transit_along`；② 规划自带走廊目标的 along（转成机动，不再追点）；③ 都没有时给默认前进步 `committed_default_step=6.0 m`。一旦开始即免疫规划变化 | node | 用户要求"防止固定前行被打断、每次都必须发生"；此前走廊目标可能被当普通目标点追踪，存在被换点/生命周期打断的窗口 |
| 21 | **四种"前行"显式分开 + 相位闸**（用户要求不得互相污染）：① **首层固定前行**＝`_control_initial_forward`，`gate is None`（一层 14.5 m，含 plane 切步态）；② **远区固定前行**＝`_control_committed_transit`，**仅** `initial_forward_complete` 且无房间锁且未闭层，且**只转换走廊前沿（LASER/CAMERA_FRONTIER）**，节点自己的 `RETURN_TO_CORRIDOR`（topology 也是 CORRIDOR）**一律不碰**；③ **电梯前往**＝电梯节点 `RETURN_TO_ELEVATOR`（对准反向→固定位移到 stand-off→A\*，独立状态机）；④ **出电梯→走廊前行**＝`_control_initial_forward`，`floor>0`（6.0 m，保留电梯 gate、恢复 plane 步态） | node | 用户："去远区的固定前行明显污染了到走廊的固定前行"——`RETURN_TO_CORRIDOR` 被当成走廊目标转成直线固定位移，机器人偏离穿门路径。现已按 kind/topology + 相位双重把关 |

### 7.2 本轮实测数字（不要再重新推演）

- **上层入场直行被整段跳过**：探索器启动时 `gate` 已由电梯节点在 `ESTABLISH_FLOOR_1_TOPOLOGY` 给好，`if gate is None` 为假。上层 ESTABLISH 自身只前进 `floor1_corridor_advance=1.80 m`。
- **覆盖率分母偏小**：`task_cells` 实测 **7865 / 8182 / 9063**，物理房间固定 **11785 格**（8.4×14.03 m @0.1 m），即分母只有 67%~77% 且按门洞 bin 切分。
- **房间提前退休**：`ROOM_L_15 combined=0.558 task_cells=7865 best_remaining_gain=11.96`；`ROOM_R_15 combined=0.563 task_cells=8182 best_remaining_gain=13.65` ⇒ 还有 ~12–14 m² 未观测就退休（用户看到"左边一片没有蓝色"）。
- **房间激光门槛仍高**：`room_laser_coverage_target=0.95`，而 `room_camera/combined=0.55` ⇒ 探索被 0.95 驱动（用户观察"还是按更高覆盖率来的"）。
- **门候选堆积**：`ROOM_L_0/ROOM_R_0 along=0.15 width=0.30`（走廊口伪门），真门为 along 7.40（前）与 21.45（后）；`actionable_portals=['ROOM_L_49']`（漂移 bin）。
- **距离真值**（layout y=7.85 对应 along 0）：走廊口 0 → 前门 7.40 → 隔墙 14.03 → **后门 21.50** → 远墙 28.06。**隔墙到后门 ≈ 6.5 m**，走廊口到后门 21.5 m。
- **规划耗时**：`last_plan_ms` 3664（run111）→ 8235（run113/117）；run112 sim 302.6→421.7 冻结 119 s（`Floor completion pending` 后无下一拍）；run113 sim 292→432 冻结 140 s，位置死钉 `(25.49,0.25)`、`gate_along=15.0`，1440 采样中 0 个在动。
- **⑥ 修复对照**：run107（修复前）196 s 内 13 次换目标、0 进门；run108（修复后）同窗口进 L_15/R_15 并退休。

## 8. 未解决问题（按优先级）

1. **门候选去重/过滤**（最高优先）：最小宽度门槛、同一物理门洞的 bin 合并、忽略走廊口接缝（`width=0.30` 的 `ROOM_L_0/ROOM_R_0`）。它同时污染 `zone_split`、覆盖率分母与目标抖动，并曾把 `active_zone` 钉在 A 区（run113 卡死）。
2. **覆盖率分母 = 物理房间完整区间**（用户已拍板）：合并 bin、以墙为界、未观测格计入分母；完成后 `0.55` 才是房间的 55%。
3. **规划耗时爆炸**（~5–8 s/次，冻结 110–140 s）：嫌疑是每个候选各自重算 `_navigation_fields`（距离变换）+ 候选数膨胀；方向是每周期缓存导航场 + 限制每周期路由候选数。
4. **`room_laser_coverage_target=0.95` 与新口径统一**：明确它是否参与房间完成/前沿生成。
5. **验证起点重锚新版**（re-plan + 从 seed 发布）：`path_start_offset` 应持续 ≈0，且 RViz 画线从机器人脚下起。
6. **三层通用验证**：固定位移兜底与入场直行必须在 floor 1/2 同样出现，不能只看第一层。
7. **T-F 命名债**：`floor1_gate/floor1_complete/floor1_topology_isolated/floor1_context_published` 实际承载所有上层楼；改名 `upper_floor_*` 且 payload 保留旧 key 一版。
8. **零增益目标污染**：run112 sim 302.6 派发 `LASER_FRONTIER/ROOM_L_49 gains=0` 后 `Dropping stuck`，把闭层推迟 ~119 s。
9. **回电梯链条对世界系 gate 的硬依赖（已修主因，建议再加防御）**：`virtual_isolation_door`（world）为 None 时，`elevator_transition_node._explorer_status_callback` 在 `if not isinstance(gate, list): return` 处提前返回，导致同一 payload 里的 `virtual_isolation_door_source` 也读不到，整条 `RETURN_TO_ELEVATOR` 链全空。已修探索器端（锁存 `lobby_entry_world`）；建议再让电梯节点在 world gate 缺失时仍读取 source gate，不要用一个可选的诊断量卡住整条流程。
10. **回电梯已改为「对准走廊反向 → 固定位移到 stand-off → A\*」（待验证）**：`RETURN_TO_ELEVATOR` 先 `_align` 到 `gate_yaw + π`，再用 `_drive_distance` 沿走廊轴走固定距离到电梯门前 stand-off（独立脚本 `staging_world`，`staging_distance=2.20 m`），到达后才交给 `_drive_planned_to`（A\*）。只用于回程；去远区仍走前沿/`next_doorway_along()` 兜底。需在含 #9 修复的新一轮里验证：4/4 后能否真的进电梯。

## 9. 当前运行

- run117（`logs/run117_doorlanding_20260914/`）在跑，带本轮改动 #1–#3、#5–#8；**#4 的"起点重规划"代码是在 run117 启动之后编辑的，run117 不含该条**。
- 已验证：floor 0 已进 `L_15 / R_15 / L_49 / R_43`，`path_offset=0.000`，`fixed_step_transit` 未触发（走的是前沿，符合"前沿优先"）。
- 待看：固定位移兜底是否在前沿耗尽时出现并落到后门；上楼后入场直行 9 m 与 plane 步态；三层通用。
