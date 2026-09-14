# Stage-B 会话交接（2026-09-14 交接点）

> **本文件用途**：跨会话交接。新会话请**先读这一份**，再读 `docs/功能模块设计.md`（活文档，含完整设计/变更记录）。
> **本轮新增**：**§10 独立电梯模块**（faithful 真值闸门、10 连场证据、定位漂移结论、跑法/工具）——电梯相关问题优先看 §10。
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


---

## 10. 独立电梯模块（本轮重点：2026-09-14 深夜 ~ 09-15 凌晨）

### 10.1 目标与当前状态
- **模块**：`team_scripts/elevator_only_test.sh` + `elevator_only_driver.py`（+ `elevator_only_matrix.sh` 跑批）。场景自起：`roscore + auto.sh(Gazebo/机器狗/控制器) + stage_b_localization`，**不启探索器、不启电梯节点**。
- **流程**（用户定义）：出生在**门外** (0.0, -3.2) 朝 **+1.5708** → 进楼门 → 走廊口 → 居中 → 掉头 → 回电梯 → 对准门中心 → 进梯 → **1→2 层** → 出梯 → 走廊口 → 居中 → 掉头 → 回电梯 → 对准门 → 进梯 → **2→3 层** → 出梯 → 走廊口 → 居中 → 掉头 → 回电梯 → 对准门 → 进梯 → **3→1 层** → 出梯 → **出门回出生点**。
- **已达成（真值辅助控制版）**：同一冻结版本 **连续 10 场 PASS**
  driver `3a2e0c7e86392c8e7880e63bd691daccd51bb74b1ffaf8853989fe997f120e1c` /
  test.sh `cb85505ecdd3ac11d9296f24292159728adaa2f081ab17cdabd5e6638e60ddd6` /
  matrix.sh `c59fcea0f5560fe4a1d4dca603249841c85a86f80c9c322513bf685cc268e27c`。
  工况覆盖：标称×4 + 进深 2.4 m + 进深 1.15 m + 横移 ±0.30 m + 出生朝向 ±0.03 rad。
  证据：`logs/FINAL_10_pass_report.txt`（独立复核，含哈希核对）。每场：3 次乘梯、乘梯 z 0.31/2.91/5.51/0.31、进梯真值横向 ≤0.15 m、终点 0.20–0.28 m、墙钟 397–507 s。
- **用户随后要求"忠实主线"**（`USE_TRUTH=0`，现为**默认**）：真值**只能监控**，控制与判定必须只用主线能拿到的信息——依据：`generated_building/team_scene_info.json` 允许服务仅 `/set_door_state`、`/call_elevator`，`forbidden_topics` 含 `/Odometry_gazebo`、`/ground_truth/*`，`forbidden_files` 含 `layout_metadata.json` 等。

### 10.2 忠实模式的实现（关键机制）
1. **单点真值闸门**：Gazebo 真值每 0.25 s 仍轮询，但只写 `gazebo_truth*`（监控）；控制/判定用的 `robot_truth*` 在 `USE_TRUTH=0` 时取**本机定位估计** `world_pose = (x, y, yaw, z)`。因此驱动里 91 处"真值"使用点**自动全部变成估计**（分区驾驶选择、进梯对准/登梯判定、excursion 深度、出梯过门、终点判定、卡滞与进度看门狗）。
2. **时间线**：`truth_*` 列**始终写 Gazebo**（监控）；`world_*` 列是控制用的估计；复核脚本另给 `max/median_ctrl_error`。
3. **乘梯到站判定**：估计 z **看不到乘梯**（机器人在轿厢内静止），故忠实模式改为**按 `/call_elevator` 回报**（`accepted` 且 `current_floor == target_floor`）；`VERDICT(lift)` 同步改用每腿 `floor_ok`。
4. **发散守卫**：始终与 Gazebo 比较（**仅守护**），忠实模式下**全域致命**（3.0 m / 2 s 去抖），不存在"真值驾驶免罚区"。
5. **轿厢出口**：由 `exit_mode` **单独决定**——`reverse`（默认，用户要求"出电梯倒着出"；6 cm 轿厢门槛正向会咬）/ `turn_forward`（轿厢内掉头按左右余量选向 + 正向出门，保留为选项）。
6. **回程**：门厅内转身到出生朝向（耐心预算 150 s + 起转看门狗）→ **倒行**穿大门与 8 cm 台阶到出生点（朝楼内特征侧倒行，利于 LIO），**门洞区域加激光两侧居中**；**终点只判位置**（官方只考核危险源与耗时，`team_scene_info.json` 只定义 `robot_start`；朝向仅记录）。
7. **进楼台阶**（`entrance_apron` 8 cm，y∈[-2.4,0]）：真值进度看门狗 + 5 s 停滞触发 + 后退重顶 + 步态复位；正向上下台阶可靠，**倒行越台阶差**（用户明确）。

### 10.3 本轮实测到的关键事实（不要重新推演）
| 事实 | 数值/结论 |
|---|---|
| 忠实模式探针（`faithful_probe2`） | 三腿乘梯全过；死在回出生点，**定位误差最大 3.44 m**（门厅/院子空旷区）→ `SPAWN_REVERSE_TIMEOUT_DIST_1.74M`。**结论：此前 10/10 有真值兜位置成分** |
| 转身可靠性（285 段实机统计） | **走廊 93%**（82/88）、**门厅 58%**（112/193）、室外 50%（3/6）→ "对准走廊中线"是关键锚 |
| 走廊入口基准 | 探索器估计 `(-0.25, 7.30, 1.5708)`（与独立脚本硬编码一致）；**真实入口 y=7.85、真实中线 x=0.00** → 偏差 0.55 m / 0.25 m（离线评估：只把机动点挪进走廊 ~0.5 m、左移 0.25 m，离墙仍余 0.54 m，不改判据性质） |
| 轿厢门槛 6 cm | 正向过会咬（`EXIT_CAR_NO_PROGRESS_AFTER_3_RETRIES_TRAVELLED_0.08M`）；**倒行 + 退避重顶稳** |
| 大门台阶 8 cm | 正向下台阶可靠；倒行越台阶差 |
| 候选电梯门洞 | 已复用主流程检测器（只读 import）+5 周期确认+合理性守卫（>0.60 m 或 >0.25 rad 拒绝）；但**地图里轿厢门是关的**（门洞被门扇占满，最长自由跨 0.9 m 且不在门位）→ 正确返回 0 候选并回退参照；驱动已改开局先开 0 层门 |
| 门口防污染 | 每场启动即 `rm -f` 清空 timeline/summary/log/verdict/env（`test.sh` + 驱动自身都清），tag 复用不再叠加两场 |

### 10.4 工具与跑法
```bash
# 忠实主线（默认，真值仅监控）
docker exec -d simenv-noetic bash -lc 'cd /workspace/SimEnv && USE_TRUTH=0 STREAK_TARGET=5 MAX_RUNS=5 CLEAN_WAIT=12   bash team_scripts/elevator_only_matrix.sh 20260902 faithful5 > /workspace/SimEnv/logs/faithful5.log 2>&1'
# 真值辅助（旧口径，快；USE_TRUTH=1）
# 逐场监视（宿主，含工况/判定/进梯真值/出梯方式/定位误差/完成时版本哈希）
python3 SimEnv/logs/watch_next_run.py 20260902
# 独立复核（不采信驱动自报，从 timeline 重算）
python3 SimEnv/logs/verify_streak.py <prefix> 20260902
# 实时看板
python3 SimEnv/logs/live_dashboard.py        # http://127.0.0.1:8090/
# 汇总
python3 SimEnv/logs/final_report.py '<tag glob>' 20260902
# 清场（标准序列）
docker exec simenv-noetic bash -lc 'pkill -f elevator_runner_; pkill -f elevator_only_test.sh; sleep 2;   cd /workspace/SimEnv && bash team_scripts/kill_sim_processes.sh'
```

### 10.5 忠实测试结果（`faithful5`，`USE_TRUTH=0`）
版本（权威，见 `logs/faithful5_code_hashes.txt`）：driver `6b9b1c50…` / test.sh `d8243078…` / matrix `c59fcea0…`。
逐场监视：`python3 SimEnv/logs/faithful5_monitor.py`（后台作业输出 `logs/faithful5_monitor.out`）。

| 场次 | 判定 | 乘梯/走廊 excursion | 定位误差 max / median | 终点距出生点 | 墙钟 |
|---|---|---|---|---|---|
| faithful5_01 | **PASS** | 3 / 3 | **0.29 m / 0.16 m** | 0.23 m | 373 s |

`faithful5_01` 详情：`WAIT_READY→ENTER_BUILDING→TO_CORRIDOR→CORRIDOR_CENTRE→TURN_AROUND→BACK_TO_LIFT→ALIGN_DOOR→ENTER_CAR→RIDE→EXIT_CAR→RETURN_SPAWN→DONE`；真值楼层平台 0.31/2.92/5.52/0.31；三次出梯**全部倒行**（`travelled` 1.04/1.02/1.01 m）；进梯真值横向 −0.121/−0.008/−0.024 m；走廊 excursion 真值深度 1.22 m×2；独立复核 clean。

> **重要更正**：早先探针的 3.44 m 定位漂移与"忠实版必漂"的结论**不成立**——本轮忠实运行定位误差仅 0.29 m（中位 0.16 m）。原因是探针当时用的是修好之前的旧版本（`fbcf92b6`，`exit_mode` 初始化崩溃前的那一版）。**忠实模式可以做到与真值辅助版同等精度。**
>
> **证据卫生**：`logs/elevfaith3_code_hashes.txt`（00:46:22）是**过期**记录（写文件时驱动正在被改写，哈希 946b7eff 无效），已被 `logs/faithful5_code_hashes.txt` 取代；`logs/run_revisions.tsv` 里 `faithful5_01` 的旧行（`fbcf92b6`，00:44:55，来自崩溃批）已修正为 `6b9b1c50`，并且 `watch_next_run.py` 已改为**按 tag 覆盖写入**（原先只在 tag 首次出现时写，导致复用 tag + 失败批会把版本归属搞错）。
>
> **中断说明（01:08:54）——按用户指示的正常停止，不是驱动缺陷**：`faithful5_02` 跑到 sim 157.7 s（`TURN_AROUND`，走廊口 y≈8.66）时，被**另一会话按用户指示"停止当前电梯单独测试"清场**（`pkill elevator_runner_/elevator_only_test.sh/elevator_only_driver` + `kill_sim_processes.sh`）终止，matrix runner 同时被杀，因此没有 fault / 没有 verdict，`logs/faithful5_summary.txt` 里只有 run 1。该会话随后把本模块方法融合进主线，见 **§11**。
> **结论**：本模块 5 场忠实测试**只完成 1 场（1/1 PASS，定位误差 0.29/0.16 m）**。若要继续跑余下 4 场，必须**等主线运行结束、容器空闲**（同一容器只有一个 Gazebo/roscore；本模块的 `clean-room` 会杀掉正在跑的主线）。本模块 `elevator_only_driver.py` **未被改动**（哈希仍 `6b9b1c50…`，已核对）；并发会话改的是 `team_scripts/two_floor_explorer_supervisor.py` 与 `src/simnav/scripts/elevator_transition_node.py`。

### 10.6 未决与下一步（按优先级）
1. **忠实版连续成功**：`faithful5_01` 已 PASS 且定位误差仅 0.29 m（见 10.5 更正）；等 5 场跑完看失败率与误差分布，再决定是否需要额外的地标锚定（走廊两墙配平、门洞两侧激光已用）。~~定位漂移 1.5–3.4 m~~ 是旧版本（`fbcf92b6`）的现象，**不再是当前结论**。
2. **走廊入口点是否对齐真实入口**（7.85 / 中线 0.00）：独立脚本应沿用主线估计才忠实；若要更准需改**主线**的门口拟合。
3. **候选门洞**：让门在地图里可见（开局开门是否足够 / 改门垛几何检测）。
4. 主线整体验收（探索 + 电梯 + 回程，`logs/run163_cmd.txt` 有命令）仍待跑；84% 与红球基线见 §3 的 T-B。

---

## 11. 独立电梯测试三段融合进主线（2026-09-15 凌晨，**已执行，未上机验证**）

> **触发**：用户指示"停止当前电梯单独测试，把最新一次测试的对应方法融合到主线"——三部分：走廊起点→电梯、电梯→从电梯出来到走廊起点（含主线固定前行的配合改动）、电梯→大门→出生点，并做接口对接（去电梯须先到走廊起点）。
> **测试状态**：`faithful5` 矩阵已被我停掉并清场（`pkill elevator_runner_/elevator_only_test.sh/elevator_only_driver` + `kill_sim_processes.sh`，复核无残留进程）。停止前 `faithful5_01` 已 **PASS**（3/3 乘梯、定位误差 0.29/0.16 m、终点 0.23 m），即用户所说"目前测试效果稳定"——本轮融合的就是这一版方法。
> **改的文件**：`src/simnav/scripts/elevator_transition_node.py`、`team_scripts/two_floor_explorer_supervisor.py`。`docs/功能模块设计.md` 已加变更记录。

### 11.1 融合了哪些方法（都来自 `elevator_only_driver.py`）

| 段 | 主线新实现 | 对应 driver 状态 |
|---|---|---|
| 走廊起点→电梯 | `_control_return_to_lift()`：`TO_CORRIDOR_START`(A\* 到闸门) → `CORRIDOR_CENTRE`(`_center_on_corridor_axis()`) → `TURN_TO_LIFT`(掉头到 `gate_yaw+π`，带 walk-and-turn 解卡) → `TO_STAGING`(`_elevator_staging_source()` = 已知车门沿门法线回退 `staging_distance`) | `CORRIDOR_CENTRE` → `TURN_AROUND` → `BACK_TO_LIFT` |
| 对门 | `_control_align_door()`：横向超 `entry_lateral_tolerance` 先滑到门轴点 `min(along,−0.30)`，再对准门法线 | `ALIGN_DOOR` |
| 电梯→走廊起点 | `ESTABLISH_FLOOR_1_TOPOLOGY` = `CLEAR_CAR`(到 staging) → `TO_CORRIDOR_START`(A\* 到闸门) → `CORRIDOR_CENTRE`；完成后 `_finish_floor_topology()` 把机器人**停在走廊起点**交给探索器 | `EXIT_CAR` → `TO_CORRIDOR` → `CORRIDOR_CENTRE` |
| 电梯→大门→出生点 | `_control_spawn_return()` 三阶段：`AXIS`(到 `(spawn_x, 1.80)`) → `FACE`(室内转向出生朝向，带解卡/超时) → `REVERSE`(倒行穿大门 + 8 cm 台阶，门洞激光居中，只判位置) | `RETURN_SPAWN` |

### 11.2 接口对接（主线配合改动）

1. **去电梯先到走廊起点**：`RETURN_TO_ELEVATOR` 第一阶段就是 A\* 到 `_corridor_gate()`（= `floor1_gate or floor0_gate_source or gate_source`，源图）。上层楼 `RETURN_TO_FLOOR_1_GATE` **复用同一函数**，删掉了 `ENTER_FLOOR_1_LOBBY` / `SEARCH_FLOOR_1_ELEVATOR` 两级旧链（监督器仍以 `RETURN_TO_FLOOR_1_GATE` 为停探索器的触发点，契约不变）。
2. **上层楼固定前行**：电梯节点现在把机器人停在**走廊起点（闸门，gate-along 0）**；`two_floor_explorer_supervisor.py` 的 `STAGE_B_UPPER_FLOOR_FORWARD` 由 **6.00 → 4.00**。依据（用户确认）：一层 14.5 m 节点、走廊起点≈虚拟闸门 `virtual_gate_forward_distance=10.5`，二者相差 **4.0 m**；只有 `floor_index > 0`（2/3 层）追加该参数，一层仍走自己的入楼 transit（14.5）。
3. **倒行出梯不翻转**：`RIDE_TO_GROUND_FLOOR` 删掉 `elevator_heading += π`。机器人进梯时朝轿厢内，`_drive_distance(reverse=True)` 沿门法线倒着出——与 driver 每腿一致；旧翻转会让倒行变成开回轿厢。
4. 新增参数（都可 rosparam 调）：`corridor_start_tolerance=0.40`、`spawn_turn_y=1.80`、`spawn_axis_tolerance=0.45`、`spawn_axis_timeout=120`、`spawn_face_tolerance=0.12`、`spawn_turn_timeout=150`、`spawn_reverse_timeout=300`、`spawn_reverse_yaw_bias=0.16`、`turn_unstick_*`。
5. 出生点用**启动时锁存的世界系位姿** `spawn_world`（`_pose_callback` 在 `WAITING` 时锁一次）；拿不到时退回源图 `(0,0)`。

### 11.3 验证到什么程度 / 还没验证什么

- ✅ `py_compile`；`check_undefined_attrs.py`、`check_read_before_assign.py` 均 OK；离线回归 **242 项全绿**；状态机"set 的每个 state 都有 handler"已用 AST 核对（`MISSION_FAULT` / `RETURNED_TO_SPAWN` / `TOP_FLOOR_COMPLETE` 为终止态，靠 `fault` / `two_floor_mission_complete` 提前 return）。
- ❌ **没有上机跑三层**。这轮只做代码融合 + 离线门禁，行为未实测。首次上机建议按 §4 受护启动，重点看：`ESTABLISH_FLOOR_1_TOPOLOGY` 是否停在走廊起点、上层 `initial_forward_progress` 是否到 4.0、`RETURN_TO_SPAWN` 三阶段、一层 `RETURN_TO_ELEVATOR` 的四个 phase 日志。
- ⚠️ 已知风险：`TO_CORRIDOR_START` 用 A\* 打到闸门点本身，若闸门格被膨胀占据，`_drive_planned_to` 会走 `route_accept_distance=1.2 m` 接受或 60 s 后 `ROUTE_UNREACHABLE`（旧的 `return_mouth_along=1.0` 实测位移写法没有这个风险，但终点不是起点）；`_center_on_corridor_axis` 的横向修正用实测位移、无独立超时；`_elevator_staging_source()` 沿用 driver 的**实时位姿对**换算（`_known_door_source()`），非成对闸门换算。

### 11.4 首次上机 run165 + 死锁修复 + 重跑 run166（2026-09-15 深夜）

**run165**（`logs/run165_fused_elevator_20260915/seed_20260902`，受护启动 health PASS，Gazebo GUI 关、RViz 开）：

- floor 0 探索正常：sim 133 `ROOM_L_15` 退休；sim 235.8 已退休 `L_15/R_15`、锁 `R_43`。
- **第一段（走廊起点→电梯）验证通过的部分**：`RETURN_TO_ELEVATOR` 自 sim 324.8 起（src (31.9, 1.3)、gate_along 21.4）回走廊，**到走廊起点最近 0.069 m**（along=−0.002 / lateral=0.069 @ sim 387.9），整段横向最小 |lateral| **0.028 m**；随后到 staging（车门框 along≈−2.2）并进 `ALIGN_ELEVATOR`（sim 397.3）。**走廊门口到达没问题。**
- **死锁**：`ALIGN_ELEVATOR` 门框横向 **0.25 m** > `entry_lateral_tolerance=0.15` ⇒ 滑移分支；但 `_drive_to(..., arrival_tolerance=0.12)` 的容差被钳到 `target_tolerance=0.35`，0.25 ≤ 0.35 立即判"已到达" ⇒ 每周期只 `_stop()` ⇒ `cmd_vel=(0,0)` 冻结 **75 仿真秒**（sim 397→472）。
- **排除"没有电梯候选"**：运行状态 JSON `elevator_portal=null`、`elevator_portal_world=null`，但**已知车门已配置**（日志 `Known car door: centre (1.65, 2.60) yaw 0.00 width 1.40`），`_approach_portal()` 返回 known door，所以候选缺失不是原因。
- 清场教训：`pkill -f "rviz -d /workspace/SimEnv/..."` 的模式会匹配到自己所在命令行，把自己 SIGKILL（exit 137）；必须写 `rvi[z]`。

**修复**：`_control_align_door()` 的横移改为实测位移（`_align` 到门轴点航向 + `_drive_distance(abs(lateral))`，`lateral_correction_distance/_heading` 跨周期记账），不再走会钳容差的 `_drive_to`。

**run166 重跑**：`logs/run166_fused_alignfix_20260915/seed_20260902`（同 seed / 55% / three_floor，受护启动）。待验证：`ALIGN_ELEVATOR` 应在 1–2 s 内完成并进 `ENTER_ELEVATOR`；之后继续核对 ⑵ 上层 4.0 m 固定前行与 ⑶ `RETURN_TO_SPAWN` 三阶段。

### 11.5 run166 结果：走廊起点 0.013 m + ALIGN 修好，但暴露"登梯守卫无符号"（2026-09-15 深夜）

**run166 好的部分**：
- 走廊起点到达 **0.013 m**（along −0.013 / lateral 0.003 @ sim 424.6），整段最小 |gate_lateral| **0.002 m** —— 比 run165（0.069 m）更好。
- `ALIGN_ELEVATOR`（sim 434.0）→ `ENTER_ELEVATOR`（sim 444.5）只用 **10.5 仿真秒**（run165 在此永久冻结）。进梯对中 `entry lateral` 从 run165 的 0.25 m 变成 **−0.05~+0.02 m**。

**run166 新的卡点（登梯，not 候选缺失）**：遥测门框 `along`：

| sim | along | 动作 |
|---|---|---|
| 444.5 | −2.365 | 开始进梯 |
| 450.5 | −0.072 | 刚过门平面 |
| 452.5 | **+0.150** | 已入轿厢 0.15 m，但 `vx=−0.25` **倒退** |
| 454.5 | −0.219 | 被"退"出来 |
| 460.5 | +1.586 | 第二次冲入 |
| 462.5 | **+1.678** | 顶到**轿厢后壁**，停死（vx=0） |

日志每 2.4 s 刷 `entry distance driven but the robot is 1.67 m from the car doorway (tolerance 1.20)`。两个 bug：
1. 第一次到 `along=+0.15` 时 `_entry_outside_car()` 要求 `along ≥ elevator_door_inset=0.35` 才算"里面"，0.15 < 0.35 ⇒ 判"在外面" ⇒ 无谓倒退。
2. 重冲顶后壁 `along=+1.67` 后，旧守卫用**无符号距离** `hypot(src−portal)`，把"门内侧 1.67 m（已进入轿厢）"当成"离门口 1.67 m（在外面）"⇒ 拒绝上梯。**独立 driver 的判据是 `along ≥ car_depth(1.20) 或 travelled ≥ enter_distance(3.00)`，有向**。

**修复**：① 守卫改有向——用 `_entry_inside_car()` 的 `along`，只在 `along < 0`（仍在门平面外）拒绝；② `enter_distance` 2.45 → **3.00**（对齐 driver 默认；也让第一次进梯落到 `along≈+0.6`，越过 0.35 门槛，不再无谓倒退）。离线门禁全绿。

**run167 重跑**：`logs/run167_fused_enterfix_20260915/seed_20260902`（同参数）。待验证：`ENTER_ELEVATOR → RIDE_TO_FLOOR_1` 是否发生、之后 ⑵ 上层 4.0 m 固定前行与 ⑶ `RETURN_TO_SPAWN` 三阶段。

### 11.6 run167 结果：登梯/乘梯/出梯都通了，但出梯后"在轿厢口原地转不动"（对上了 driver 的 TO_CORRIDOR 子段 0）

**run167 通的部分**：走廊起点 0.079 m；`ALIGN_ELEVATOR`（sim 391.9）→ `ENTER_ELEVATOR`（405.5）→ **`RIDE_TO_FLOOR_1`（417.6）** —— 登梯有向守卫 + `enter_distance=3.00` 生效，**真的上楼了**；`ALIGN_FLOOR_1_EXIT` → `EXIT_ELEVATOR`（439.5）也正常。

**run167 卡点**：`ESTABLISH_FLOOR_1_TOPOLOGY` 的 `CLEAR_CAR` 相位，遥测：

| sim | src | yaw | vx / wz |
|---|---|---|---|
| 449.9 | (5.83, −0.37) | −1.54 | 0 / 0 |
| 451.9 | (5.80, −0.36) | −1.93 | 0 / **−0.45** |
| 453.9 → 535.9 | (5.79, −0.36) 冻住 | **−1.97 冻住** | 0 / **−0.45** |

即：离车门平面 **1.28 m**、在轿厢口，命令持续转 150 仿真秒但 yaw 不动 ⇒ **物理上转不动（楼梯核心与电梯井之间夹住）**。最后：
```
18:03:55 [WARNING] Floor 1 topology not reached within 150s
         (route_retry_count=0, front_clearance=1.41); continuing
18:03:55 state -> FLOOR_1_READY
```
150 s 超时兜底放行 → 监督器起二层探索器 → 机器人才动。**所以"它又开始走了"是超时兜底，不是修复**；而且探索器在轿厢口起锚，**二层入场的 4.0 m 不是从走廊起点开始，第 ② 段这轮无效**。

**根因（没对上独立 driver）**：driver 的 `TO_CORRIDOR` 子段 0（`excursion_leg == 0` 且 `exit_reverse`）明确写：
> "倒行出梯后机器人朝轿厢内、staging 航点在正后方，在那里掉头会把机器人卡死（probe 02），所以**先沿当前朝向倒车直走到 staging 航点，到了有空间再转**。"

主线 `CLEAR_CAR` 却调 `_drive_planned_to(staging)`；距离 ≤ `direct_entry_max_distance=4.0` 时走 `_drive_direct`，而 **`_drive_direct` 是先原地转向再前进** ⇒ 正好在轿厢口打转。

**修复**：`CLEAR_CAR` 用已知车门的门框 `along` 算出还需外移多少，用 `_drive_distance(..., reverse=True)` **沿门法线倒车直走**到 `along = −staging_distance`；门框不可用时才回退 `_drive_planned_to`。转向改在 staging（离车 2.2 m、有空间）处发生，与 driver 一致。离线门禁全绿。

**run168 重跑**：`logs/run168_fused_exitfix_20260915/seed_20260902`。待验证：`CLEAR_CAR` 应在 1–2 s 内倒车完成、`TO_CORRIDOR_START` 到达走廊起点（gate_along≈0）、随后探索器 `initial_forward_distance=4.0` 从走廊起点起算。

### 11.7 run168：出电梯修复验证通过（2026-09-15 深夜）

状态链（无任何告警）：
```
18:11:40 RETURN_TO_ELEVATOR
18:13:03 ALIGN_ELEVATOR
18:13:18 ENTER_ELEVATOR
18:13:31 RIDE_TO_FLOOR_1
18:13:56 ALIGN_FLOOR_1_EXIT → EXIT_ELEVATOR
18:14:06 ESTABLISH_FLOOR_1_TOPOLOGY
18:14:34 FLOOR_1_READY            ← 28 s（run167 卡 150 s 超时）
```

`ESTABLISH` 遥测（出梯→走廊起点）：
| sim | src | gate_along | 动作 |
|---|---|---|---|
| 436.3 | (5.82, −0.25) | −4.676 | 进入 ESTABLISH |
| 438.3 | (5.83, **+0.21**) | −4.670 | **倒车直走到 staging（不在轿厢口打转）** |
| 440–448 | (5.85, 0.15) | −4.64 | 在 staging 转身（有空间） |
| 450.3 | (6.37, 0.15) | −4.132 | 前进 |
| 460.3 | (10.06, 0.50) | **−0.440** | 到走廊起点（横向 0.097） |

二层探索器：`initial_forward_distance=4.0`、`initial_forward_active=True`、`topology_region=CORRIDOR` ✅

**同时确认**：走廊起点→电梯（ALIGN 15 s → ENTER 13 s → RIDE）、登梯有向守卫（真的上梯）都通过。

**run168 暴露的一个小瑕疵（已修，只影响后续轮次）**：`CLEAR_CAR` 里 `distance` 每周期重算、而 `_drive_distance` 的 `travel_anchor` 只设一次，两者相互缩水 ⇒ **只倒了一半**（实测 0.46 m / 需要 0.80 m），少了 0.34 m（这次有空间所以没影响）。修复：新增 `establish_clear_distance` **只 latch 一次**，`_set_state` 一并复位。离线门禁全绿。

**任务聚焦**：噪点问题（已量：漂移 max 0.44 m / median 0.36 m、回环无发布者 count=0、地图实心占 62.7% 而孤立点仅 1.2%）按用户指示暂停，先把三层测试跑通。
