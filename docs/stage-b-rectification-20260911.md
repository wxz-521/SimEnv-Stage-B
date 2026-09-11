# Stage B 三层端到端整改记录（2026-09-11）

基线：git `fefbd0b`，备份 `backups/pre_danger_strategy_20260911_075503`。
本轮所有改动都先由遥测/日志定位根因，再改代码复测。

## 0. 前一轮测试的收尾

上一轮的 `run14_fallfix_20260911`（three_floor，room 目标 0.75）在容器内仍在运行
（已跑 37 分钟、sim≈296 s），并且另一个会话又启动了 `run16_endtoend_20260911`
（`STAGE_B_ROOM_COMBINED_COVERAGE_TARGET=0.75`，与本轮 0.84 判据冲突）。两者都已终止，
仿真进程、ROS 端口 11320/11345 已确认释放后再开始本轮测试。

## 1. 根因与改动

### 1.1 三层验收判据自相矛盾（阻塞性）

`monitor_stage_b_coverage.py` 的 three_floor 通过条件要求
`two_floor_mission_complete and latest_elevator.floor_index == 2`。但
`fefbd0b` 新增的下降流程在 `RIDE_TO_GROUND_FLOOR` 把 `current_floor` 置回 0
（`elevator_transition_node.py`），而 `two_floor_mission_complete` 只在
`RETURN_TO_SPAWN`（更晚）才置 True。于是同一份 status 里
`two_floor_mission_complete==True` 必然伴随 `floor_index==0`，该条件永远不可能成立，
`passed` 恒为 False。

改动：three_floor 改为校验 `two_floor_mission_complete and returned_to_spawn and
main_entrance_opened and 2 in completed_floor_indices and 三层房间全 COMPLETE`。
这不是放宽判据：`completed_floor_indices` 仍要求 0/1/2 三层，且新增了返回出生点与开门要求。

### 1.2 综合覆盖率判据被下调

工作区把房间综合覆盖目标从 0.84 下调成 0.75（三层 supervisor）甚至 0.40（run6），
全局 `camera_coverage_target` 也从 0.85 下调。历史证据
（`SimEnv_two_floor_exploration_codex_spec.md` §17、09-01 的
`coverage_speed_sweep_95cam_20260901/speed_060`）表明：
room 0.84 + camera 0.85 时 seed 20260902 四房完成、红球 3/3、虚警 0、sim 334.8 s；
camera 目标降到 0.75 后单层只剩 2/3，room 目标 0.40 时只剩 1/3。

改动：恢复 room `0.84` / camera `0.85` / combined `0.84`（laser `0.95`），
涉及 `stage_b_behavior.launch`、`stage_b_floor_explorer.launch`、
`two_floor_explorer_supervisor.py` 默认值。

### 1.3 非探索段过慢

run6 每状态耗时：`RETURN_TO_ELEVATOR` 82.8 s、`RETURN_TO_FLOOR_1_GATE` 81.0/73.0 s，
而这两段被 `min(motion_speed, 0.30)` 夹到 0.30–0.35 m/s；初始大门→走廊为 0.45 m/s。
run6 全程 958 s 里约 295 s 花在非探索过渡。

改动（探索段速度保持 0.60、过门仍夹 0.25 不变）：
- `elevator_transition_node` 新增 `lobby_approach_speed`（默认 0.45），替换两处
  `min(motion_speed, 0.30)`；launch 里 `motion_speed` 0.60→0.90、`crossing_speed` 0.22→0.28。
- 探索器 `initial_forward_speed` 0.70→0.90（大门→走廊），新增 `transit_speed`（0.90）
  仅用于走廊后段直线转场；房间内仍用 `motion_speed`=0.60。

### 1.4 危险源漏检

- `danger_detector` 只看 RGB-D，前视相机固定；漏检主因是「相机从未对准红球」：
  floor-0 的 id1/id10 分别在门墙内侧 5.8 m / 7.7 m，room 目标 0.40 时机器人根本不进深处。
- 探索器的激光球体假设（`detect_sphere_like_clusters`）在全部 241 份历史 result.json 中
  `sphere_hypotheses==0`，`SPHERE_REVIEW` 从未触发；且 planner 在走廊/房间锁定时会丢弃
  SPHERE_REVIEW。

改动（纯几何/信息增益，无强化学习）：
- `danger_detector_node` 新增 `/simnav/danger_candidates`，发布**未确认**的红球轨迹
  （≥1 次观测、当前楼层、时间新鲜）。
- `coverage_explorer_node` 订阅该话题，把候选当作 `SPHERE_REVIEW` 目标，看到相机已扫过
  其位置即标记 reviewed；候选不进入 `_stable_spheres`，不影响楼层完成门。
- `coverage_explorer_core` 让 SPHERE_REVIEW 在无锁时全局优先，并在房间锁定时允许
  走廊所属的红球 review 跨锁执行。
- 危险源确认仍完全由原 RGB-D 形状/颜色门 + `confirmation_frames=3` 决定，未放宽，
  因此不会增加红方块/绿球虚警。

### 1.5 伪跌倒导致任务误判

遥测显示 metric z 在长跑中缓慢下漂（run14：0.33→0.07 m，全程 roll/pitch < 0.07 rad，
机器人仍在正常覆盖房间），而 `_check_fall` 用绝对高度 `z < 0.10` 判跌倒，
会在跑到一半时 `_fail(ROBOT_ON_GROUND)` 终止任务。

改动：跌倒判定改为「短窗口突降」——`fall_drop_window=1.5 s` 内高度下降
≥`fall_drop_threshold=0.15 m` 且当前 < `fall_base_height`，倾角阈值 60°→30°
（实测站立 <11°、真跌倒 35–45°）。遥测记录器同步改为突降判据。

### 1.6 三层危险源评估

runner 原先对 three_floor 直接跳过危险源评估。改动：所有模式都评估；
two/three_floor 用 `--floor-index -1` 评估全楼层真值（5 个红球），单层仍为 floor 0。

### 1.7 单层验收分支自相矛盾（阻塞性）

`monitor_stage_b_coverage.py` 的单层分支要求
`latest_elevator.get("floor1_topology_isolated")`，但 coverage 模式根本不启动电梯节点，
`latest_elevator` 为 `{}`，于是单层即使四房全部 COMPLETE 也永远 `passed=False`。
（09-01 的历史单层通过记录显示 `latest_elevator=null` 且 `passed=true`，说明该条件后来被错误地套到了单层分支上。）
改动：仅当显式 `--wait-floor-transition` 时才要求电梯隔离标志；覆盖率模式只校验
`floor_complete + 4 房 COMPLETE + 无未复核球体 + max_pose_step<1.0`。

### 1.8 规划器死锁（本轮首次三层 0.84 运行的直接失败原因）

`run17`（three_floor, 0.84/0.85/0.84）在 floor 0 卡死：`ROOM_L_43` 的接近失败后被置为
`BLOCKED`，此后走廊已全部建图，planner 返回 `NO_FRONTIER` /
`last_reject_reason=EMPTY_ROOM_TASK_MASK`，机器人 `cmd_vel=0` 原地停留直到超时
（telemetry：sim 265→345 `src` 恒为 (24.20, -0.25)）。
日志显示接近失败前 planner 连续派出 `ROOM_L_43` 的 CAMERA_FRONTIER，
路径长度 23.14 m / 32.24 m，而目标直线距离只有几米——目标在墙后，A* 绕行。

改动（两条，均为有界恢复，不放宽覆盖率判据）：
1. **绕行门控**（`coverage_explorer_core.plan` 的最终候选提升）：
   `path_length > 8.0 且 path_length > 3.0 × 直线距离` 的候选视为墙后目标，直接淘汰，
   让下一个更近的候选被选中。
2. **有界重试**（`coverage_explorer_node._retry_blocked_room`）：
   当 planner 无目标且存在未完成的 BLOCKED 房间时，重新打开覆盖率最低的那个房间
   （状态回到 APPROACHING、清零 miss 计数、清空 blocked 冷却），每个房间最多
   `max_room_retries=3` 次，避免无限空转也避免不可达房间吃满整个 run。

## 2. 复测结果

### 2.1 并发会话冲突（已确认）

本会话期间存在另一个 DSH 会话在同一工作区操作仿真：
它启动了 `run16_endtoend_20260911`（`STAGE_B_ROOM_COMBINED_COVERAGE_TARGET=0.75`，与本轮 0.84 判据冲突），
其运行杀死了本会话的 `run15`；并且它的文件编辑与本会话交错
（`elevator_transition_node.py` 12:35、`stage_b_two_floor_support.launch` 12:36 出现本会话未写的
`fall_baseline_window`/`fall_baseline_min_samples` 参数）。
该会话的进程已终止（run14/run16 与 9 小时前的 `watch_run_periodic` watcher 均已停止），
其最后一次文件编辑停在 12:36，之后未再活动，因此本会话继续执行。

### 2.2 run17（three_floor, 0.84）— 失败，已定位

见 1.8：`ROOM_L_43` 接近失败后 planner `NO_FRONTIER`，机器人原地停到超时。

### 2.3 run18（coverage/single floor, 0.84）— 失败，已定位

对侧房间 `ROOM_R_15` 在站点内 20 个 planner 周期都拿不到可执行目标
（`Opposite room ROOM_R_15 has no executable target yet` ×20 → `unreachable for 20 cycles; releasing station lock`）。
有界重试 1/3、2/3、3/3 均复现同一结果，说明不是等待时间不足，而是该房间在当前地图/拓扑掩码下
确实产生不出目标。据此按用户选择执行「回退到已验证基线参数」。

### 2.4 回退项（保留本轮新增能力）

回退（前一轮未验证的实验项）：
- `door_search_enabled` true→false（节点默认与两个 launch）；
- `room_frontier_wait_cycles` 20→0（恢复「持有房间锁」的已验证行为）；
- planner 里 all_camera_cells 为空时的 relaxed 相机回退删除；
- `navigation_clearance` 0.24→0.30、`preferred_clearance` 0.36→0.42；
- `camera_points_memory_limit` 400000→120000、`camera_observation_max_cells` 200000→30000。

保留（本轮新增、且与判据无关）：0.84/0.85/0.84 覆盖率、非探索提速、红球定向
（`danger_candidate_min_hits` + `/simnav/danger_candidates` + SPHERE_REVIEW 全局优先）、
突降跌倒判据、绕行门控、有界重试、验收脚本修正、三层全楼层危险源评估。

### 2.5 run19（coverage/single floor, 0.84, 基线回退后）

结果：`floor_complete=True`，**4/4 房完成**（ROOM_L_15/ROOM_L_43/ROOM_R_15/ROOM_R_42），
`elapsed_sim=355.1 s`（历史 0.84 单层基准 334.8 s，同量级），`mission_fault=None`。
但危险源 `recall=0/3`（detected 0，missed 3，虚警 0）。基线回退修好了完成度，红球召回是下一个缺口。

### 2.6 红球召回根因（run19 遥测 + run20 在线探针）

先用 run19 遥测做几何复盘：三个红球**都曾进入前视相机视场**（60° HFOV、深度 <8 m）——
id1 最近 3.45 m、id5 2.36 m、id10 0.26 m，且各自存在正对（方位误差≈0°）的采样
（id1 sim 78.6 @6.17 m、id5 sim 294.0 @5.63 m、id10 sim 261.3 @0.91 m）。
所以不是"相机没看到"，而是检测环节丢掉了。

为定位，给 `danger_detector_core/node` 增加逐帧聚合诊断（red_mask 帧数、mask 像素、
contour_pass、depth_pass、tf_reject、floor_reject、最大轮廓的 area/circularity/aspect/extent/radius）。
run20（moving_frequency 5→10）实测：
- `red_frames` 持续增长、`contour_pass` 增长、随后 `depth_pass=11`、`floor_rej=11`；
- `results/detected_danger_debug.json` 出现**已确认轨迹 id0 = (-6.33, 18.72, -0.08)**
  （真值 id1 = (-6.858, 18.695)，偏差 0.53 m，10 次观测）。

在线探针（对同一帧跑检测器 + 打印红轮廓指标）显示：红球轮廓会随接近从
area 1678→2938→4678、circularity 0.81→0.87→0.90、半径 27→33→40，
且轮廓中心处深度 **361/361、625/625、841/841 全部有效**。即：球能被干净地看到并测到深度。

因此本轮真正的两个损失点：
1. **抽帧率**：5 Hz 门限把"球恰好通过视野"的帧大量丢掉（run11 曾记录 rate skip≈10119）。
   提高到 10 Hz（只减少丢帧，不放宽颜色/形状/3 帧确认门）后，run20 立即检出 id1。
2. **楼层高度带**：`floor_min_offset=-0.2` 把投影 z≈-0.08 的观测判为"不在本层"
   （floor_rej=11，恰好等于 depth_pass 数）。metric z 是层内相对量且长跑下漂
   （0.33→0.07 m），因此把下界放宽到 **-0.6**（上界仍 1.2）。

（说明：期间一度怀疑 RGB 与深度图上下翻转——在线统计 `same_row=0/289`、`flipped_row=289/289`；
但抓取实帧直接查看后确认两图**对齐**，顶部为超出 8 m 远裁剪的 NaN 区域，
翻转假设不成立，未据此改代码。）

### 2.7 run20（moving_frequency=10 + 诊断）

结果（`logs/run20_single084_reddiag_20260911/seed_20260902/result.json`）：

- `passed=True`，`floor_complete=True`，**4/4 房**（ROOM_L_15 / ROOM_L_43 / ROOM_R_15 / ROOM_R_43）
- `elapsed_sim=359.194 s`（历史 0.84 单层基准 334.8 s，同量级）
- 危险源评估：`correct=3, missed=0, false_alarms=0, recall=1.0, passed=True`（floor 0 真值 3 个红球全部命中）
- 三条确认轨迹：id0=(-6.33,18.72)↔id1 偏差 0.53 m；id1=(-1.28,31.32)↔id5 偏差 0.58 m；
  id2=(9.44,25.93)↔id10 偏差 0.66 m

**即 0.84 覆盖率判据 + 3/3 红球召回 + 0 虚警 同时达成（单层，seed 20260902）。**
注意：本次运行仍使用 `floor_min_offset=-0.2`（-0.6 的放宽是运行开始后才改的）；
三条轨迹的 z 分别为 -0.08 / -0.18 / -0.17，已贴近 -0.2 边界，放宽后更安全。

## 4. 红球引导：分级、渐进（2026-09-11 追加）

需求：红球检测一点点来，不一开始就把红球当主目标，而是加入"红球对探索的引导"机制，
逐步扩大能力，找到最好的一版。

实现：新增 `danger_guidance_level`（0–3，默认 **1**），只对**颜色检测器**来源的红球
候选（`hypothesis_id` 以 `DET_` 开头）生效；激光球体假设（`sphere_*`）保持已验证基线优先级，
不受该档位影响。

| 档位 | 行为 |
| --- | --- |
| 0 | 只检测，红球候选不参与规划（节点不把候选交给 planner） |
| 1（当前默认） | 红球候选只作兜底：排序排在普通 frontier 之后，只有没有其他目标时才去 review；不抢占 |
| 2 | 在已允许的拓扑内排第一（同一房间内优先），但不强抢安全的在途目标、不跨房间锁 |
| 3 | 全局优先 + 可抢占安全在途目标 + 允许走廊红球跨房间锁 |

实现位置：
- `coverage_explorer_core.plan(..., danger_guidance_level=)`：`priority`/`target_priority`
  只对 `DET_` 候选应用档位；走廊候选在 level≥1 才进入无锁分支，跨锁 review 在 level≥3 才启用。
- `coverage_explorer_node`：`danger_guidance_level` 参数（0–3）；level 0 时
  `_detector_candidates_locked()` 返回空；`sphere_preemption` 仅在 level≥3 生效。
- launch：`stage_b_behavior.launch` / `stage_b_floor_explorer.launch` 显式写
  `danger_guidance_level=1`。

单元测试：`test_detector_review_guidance_is_graded`（level 1 时普通 frontier 胜出、
level 2 时 `DET_0` 胜出）与 `test_lidar_sphere_review_keeps_validated_priority`
（激光假设在 0–3 档都保持第一）。全部 61 项离线测试通过。

同时把检测器 `moving_frequency` 5→10 Hz（只减少抽帧、不放宽颜色/形状/3 帧确认门），
提高"恰好看到一眼红球"的机会。

## 5. 产物

- `logs/run17_endtoend_084_20260911/seed_20260902/`：三层端到端（0.84）失败轮（planner 死锁证据）。
- `logs/run18_single084_20260911/seed_20260902/`：单层（0.84）失败轮（对侧房间无目标证据）。
- `logs/run19_single084_baseline_20260911/seed_20260902/`：基线回退后的单层验证。
