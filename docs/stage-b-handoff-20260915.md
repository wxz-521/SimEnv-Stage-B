# Stage-B 会话交接（2026-09-15 深夜，碰撞倒退版）

> **用途**：本轮跨会话交接。新会话先读这一份，再读 `docs/功能模块设计.md`（活设计文档，含全部变更记录）。
> **一句话状态**：三层主线的电梯三段融合 + 三次死锁修复已离线验证通过；本轮又加了「碰撞倒退 + 自身点过滤 + MAP_TRAPPED 告警」，**代码+单测就绪，但尚未在活体运行中确认触发**。当前有一轮在跑。

---

## 0. 目标与验收（不变）

- **验收场景 seed = `20260902`**、覆盖 55%、三层、回出生点；跑通后切 84% 并把红球纳入验收（坐标容差 1.0 m）。
- 判据：不「进房→出门→再进房」、目标不在左右房横跳、三层各房覆盖达标 → 电梯上下 → 回出生点。
- ⚠️ **重要**：当前 `generated_building/` 是 **seed 20260915** 生成的场景（本轮测试顺手用了新 seed，4/4/4 房）。**跑验收前必须重新生成 seed 20260902 的场景**，否则场景不是验收那栋楼。

---

## 1. 本轮代码改动（都已落地，均通过离线门禁）

| # | 改动 | 文件 | 状态 |
|---|---|---|---|
| 1 | **电梯三段融合进主线**：① 走廊起点→电梯 `_control_return_to_lift()`（A\* 到闸门 → `_center_on_corridor_axis` → 掉头 → 车门前 staging）；② 电梯→走廊起点 `ESTABLISH_FLOOR_1_TOPOLOGY` = `CLEAR_CAR → TO_CORRIDOR_START → CORRIDOR_CENTRE`；③ 电梯→大门→出生点 `_control_spawn_return()`（AXIS→FACE→REVERSE 倒行回出生点） | `elevator_transition_node.py` | ✅ 活体验证（run166/167/168） |
| 2 | **登梯守卫改有向 + 进梯距离**：`ENTER_ELEVATOR` 用门框 `along<0` 判"在外面"；`enter_distance` 2.45→**3.00**；`RIDE_TO_GROUND_FLOOR` 去掉 `+π` 翻转 | 同上 | ✅ 活体验证（run167 真的上梯） |
| 3 | **出梯后倒车直走到 staging 再转向**（对上了 driver `TO_CORRIDOR` 子段 0，避免在轿厢口原地打转卡 150 s）；`establish_clear_distance` latch 一次 | 同上 | ✅ 活体验证（run168：ESTABLISH 28 s，无超时） |
| 4 | **上层固定前行 6.00→4.00**（仅 2/3 层；走廊起点≈闸门 10.5，一层 14.5 节点即闸门后 4.0 m） | `two_floor_explorer_supervisor.py` | ✅ 活体验证（run168 起锚 4.0） |
| 5 | **碰撞倒退**：IMU 冲击检测（|比力|相对基线突跳 ≥5 m/s² 或角速率 ≥3 rad/s）+ 沿"面包屑轨迹"倒退 0.8 m（速度 0.22、重试上限 3、超时 8 s、失速 1.5 s）+ 前方受阻/无位移时也触发 | `coverage_explorer_node.py` + `coverage_explorer_core.py` | ⚠️ 单测就绪，**活体未确认触发** |
| 6 | **自身点过滤**：`lio_occupancy` 投影前丢弃距机器人 <0.55 m 的点（机器人自身腿/身体不把自己格子标 occupied，否则 `end_pose_clearance=0` 规划器永不规划） | `lio_occupancy_node.py` + `map_floors_core.py` | ⚠️ 单测就绪，活体未确认 |
| 7 | **MAP_TRAPPED 告警**：`navigation_reachable_cells<8` 或起点净空 0 且仍有未完成房间 → 打 `rospy.logerr`（纯观测，不改行为） | `coverage_explorer_node.py` + core | ⚠️ 单测就绪，活体未确认 |

新增纯函数 + 单测：`collision_impact_detected` / `retreat_trail_target` / `map_is_trapped`（`coverage_explorer_core.py`）、`filter_self_returns`（`map_floors_core.py`），测试在 `src/simnav/test/test_collision_retreat_core.py`（17 项）。

### 关键新参数（节点 `~` 参数，有默认值，未写入 launch）

`collision_impact_accel=5.0`、`collision_impact_gyro=3.0`、`collision_impact_hold=1.0`、`collision_retreat_distance=0.80`、`collision_retreat_speed=0.22`、`collision_retreat_limit=3`、`collision_retreat_timeout=8.0`、`collision_retreat_stall_seconds=1.5`、`pose_trail_spacing=0.10`、`self_filter_range=0.55`（`lio_occupancy`）。

---

## 2. 离线门禁（本轮全部绿）

- `py_compile`（5 个脚本）✅
- `check_undefined_attrs.py` / `check_read_before_assign.py`（node + lio_occupancy + elevator_transition）✅
- **离线回归 259 项全绿**（原 242 + 新增 17）✅

命令（容器内）：
```bash
docker exec simenv-noetic bash -lc 'cd /workspace/SimEnv && source /opt/ros/noetic/setup.bash && \
  source .simenv_build/devel/setup.bash 2>/dev/null && \
  export PYTHONPATH=/workspace/SimEnv/src/simnav/scripts:$PYTHONPATH && \
  python3 -m unittest discover -s src/simnav/test -p "test_*_core.py"'
```

---

## 3. 当前运行（交接时仍在跑）

- 目录 `logs/three_floor_collision_gate_20260915/seed_20260915`，**seed 20260915 / 覆盖 0.40 / three_floor**（碰撞门测试，不是验收参数）。
- 启动命令（**直接 `run_stage_b_seed.sh`，非受护启动**）：
  ```bash
  env STAGE_B_GUI=false STAGE_B_START_RVIZ=1 STAGE_B_ROOM_COMBINED_COVERAGE_TARGET=0.40 \
    setsid ./team_scripts/run_stage_b_seed.sh 20260915 2400 logs/three_floor_collision_gate_20260915 three_floor
  ```
- 进度：~38 min 墙钟跑到 **sim ≈ 1988**，正在 `ENTER_ELEVATOR`（第 4 次进梯，已上到高层）；Gazebo GUI 关、RViz 开。
- **碰撞倒退尚未观察到触发**：`two_floor_supervisor.log` 里没有 `Collision retreat` / `MAP_TRAPPED` 日志（说明这轮还没发生碰撞，或阈值未命中）——这是第 5/6/7 项仍属"未活体验证"的直接证据。

---

## 4. 自审查：哪些已验证 / 哪些还没（诚实版）

**已验证（活体）**：电梯三段融合、登梯有向守卫、出梯倒车、上层 4.0 m。run166/167/168 的证据在 `docs/stage-b-handoff-20260914.md` §11.5–11.7。

**仅单测、未活体验证**：碰撞倒退（#5）、自身点过滤（#6）、MAP_TRAPPED 告警（#7）。这三项的触发依赖真实碰撞/漂移，本轮还没撞，所以**实际效果未知**。

**已知风险 / 待办**：
1. **碰撞倒退阈值是经验值**（IMU 5 m/s² / 3 rad/s），可能误报（急转/台阶）或漏报（慢速顶墙）。`_impact_recent` 只作用于 `initial_forward_complete` 之后，入场 trans转瞬态被排除。
2. **自滤半径 0.55 m** 会滤掉 <0.55 m 的所有点——导航净空 0.30 下正常不会在 <0.55 m 处贴墙，但极窄门洞需复核。
3. 电梯 `CLEAR_CAR` / staging / 对中现已改用**源图候选**（`_approach_portal()`），参考只做校验、不控制（见 §6）。
4. 改 explorer 后 `run_stage_b_seed.sh` 会打 "frozen floor-0 manifest differs" **告警（非致命）**。
5. 旧代码 `_elevator_staging_target`、`return_mouth_along`/`return_standoff_*` 等仍保留（dead code，未删）。
6. 本轮临时探针脚本（`.dsh_*`）已清理；`team_scripts/elevator_only_driver.py`/`elevator_only_test.sh` 的改动是更早会话留下的，未动。

---

## 5. 下一步建议

1. 让当前 `three_floor_collision_gate_20260915` 跑完或跑出一次碰撞，**确认碰撞倒退真的触发**（看 `two_floor_supervisor.log` 里的 `Collision retreat` 与状态字段 `collision_impacts/retreats`）。
2. 确认后，**重新生成 seed 20260902 场景**，用受护启动跑正式验收（55%）。
3. 若碰撞倒退表现不稳，优先调 `collision_impact_accel/gyro` 与 `collision_retreat_distance`。

---

## 6. 去硬编码：电梯只通过"电梯候选"识别（2026-09-15，已执行，未上机验证）

**问题**：主线 `elevator_transition_node.py` + `stage_b_two_floor_support.launch` 把电梯门世界坐标 `~elevator_door=[1.65, 2.60, 0.0, 1.40]` **直接用于控制**（approach/align/entry centring/staging/CLEAR_CAR 全部算在 known door 上），即"提前知道坐标"。

**改法（对照独立脚本忠实测试 `elevator_only_driver.py` 的 `_confirm_door_candidate`）**：
- `_approach_portal()` → 只返回 `self.elevator_portal`（`detect_wide_lobby_openings` 时序确认的**候选**，源图 x/y/yaw/width）。**控制只用候选**。
- `_entry_inside_car()` / `_entry_outside_car()` / `_drive_distance` 对中 / `_elevator_staging_source()` / `CLEAR_CAR` 全部改用**源图候选**。
- 删除 `_known_door_source()` / `_known_door_side()`。
- **保留 `~elevator_door` 作为"合理性标尺"（sanity yardstick），只用于校验、绝不用于控制**：新增 `_reference_source()`（用成对闸门把世界参考映到源图），在 `_navigation_map_callback` 确认候选后，校验 `|lateral| ≤ elevator_candidate_max_lateral(0.60)`、`|yaw err| ≤ 0.25`、`width ∈ [0.90, 3.80]`；不通过则 REJECT（这正是忠实测试防止 run159 锁错墙的做法）。
- 保留 `elevator_door_inset=0.35`（行为阈值，非坐标）。

**验证**：`py_compile` + `check_undefined_attrs` / `check_read_before_assign` OK；离线回归 **259 项全绿**。

**注意 / 风险**：
- 候选确认依赖 `elevator_floor_0` 初始开门（`initial_open: true`，已核实）；若 floor 0 完成时候选仍未确认，`WAITING` 会一直 `Waiting for map-confirmed elevator portal`（run117 老风险）。**没有"参考兜底控制"**（这是比独立 driver 更严的忠实：driver 候选失败会退回参考，主线不退回）。
- 候选是 ~3.5 m 门厅开口，中心可能偏离真实 1.4 m 轿门 ~0.41 m（run152）；对中/登梯以候选中心为准，可能夹门框，需活体验证。
- 仍保留的"结构参数"（非电梯绝对坐标）：`elevator_lobby_wall_offset=1.65`（候选搜索横向带）、`spawn_turn_y=1.80`（回出生点门厅 U 转世界 y）。若也要去，另行讨论。
- **当前在跑的 `three_floor_collision_gate_20260915` 还是旧代码（带 known door 控制），改完不影响已启动进程；要验证这条需重跑。**

**提交后待办**：以 seed 20260902 重跑，验证"仅靠候选 + 参考校验"能进电梯/出电梯/回出生点。
