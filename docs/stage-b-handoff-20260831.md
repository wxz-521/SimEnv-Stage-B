# Stage B 当前实现交接与问题审计

> 2026-08-31 最终追加：已收窄门洞缓存，只缓存 planner 已接纳的 actionable/front-station 门；RETURNING 仅允许 `RETURN_TO_CORRIDOR`；进门与返回仅在确认门宽内平移穿门线，返回门线只忽略连通簇不超过 2 格的 LIO endpoint 孤点，连续墙体、未知格、一般导航净空与实时激光急停均保持。离线核心测试 31/31 通过。真实无界面 Gazebo seed `20260902` 在 `523.186 s` 完成四房进入、局部覆盖和退出，`floor_complete=true`，危险物 3/3、漏检 0、误报 0。单 seed 功能闭环已通过；5-seed 稳定验收尚未完成。

> 交接日期：2026-08-31（Asia/Shanghai）  
> 工程目录：/home/xiejiaxue/SimEnv_20260806_transfer/SimEnv  
> 工作分支：two-floor-navigation  
> 运行环境：simenv-noetic（ROS Noetic + Gazebo Classic）  
> 当前状态：本轮真实 Gazebo 后端已停止，Gazebo、RViz、ROS、Stage B 进程已清理。  
> 本文件用途：完整说明当前实际代码、与总方案的偏差、已经做过的修改、测试证据和未解决的问题，供后续接手者使用。

## 1. 结论先行

当前版本不能宣称 Stage B 已完成。

已经证实：

- 真实 Gazebo 后端可以启动，Gazebo GUI 可以关闭，RViz 可以同时打开；
- 开局楼梯 RL policy 可以完成固定前进；
- 固定前进停止后可以切换到平地 RL policy；
- 部分 seed 可以确认首个右侧门并真正进入房间；
- 房间综合覆盖率已经采用激光 10% + 相机 90%；
- 房间达到综合覆盖阈值后会尝试生成 RETURN_TO_CORRIDOR；
- 每轮测试可以保存 RViz 截图、探索地图和运行日志；
- 多 seed 测试期间没有修改代码，使用了冻结代码哈希。

尚未证实且当前明确失败：

1. 门洞确认后，部分 seed 无法生成任何门内前沿目标，持续 NO_FRONTIER，机器人没有进房。
2. 有的 seed 可以完成首房覆盖，但 RETURN_TO_CORRIDOR 过程中又被 CAMERA_FRONTIER 覆盖，机器人重新深入房间，无法退出。
3. 本轮三个正式 seed 都没有完成四个真实房间，均没有 floor_complete=true。
4. 本轮危险物结果没有通过；进入房间的 seed 仍为 0/3。
5. 后方两个房间尚未进行有效验证，因为前方房间没有稳定完成并回到走廊。

准确状态应写成：

> 房间局部覆盖、门洞确认和退出骨架已经形成，但“确认门洞后生成门内目标”和“房间完成后返回状态绝对隔离”仍未收敛。先修这两个根因，再继续增加机制。

## 2. 总方案和当前实现的关系

总方案文件：

    SimEnv_two_floor_exploration_codex_spec.md

总方案要求的最终数据流为：

    定位后端
      -> Navigation Map / Exploration Map
      -> CorridorEstimator
      -> Scan/Map DoorEvidence
      -> DoorFusion
      -> DoorCandidateManager
      -> SingleFloor FSM
      -> 危险物检测与 ResultWriter
      -> 单层稳定后再进入双层任务

总方案规定：

- 算法不能把 Gazebo 真值接入运行时规划；
- 走廊不使用普通全局 Frontier；
- 房间内部使用拓扑约束的信息增益前沿；
- 四个真实房间必须全部进入、扫描、退出；
- 危险物不能漏检；
- 单层稳定通过前不得进入双层集成；
- 失败先分类，不能根据单个 seed 无限增加特殊分支。

当前版本是为了快速验证 Stage B 的过渡实现，主要逻辑集中在：

    src/simnav/scripts/coverage_explorer_core.py
    src/simnav/scripts/coverage_explorer_node.py

它还不是总方案中完整拆分后的 DoorFusion、DoorCandidateManager、RecoveryManager、RoomFrontierPlanner 模块化结构。

## 3. 与总方案不同的地方

| 总方案要求 | 当前实际实现 | 性质 | 风险 |
|---|---|---|---|
| 最终使用 Livox + IMU LIO | Stage B 主链仍是 Hector 提供 x/y，IMU 相对航向提供 yaw；本轮没有切换到 FAST-LIO 主定位 | 未完成 | 长走廊尺度压缩、房内平移漂移仍会影响门和危险物坐标 |
| Scan/Map Detector 只输出 Evidence，由独立候选管理器统一管理 | 当前覆盖探索节点直接完成地图门洞检测、确认计数、拓扑 ID 和调度 | 过渡实现 | 门洞识别、候选生命周期和拓扑调度互相耦合 |
| 不以覆盖率作为完成条件 | 当前房间以局部综合覆盖率完成，楼层以四个已完成拓扑房间完成 | 这是你后续明确提出的变更 | 与总方案旧文字不一致；必须保持所有文档口径一致 |
| 房内使用局部信息增益前沿 | 当前生成 CAMERA_FRONTIER，并按 camera_weight=0.90 选择；仍依赖全局地图可达性、任务 mask 和 topology_id_for_point | 部分实现 | 可能出现门已确认但没有可用门内前沿 |
| 走廊只运输和找门 | 当前有 FRONT_DOOR、REAR_TRANSIT、虚拟入口 gate 和门站调度；仍保留全局覆盖快照及激光 Frontier fallback | 部分实现 | 走廊边界、门站分配和前沿候选仍可能串联 |
| 进房时建立 Room Entry Frame，并用局部回环辅助退出 | 当前主要缓存已确认门洞并使用相对门中心生成退出路径；完整 Entry Frame/ICP 不是本轮主退出链 | 部分实现 | Hector 漂移会使退出路径与实际门位置不一致 |
| 相机在房内观察期间持续工作 | 当前只有 topology_lock 不为空时开启相机覆盖和球形点云处理；走廊运输阶段不作为探索目标 | 按当前策略实现 | 进房失败时无法评价相机 detector |
| 前方房间完成后再解锁后方房间 | 当前需要前方 L/R 两房都回到走廊并 COMPLETE，然后 REAR_TRANSIT 前进约 12 m | 按你的拓扑要求实现 | 前方一个房间退出失败会连锁阻塞后方 |
| 使用公开接口和传感器，不读真值 | 运行时没有把 model_states、danger_truth、layout_metadata 接入算法；测试后才读取真值人工核对 | 符合要求 | 后续修复不得将真值坐标接回规划 |
| Stage B 稳定后再双层 | 当前仍只做单层；尚未接自动电梯/楼梯/下一层 | 符合要求 | 不应现在开始双层集成 |

launch 文件中还保留一条过期注释“Keep the harmonic gate”。实际代码已改成线性加权，不是调和平均。后续以代码和本交接文件为准。

## 4. 当前实际行为链

入口节点：

    src/simnav/scripts/coverage_explorer_node.py

纯逻辑：

    src/simnav/scripts/coverage_explorer_core.py

实际顺序：

    启动
      -> stair RL policy
      -> 固定前进约 14.5 m
      -> 零速度等待约 0.8 s
      -> /unitree/select_plane_policy(True)
      -> plane RL policy
      -> FRONT_DOOR_RESCAN / SEARCH / CONFIRM
      -> 发现并跨帧确认单侧门洞
      -> 锁定 ROOM_L_xx 或 ROOM_R_xx
      -> 生成 CAMERA_FRONTIER
      -> 房内综合覆盖达到 0.85
      -> 生成 RETURN_TO_CORRIDOR
      -> 几何确认回到走廊后标记 COMPLETE
      -> 前方左右房间都完成后 REAR_TRANSIT
      -> 搜索后方房间
      -> 四个 ROOM 拓扑完成且没有未复核球形假设
      -> FLOOR_COMPLETE

当前关键参数：

| 参数 | 当前值 |
|---|---:|
| initial_forward_distance | 14.5 m |
| initial_centering_start_distance | 10.5 m |
| virtual_gate_forward_distance | 10.5 m |
| virtual_gate_half_width | 1.1 m |
| virtual_gate_depth | 0.30 m |
| task_entry_buffer | 0.45 m |
| task_back_extension | 0.60 m |
| task_forward_depth | 35.0 m |
| task_corridor_half_width | 1.1 m |
| navigation_clearance | 0.20 m |
| preferred_clearance | 0.32 m |
| door_search_step_distance | 1.0 m |
| door_search_front/rear_limit | 3.0 m |
| portal_confirm_cycles | 3 |
| room_combined_coverage_target | 0.85 |
| expected_rooms_per_floor | 4 |
| camera_weight | 0.90 |
| motion_speed | 0.45 m/s |
| door crossing speed | 代码中约 0.25 m/s |
| rear_transit_distance | 12.0 m |
| sphere_min_hits | 3 |

注意：door crossing 的 0.25 m/s 是节点中的行为逻辑，不是 launch 参数。

## 5. 本轮实际做过的修改

以下是本轮冻结前能够由源码和 frozen_changes.patch 核对的修改。工作区中的其它历史变化不要自动归入本轮。

### 5.1 综合覆盖率改成相机主导的线性 10/90

相关文件：

    src/simnav/scripts/coverage_explorer_core.py
    src/simnav/scripts/coverage_explorer_node.py
    src/simnav/launch/stage_b_behavior.launch

实际公式：

    combined = 0.10 * laser + 0.90 * camera

房间完成条件：

    topology state == EXPLORING
    and local.combined >= 0.85

这不是“激光达到 95% 且相机达到 85% 的 AND 条件”。例如激光为 100% 时，相机至少约 83.33% 才能令综合值达到 85%。

20260902 触发首房返回时的现场数据约为：

    laser    = 96.5%
    camera   = 83.8%
    combined = 85.0%

这符合你提出的“激光只占 0.1、相机为主”的要求，但偏离总方案早期“不以覆盖率作为完成条件”的旧表述。后续只能选择一个统一口径，不能一部分代码用旧 AND 条件、一部分用新线性条件。

### 5.2 楼层完成改成四个已完成 ROOM 拓扑

增加 topology_completion_ready()。

只有满足以下条件才允许 floor complete：

    至少 expected_rooms_per_floor 个唯一 ROOM_* 拓扑
    每个房间都实际回到走廊后进入 COMPLETE
    没有未复核的稳定球形假设

仅在房内达到覆盖率，不会直接加入 completed_topologies。

这修正了旧漏洞：

    4 个候选 + 4 次 VISITED != 4 个真实房间

但门洞假阳性仍可能让错误拓扑进入计数，真实门结构校验仍未完成。

### 5.3 缓存进门所使用的门洞

加入：

    self.topology_portals = {}

已确认门洞通过 setdefault() 保留。房内稀疏扫描或地图更新不能立即删除唯一安全出口。

退出时优先使用当前计划中的 portal，否则使用缓存的 portal。路径结构是：

    房内普通 A*
      -> 已跨过的门洞附近
      -> 走廊中心直线穿越

门洞穿越段允许较低的 portal clearance=0.12 m；普通房内路径仍用 navigation_clearance。原因是门中心在稀疏 navigation map 中可能低于全局 0.20 m clearance，导致完整退出路径被判不可行。

### 5.4 门口简单朝向修正

根据：

- 门洞 side；
- 走廊 yaw；
- 门中心相对位置；

计算一个简单 heading correction。进门时低速直行。它不是视觉伺服，也不是独立语义门识别器。

### 5.5 保留已进入房间的拓扑状态

加入 topology_state_for_new_target()。

当房间已经越过门洞进入 EXPLORING 后，同房间新的 CAMERA_FRONTIER 不应把状态重置为 APPROACHING。否则覆盖完成和退出分支永远不能稳定触发。

### 5.6 stair/plane 两个 RL policy 预加载和切换

相关文件：

    src/unitree_guide/unitree_guide/unitree_guide/include/FSM/State_RL_test.h
    src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp

实际变化：

- UNITREE_POLICY_PATH 指定 stair policy，默认仍为 stair policy；
- UNITREE_PLANE_POLICY_PATH 指定 plane policy；
- 控制器启动时预加载两个 TorchScript 模型；
- 新增 /unitree/select_plane_policy，类型为 std_srvs/SetBool；
- 仅当前速度命令总幅值不超过 0.03 时接受切换；
- 推理线程使用 atomic flag 选择当前模型；
- 仿真 RL 观测中的四元数/陀螺仪改从 _lowState->imu 读取，不使用控制器中可能带真值性质的基座字段。

编译产物：

    .simenv_build/devel/lib/unitree_guide/junior_ctrl

该机制证明了“可切换 policy”，不等于每个 seed 的腿式运动都稳定。出生飞起、姿态异常或固定前进无位移时，应保存证据、清理进程、同 seed 重跑，不要先改探索算法。

### 5.7 相机覆盖点累计和开启条件

节点订阅：

    /simnav/camera_coverage

相机看到的点被累计为有界 floor-wide union，避免从房间切回走廊后旧房间覆盖被清零。

相机探索真正开启条件：

    topology_lock is not None

所以走廊运输阶段不选择相机探索目标；锁定一个房间后才开始房内相机主导探索。退出过程中 topology_lock 继续保留，直到确认回到走廊。

### 5.8 RViz 标记和覆盖颜色

当前 launch 设置：

    show_room_virtual_doors=true

门洞 marker 是诊断标记，不写入 navigation_map，不应直接阻挡 A*。颜色含义：

| 颜色 | namespace/状态 | 含义 |
|---|---|---|
| 蓝色 | laser_only | 激光已覆盖，相机未覆盖 |
| 绿色 | camera_only | 相机已覆盖，激光未覆盖 |
| 黄色 | laser_and_camera | 两者都已覆盖 |
| 橙色 | READY | 已发现但尚未进入/配对的门洞 |
| 淡黄色 | PAIRED | 与另一侧按纵向位置配对 |
| 紫色/品红 | APPROACHING | 正在接近或准备进门 |
| 绿色短线 | EXPLORING | 当前房间探索中 |
| 蓝色短线 | RETURNING | 房间覆盖完成，正在返回走廊 |
| 青色短线 | COMPLETE | 房间完成且回到走廊 |
| 红色短线 | BLOCKED | 规划或进门失败后的阻塞状态 |
| 品红箭头 | coverage_target | 当前目标，不是门也不是障碍墙 |

门 marker 的长度使用估计门宽，方向按走廊轴显示。它只是 Marker；但任务 mask 仍会影响前沿候选，所以“不是物理障碍”不代表线附近一定可被当前 planner 选择。

### 5.9 已确认的“自己加机制后产生的回归”

这些不是 Gazebo 随机性，也不应继续归咎于环境：

1. **虚拟入口 gate 的语义边界曾表现成“堵路”。** gate 没有写入 navigation_map，物理上不会挡住机器人；但它参与 task mask 和覆盖候选过滤。gate 位置、`task_entry_buffer` 或走廊法向稍有偏差，就会把线以下的房间区域从 eligible/frontier 集合中排除。RViz 看起来像一条线挡住了探索，实际是“覆盖/目标过滤挡住了候选”。当前仍保留该 gate，因此 `NO_FRONTIER` 首先要检查 gate 后的单元数，而不是先改膨胀半径。
2. **为了观察门而打开的 room virtual door marker 增加了诊断噪声。** 橙色、淡黄色、紫色、绿色、蓝色短线都是 Marker，不是占用栅格；但 marker 状态颜色和覆盖层颜色相近，实际调试时很容易把“诊断线”误认成物理墙或覆盖边界。后续应默认关闭，或把门 marker 与覆盖 marker 使用更容易区分的 namespace/颜色。
3. **允许单侧确认门独立拥有房间后，配对约束变弱。** 这样可以避免“对面门没先确认就永远进不了第一房”，但也使单个误门有机会先锁定房间。当前靠前后门站距离和最终四房数约束补偿，仍不能替代真实门框结构验证。
4. **为解决地图稀疏门中心不可达而把返回穿门 clearance 降到 0.12 m，增加了安全边界不一致。** 普通房内 A* 使用 0.20 m，已跨过门的直线回程使用 0.12 m。它解决过“出口明明存在却被 navigation clearance 拒绝”，但也可能让返回路径接受过窄单元；后续必须用机器人实际 footprint 和门宽验证，不能继续无条件降低。
5. **目标替换保护没有覆盖完整的返回时序。** 我已经对现存 `RETURN_TO_CORRIDOR` 做了不替换保护，也让同房间新前沿保留 `EXPLORING`；但返回目标被清空后的下一次 planner 分配仍能产生 `CAMERA_FRONTIER`。这正是 20260902 观察到的 `RETURNING + CAMERA_FRONTIER`，属于状态机修改没有封住所有入口的自引入回归。
6. **前方门站先完成、后方门站再解锁是人为拓扑限制。** 这是为了避免前两个房间反复切换和走廊/房间串线而加的阶段边界，不是场景本身要求。它的副作用是：只要前方任一房间退出失败，后方两个房间必然不可达。不能把这一连锁结果误判为后方门检测或后方 A* 已经失败。
7. **覆盖率口径在修改后出现旧参数和新参数并存。** 实际房间完成使用 0.85 的线性 10/90；launch 里仍有 `combined_coverage_target=0.84` 等旧全局参数和 harmonic 注释。它们目前不直接决定房间完成，但会误导日志阅读和后续修改，属于文档/参数清理欠账。
8. **stair/plane policy 自动切换增加了新的启动耦合。** 控制器需要同时加载两个 TorchScript 模型，并依赖“零速度时才能切换”的服务时序。20260901 首次出现出生姿态异常和固定前进为 0，按规则同 seed 重跑后正常；这类故障不应通过继续修改探索器掩盖，必须单独记录为 locomotion/启动问题。

上述回归点中，当前最直接影响功能的是第 5 项，其次是第 1 项和第 3 项。它们都属于我在探索层增加约束或状态保护后留下的耦合，不应被包装成“传感器天然不稳定”。

## 6. 三个 seed 的测试证据

测试目录：

    logs/multiseed_combined_90_20260831/

启动方式：

    docker exec -e DISPLAY=:0 -w /workspace/SimEnv simenv-noetic bash -lc \
      'STAGE_B_GUI=false STAGE_B_START_RVIZ=1 \
       ./team_scripts/run_stage_b_seed.sh SEED 600 \
       logs/multiseed_combined_90_20260831 coverage'

含义：

- 使用真实 Gazebo 后端；
- Gazebo GUI 关闭；
- RViz 打开；
- 600 是仿真秒上限，不是主机墙钟秒数；
- runner 在结束前保存探索地图和 RViz 截图。

### 6.1 seed 20260901

第一次启动属于你要求的“直接重跑”类型：

    位姿 z 约 0.18 m
    yaw 约 -1.85 rad
    位置约 (0.14, -1.84)
    固定前进进度为 0
    控制器主要原地转向

这被归为物理启动异常，不作为探索策略失败。已保存并清理，然后使用相同 seed 重跑。

重跑启动正常：

    z 约 0.31 m
    固定前进进度约 14.59 m
    plane_policy_active=true

重跑后稳定卡住：

    ROOM_R_15 confirmed=true
    portal evidence=35
    actionable_portals=[ROOM_R_15]
    active_target_kind=null
    last_plan_reason=NO_FRONTIER
    targets_reached=0
    navigation_blocks=0
    plan_failures=0
    navigation_reachable_cells 约 17565

结果快照：

    仿真时间约 69.194 s
    相机覆盖 0%
    激光覆盖约 7.69%
    综合覆盖约 0.77%
    危险物真值 1
    检出 0
    漏检 1

判断：

- 门不是没确认；
- A* 不是全图不可达；
- planner timer 没有异常退出；
- 机器人没有拿到门内目标；
- 属于“确认门洞后房间前沿生成为空”。

保存位置：

    seed_20260901/startup_anomaly_attempt_1/
    seed_20260901/early_plan_failure_attempt_2/
    seed_20260901/stable_failure_rerun/
    seed_20260901/rviz_final.png
    seed_20260901/result.json

### 6.2 seed 20260902

启动、固定前进和 stair -> plane 切换正常。成功锁定并进入 ROOM_R_15。

房间内曾达到：

    激光约 96.5%
    相机约 83.8%
    综合约 85.0%

随后正确触发：

    topology_state=RETURNING
    returning_topology=ROOM_R_15
    active_target_kind=RETURN_TO_CORRIDOR

但之后出现关键错误：

    topology_state 仍是 RETURNING
    topology_lock 仍是 ROOM_R_15
    active_target_kind 却变成 CAMERA_FRONTIER

机器人没有沿记忆门洞退出，反而继续进入房间，实测位置一度约：

    x=7.24, y=11.10

最后：

    last_plan_reason=NO_FRONTIER
    completed_topologies=[]
    floor_complete=false

最终结果快照：

    仿真时间约 170.31 s
    全局相机覆盖约 11.84%
    全局激光覆盖约 25.36%
    全局综合覆盖约 13.19%
    危险物真值 3
    检出 0
    漏检 3

注意：运行中 room_coverages 的局部峰值约 94.7%，结果文件最后的全局覆盖率不能代表房间峰值。评价房间退出应看 room_coverages 和 topology_states，而不是只看结果 JSON 最后一行全局覆盖率。

保存位置：

    seed_20260902/stable_failure/
    seed_20260902/rviz_final.png
    seed_20260902/result.json

### 6.3 seed 20260903

启动正常：

    固定前进进度约 14.55 m
    plane_policy_active=true

右侧首门确认约 20 次：

    ROOM_R_15 confirmed=true
    portal evidence=20
    actionable_portals=[ROOM_R_15]
    active_target_kind=null
    last_plan_reason=NO_FRONTIER
    targets_reached=0

结果快照：

    仿真时间约 54.694 s
    相机覆盖 0%
    激光覆盖约 7.55%
    综合覆盖约 0.75%
    危险物真值 0
    检出 0

危险物真值为 0，所以该 seed 不能评价 detector；但它再次复现确认门后没有门内前沿的问题。

保存位置：

    seed_20260903/stable_failure/
    seed_20260903/rviz_final.png
    seed_20260903/result.json

## 7. 问题定位

### 7.1 确认门洞后 NO_FRONTIER

该问题在 20260901 重跑和 20260903 重复出现，不能归为偶然。

已知事实：

- observed_portals 有 confirmed=true；
- actionable_portals 包含 ROOM_R_15；
- evidence 远大于确认门槛 3；
- navigation_reachable_cells 约 1.7 万；
- plan_failures=0；
- navigation_blocks=0；
- active_target_kind=null。

首要排查链：

    detect_room_portals
      -> confirmed_topologies
      -> assignment_portals
      -> topology_id_for_point
      -> task_mask / camera_unseen
      -> 房间前沿候选过滤

重点怀疑：

1. 门洞 ID 与门内前沿点的 topology 归属不一致；
2. gate、task_entry_buffer、corridor_half_width 或 room ROI 把门内单元排除了；
3. reachable 使用 navigation map，而候选又经过 task mask 和 topology_id_for_point 二次过滤，空间定义不一致；
4. camera_seen 坐标转换错误，门内未知点被误标为已观察；
5. 门口地图只有少量自由格，未知区域没有形成满足最小距离、净空和信息增益条件的 viewpoint。

下一步先增加只读诊断字段，不放宽策略：

    assignment_portal_count
    room_task_cells
    room_eligible_cells
    room_camera_unseen_cells
    room_reachable_cells
    room_reachable_camera_unseen_cells
    candidate_reject_counts
    last_reject_reason

在定位前不要先改膨胀半径、门宽、速度或 door crossing；当前机器人还没有拿到门内目标。

### 7.2 RETURNING 被 CAMERA_FRONTIER 覆盖

20260902 已直接观测：

    topology_state=RETURNING
    active_target_kind=CAMERA_FRONTIER

设计上应该是：

    local.combined >= 0.85
      -> state=RETURNING
      -> active_target=RETURN_TO_CORRIDOR
      -> 直到走廊几何确认才清 lock

实际说明：返回路径的目标清空、下一轮计划和普通目标替换之间没有形成硬隔离。_should_replace_active() 对已有 RETURN_TO_CORRIDOR 有保护，但不能覆盖所有时序；返回子段完成、active_target 清空或 planner 刷新后，同一个锁定房间仍能产生 CAMERA_FRONTIER。

正确约束应为：

    topology_state == RETURNING
      -> 只允许 RETURN_TO_CORRIDOR
      -> 禁止 CAMERA_FRONTIER
      -> 禁止 SPHERE_REVIEW
      -> 禁止新门洞调度
      -> 只有确认回到 corridor lateral 带才清 topology_lock

这是最明确的代码级根因。下一步只修这个隔离，用单房进入—完成—退出回归验证；不要同时改门检测和覆盖公式。

### 7.3 后方房间未验证

后方房间解锁需要：

    front_station_topologies 有 L/R
    completed_front_sides == {L,R}
    机器人在走廊中心

前方任一房间无法退出时，rear_rooms_unlocked=false，后方房间不会进入调度。因此现在“后方去不了”是前方退出失败的连锁结果，不是已经独立证实的后方路径故障。

在退出隔离修好前，不要先改 rear_transit_distance 或 far_room_first。

### 7.4 危险物漏检不能直接归因到视觉阈值

20260901、20260903 没进房，不能评价 RGB-D detector。

20260902 进入了房间但 0/3，应该按以下顺序拆分：

    RealSense RGB 是否有消息
    depth 有效比例是否足够
    danger_detector 是否收到同步帧
    red mask 是否产生
    圆形轮廓是否通过
    深度点是否有效
    TF 到 world 是否成功
    track 是否达到 confirmation_frames
    runner 清理前结果文件是否写完

测试后可以用 danger_truth 做人工评估，但不能让真值进入节点。不要直接扩大颜色阈值或降低球形筛选门槛。

### 7.5 覆盖率和 RViz 颜色容易误解

当前有三层不同数字：

1. 全局任务覆盖率：整个 task envelope；
2. 房间局部覆盖率：room_coverages[ROOM_*]；
3. RViz 颜色分类：蓝、绿、黄只是传感器覆盖组合，不是百分比刻度。

因此：

- 蓝色多不代表相机覆盖高；
- 黄色表示激光和相机都覆盖；
- 全局综合 14% 不代表当前房间只有 14%；
- 房间完成看当前锁定房间局部 combined 和 EXPLORING 状态；
- 楼层完成看四个 COMPLETE 拓扑房间，不看全局 combined。

## 8. 冻结证据和工作区注意事项

多 seed 启动前保存：

    logs/multiseed_combined_90_20260831/frozen_code.sha256
    logs/multiseed_combined_90_20260831/frozen_changes.patch

本轮相关源码哈希：

    team_scripts/run_stage_b_seed.sh
    15d21aab462764193c0d39808d5e2daa87c0c52c90138edd3f4b7fee0e0208a9

    src/simnav/scripts/coverage_explorer_core.py
    fc77ee103c432d3bc93984e79c209e1912951ef43e92d7f0af949c2b410b8311

    src/simnav/scripts/coverage_explorer_node.py
    b73d0fa93ab559061e618eaed20c4f334303b48c1ec1f84649d792655bd12b84

    src/simnav/launch/stage_b_behavior.launch
    a72cf272be6d283ff5648a1217e71fabb89b8c6507aa9d232ee104a7270e8716

    State_RL_test.h
    294e57ef73f21f4dee26b72c2708fd8c3961878cb5b572b8bb9230eb81e69633

    State_RL_test.cpp
    b008c63aa209c84508a09a073f5078530ef1a3c76c0900d44ba64a3ef8375fad

实际校验建议直接执行：

    cat logs/multiseed_combined_90_20260831/frozen_code.sha256

本轮核心测试、Python 编译、bash 语法检查和空白检查均通过；冻结前记录为 57 项测试通过。

工作区整体很脏，包含历史源码、生成场景、build/devel 产物、旧日志和多个未跟踪目录。禁止：

    git reset --hard
    git clean -fd
    全局批量回退
    用整个 git diff 推断本轮修改

后续只能针对明确文件和明确修改做审计。

## 9. 运行和清理状态

本轮结束时已清理：

    run_stage_b_seed.sh
    roslaunch simnav stage_b_behavior.launch
    junior_ctrl
    gzserver
    roscore
    rviz

启动前仍建议执行：

    pgrep -af 'run_stage_b_seed|stage_b_behavior|junior_ctrl|gzserver|roscore|rviz'

不能只杀外层 docker exec。容器内 ROS launch、Gazebo 和控制器可能脱离外层进程组继续运行，历史上这会造成：

- 话题串线；
- 节点同名替换；
- 旧监控器监听新运行；
- CPU/GPU 竞争；
- 日志和结果目录错配。

## 10. 推荐后续顺序

### P0-1：返回状态硬隔离

只做一件事：

    房间 combined >= 0.85
      -> 永远只允许 RETURN_TO_CORRIDOR
      -> 回到 corridor lateral 带后才 COMPLETE

单房回归必须看到：

    RETURN_TO_CORRIDOR
      -> corridor
      -> COMPLETE

不得出现：

    RETURNING + CAMERA_FRONTIER
    RETURNING + SPHERE_REVIEW

### P0-2：给 NO_FRONTIER 增加过滤计数

只加日志或 status 字段，不改变决策：

    房间 ROI eligible 单元数
    camera_unseen 单元数
    reachable 单元数
    reachable & camera_unseen 单元数
    assignment_portals 数量
    各过滤阶段拒绝数量
    最终 NO_FRONTIER 原因

### P0-3：单房闭环回归

在已知能进房的 seed 上验证：

    单门确认
      -> 进门
      -> 相机主导覆盖达到 0.85
      -> 使用记忆门洞返回
      -> corridor 几何确认
      -> COMPLETE

连续多个 seed 首房闭环稳定后，才测试前方左右门站。

### P1：验证危险物检测

在稳定进出单房的前提下，逐层检查 RGB、Depth、同步、红色 mask、球形轮廓、深度、TF、跟踪和结果写出。真值只用于测试后评估。

### P1：验证后方房间

前方 L/R 都 COMPLETE 后，再验证 REAR_TRANSIT 和后方两个房间。不要在前房退出未修好时修改远门策略。

### P2：至少五个 seed 正式验收

每个 seed 应同时满足：

    真实房间 4/4
    floor_complete=true
    UNREACHABLE=0
    无额外假门
    危险物 missed=0
    红方块/绿色球误报=0
    无 >1 m 非物理 pose jump
    无人工干预
    单层时间在规定范围内

单个 seed 成功不等于稳定通过，单个 seed 失败也不应直接增加特殊分支。

## 11. 接手时明确不要做的事

- 不要把 NO_FRONTIER 直接解释成膨胀半径问题；当前证据是门内目标根本没生成。
- 不要允许 RETURNING 状态被 CAMERA_FRONTIER 或 SPHERE_REVIEW 抢占。
- 不要为了补足四个数量读取 layout_metadata 或 danger_truth 生成目标。
- 不要让走廊端点、墙缝候选替代真实房门。
- 不要把虚拟 gate marker 写入 navigation occupancy grid。
- 不要在同一轮同时改定位、门检测、覆盖公式和 locomotion policy。
- 不要因一次出生飞起就修改探索算法；先保存证据并同 seed 重跑。
- 不要中断外层 Docker 命令后假定容器内进程都结束。
- 不要用结果 JSON 的最后全局覆盖率代替 room_coverages 和状态转移检查。
- 不要把当前版本称为双层探索完成。

## 12. 关键文件索引

主探索链：

    src/simnav/scripts/coverage_explorer_core.py
    src/simnav/scripts/coverage_explorer_node.py
    src/simnav/launch/stage_b_behavior.launch
    src/simnav/scripts/danger_detector_core.py
    src/simnav/scripts/danger_detector_node.py
    src/simnav/launch/stage_b_localization.launch

RL policy 切换：

    src/unitree_guide/unitree_guide/unitree_guide/include/FSM/State_RL_test.h
    src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp

运行、监控、结果保存：

    team_scripts/run_stage_b_seed.sh
    team_scripts/monitor_stage_b_coverage.py
    team_scripts/evaluate_stage_b_danger.py

规范和历史：

    SimEnv_two_floor_exploration_codex_spec.md
    SimEnv_Stage_B_Adjustment_Plan_Anti_Overfitting.md
    docs/stage-b-progress.md
    docs/stage-b-closeout-issues.md
    docs/SimEnv工程当前进度与功能机制说明.md

本轮测试证据：

    logs/multiseed_combined_90_20260831/frozen_code.sha256
    logs/multiseed_combined_90_20260831/frozen_changes.patch
    logs/multiseed_combined_90_20260831/seed_20260901/
    logs/multiseed_combined_90_20260831/seed_20260902/
    logs/multiseed_combined_90_20260831/seed_20260903/

## 13. 最终交接判断

本轮最有价值的改动：

- 真实 Gazebo + RViz 可重复运行入口；
- stair -> plane policy 实际切换；
- 相机主导的 10/90 综合覆盖；
- 已确认门洞坐标缓存；
- 房间完成后明确尝试返回走廊；
- 每轮地图、截图、日志、结果和冻结哈希可审计。

当前最危险的部分：

- 门确认后房间前沿生成为空；
- RETURNING 没有完全隔离普通前沿；
- 结果同时存在全局和局部覆盖口径；
- 危险物 detector 尚未在稳定房间闭环中验收；
- Hector 平移仍是长走廊和房内定位风险；
- 工作区有大量历史/生成/构建脏文件。

下一位接手者的第一目标必须是：

    确认门洞 -> 能生成门内目标
    房间完成 -> 只能返回走廊直到 COMPLETE

这两点通过前，不应继续增加新的虚拟门、全局 Frontier、后方优先级、房间外围扫描或双层逻辑。
