# SimEnv 系统架构与模块化设计

> **版本**：2026-09-14
> **范围**：本仓库（`SimEnv`）当前可运行的全部功能块——比赛仿真环境 + Stage B 三层自主探索算法 + 监督/评估工具链。
> **写作目的**：① 说明"系统由哪些块组成、每块怎么设计"；② 说明"模块化的判据、契约、边界与现状缺口"。第 2 章与第 6 章是模块化设计的核心。
> **配套文档**：场景与规则见 `比赛场景背景说明_论文用.md`；目标架构与精简台账见 `SimEnv_模块化功能块设计_20260913.md`；反复出现的困难见 `stage_b_recurring_difficulties.md`。
> **约定**：文中所有参数默认值取自 `src/simnav/launch/*.launch` 与 `generated_building/scene_manifest.json`；节点名、话题名取自代码与运行时 `rosnode/rostopic` 实测。

---

## 目录

1. [系统总览](#1-系统总览)
2. [模块化设计总则](#2-模块化设计总则)
3. [功能块详述](#3-功能块详述)
4. [模块间接口契约总表](#4-模块间接口契约总表)
5. [关键数据流](#5-关键数据流)
6. [模块化现状评估与目标架构](#6-模块化现状评估与目标架构)
7. [附录](#7-附录)

---

## 1. 系统总览

### 1.1 系统是什么

本仓库同时承载**两个交付物**，二者必须严格分离：

| 交付物 | 内容 | 谁可以使用 |
|---|---|---|
| **A. 比赛仿真环境（Environment）** | 随机多楼层场景生成、Gazebo 物理世界、A1 机器人与其控制链、传感器仿真、门/电梯服务、真值与评估 | 参赛算法**只能**通过公开话题/服务/`team_scene_info.json` 使用 |
| **B. 自主探索算法（Stage B Solution）** | `simnav` 包：定位与建图、占用地图、覆盖探索决策、危险源检测、电梯换层、任务编排与监督 | 本项目自己的实现，用于验证环境与产出成绩 |

环境层与算法层的**边界**就是第 4 章的接口总表；任何"算法直接读环境内部数据"的行为都是违规（例如读 `layout_metadata.json`、`/ground_truth/*`）。

### 1.2 分层架构

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│ L6 编排与监督   run_stage_b_seed.sh · two_floor_explorer_supervisor · watch_* │
├──────────────────────────────────────────────────────────────────────────────┤
│ L5 评估与记录   evaluate_stage_b_danger · monitor_stage_b_coverage ·          │
│                 record_stage_b_telemetry · verify_three_floor_run            │
├──────────────────────────────────────────────────────────────────────────────┤
│ L4 任务决策     coverage_explorer(core+node)      │  elevator_transition     │
│                 「覆盖探索：目标/路径/走廊-房间」   │  「垂直交通：对准/进出/收尾」│
├──────────────────────────────────────────────────────────────────────────────┤
│ L3 任务感知     danger_detector(core+node)        │  map_floors / pose_continuity │
│                 「RGB-D 红球 → 世界坐标」          │  「分层与位姿连续性」        │
├──────────────────────────────────────────────────────────────────────────────┤
│ L2 定位与建图   FAST-LIO · lio_localization_bridge · lio_health_monitor ·     │
│                 pointcloud_to_laserscan · lio_occupancy · map_views          │
├──────────────────────────────────────────────────────────────────────────────┤
│ L1 机器人平台   unitree_guide(A1 + junior_ctrl + RL policy) · Mid360 插件 ·    │
│                 RealSense 插件 · IMU · 真值插件（裁判专用）                    │
├──────────────────────────────────────────────────────────────────────────────┤
│ L0 场景与物理   building_generator_core(约束/布局/生成/导出) ·                │
│                 building_generator_classic(门/电梯服务) · Gazebo Classic      │
└──────────────────────────────────────────────────────────────────────────────┘
        ▲ 控制指令向上：/cmd_vel        ▼ 观测向下：点云/图像/IMU/关节状态
```

**依赖方向（设计约束）**：L0→L1→L2→L3→L4→L6 单向依赖；**任何下层不得调用上层**。唯一的例外是 L4 的电梯块与探索块都向 `/cmd_vel` 写指令——二者通过状态互斥（谁在跑由 supervisor 决定）而不是通过共享内存协调。

### 1.3 代码结构与分层的映射

| 目录 | 层 | 角色 |
|---|---|---|
| `src/building_generator_core/` | L0 | 场景生成纯逻辑：`constraints.py`（约束）、`generator.py`（布局/家具/签名）、`layout.py`（几何数据类）、`exporter.py`（SDF 与配置导出 + 8 类一致性校验）、`cli.py` |
| `src/building_generator_classic/` | L0 | 门/电梯运行期服务：`control_server.py`、`control_runtime.py`、`classic_export.py` |
| `src/building_generator_interfaces/` | L0 | 服务定义 `SetDoorState.srv`、`CallElevator.srv` |
| `src/building_obstacles/` | L0/L5 | 场景片段生成与**评估脚本** `evaluate_danger.py` |
| `src/unitree_guide/` | L1 | A1 描述（`a1_description/xacro`）、控制器 `junior_ctrl`、`multi_floor_gazeboSim.launch`、`pointcloud2livox.py` |
| `src/Mid360_imu_sim/` | L1 | Livox MID-360 仿真插件（`livox_points_plugin.cpp`、`livox_ode_multiray_shape.cpp`）与扫描模式 CSV |
| `src/FAST_LIO/` | L2 | LiDAR-惯性里程计 `fastlio_mapping` |
| `src/simnav/` | L2–L4 | 本项目全部算法节点、launch、RViz 配置与离线测试 |
| `team_scripts/` | L6/L5 | 种子运行器、监督进程、看门狗、遥测/监控/校验/复现工具 |
| `generated_building/`, `results/`, `logs/` | 数据 | 场景工件、结果、运行日志（每轮一个目录） |
| `config/` | 治理 | 冻结清单（`stage_b_entry_frozen.sha256`、`stage_b_floor0_frozen.sha256`） |

### 1.4 运行拓扑

一次三层任务运行时（`three_floor` 模式）实际存在的进程/节点：

| 进程 | 节点名 | 启动方 | 作用 |
|---|---|---|---|
| `roscore` | — | runner（独立端口，默认 11320） | ROS master |
| `gzserver` | `/gazebo` | `auto.sh` → `multi_floor_gazeboSim.launch` | 物理世界 + 全部传感器插件（Gazebo master 默认 11345） |
| `junior_ctrl`（可执行文件） | `/unitree_gazebo_servo` | runner | RL 步态策略执行 `/cmd_vel`；发布 `/simnav/controller_health` |
| `building_generator_classic_control` | `/building_generator_classic_control` | `auto.sh` | `/set_door_state`、`/call_elevator` |
| `fastlio_mapping` | `/fast_lio` | `stage_b_localization.launch` | LIO 里程计与注册点云 |
| `pointcloud_to_laserscan` | `/simnav_cloud_to_scan` | 同上 | 3D 点云 → `/scan_2d` |
| `lio_localization_bridge_node.py` | `/localization_bridge` | 同上 | `/simnav/odom`、`/simnav/world_pose`、`/simnav/world_pose_metric`、回环 |
| `lio_health_monitor_node.py` | `/lio_health_monitor` | 同上 | `/simnav/lio_health` |
| `lio_occupancy_node.py` | `/lio_occupancy` | 同上 | `/map`（按层占用栅格）、`/simnav/map_floors` |
| `map_views_node.py` | `/map_views` | 同上 | `/exploration_map`、`/navigation_map` |
| `danger_detector_node.py` | `/danger_detector` | `stage_b_two_floor_support.launch` | 红球检测与结果文件 |
| `elevator_transition_node.py` | `/elevator_transition` | 同上 | 垂直交通状态机、下降与收尾 |
| `two_floor_explorer_supervisor.py` | `/two_floor_explorer_supervisor` | runner | 每层起停探索器、转发事件 |
| `coverage_explorer_node.py` | `/coverage_explorer` | supervisor（每层一个实例） | 覆盖探索决策 + `/cmd_vel` |
| `rviz` | `/rviz_*` | runner（可选） | 可视化（不影响算法） |
| 看门狗 `watch_*.py`（多个） | — | runner | 停滞/超时/冻结判定（**统一仿真时钟**） |
| `monitor_stage_b_coverage.py`、`record_stage_b_telemetry.py` | — | runner | 结果判定 `result.json`、`telemetry.csv` |

**端口与隔离**：ROS master 与 Gazebo master 使用独立端口；runner 启动前做进程/端口占用检查与单实例锁（`.stage_b_runner.lock`），退出时按本轮 PID 递归清理，不做全局清理。这一条对"可复现"很关键：两次运行不会互相污染，也不会误杀同机其他任务。

---

## 2. 模块化设计总则

> 本章是本文档的重点。它回答"为什么这样分块、块之间靠什么契约、怎么验证一个块是否做对了"。

### 2.1 为什么必须模块化：来自运行成本的硬约束

三层任务的实测实时因子 **RTF ≈ 0.07–0.15**（瓶颈是 10 Hz × 2400 条射线的雷达光线投射；关闭雷达后 RTF 回到上限 1.00）。这意味着：

- 一次 2400 仿真秒的端到端实验 = **4–9 小时墙钟**；
- "改一行代码 → 跑一轮 → 看结果"的反馈环极长，任何"每轮只暴露一个缺陷"的迭代方式都会把代价放大 5–10 倍；
- 因此**唯一可行的工程方式是分层验证**：把能在秒级离线验证的部分（纯逻辑）与只能在分钟/小时级验证的部分（ROS 集成、端到端）严格分开。

模块化不是为了"好看"，而是因为在 RTF≈0.1 的现实下，**跨块耦合的每一次调试都要按小时计费**。

### 2.2 划分判据：容差与失败代价

同一台机器人、同一套传感器，本系统里存在两类**性质完全不同**的精细作业：

| | 房间（区域覆盖） | 电梯 / 大堂（精确落位） |
|---|---|---|
| 任务 | 在区域内做覆盖，站在哪里都算干活 | 精确接近并落到轿厢口 |
| 容差 | **米级** | **厘米级** |
| 几何可分辨性 | 房门宽约 0.8–1.1 m，与 0.25 m 栅格同量级 | 大堂/电梯口宽 3.5–3.6 m，无歧义 |
| 结论 | **不做门身份识别**：进出由"区域成员判定"决定 | **保留识别 + 对准**：唯一合理的识别场景 |

这条判据直接决定了两个大块的**内部风格差异**：探索块应尽量"无状态识别、幂等判定"，电梯块必须"有对准门控、有速度特例、有重试预算"。把两者的机制互相搬用（例如给房间加对准特例、给电梯加覆盖逻辑）就是模块边界被破坏。

### 2.3 分层模式：`core`（纯逻辑）+ `node`（ROS 适配）

`simnav` 的每个算法块都按同一模式组织：

| 层 | 文件示例 | 允许做什么 | 禁止做什么 |
|---|---|---|---|
| **core** | `coverage_explorer_core.py`(3685 行)、`elevator_transition_core.py`(323)、`danger_detector_core.py`(400)、`map_floors_core.py`(115)、`pose_continuity.py`(134) | 纯函数、数据类、几何/判据/状态迁移；不导入 `rospy` | 不订阅/发布、不读参数服务器、不写文件、不调用 `time` 以外的系统时钟 |
| **node** | `coverage_explorer_node.py`(3413)、`elevator_transition_node.py`(1878)、`danger_detector_node.py`(879)、`lio_occupancy_node.py`(265)、`map_views_node.py`(219)、`lio_localization_bridge_node.py`(427) | ROS 订阅/发布、参数、定时器、TF、日志、诊断字段组装 | 不实现领域判据（应调用 core）；不在回调里写复杂几何 |

**收益（可量化）**：`src/simnav/test/` 下 21 个 `test_*_core.py` 全部是**离线单元测试**，无需 Gazebo/ROS 即可在秒级运行（`team_scripts/run_stage_b_offline_tests.sh`）。它们直接覆盖领域规则：门洞几何、走廊分区、覆盖计算、速度钳制、电梯进入重试、占用图分层、脚本卫生等。

**边界纪律（来自实际教训）**：
- 纯逻辑必须在 core。曾把辅助函数误插入类作用域，导致类被提前截断、后续方法变成嵌套函数——由 `test_script_hygiene_core.py`（PlannerApiTest）在离线测试中拦截。
- node 层只做适配。node 出现过 `UnboundLocalError: heading_tolerance` 导致 `rospy.Timer` 回调线程永久死亡（run44），因此现在 `_control` / `_plan` 都有异常护栏（`CONTROL_FAULT` / `PLAN_FAULT` 计数）。
- 静态检查补位：`team_scripts/check_undefined_attrs.py`（属性拼写/未定义属性，run60 的 `stop_distance` 误用即此类）、`check_read_before_assign.py`。

### 2.4 契约设计：三类接口 + 四条命名纪律

**接口只有三类**，任何跨块通信必须落在其中：

| 类别 | 载体 | 特点 | 例子 |
|---|---|---|---|
| ① 话题 | ROS topic | 单向、可丢、带时间戳 | `/simnav/odom`、`/map`、`/cmd_vel`、`/simnav/*_status` |
| ② 服务 | ROS service | 请求-应答、同步 | `/set_door_state`、`/call_elevator` |
| ③ 文件 | JSON/YAML | 跨进程/跨轮次、可审计 | `team_scene_info.json`、`result.json`、`telemetry.csv`、`elevator_phase.csv` |

**四条纪律**：
1. **单位写进名字**：`*_meters`、`*_seconds`、`*_deg`；速度用 `motion_speed`、`turn_speed`、`transit_speed_cap`。
2. **坐标系写进语义**：`source`（源地图帧）、`world`（全局帧）、`metric`（LIO 度量帧）、`base`。`/simnav/odom` 给 `base` 帧位姿，`/simnav/world_pose_metric` 给 world 帧度量位姿。
3. **时钟唯一**：任务与看门狗**一律用仿真时间**（`/clock`）。这是硬纪律：run44/57 的看门狗误杀就是因为用了真实秒（420 真实秒 ≈ 38 仿真秒，落在正常等待窗口内）。
4. **状态要"可区分沉默与停滞"**：`/simnav/*_status` 为 latch 的 JSON 快照（每周期或每次变化发布），并带周期计数器/时间戳；看门狗判"停滞"必须先确认"状态仍在更新"。

### 2.5 单一事实来源（SSOT）

跨块共享的四个概念，必须各有一个 owner；其它块**只消费、不重算**：

| 概念 | 唯一 owner（目标） | 现状 | 风险 |
|---|---|---|---|
| **地图** | `lio_occupancy` 产 `/map`（分层栅格）；`map_views` 产 `/exploration_map`、`/navigation_map` 两个**派生视图** | 已分离，但"去斑图 vs 原始图"两个视图都参与判据 | 路由用去斑图、校验用原始图 → 合法路径被判不安全（run47） |
| **位姿** | `lio_localization_bridge` 产 `/simnav/odom`(base)、`/simnav/world_pose`、`/simnav/world_pose_metric` | 已有，但姿态判据两处口径（IMU vs metric） | 倾倒误判（run63：metric 单点跳 0.45 rad vs IMU 0.12 rad） |
| **时钟** | `/clock`（Gazebo 仿真时间） | 已统一到看门狗与任务 | 历史误杀（run44/57） |
| **楼层上下文** | `elevator_transition` 产 `/simnav/floor_exploration_context`（楼层号、几何提示、复用门户） | 已建立；上层几何以"提示"方式注入探索器 | 换层后沿用上一层地图规划（run52） |
| **门/电梯几何** | 目标态由电梯块唯一计算并发布 | 现状**双方各算一份**（探索器也有 `elevator_portal` / `virtual_isolation_door*`） | run62：两边不一致导致整层快照被跳过 |

### 2.6 活性（liveness）契约

模块不仅要"算得对"，还要"活着并且有进展"，二者分离：

| 关注点 | 表达方式 | 处理者 |
|---|---|---|
| 组件是否活着 | 心跳：状态话题周期发布 + 迭代计数（`plan_cycles`、`control_faults`） | 监督层/看门狗 |
| 是否在进展 | 进度量：`initial_forward_progress`、`progress_idle_seconds`、`targets_reached` | 块内不变量 + 兜底 |
| 是否失败 | 显式故障：`mission_fault`（字符串）、`MISSION_FAULT` 状态 | 编排层 |

看门狗据此分离**判定**与**动作**：先写 verdict JSON（证据），终止运行是少数显式规则。看门狗覆盖位置停滞、楼层截止时间、电梯进度、状态冻结、周期巡检等，全部以仿真秒为主时钟并带墙钟护底。

### 2.7 分层验证策略

| 层级 | 手段 | 成本 | 覆盖内容 |
|---|---|---|---|
| V1 静态 | `check_undefined_attrs.py`、`check_read_before_assign.py`、`test_script_hygiene_core.py` | 秒 | 属性/接口/方法完整性 |
| V2 单元 | `src/simnav/test/` 21 个 core 测试 + `run_stage_b_offline_tests.sh`（另跑 `offline_stage_b_logic.launch` 并断言结果文件 `"passed": true`） | 秒 | 全部领域判据与状态迁移 |
| V3 回放 | `capture_stall_scene.py`、`offline_sensor_sim.py`（自带 `/clock`+`/scan_2d`+`/trunk_imu` 的确定性数据源） + core | 秒–分 | 用真实现场数据复现缺陷，无需 Gazebo |
| V4 单层 | `run_stage_b_seed.sh ... coverage` | 小时 | 一层覆盖与危险源 |
| V5 换层 | `TRANSITION_ONLY=1`（只跑电梯链）+ `elevator_only_test.sh` | 分钟 | 电梯对准/进出/跨门槛 |
| V6 端到端 | `three_floor` 全程 | 4–9 小时 | 全链路 |
| V7 冻结 | `config/stage_b_*_frozen.sha256` 清单 | 秒 | 入口/楼层行为不被静默改动 |

**原则**：任何改动先过 V1–V3；V5 用于尾段（换层/收尾）缺陷，避免被 0 层缺陷遮蔽。

### 2.8 模块边界负面清单（什么不该放在哪一块）

| 机制 | 不应出现的位置 | 原因 |
|---|---|---|
| 房间门的身份/证据/配对 | 探索块（房间侧） | 米级容差下门身份不稳定，5 套匹配器相互矛盾 |
| 朝向/对准特例 | 房间内 | 房间不需要精确落位；只在电梯块内合理 |
| 覆盖/前沿逻辑 | 电梯块 | 电梯任务与信息增益无关 |
| 看门狗自己的"正常等待"定义 | 各看门狗内部 | 必须来自共享的任务状态语义 |
| 电梯几何的第二种算法 | 探索块 | 双 owner 必然不一致 |

### 2.9 模块划分表（现状 10 块）

| # | 模块 | 一句话职责 | 层级 | 关键接口 | 状态机 |
|---|---|---|---|---|---|
| M1 | 场景与物理环境 | 随机生成可通行的三层楼栋与动态门/电梯 | L0 | 文件产物 + Gazebo | — |
| M2 | 机器人平台与控制 | 把 `/cmd_vel` 变成稳定步态位移 | L1 | `/cmd_vel` | 站立/行走/复位 |
| M3 | 传感与数据链路 | 发布雷达/深度/IMU 与派生点云 | L1 | `/scan`、`/livox/*`、`/real_sense/*` | — |
| M4 | 定位与建图 | 度量位姿、world 对齐、分层占用栅格 | L2 | `/simnav/odom`、`/map`、`/simnav/lio_health` | LIO 健康状态 |
| M5 | 地图视图与门禁几何 | 派生导航/探索地图与虚拟门分区 | L2 | `/navigation_map`、`/exploration_map`、`/simnav/entrance_gate` | — |
| M6 | 覆盖探索决策 | 选目标、规划路径、驱动、判完成 | L4 | `/simnav/explorer_status`、`/cmd_vel` | 拓扑生命周期 + 目标生命周期 |
| M7 | 危险源感知 | RGB-D 红球检测 → world 坐标 → 结果文件 | L3 | `/simnav/danger_*`、`detected_danger.json` | 检测生命周期 |
| M8 | 垂直交通与收尾 | 换层、下降、开大门、返回出生点 | L4 | `/simnav/elevator_status`、`/set_door_state`、`/call_elevator` | **跨层状态链（约 20 态）** |
| M9 | 任务编排与监督 | 决定谁在跑、何时换层、异常终止 | L6 | `/simnav/floor_complete`、`/simnav/floor_exploration_context` | 任务阶段 |
| M10 | 评估、记录与验证 | 结果判定、遥测、危险源评分、复现 | L5 | `result.json`、`telemetry.csv`、`evaluation_result.json` | — |

---

## 3. 功能块详述

### 3.1 M1 场景与物理环境块

**职责**：由 seed 生成一个**保证可通行、可评估**的三层楼栋，并写出"算法可见"与"裁判可见"两类工件。

- **输入**：seed + 约束配置（层数、每层房间数、占地、源数量区间、房间类型配比）。
- **输出**：`competition_scene.world` / `world.sdf` / `model.sdf`、`door_config.yaml`、`elevator_config.yaml`、`layout_metadata.json`、`building_config.json`、`scene_manifest.json`、`team_scene_info.json`、`danger_truth.json`、`generation_checks.json`。
- **四阶段**：`constraints.py` → `generator.generate_layout()` → `exporter.export_sdf()` → `_validate_*()`（8 类校验：电梯楼层位姿、初始位姿、门槛填充、厅门配置、动态门模型、楼梯几何等）。
- **数据类**：`Rect2D`、`FurnitureSpec`、`RoomSpec`、`DoorSpec`、`ElevatorSpec`、`FloorLayout`、`BuildingLayout`、`ArtifactPaths`——导出与校验都只读这些对象，因此场景生成器可完全离线单测（`test_constraints.py`、`test_generator.py`、`test_exporter.py`）。
- **运行期服务**：`building_generator_classic_control` 提供 `/set_door_state`（门板插值，约 25 s 开合）与 `/call_elevator`（轿厢换层）。
- **边界**：环境层不感知算法；算法层不读环境内部文件。

### 3.2 M2 机器人平台与控制块

**职责**：把统一速度指令转成稳定的四足步态位移，并**不暴露真值**。

- **模型**：`src/unitree_guide/unitree_ros/robots/a1_description`（xacro 描述，含 4 条腿、雷达/相机/IMU link、足端碰撞体）。
- **控制链（四段）**：`junior_ctrl`（控制周期 `UNITREE_CTRL_DT=0.002 s` = 500 Hz）→ RL 步态策略（TorchScript，**策略推理线程 50 Hz**，观测为 45 维 × 5 帧历史 = 225 维，动作缩放 0.25 后叠加到默认关节角）→ 关节目标经 `unitree_legged_control/UnitreeJointController`（12 个关节 PID，`joint_state_controller` 1000 Hz）在 `gazebo_ros_control` 下执行（命名空间 `/a1_gazebo`）。
- **两套策略**：默认 stair policy（`policy_act_inference_stair.pt`）与平地 policy（`policy_act_inference_plane.pt`），可用 `UNITREE_POLICY_PATH` / `UNITREE_PLANE_POLICY_PATH` 覆盖，并可在运行时通过 `/unitree/select_plane_policy`（`std_srvs/SetBool`）切换——探索器在起始段结束后调用它切到平地策略。
- **模式**：`2` 站立、`4` RL 键盘、`6` RL `/cmd_vel`、`8` 摔倒复位。算法只使用模式 `6`；`/cmd_vel` 经 `FSMState::cmdVelCallback` 直接写入策略的命令张量（竞赛算法**不**下发关节指令）。
- **公平性**：控制链已移除对 `/ground_truth/base_w`、`/ground_truth/base_trunk` 与四足真值位姿的订阅；RL 观测使用机体 IMU。
- **接口**：`/cmd_vel`（`geometry_msgs/Twist`）+ 12 路关节指令/状态 `/a1_gazebo/{FR,FL,RR,RL}_{hip,thigh,calf}_controller/{command,state}`（`unitree_legged_msgs/MotorCmd|MotorState`，后者由 `UnitreeJointController` 以 `mode==PMSM` 生效，`tau = Kp(q_des−q) + Kd(dq_des−dq) + tau_ff`）+ `/a1_gazebo/joint_states`（1000 Hz）。
- **命名澄清**：`junior_ctrl` 是**可执行文件名**，它内部的 ROS 节点名是 **`unitree_gazebo_servo`**；`unitree_legged_control` 不是节点而是 **controller pluginlib 库**（12 个关节控制器实例）。
- **健康接口**：`junior_ctrl` 的 FSM 还发布 latch 的 `/simnav/controller_health`（`WAITING/INVALID/FALL/PASSIVE/RL/ACTIVE/RESET`），可用于区分"控制器没起来"与"算法没动"。
- **已知特性（影响上层设计）**：腿式底盘不是理想速度源——指令到位移存在步态周期延迟与打滑；电梯轿厢口有 6 cm 门槛，实测 **0.45 m/s 可通过、0.35 m/s 会失败**（因此电梯块对 `crossing_speed` 有独立参数）。

### 3.3 M3 传感与数据链路块

**职责**：为上层提供带噪声、带稀疏性的传感器流。

| 传感器 | 话题 | 规格 | 备注 |
|---|---|---|---|
| 机体 IMU | `/trunk_imu` | 1000 Hz | 与 trunk 重合 |
| Livox MID-360 | `/scan`（`sensor_msgs/PointCloud`） | 10 Hz，每帧 2400 条射线（`samples=24000, downsample=10`），0.1–40 m，σ=0.005 m，倾斜 45° | 由 `liblivox_laser_simulation.so` 生成 |
| 点云转换 | `/livox/Pointcloud2`、`/livox/lidar2` | ~10 Hz | `pointcloud2livox.py` |
| Livox IMU | `/livox/imu` | 1000 Hz | LIO 惯性源 |
| RealSense RGB/深度/点云 | `/real_sense/{rgb,depth}/image_raw`、`/real_sense/depth/points` | 10 Hz，640×480，深度 0.05–8 m | `libgazebo_ros_openni_kinect.so` |
| 真值（禁） | `/ground_truth/*`、`/Odometry_gazebo` | 100 Hz | 裁判/调试专用 |

**插件级设计要点（Livox）**：`livox_points_plugin.cpp` 在 `Load()` 中把 ray sensor 先置为 inactive，待世界完成一次 update 后再激活（降低启动竞争）；关闭射线可视化；用扫描模式 CSV（80 万行 Azimuth/Zenith）抽稀成每帧 2400 条射线；`livox_ode_multiray_shape.cpp` 提供 ODE 多射线形状（射线空间的 AABB 宽相位是仿真 RTF 的主要成本来源，见 §6.4）。

### 3.4 M4 定位与建图块

| 子模块 | 节点 | 订阅 | 发布 | 设计要点 |
|---|---|---|---|---|
| LIO | `/fast_lio`（`fastlio_mapping`） | **`/livox/lidar2`（CustomMsg）**、`/livox/imu` | `/Odometry`、`/cloud_registered`、TF | Stage B 使用专用配置 `src/FAST_LIO/config/simenv.yaml`：`filter_size_surf/map=0.25`、`max_iteration=3`、`cube_side_length=120`、`point_filter_num=1`、`blind=0.3`、`det_range=40.0`、`dense_publish_en=false`（注意与通用 `mid360.yaml` 的 `det_range=100` 不同）；长走廊几何退化下靠 IMU 约束 |
| 健康监控 | `/lio_health_monitor` | `/cloud_registered` | `/simnav/lio_health` | `minimum_effective_points=5`；输出 `GOOD/DEGRADED/...` 供危险源与探索使用 |
| 定位桥 | `/localization_bridge`（`lio_localization_bridge_node.py`） | `/Odometry`、`/cmd_vel`、`/simnav/explorer_status`、`/simnav/metric_corridor_constraint` | `/simnav/odom`、`/simnav/world_pose`、`/simnav/world_pose_metric`、`/simnav/lio_map_transform`、`/simnav/local_loop_closure_{request,applied}`、TF | 关键机制：`command_response_scale=0.95`、`command_timeout=0.35`、`minimum_motion_fraction=0.20`（指令-响应一致性）、`max_loop_correction=1.0`、`max_loop_rotation=0.12`、`max_corridor_constraint=3.0`；world 对齐只用公开起点 |
| 3D→2D | `/simnav_cloud_to_scan` | `/livox/Pointcloud2` | `/scan_2d` | 高度带 0.08–1.50 m、`range_max=12.0`、`angle_increment=0.0087 rad`、`concurrency_level=1` |
| 占用图 | `/lio_occupancy`（+`map_floors_core.py`） | `/cloud_registered`、`/simnav/odom`、`/simnav/lio_map_transform`、`/simnav/floor_exploration_context` | `/map`、`/simnav/map_floors` | 按层保留独立栅格；`trace_ray_free` 只释放 **unknown** 格（occupied 永不被释放）；点云 `point_stride=4`；换层不改写历史层 |
| 位姿连续性 | `pose_continuity.py` | （被 node 调用） | — | 位姿跳变检测，用于回环/重定位后的连续性判定 |

**边界**：本块只回答"我在哪、地图是什么"，不做任何任务决策；`/simnav/odom` 是唯一位姿入口（探索/电梯/危险源三块都订阅它）。

### 3.5 M5 地图视图与门禁几何块

- **节点**：`/map_views`（`map_views_node.py`）。
- **订阅**：`/map`、`/simnav/floor_exploration_context`、`/simnav/entrance_gate`、`/simnav/defer_zone`；**发布**：`/exploration_map`、`/navigation_map`。
- **纯函数**：`binary_dilate/erode`、`close_small_gaps`、`_rasterize_polygon`、`_draw_line`、`_inside_polygon`。
- **设计含义**：**探索语义地图与导航代价地图分离**——探索地图保留未知区（供前沿/增益计算），导航地图做膨胀/闭运算（供安全路径），虚拟门/大堂边界以多边形形式写入探索视图而不写进物理世界。
- **边界**：本块不产生目标，只提供视图。

### 3.6 M6 覆盖探索决策块（核心块）

**职责**：在"单层走廊 + 若干小房间"的结构里，持续产出"下一个该去哪 + 怎么去"，直到该层覆盖率达标并回到走廊。

#### 3.6.1 双层结构

| 层 | 文件 | 内容 |
|---|---|---|
| core | `coverage_explorer_core.py`（3685 行） | 数据类 `GridView`/`CoverageSnapshot`/`FrontierTarget`/`CoveragePlan`/`RoomPortal`/`PortalStation`/`TaskExtent`；约 30 个模块级纯函数；巨型类 `TaskCoveragePlanner`（≈2200 行） |
| node | `coverage_explorer_node.py`（3413 行） | 类 `CoverageExplorer`：订阅/发布、参数、`_plan_occupied`/`_control_*` 定时器、诊断与 RViz 标记 |

#### 3.6.2 数据模型（core）

- `RoomPortal`：一个候选门洞的**几何瞬时量**——`id`、`side`、`along`（沿走廊轴位置）、`lateral`（横向）、`measured_wall`、`width`、`evidence`、`confirmed`、`actionable`。
- `PortalStation`：门站（同一 `along` 上左右两个门构成的"对门"站）。
- `TaskExtent`：任务包络（走廊入口门 `virtual gate` 前后界、横向半宽、置信度）。
- `CoveragePlan`：一次规划的输出——目标点/类型/所属拓扑、路径、诊断字段（拒绝原因、候选计数、门带信息）。
- `FrontierTarget`：前沿候选（激光/相机/门口/巡游等 kind）。

#### 3.6.3 覆盖与完成判据（纯函数）

| 函数 | 语义 |
|---|---|
| `weighted_linear_coverage(laser, camera, camera_weight)` | 覆盖率主口径 |
| `weighted_harmonic_coverage(...)` | 谐波口径（防止一侧掩盖另一侧） |
| `coverage_snapshot` / `coverage_classification` | 生成/分类覆盖率快照（`laser`/`camera`/`combined`） |
| `topology_completion_ready(rooms, unreviewed)` | 完成判据：房间数 ≥ 期望 且 未复核为 0 |
| `floor_regions_complete(statuses, expected_rooms)` | 楼层完成 |
| `region_status(combined, target, active)` | 区域状态 `UNSEEN/ACTIVE/COVERED` |
| `task_region_mask` / `topology_region_mask` | 任务包络/单拓扑区域掩膜 |
| `distinct_room_count` / `front_stations_complete` | 去重房间数与门站完成（**几何匹配**，容差 2.5 m） |

默认目标（`stage_b_floor_explorer.launch`）：`laser_coverage_target=0.95`、`camera_coverage_target=0.85`、`combined_coverage_target=0.84`、`room_combined_coverage_target=0.84`、`camera_weight=0.95`。控制器可用环境变量（如 `STAGE_B_ROOM_COMBINED_COVERAGE_TARGET=0.55`）整体下调目标，用于"降低覆盖要求看衔接"的实验。

#### 3.6.4 目标生成（过滤器链，现状）

规划器 `TaskCoveragePlanner.plan()` 的候选来自多个族，再经一串过滤器排序：

| 族 | 生成方式 |
|---|---|
| 激光前沿 | `GridView` 上的前沿簇（`_clusters`、`frontier_cluster_radius=0.45`、`frontier_revisit_radius=0.70`） |
| 相机前沿 | 由 `/simnav/camera_coverage` 提供的"未看区域"（`information_radius=2.5`、`camera_weight`） |
| 门口观测/穿越 | `verified_door_band`、`door_crossing_along_offsets`（±0.30 m 窄搜 + ±2.5 m 宽搜，步长 0.5）、`_door_approach_target` |
| 走廊巡游 | `corridor_wander_target`（闭合，可达 ±2…±14 m） |
| 危险源复核 | `_review_target`（由 `/simnav/danger_candidates` 驱动） |

关键判据函数：`target_switch_allowed`（目标切换防抖）、`target_replacement_gain_ok`（替换增益门槛 `stalled_replace_gain_ratio=0.50`）、`unsafe_path_replan_needed`、`prefer_room_approach`、`interior_targets_only`、`door_approach_is_new`、`doorways_match`。
分区与顺序：`zone_split_along`（走廊纵向中点）/`zone_of_along`/`active_zone`/`zone_admits`——实现"先下半区、下半区完成才去上半区"。

#### 3.6.5 路径与安全（core）

- `navigation_path` / `navigation_path_via` / `navigation_path_through_portal` / `navigation_path_from_room_through_portal`：A*（`_astar_cells`，含初始航向、转弯代价）+ `_shortcut_cells` 拉直 + `_resample_world_path`（0.40 m 采样）。
- A* 细节：8 邻域**带朝向的状态** `(row, column, direction)`，拒绝对角切角，代价 `step·(1 + 1.4·clearance_deficit) + 0.10·turn_angle`，启发式用欧氏距离；随后 `_shortcut_cells` 视线拉直、`_resample_world_path(maximum_spacing=0.40)` 重采样。
- 安全判据：`path_is_safe(..., despeckle=)`（净空 ≥ `navigation_clearance=0.30`，期望 `preferred_clearance=0.42`）、`_line_is_safe`、`_navigation_fields`（距离场/可达域，A* 失败时给出"可达格数"诊断）。
- 门口斑点清理：`clear_doorway_speckle(data, mask, max_component=2, dilation=1)`——只在门掩膜内释放**完全落在掩膜内**的小连通块（≤2 格），避免"门被噪点堵死"；这是对占用图"occupied 永不被释放"策略的定点补偿。
- 相机候选的采样纪律：按格抽样（步长 `ceil(max(0.8, revisit_radius)/resolution)`）、剔除过近（<0.6 m）、**按拓扑分层保留**（每拓扑至少 `max(8, ceil(64/N))` 个）、总量上限 128 个，并为每个候选计算 `look_at`（16 个 22.5° 朝向分箱中选最"值得看"的方向）。

#### 3.6.6 控制（node）

- 定时器：`_plan_guarded`（规划，`replan_period=1.0` 仿真秒）与 `_control`（控制在 20 Hz），二者都在异常时记 `PLAN_FAULT`/`CONTROL_FAULT` 而不静默死亡。
- 速度体系：`motion_speed=0.60`、`max_linear_speed=0.60`（`_publish_command` 内统一钳制）、过门限速 `transit_speed_cap=0.35`（仅 `transit_jamb_zone=2.0 m` 内生效）、转向 `turn_speed=0.65`、起步斜坡 `_ramped_forward_speed`（settle 1.0 s、ramp 2.0 s、0.25→0.60）、停止距离 `motion_stop_distance=0.48`。
- 卡住与恢复：`stuck_target_seconds=5.0` + `stuck_target_cooldown_seconds=120`、`collision_block_seconds=45`、`transit_timeout_seconds=60`、`unsafe_path_*`、`max_align_seconds=8.0`、`_recover_idle_lock`、`_retry_blocked_room`。
- 末端对准：`CAMERA_FRONTIER`/`SPHERE_REVIEW` 带 `look_at` 时，进入航向误差 ±0.12 rad 并保持 0.6 仿真秒（球体复核用 `sphere_review_hold=2.0`）后再结束；门带内航向容差收紧到 0.12 rad。
- 最后厘米：`creep_stop_distance=0.20` + `creep_speed=0.25`（前沿目标在净空允许时以蠕行收尾），停止由 `motion_stop_distance=0.48` 的前向净空门控。
- 起始机动：`_control_initial_forward`（沿走廊前进 `initial_forward_distance=14.5 m` 并居中，速度按 settle 1.0 s + ramp 2.0 s 从 0.25 升到 0.60）——它同时承担"离开大堂、建立走廊参考"的职责；`transit_timeout_seconds` 后采用暂定门（`_adopt_provisional_gate`）以免永远起不来。
- **复用关系**：`TaskCoveragePlanner` 同时被两个客户端使用——探索器把它当"覆盖/拓扑规划器"，电梯块只借它的 A*/路由能力（`navigation_path`）。这是当前最值得显式化的复用点（应拆成独立的 `PathPlanner` 服务）。

#### 3.6.7 状态与生命周期（字面状态名）

- **节点级阶段** `status.state`：`INITIAL_FORWARD`（尚未确定虚拟门）→ `COVERAGE_EXPLORATION` → `FLOOR_COMPLETE`。
- **区域阶段** `topology_region`：`LOBBY_TRANSIT` / `CORRIDOR` / `ROOM_APPROACHING` / `ROOM_EXPLORING` / `ROOM_RETURNING` / `OPPOSITE_ROOM_APPROACHING`。
- **单房间状态** `topology_states[id].state`：`APPROACHING` → `EXPLORING` → `RETURNING` → `COMPLETE`；失败/不可达为 `BLOCKED`。
  - `APPROACHING→EXPLORING`：位姿落入该房间区域（区域成员判定）或门平面回退条件成立。
  - `EXPLORING→RETURNING`：房内 `combined` 达标（或红球提前退出）且增益/宽限判据通过，转入 `_start_corridor_return`。
  - `RETURNING→COMPLETE`：位姿回到走廊带（`|lateral| ≤ floor_end_corridor_tolerance=0.35`）且终点格在**导航图**上净空达标（`≥ navigation_clearance`）。
  - `COMPLETE` 后立即用 `opposite_room_portal` 锁定**对面房间**为 `APPROACHING`（对门配对探索）。
- **走廊分区** `zone_of_along` 返回 `"A"`/`"B"`（以门站带中点分割），`active_zone` 决定当前只服务哪一半——实现"下半区完成才去上半区"。
- 生命周期用 `completed_topologies`、`retired_topologies`、`room_entry_counts`、`blocked_targets`、`route_refresh_counts` 记账；进门/出门判定在**控制周期（20 Hz）**执行（`_mark_room_entered_locked` / `_finish_corridor_return_locked`），不在 1 Hz 规划周期，避免事件被重规划漏掉。
- 完成闸门：`_check_completion` → `topology_completion_ready(completed, expected_rooms, 0)` + `completion_stable_duration=3.0` 仿真秒去抖 → `/simnav/floor_complete`。
- 危险源耦合：`danger_guidance_level`（0 仅检测 / 1 兜底复核 / 2 拓扑内优先 / 3 全局优先+抢占）、`danger_early_exit_coverage`（默认 0 关闭）、`danger_candidate_min_hits`；激光独立簇检测由 `detect_sphere_like_clusters` 提供（低矮各向同性点簇，`sphere_min_hits=3`）。
- **服务调用**：本块还调用 `/unitree/select_plane_policy`（`std_srvs/SetBool`），在固定起始段结束后切换到平地策略。

#### 3.6.8 诊断（模块可观测性）

`/simnav/explorer_status`（latch JSON）是全系统的"飞行数据记录器"，键按组：

| 组 | 键（示例） |
|---|---|
| 覆盖 | `laser_coverage`、`camera_coverage`、`combined_coverage`、`*_target`、`room_coverages`、`room_region_status` |
| 拓扑/门户 | `observed_portals`、`actionable_portals`、`portal_evidence`、`front_station_along`、`front_rooms_complete`、`completed_front_sides`、`completed_topologies`、`reused_topology_ids`、`topology_states` |
| 目标/规划 | `active_target`、`active_target_kind`、`active_target_topology`、`planner_diagnostics`、`plan_cycles`、`plan_failures`、`last_plan_reason`、`last_plan_ms`、`target_switches`、`targets_reached` |
| 分区 | `zone_*`、`floor_index`、`task_forward_limit`、`task_extent_confident` |
| 危险源 | `sphere_hypotheses`、`unreviewed_sphere_hypotheses`、`candidate_topologies` |
| 活性/故障 | `control_faults`、`plan_faults`、`progress_idle_seconds`、`stuck_target_drops`、`empty_path_cycles`、`unsafe_path_*`、`route_refreshes`、`lock_release_counts`、`room_interior_retries`、`navigation_blocks` |
| 指令 | `cmd_vel.linear_x`、`cmd_vel.angular_z` |

**边界**：本块**不**写门/电梯服务、**不**做红球判定、**不**决定换层；它只回答"这一层下一步去哪"。

### 3.7 M7 危险源感知块

| 层 | 内容 |
|---|---|
| core（`danger_detector_core.py`） | `CameraIntrinsics`（fx/fy/cx/cy）、`BallObservation`、`DangerTrack`（含 `position_variance`）、`RedBallDetector`（HSV 红分割 + 圆度 `min_circularity=0.82`、面积/半径门限）、`_center_depth`（球心区域深度中值）、`rasterize_planar_ray`（观测射线栅格化，供覆盖统计）、`DangerTracker`（多帧确认 `confirmation_frames=3`、空间聚类 `cluster_radius=0.75`、平移/合并/锚定）、`position_on_floor`（地面高度过滤 `floor_min_offset=-0.6…+1.2`）、`project_pose_from_anchor`、`ResultWriter`（**原子写** `detected_danger.json` + debug 版） |
| node（`danger_detector_node.py`） | RGB-D 同步（`ApproximateTimeSynchronizer`，`slop=0.08`，队列 8）、`moving_frequency=10.0` 门控、`world_pose_metric` + `camera_info` → 反投影、多帧跟踪与确认、结果文件写出、发布诊断 |

**输出**：`/simnav/danger_poses`（PoseArray）、`/simnav/danger_markers`、`/simnav/danger_tracks`、`/simnav/danger_candidates`、`/simnav/danger_confirmation_active`、`/simnav/camera_coverage`（相机覆盖，供探索块）、`/simnav/camera_observed_markers`、`/simnav/danger_detection_lifecycle`、`/simnav/danger_valid_frame`、`results/detected_danger.json`。

**关键设计点**：
1. 颜色**不是**充分条件（红方块是干扰源）→ 必须有圆度/轮廓/深度一致性判别（`min_circularity=0.82`、长宽比 0.78–1.28、extent ≤ 0.88）。
2. 三维位置依赖定位 → 使用 `/simnav/world_pose_metric` 反投影，并把"定位健康度"记进每条轨迹（`DangerTrack.localization_health_counts`）。
3. 写文件必须**原子**（先写临时文件再 `replace`），避免评估脚本读到半截 JSON。
4. **⚠ 已发现的接口断裂（需修）**：检测节点订阅的是 `/simnav/localization_health`，而健康监控节点发布的是 `/simnav/lio_health`（`lio_health_monitor_node.py:16` vs `danger_detector_node.py:169`）。两者名字不匹配且都不完整，导致 `localization_health` 恒为 `"STALE"`、轨迹里的健康度统计失去意义。这是当前包内**最明确的一处跨模块接口错误**，修复方式是把两端统一到同一话题名并把状态词表对齐。

### 3.8 M8 垂直交通与任务收尾块

**职责**：楼层完成之后的一切——回电梯、进轿厢、跨层、出轿厢、建立新楼层上下文、封顶后下降到一层、开主入口门、返回出生点。

#### 3.8.1 状态链（literal 状态名，代码位置 `elevator_transition_node.py`）

```text
WAITING / WAIT_FLOOR_COMPLETE
  → RETURN_TO_ELEVATOR → ALIGN_ELEVATOR → ENTER_ELEVATOR
  → RIDE_TO_FLOOR_1 → ALIGN_FLOOR_1_EXIT → EXIT_ELEVATOR
  → ESTABLISH_FLOOR_1_TOPOLOGY → FLOOR_1_READY
  → RETURN_TO_FLOOR_1_GATE → ENTER_FLOOR_1_LOBBY → SEARCH_FLOOR_1_ELEVATOR
  → ALIGN_FLOOR_1_ELEVATOR_RETURN → ENTER_FLOOR_1_ELEVATOR_RETURN
  → (RIDE_TO_NEXT_FLOOR | RIDE_TO_GROUND_FLOOR)
  → ALIGN_GROUND_FLOOR_EXIT → EXIT_GROUND_FLOOR
  → OPEN_MAIN_ENTRANCE → RETURN_TO_SPAWN → RETURNED_TO_SPAWN
```

异常/终态：`TOP_FLOOR_COMPLETE`、`MISSION_FAULT`、`ROBOT_ROLLED`、`ROBOT_ON_GROUND`、`NO_STATUS`，以及失败原因字符串（如 `ELEVATOR_ENTRY_NO_PROGRESS`、`NO_SENSOR_CONFIRMED_FLOOR_1_ELEVATOR_OPENING`、`WIDE_PORTAL_MAP`、`REFERENCE_TOPOLOGY_EMPTY`）。

> 命名说明：状态名沿用"FLOOR_1"表示"上一层"（历史命名），实际语义是**楼层无关**的；阅读时把 `FLOOR_1` 理解为 `NEXT_FLOOR`。

#### 3.8.2 core 纯函数（`elevator_transition_core.py`）

`door_frame_offset`、`point_from_gate`、`portal_staging_point`、`target_heading`、`choose_opening_heading`（选择开口朝向）、`height_transition_complete`（跨层高度完成）、`entry_stall_confirms_containment`（进入停滞=已进轿厢）、`entry_blocked_retry_allowed`、`direct_entry_applies`、`direct_alignment_ready`（对准就绪，容差 0.25）、`tilt_fault_state`（IMU 倾倒判定，需持续 `fall_tilt_persist`）、`establish_budget_exceeded`、`elevator_door_id(floor_index)`、`transform_pose_between_frames`、`detect_wide_lobby_openings`、`clamp_linear_speed`。

#### 3.8.3 关键机制

| 机制 | 参数/说明 |
|---|---|
| 门与电梯控制 | `/set_door_state`（`elevator_floor_{0,1,2}`、`main_entrance`）、`/call_elevator(elevator_main, target_floor, open_doors)`；`_ensure_elevator_door_open` 在对准/进出前确保厅门已开 |
| 精确接近 | `elevator_approach_tolerance=0.80`、`direct_alignment_ready` 容差 0.25、`crossing_speed=0.45`（跨 6 cm 门槛的实测最优）、`lobby_approach_speed=0.45`、`route_accept_distance=1.2` |
| 重试预算 | `max_route_retries=40`、`route_unreachable_timeout=60`、`route_open_loop_grace=6.0`、进入受阻回退重试 |
| 倾倒/跌落保护 | `fall_roll_limit_deg=30`、`fall_pitch_limit_deg=30`、`fall_baseline_window=20`、`fall_grace_seconds=15`（基线自适应，容忍度量 z 长期漂移） |
| 楼层上下文发布 | `_publish_floor_context(source)` → `/simnav/floor_exploration_context`（楼层号、来源、几何提示、可复用门户），上层探索器据此换图 |
| 收尾 | `main_entrance` 需 `accepted && state=="open"` 才记成功（`main_entrance_max_attempts=6`）；`return_to_spawn_tolerance=0.45` |
| 速度纪律 | `motion_speed=0.60`、`max_linear_speed=0.60`（与探索块一致的全局上限） |

#### 3.8.4 输出

`/simnav/elevator_status`（latch JSON，含状态、目标、失败原因、门状态、对准误差）、`/simnav/floor_transition_complete`、`/simnav/two_floor_mission_complete`、`/simnav/mission_fault`、`/simnav/floor_exploration_context`、`logs/*/elevator_phase.csv`（每状态耗时与动作质量）。

### 3.9 M9 任务编排与监督块

| 组件 | 职责 | 关键接口 |
|---|---|---|
| `run_stage_b_seed.sh`（454 行） | 单实例锁、资源检查、生成场景、起 `auto.sh`（Gazebo+控制器）、起定位链、起支撑链（危险源+电梯）、起 supervisor、起监控/看门狗、收集日志、调用评估 | 环境变量控制速度/覆盖率目标/楼层数/GUI/RViz |
| `activate_stage_b_controller.py` | 通过 ROS 服务把 `junior_ctrl` 切到站立/RL 模式 | `/a1_gazebo/*` |
| `two_floor_explorer_supervisor.py` | 订阅 `/simnav/floor_complete` 与 `/simnav/elevator_status`；**每层起停一个探索器实例**（`start_explorer(floor_index)`/`stop_explorer`），保证换层时探索器状态干净 | `/simnav/floor_complete`、`/simnav/elevator_status` |
| `watch_*.py` | 位置停滞、楼层截止、电梯进度、状态冻结、周期巡检；**仿真时钟 + verdict JSON** | 各状态话题、`telemetry.csv` |
| `testonly_*` / `elevator_only_*` | 只跑尾段的专项测试（`TRANSITION_ONLY=1`、`elevator_only_test.sh`：真值 z 上升 ≥1.5 m + 完成进/出/跨门槛即 PASS） | — |

**看门狗的判定与预算（要点）**：`watch_position_stall.py`（180 仿真秒内位移 ≤0.30 m，电梯非 `WAIT_FLOOR_COMPLETE` 或楼层已完成时跳过；小位移但航向变化不计入——`--yaw-threshold` 参数被记录但未参与判定）；`watch_floor_deadline.py`（每层 600 仿真秒上限）；`watch_elevator_progress.py`（**逐状态**仿真预算，如 `ENTER_ELEVATOR 90 s`、`ESTABLISH_FLOOR_1_TOPOLOGY 170 s`、`OPEN_MAIN_ENTRANCE 60 s`，另加 1800 秒墙钟护底与"60 秒无动作"判据）；`watch_stage_b_freeze.py`（覆盖率 20 仿真秒不变且 `NO_FRONTIER`）；三者都会写 `*_verdict.json`（判定与证据），其中三个可终止运行。

**设计要点**：编排层**不持业务状态**，只做"谁在跑、何时切换、何时终止"；业务真相来自各块的 status 话题。
**注**：`testonly_launch_guarded.sh`、`testonly_startup_health.py` 自带 "TEST-ONLY, delete for the final version" 标记，属于临时护栏，不计入正式架构。

**设计要点**：编排层**不持业务状态**，只做"谁在跑、何时切换、何时终止"；业务真相来自各块的 status 话题。

### 3.10 M10 评估、记录与验证块

| 工具 | 输出 | 说明 |
|---|---|---|
| `monitor_stage_b_coverage.py`（+`stage_b_monitor_core.py`） | `result.json` | 汇总 `completed_floor_indices`、`floor_complete`、`returned_to_spawn`、`main_entrance_opened`、`mission_fault`、`elapsed_sim_time`、`mean_real_time_factor`、`max_pose_step`、`state_sequence`/`state_durations`、`room_entries/exits`、覆盖率、危险源召回等 |
| `record_stage_b_telemetry.py` | `telemetry.csv` | 列：`wall,sim,src_x,src_y,src_yaw,wx,wy,wz,wroll,wpitch,wyaw,imu_roll,imu_pitch,imu_gx,imu_gy,imu_gz,cmd_vx,cmd_wz,elevator_state,gate_along,gate_lateral,region_lock,entry_total` |
| `evaluate_stage_b_danger.py` / `src/building_obstacles/scripts/evaluate_danger.py` | `danger_evaluation.json` / `evaluation_result.json` | 1 m 阈值 + 贪心一对一匹配；召回、虚警、时间分 |
| `verify_three_floor_run.py` | 判定与证据 | 三层端到端校验（楼层、收尾、危险源） |
| `capture_stall_scene.py` / `offline_sensor_sim.py` | 现场快照/离线回放 | 把"卡住"现场变成可离线复现的输入 |
| `run_stage_b_offline_tests.sh` | 测试结果 | 21 个 core 单测 + 静态检查 |

**设计要点**：所有判定产物都是**结构化 JSON**（不是日志文本），这样"这轮到底是死了、慢了，还是判据没过"可以一眼区分。

**通过判据（三层模式，`monitor_stage_b_coverage.py`）**：`mission_fault is None` ∧ `two_floor_mission_complete` ∧ `returned_to_spawn` ∧ `main_entrance_opened` ∧ `completed_floor_indices ∋ {2}` ∧ 三层各自"房间全部 `COMPLETE`"（≥ `expected_rooms_per_floor=4`）∧ `max_pose_step < 1.0`；再与危险源评估（`recall ≥ 1.0` 且 `FAR ≤ 0.1`）做逻辑与。**注意**：内部判据（召回 1.0、逐层房间完成）比比赛客观分（召回门槛 0.6、无覆盖率要求）更严，论文引用时必须区分两套口径。

**已知脆弱点**：监控器只在**运行结束时打印一次** `result.json`，因此被看门狗杀停的运行会留下 **0 字节** `result.json`（历史归档中 72/449 如此）；这类轮次的结论只能从看门狗的 verdict JSON 读取。`verify_three_floor_run.py` 是**人工检查清单**（房间不重复进入、无 `navigation_blocks`、检出红球、≥2 层、返回出生点、12 个房间 combined ≥0.55），**总是返回 0**，不参与判定。

---

## 4. 模块间接口契约总表

### 4.1 话题（Topic）

| 话题 | 类型 | 发布者 | 主要订阅者 | 语义 |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | 全部 | 仿真时钟（唯一任务时钟） |
| `/cmd_vel` | `geometry_msgs/Twist` | 探索器 / 电梯块 | `junior_ctrl` | 速度指令 |
| `/scan` | `sensor_msgs/PointCloud` | Livox 插件 | `pointcloud2livox` | 原始点云 |
| `/livox/Pointcloud2`、`/livox/lidar2` | `PointCloud2` / `CustomMsg` | `pointcloud2livox` | cloud_to_scan、FAST-LIO、RViz | 派生点云 |
| `/livox/imu`、`/trunk_imu` | `sensor_msgs/Imu` | 插件 | FAST-LIO、定位桥、电梯块 | 惯性 |
| `/real_sense/rgb/image_raw`、`/real_sense/depth/image_raw`、`/real_sense/*/camera_info`、`/real_sense/depth/points` | `Image`/`CameraInfo`/`PointCloud2` | RealSense 插件 | 危险源检测、RViz | 视觉 |
| `/Odometry`、`/cloud_registered` | `nav_msgs/Odometry`、`PointCloud2` | FAST-LIO | 定位桥、健康监控、占用图 | LIO 原始输出 |
| `/scan_2d` | `sensor_msgs/LaserScan` | `simnav_cloud_to_scan` | 探索器、电梯块 | 2D 激光（range_max 12 m） |
| `/map` | `nav_msgs/OccupancyGrid` | `lio_occupancy` | `map_views`、探索器 | 分层占用栅格（当前层） |
| `/exploration_map`、`/navigation_map` | `OccupancyGrid` | `map_views` | 探索器、电梯块、RViz | 探索视图 / 导航视图 |
| `/simnav/map_floors` | `std_msgs/String`(JSON) | `lio_occupancy` | 监控 | 各层栅格统计 |
| `/simnav/odom` | `nav_msgs/Odometry` | 定位桥 | 探索器、电梯块、危险源、占用图、遥测 | base 位姿（唯一入口） |
| `/simnav/world_pose`、`/simnav/world_pose_metric` | `PoseStamped` | 定位桥 | 探索器、危险源、电梯块 | world 对齐位姿 / 度量位姿 |
| `/simnav/lio_health` | `std_msgs/String`（latch） | `lio_health_monitor` | **包内无订阅者** | `{state:"GOOD"\|"NO_EFFECTIVE_POINTS"\|"STALE", effective_points, timestamp}` |
| `/simnav/localization_health` | `String` | **无发布者** | 危险源检测（订阅） | 期望的健康契约；当前断裂（见 §3.7 与 §6.2） |
| `/simnav/lio_map_transform` | `geometry_msgs/TransformStamped` | 定位桥 | 占用图、探索器 | LIO→地图对齐 |
| `/simnav/local_loop_closure_request`、`/simnav/local_loop_closure_applied` | `String` | 定位桥 | 探索器、危险源 | 局部门口回环 |
| `/simnav/metric_corridor_constraint` | `String` | 探索器 | 定位桥 | 走廊方向约束 |
| `/simnav/entrance_gate` | `geometry_msgs/PolygonStamped` | 探索器 | `map_views` | 虚拟大堂/走廊门 |
| `/simnav/defer_zone` | `String` | 探索器 | `map_views` | 暂缓区域 |
| `/simnav/explorer_status` | `String`(JSON, latch) | 探索器 | 电梯块、危险源、定位桥、看门狗、RViz | 探索诊断全量快照 |
| `/simnav/coverage_status`、`/simnav/coverage_markers`、`/simnav/coverage_layers`、`/simnav/coverage_path`、`/simnav/room_entry` | JSON/MarkerArray/Path/String | 探索器 | RViz、监控 | 覆盖可视化与进门记录 |
| `/simnav/floor_complete` | `std_msgs/Bool`(latch) | 探索器 | supervisor、电梯块、危险源 | 该层完成 |
| `/simnav/floor_exploration_context` | `String`(JSON) | 电梯块 | 探索器、占用图、map_views、危险源 | 楼层上下文（楼层号、几何提示、复用门户） |
| `/simnav/elevator_status` | `String`(JSON, latch) | 电梯块 | supervisor、看门狗、RViz | 状态链快照 |
| `/simnav/floor_transition_complete`、`/simnav/two_floor_mission_complete` | `Bool`/`String` | 电梯块 | 监控 | 换层/任务完成 |
| `/simnav/mission_fault` | `String` | 电梯块、危险源 | 监控、看门狗 | 任务级故障 |
| `/simnav/camera_coverage` | `String`(JSON) | 危险源 | 探索器 | 相机已看单元 |
| `/simnav/danger_poses`、`/simnav/danger_markers`、`/simnav/danger_tracks`、`/simnav/danger_candidates`、`/simnav/danger_confirmation_active`、`/simnav/danger_detection_lifecycle`、`/simnav/danger_valid_frame`、`/simnav/camera_observed_markers` | PoseArray/MarkerArray/String/Bool | 危险源 | 探索器、RViz | 危险源检测与复核 |
| `/ground_truth/*`、`/Odometry_gazebo` | — | Gazebo 真值插件 | **禁止算法订阅** | 裁判/调试 |

### 4.2 服务（Service）

| 服务 | 类型 | 提供者 | 消费者 | 语义 |
|---|---|---|---|---|
| `/set_door_state` | `SetDoorState` | `building_generator_classic_control` | 电梯块 | `door_id` + `open`；门板插值约 25 s 后返回 |
| `/call_elevator` | `CallElevator` | 同上 | 电梯块 | `elevator_id`、`target_floor`、`open_doors` |

### 4.3 文件（File）

| 文件 | 写方 | 读方 | 内容 |
|---|---|---|---|
| `generated_building/team_scene_info.json` | 生成器 | **算法（唯一允许）** | 起点、公开门/电梯 ID、允许话题/服务、结果路径、禁止清单 |
| `layout_metadata.json` / `building_config.json` / `scene_manifest.json` / `competition_scene.world` / `door_config.yaml` / `elevator_config.yaml` | 生成器 | 环境/裁判 | 内部布局与配置（算法禁读） |
| `results/detected_danger.json` | 危险源块 | 评估脚本 | `exploration_time` + `detected_danger_sources[].position` |
| `results/detected_danger_debug.json` | 危险源块 | 调试 | 轨迹级调试信息 |
| `results/danger_truth.json` | 生成器 | 评估脚本 | 真值（算法禁读） |
| `results/evaluation_result.json` | 评估脚本 | 报告 | 召回/虚警/时间得分 |
| `logs/<run>/.../result.json` | 监控 | 报告/校验 | 系统级判定与统计 |
| `logs/<run>/.../telemetry.csv` | 遥测 | 取证 | 23 列高频状态 |
| `logs/<run>/.../elevator_phase.csv` | 电梯块 | 取证 | 每状态耗时与动作质量 |
| `config/stage_b_*_frozen.sha256` | 人工 | runner | 关键文件冻结清单 |

---

## 5. 关键数据流

### 5.1 探索主环（单层，20 Hz 控制 / 1 Hz 规划）

```text
Gazebo(雷达/深度/IMU) ──► Livox插件(/scan) ──► pointcloud2livox ──┬─► /livox/Pointcloud2 ─► pointcloud_to_laserscan ─► /scan_2d ─┐
                                                                  └─► /livox/lidar2(CustomMsg) ─► FAST-LIO                   │
FAST-LIO ─► 定位桥 ─► /simnav/odom + /simnav/world_pose_metric ─────────────────────────────────────────────────────────────────┤
FAST-LIO ─► /cloud_registered ─► 占用图 ─► /map ─► map_views ─► /exploration_map,/navigation_map ────────────────────────────────┤
RealSense ─► 危险源块 ─► /simnav/camera_coverage ────────────────────────────────────────────────────────────────────────────────┤
                                                                                                                                ▼
                                                        ┌──────────────────────────────────────────────────┐
                                                        │ coverage_explorer                                 │
                                                        │  plan(): 候选族 → 过滤 → A* → 安全校验 → CoveragePlan│
                                                        │  control(): 路径跟踪 → /cmd_vel                    │
                                                        └───────────────┬──────────────────────────────────┘
                                                                        ▼
                                                             /cmd_vel → junior_ctrl(RL 策略) → 关节
                                                        状态：/simnav/explorer_status（诊断全量）
```

### 5.2 换层链（楼层完成之后）

```text
coverage_explorer ──/simnav/floor_complete(true)──► supervisor ──(停当前探索器)
                                        │
                                        ├──► elevator_transition: WAIT_FLOOR_COMPLETE → RETURN_TO_ELEVATOR → ALIGN → ENTER
                                        │        │
                                        │        ├─ /set_door_state(elevator_floor_n, open)
                                        │        ├─ /call_elevator(elevator_main, target_floor)
                                        │        └─ 精确对准（crossing_speed=0.45，容差 0.25/0.80）
                                        ▼
                         EXIT_ELEVATOR → ESTABLISH_FLOOR_1_TOPOLOGY → FLOOR_1_READY
                                        │
                                        └─ /simnav/floor_exploration_context ─► lio_occupancy（切层）/ map_views / 危险源
                                        │
                        supervisor ──► 起新的 coverage_explorer（楼层号 + 复用门户提示）
```

封顶之后：`RIDE_TO_GROUND_FLOOR → EXIT_GROUND_FLOOR → OPEN_MAIN_ENTRANCE(/set_door_state main_entrance) → RETURN_TO_SPAWN → RETURNED_TO_SPAWN`。

### 5.3 危险源链

```text
RealSense RGB+Depth(同步 slop=0.08) ─► RedBallDetector(HSV 红 + 圆度 + 深度) ─► BallObservation
        │                                                                          │
   /simnav/world_pose_metric ─────────────────────────────────────────────► DangerTracker(多帧确认 3 / 聚类 0.75 m)
                                                                                   │
                                    /simnav/danger_candidates ◄── 未确认轨迹复核目标 ─┤
                                    /simnav/camera_coverage  ◄── 相机覆盖（供探索）  │
                                                                                   ▼
                                                    ResultWriter（原子写）→ results/detected_danger.json
```

---

## 6. 模块化现状评估与目标架构

### 6.1 已经做到的

1. **core/node 双层**在 5 个算法块上一致落地，21 个离线测试可在秒级回归领域逻辑。
2. **接口收敛**为话题/服务/文件三类，并有明确命名与坐标系约定。
3. **环境与算法分离**：真值隔离、控制器去真值、公开信息只有一份 `team_scene_info.json`。
4. **时间纪律**：任务与看门狗统一仿真时钟（历史误杀的直接修复）。
5. **可观测性**：`/simnav/*_status` 全量 JSON + `telemetry.csv` + `elevator_phase.csv`，使"卡住"可被遥测定位而不是靠猜。
6. **分层验证**：V1–V7 七级（静态→单元→回放→单层→换层→端到端→冻结清单）。

### 6.2 已知耦合与重复（按根因归类）

| 根因 | 具体表现 | 证据 |
|---|---|---|
| **身份由几何瞬时量派生** | 同一物理门在不同时刻叫 `ROOM_L_10/L_15/L_43/L_47/L_38`；4 处各写一遍几何容差匹配（`doorways_match`、`front_stations_complete`、宽域沿搜索、锁定家族） | run46/48/50/55/62/64 |
| **同一概念多视图** | 路由用去斑图、校验用原始图；姿态用 metric 与 IMU 两套；时间用仿真秒与真实秒两套 | run47/63/44/57 |
| **几何双 owner** | 探索器自算 `elevator_portal`/`virtual_isolation_door*`，电梯块也自算一份 | run62 |
| **活性非一等契约** | `_control`/`_plan` 抛异常导致线程静默死亡；状态事件驱动导致"沉默=停滞" | run44/57/60 |
| **参数蔓延** | `stop_distance` vs `motion_stop_distance` 误用；多个 `*_tolerance` 语义重叠 | run60 |
| **兜底链替代观测** | 相机兜底→巡游→门口接近→释放锁→BLOCKED，过滤器链可组合出"零候选" | run46/55/58/61/64 |
| **完成概念分散** | `completed_topologies`/`retired_topologies`/`state==COMPLETE`/`completed_front_sides` 四套 | run46/55/62 |

### 6.2.1 接口级缺陷清单（逐条代码核对）

| # | 缺陷 | 位置 | 影响 |
|---|---|---|---|
| 1 | **健康话题两端不接**：发布 `/simnav/lio_health`，订阅 `/simnav/localization_health` | `lio_health_monitor_node.py:16` ↔ `danger_detector_node.py:169` | 定位健康度永远 `STALE`，轨迹健康统计失效；感知失去唯一的定位质量信号 |
| 2 | **同名节点两套实现**：Hector 与 FAST-LIO 两个桥都 `init_node("localization_bridge")` 并发布同一组 `/simnav/odom`、`/simnav/world_pose` 与同名 TF | `localization_bridge_node.py:225`、`lio_localization_bridge_node.py:425` | 只能靠"哪个 launch 起了哪个"来区分，缺少显式的 front-end 选择契约 |
| 3 | **两个同名速度钳制函数** | `coverage_explorer_core.py:249`、`elevator_transition_core.py:316` | 同一任务级限速规则两份实现；`test_speed_clamp_core.py` 只覆盖前者 |
| 4 | **地板高度带规则两份**（核心默认 −0.2/+1.2，节点参数 −0.6/+1.2） | `danger_detector_core.py:303` vs `danger_detector_node.py:834` | 核心默认永不生效，行为由 launch 值决定，语义分散 |
| 5 | **launch 参数被静默忽略**：`portal_merge_radius` 从未被节点读取 | `stage_b_floor_explorer.launch:129` ↔ `coverage_explorer_node.py`（无读点） | launch 注释与实际行为相反；门合并始终按 planner 默认 2.5 m 运行 |
| 6 | **死亡代码**：`_check_fall` 在 `:664` 无条件返回，其后 `ROBOT_ON_GROUND` 分支与高度/跌落判据不可达 | `elevator_transition_node.py:664-696` | `fall_base_height`、`fall_drop_threshold` 等参数与 `fall_height_history` 采集均为无效负担 |
| 7 | **fire-and-forget 话题**：`/simnav/defer_zone`、`/simnav/local_loop_closure_request`、`/simnav/metric_corridor_constraint` 有订阅无发布；`/simnav/world_pose`、`/simnav/elevator_status`、`/simnav/map_floors`、`/map/floor_N`、`/simnav/coverage_status`、`/simnav/danger_confirmation_active` 有发布无订阅 | 多处 | 契约存在但无人履行；`danger_confirmation_active` 被硬编码为恒 `False` |
| 8 | **会话/状态复用未生效**：`reused_topology` 仍在发布，但探索器把它丢弃（`reused_portals = ()`） | `elevator_transition_node.py:1146` ↔ `coverage_explorer_node.py:735` | "上层复用下层门提示"的接口是活的但被短路，与"提而不照搬"的设计意图不一致 |
| 9 | **未使用/仅测试的纯函数**：`weighted_harmonic_coverage`、`distinct_room_count`、`floor_regions_complete`、`constrain_to_corridor` | `coverage_explorer_core.py`、`pose_continuity.py` | 生产路径用 `topology_completion_ready(..., 0)` 绕过了"区域完成"契约，文档与实现口径不同 |
| 10 | **节点级零测试**：21 个测试全部针对 core，`CMakeLists.txt` 未注册任何测试 | `src/simnav/test/` | 话题名、JSON 键、TF 这类集成契约（正是缺陷 1/5/7/8 的类型）无法被回归发现 |
| 11 | **跨模块状态几乎全部是 JSON 字符串**：`explorer_status`（约 90 键）、`elevator_status`（约 30 键）、`camera_coverage`、`danger_tracks/candidates`、回环事件、楼层上下文 | 多处 | 消费者用 `.get()` 取值、解析失败即静默返回 → **schema 变更会静默失效（fail-open）** |
| 12 | **相机作用域由三处驱动**：检测节点解析探索器 `topology_lock`、探索器自己的进门判定、`/simnav/room_entry` 触发重置 | `danger_detector_node.py:218-250,340-350`、`coverage_explorer_node.py:1915-2011` | 同一状态无单一 owner |

> 这 12 条是"模块化缺口"的具体形态。它们的共同点是：**接口存在但没有唯一 owner、或两端名字/口径不一致**——恰好对应 §6.4 要补的四项基础设施（身份、视图/时钟、心跳、状态机不变量）。

### 6.3 目标架构（7 块提案，来自 `SimEnv_模块化功能块设计_20260913.md`）

| 块 | 职责 | 识别？ |
|---|---|---|
| **A 定位** | 可用位姿与不确定度 | — |
| **B 房间区域** | 把地图切成走廊 + 小房间区域；区域成员判定进出；每区域覆盖/状态 | ❌（门=区域边界，无身份） |
| **C 电梯门户** | 大堂/电梯口识别、对准、进出轿厢、跨层；**几何唯一 owner** | ✅ |
| **D 规划** | 由 B 的区域阶段决定"下一个目标 + 一条路径" | — |
| **E 导航** | 单控制器、单限速权威，沿路径驱动、不碰撞 | — |
| **F 感知** | 房间内相机覆盖 + 红球检测（走廊不派相机前沿） | — |
| **G 交接** | 编排 A→F：完成→C→新楼层 | — |

**迁移顺序（按块推进，不设指标门槛）**：B0 走廊不计相机收益 → P0.1 进门改"位姿状态"（幂等）→ B1 区域表（`status` 取代 4 套完成集合 + 2 套状态）→ D1 目标由区域阶段决定、删替换族 → E1 单控制器（删走廊控制器拷贝与多余限速）→ C1 电梯几何归属收敛到 C → 清理台账。

### 6.4 需要补的四项基础设施（横切关注点收敛）

1. **持久身份登记表（doorway registry）**：按几何在首次观测时分配**持久 id**，之后邻近观测归并；所有消费者只认持久 id，禁止各自几何匹配。
2. **单一视图/时钟访问器**：地图、姿态、时钟、状态各一个访问器，禁止就地构造第二视图；启动时打印"本轮使用的视图"。
3. **组件心跳与显式监督**：每个长生命周期组件发布心跳 + 迭代计数 + 最近成功时间；"无进展"由不变量处理，"缺心跳"由监督层处理；判定与动作分离（verdict JSON 先行）。
4. **显式状态机 + 不变量测试**：楼层/房间/门生命周期写成显式状态机（状态、迁移、守卫、不变量）；用模型化测试枚举状态×事件；把"锁着房间 ⇒ 必有可派发动作"等不变量放进运行时自检。

### 6.5 面向性能的模块化注记（RTF 归因实测）

本轮对仿真成本做了隔离基准（同一世界 + 同一机器人，私有 ROS/Gazebo 端口，不影响在跑任务）：

| 配置 | 实测 RTF |
|---|---|
| 完整配置（Livox + RealSense + 机器人 + 物理） | **0.07–0.08**（本机限制下） |
| 关闭 RealSense，保留 Livox | 0.07–0.08 |
| **关闭 Livox，保留 RealSense** | **1.00（上限）** |
| 两者都关 | 1.00 |

结论：**实时因子几乎完全由 Livox 射线投射（10 Hz × 2400 条射线的 ODE 宽相位）决定**，与物理求解、深度相机渲染无关。这给模块化提出了一个直接的工程含义：**传感器仿真块（M3）的成本决定了整条验证链的时间预算**，因此"缩短射线/降低扫描密度/替换光线投射实现"属于**平台级优化**，必须在 M3 内部解决，而不应通过在 M4–M6 里加逻辑来"绕开慢仿真"。

---

## 7. 附录

### 7.1 目录速查

```text
SimEnv/
├── auto.sh                      # 基础环境入口（Gazebo + A1 + 控制器 + 门/电梯服务）
├── setup_multi_floor_simulation.py
├── src/
│   ├── building_generator_core/       # 场景生成（纯逻辑 + 导出 + 校验）
│   ├── building_generator_classic/    # 门/电梯运行期服务
│   ├── building_generator_interfaces/ # 服务定义
│   ├── building_obstacles/            # 场景片段与评估脚本
│   ├── unitree_guide/                 # A1 描述、junior_ctrl、启动文件
│   ├── Mid360_imu_sim/                # Livox 仿真插件与扫描模式
│   ├── FAST_LIO/                      # LiDAR-惯性里程计
│   └── simnav/                        # 本项目算法
│       ├── launch/  (9 个 launch)      # localization / two_floor_support / floor_explorer / behavior ...
│       ├── scripts/ (13 个节点/核心)    # 见 §1.3
│       ├── rviz/stage_b.rviz
│       └── test/    (21 个离线测试)
├── team_scripts/                # runner、supervisor、看门狗、遥测、校验、专项测试
├── generated_building/          # 本轮场景工件（team_scene_info 为唯一公开件）
├── results/                     # detected_danger.json / danger_truth.json / evaluation_result.json
├── logs/<run>/                  # result.json、telemetry.csv、elevator_phase.csv、各节点日志
├── docs/                        # 规则、接口、评估、设计文档
└── config/                      # 冻结清单
```

### 7.2 参数总表（关键项）

**探索器（`stage_b_floor_explorer.launch` / `stage_b_behavior.launch`）**

| 参数 | 默认 | 含义 |
|---|---|---|
| `motion_speed` / `max_linear_speed` | 0.60 / 0.60 | 巡航速度与全局上限 |
| `initial_forward_speed` / `_start_speed` / `_ramp_seconds` / `_settle_seconds` | 0.60 / 0.25 / 2.0 / 1.0 | 起始直行与斜坡 |
| `initial_forward_distance` / `initial_centering_start_distance` | 14.5 / 10.5 | 起始前进距离与居中起点 |
| `turn_speed` | 0.65 | 转向角速度 |
| `transit_speed_cap` / `transit_jamb_zone` | 0.35 / 2.0 | 门口限速与作用区 |
| `motion_stop_distance` | 0.48 | 停止距离 |
| `target_tolerance` / `heading_tolerance` | 0.35 / 0.25 | 到达/航向容差 |
| `replan_period` | 1.0 | 规划周期（仿真秒） |
| `navigation_clearance` / `preferred_clearance` / `robot_radius` | 0.30 / 0.42 / 0.38 | 净空与半径 |
| `frontier_cluster_radius` / `frontier_revisit_radius` / `information_radius` | 0.45 / 0.70 / 2.5 | 前沿聚类/复访/信息半径 |
| `laser_coverage_target` / `camera_coverage_target` / `combined_coverage_target` | 0.95 / 0.85 / 0.84 | 覆盖目标 |
| `room_combined_coverage_target` / `camera_weight` | 0.84 / 0.95 | 房间目标与权重 |
| `completion_stable_duration` | 3.0 | 完成稳定时长 |
| `expected_rooms_per_floor` / `minimum_room_stations` | 4 / 1 | 房间数/门站数 |
| `stuck_target_seconds` / `stuck_target_cooldown_seconds` / `collision_block_seconds` | 5.0 / 120 / 45 | 卡住与冷却 |
| `unsafe_path_replan_cycles` / `unsafe_path_block_seconds` | 3 / 45 | 不安全路径处理 |
| `room_exhaust_grace_seconds` / `room_exhaust_min_gain` / `room_exit_requires_gain` | 60 / 1.5 / false | 房间收尾 |
| `danger_guidance_level` / `danger_early_exit_coverage` / `danger_candidate_min_hits` | 1 / 0.0 / 1 | 危险源引导 |
| `zone_*`（走廊纵向中点分区） | 由门站中点推导 | 先下半区后上半区 |
| `expected_rooms_per_floor`、`floor_end_corridor_tolerance` | 4 / 0.35 | 楼层收尾闸门 |

**电梯块（`stage_b_two_floor_support.launch`）**

| 参数 | 默认 | 含义 |
|---|---|---|
| `motion_speed` / `max_linear_speed` | 0.60 / 0.60 | 巡航与上限 |
| `crossing_speed` / `lobby_approach_speed` | 0.45 / 0.45 | 跨门槛/大堂接近（实测 0.35 会失败） |
| `elevator_approach_tolerance` | 0.80 | 接近容差 |
| `return_to_spawn_tolerance` | 0.45 | 返回出生点容差 |
| `max_route_retries` / `route_unreachable_timeout` / `route_accept_distance` | 40 / 60 / 1.2 | 路由重试 |
| `route_replan_period` | 2.0 | 路由重规划周期 |
| `fall_roll_limit_deg` / `fall_pitch_limit_deg` / `fall_baseline_window` / `fall_grace_seconds` | 30 / 30 / 20 / 15 | 倾倒判定 |
| `elevator_candidate_confirm_cycles` / `elevator_candidate_lateral_min|max` | 5 / 0.5 / 2.0 | 电梯候选确认 |
| `elevator_lobby_wall_offset` | 1.65 | 大堂墙偏移 |

**危险源（`stage_b_two_floor_support.launch` / `stage_b_behavior.launch`）**

| 参数 | 默认 | 含义 |
|---|---|---|
| `confirmation_frames` / `cluster_radius` | 3 / 0.75 | 多帧确认与空间聚类 |
| `moving_frequency` | 10.0 | 处理频率门控 |
| `floor_min_offset` / `floor_max_offset` | -0.6 / 1.2 | 地面高度过滤 |
| `camera_observation_resolution` / `_pixel_stride` / `_max_cells` | 0.25 / 32 / 30000 | 相机覆盖栅格 |
| `sphere_min_hits` / `sphere_review_hold` / `sphere_process_period` | 3 / 2.0 / 0.5 | 球体确认 |

### 7.3 状态常量表

- **探索器拓扑状态**：`UNSEEN` / `ACTIVE` / `COVERED`（区域），配合 `topology_state_for_new_target` 限制目标类型。
- **电梯状态链**：见 §3.8.1（含 `TOP_FLOOR_COMPLETE`、`MISSION_FAULT`、`ROBOT_ROLLED`、`ROBOT_ON_GROUND`、`NO_STATUS` 及失败原因字符串）。
- **定位健康**：`/simnav/lio_health`（`GOOD` / 降级态），由 `minimum_effective_points=5` 等判据驱动。
- **任务级故障**：`/simnav/mission_fault`（字符串原因），由电梯块或危险源块发布。

### 7.4 离线测试清单（`src/simnav/test/`）

`test_coverage_explorer_core.py`、`test_corridor_zone_core.py`、`test_corridor_exhaustion_core.py`、`test_door_approach_core.py`、`test_door_crossing_offsets_core.py`、`test_doorway_speckle_core.py`、`test_front_station_completion_core.py`、`test_portal_merge_core.py`、`test_region_of_point_core.py`、`test_navigation_despeckle_core.py`、`test_unsafe_path_replan_core.py`、`test_verified_door_band_core.py`、`test_speed_clamp_core.py`、`test_elevator_transition_core.py`、`test_elevator_entry_recovery_core.py`、`test_danger_detector_core.py`、`test_lio_occupancy.py`、`test_localization_bridge_core.py`、`test_map_floors_core.py`、`test_map_views.py`、`test_script_hygiene_core.py`。

其中"把一次真实故障钉成回归"的代表性用例（论文/工程报告可用）：

| 测试 | 锁定的历史缺陷 |
|---|---|
| `test_corridor_exhaustion_core.py` | run17：走廊只认激光前沿，导致"相机未看的未完成房间"永远产不出目标 → 死锁 |
| `test_door_crossing_offsets_core.py` | run48/50：门洞 bin 漂移，窄域搜索无法穿越，必须窄搜后再宽搜（±2.5 m / 0.5 m 步长） |
| `test_doorway_speckle_core.py` | 占用图"occupied 永不释放"导致门被噪点堵死；只允许释放完全落在门掩膜内的 ≤2 格连通块 |
| `test_front_station_completion_core.py` | run46：门 id 漂移导致"首对房间永远记不上完成"，改为几何容差匹配（2.5 m） |
| `test_region_of_point_core.py` | run79：同一门在 9 个不同 id 下被当成 9 个房间 → 区域成员判定 + 去重 |
| `test_portal_merge_core.py` | run38：同一物理门在 0.5 m 分箱下被反复重新接近 |
| `test_navigation_despeckle_core.py` | run47：路由用去斑图、校验用原始图，合法路径被恒判不安全 |
| `test_elevator_entry_recovery_core.py` | run49：`ELEVATOR_PATH_BLOCKED_AFTER_0.93M` → 开门 + 进入重试预算 + 对准门控 |
| `test_map_floors_core.py` | run52：换层后仍用上一层地图规划（A* 失败 654 次） |
| `test_script_hygiene_core.py` | run44：`UnboundLocalError` 杀死控制定时器线程；静态 AST 检查 read-before-assign 与未定义 `self` 属性 |
| `test_speed_clamp_core.py` | 速度上限必须全局一致（0.60 m/s），所有指令出口统一钳制 |

**已知覆盖缺口**：21 个测试全部针对 core（纯逻辑），**没有任何测试实例化节点**（`CoverageExplorer`、`ElevatorTransition`、`DangerDetector`、`MapViewsNode`、两个定位桥），`CMakeLists.txt` 也未注册测试。因此话题名、JSON 键、TF 这类**集成契约**没有自动回归能力——§6.2.1 中的缺陷 1/5/7/8 正是这一缺口的产物。

### 7.5 运行命令

```bash
# 端到端（三层）
docker exec -w /workspace/SimEnv simenv-noetic bash -lc \
  './team_scripts/run_stage_b_seed.sh 20260902 2400 logs/stage_b_run three_floor'

# 只跑换层/收尾链（尾段专项）
docker exec -w /workspace/SimEnv simenv-noetic bash -lc \
  'TRANSITION_ONLY=1 ./team_scripts/run_stage_b_seed.sh 20260902 600 logs/transition two_floor'

# 离线核心测试 + 静态检查
docker exec -w /workspace/SimEnv simenv-noetic bash -lc './team_scripts/run_stage_b_offline_tests.sh'

# 危险源评估
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

---

### 附：本文档与其它文档的分工

| 文档 | 回答的问题 |
|---|---|
| **本文** | 系统由哪些块组成、每块怎么设计、模块化判据与缺口 |
| `比赛场景背景说明_论文用.md` | 场景/规则/平台/评分（论文用） |
| `SimEnv_模块化功能块设计_20260913.md` | 目标架构（7 块）与精简台账（删什么、合并什么） |
| `stage_b_recurring_difficulties.md` | 反复出现的困难及其方法层根因 |
| `stage_b_current_work.md` | 当前进行中的工作与机制清单 |
| `docs/reference.md`、`doors-and-elevator.md`、`sensors-and-topics.md`、`evaluation.md` | 接口/规则/评估的原始说明 |
