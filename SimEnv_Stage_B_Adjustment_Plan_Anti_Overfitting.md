# SimEnv 阶段 B 调整方案：单层四房稳定化与防止机制过度堆叠

> 适用工程：SimEnv  
> 官方仓库：https://gitee.com/guoyulun/SimEnv.git  
> 当前目标：阶段 B —— 第一层四个房间稳定自主访问、危险源检测与正式结果闭环  
> 当前工作分支：`two-floor-navigation`  
> 当前阶段原则：**先把单层做稳，再进入双层；先解决根因，再增加机制；禁止为了修一个现象不断叠加补丁。**

---

## 1. 阶段 B 的重新定义

阶段 B 不再追求“尽可能理解整层建筑”或“把未知区域全部探索完”。

阶段 B 只解决一个明确问题：

> **机器人进入第一层后，能够稳定沿主走廊发现四个真实小房间，逐个进入、完成房内危险源扫描、可靠退出，并在四房完成后发布 `FLOOR_COMPLETE`。**

阶段 B 不负责：

- 完整地图覆盖；
- 普通 Frontier Exploration；
- 楼梯大厅探索；
- 电梯内部探索；
- 未知空间最大信息增益；
- 自动上楼；
- 多楼层地图管理；
- 完整全楼任务调度。

这些内容全部留到单层稳定以后处理。

---

# 2. 当前阶段最重要的问题判断

当前系统已经证明：

- 单层四房主闭环是可行的；
- 固定场景中已经出现 `VISITED=4`、`floor_complete=true` 的成功结果；
- 当前瓶颈已经不是“机器人能不能进房”；
- 当前瓶颈是：
  1. Hector 长走廊和进房阶段定位退化；
  2. Livox 稀疏扫描导致门洞证据不稳定；
  3. Scan 与 Map 双门洞链存在候选管理重复；
  4. 一些状态为补偿定位误差不断增加恢复逻辑；
  5. 进房和退出仍较依赖固定时间；
  6. 不同 seed 下稳定性不足；
  7. 正式结果格式尚未完整闭环。

因此阶段 B 后续的核心任务不是继续增加探索功能，而是：

> **减少耦合、统一职责、保留必要的异构冗余、消除重复状态和重复候选管理，并逐步将定位问题从探索逻辑中剥离。**

---

# 3. 阶段 B 的总设计原则

后续任何代码调整都遵循以下原则。

## 3.1 不因为一个失败案例立即增加新机制

出现失败时，必须先判断属于哪一层：

```text
定位
↓
局部激光感知
↓
门洞证据
↓
候选融合
↓
状态机
↓
运动执行
↓
危险源检测
```

只有确认现有机制无法覆盖该失效模式时，才允许新增机制。

禁止出现：

```text
误门
→ 加一个规则
→ 新 seed 又误门
→ 再加一个规则
→ 新规则导致漏门
→ 再加补偿
```

这种无限补丁式开发。

---

## 3.2 保留异构冗余，但消除重复职责

Scan 门洞和 Map 门洞是互补关系，必须保留。

原因：

### Scan 能补 Map

当 Hector：

- 将真实门洞刷成连续占用墙；
- 地图发生局部拉伸；
- 横向漂移；
- 门口地图结构失真；

Scan 仍有机会通过实时侧向开放检测真实门。

### Map 能补 Scan

当 Livox：

- 正侧窄扇区没有有限回波；
- 某一帧非重复扫描模式没有打到门框；
- 门洞 open-close 证据晚于机器人通过门中心；
- 单帧局部扫描不完整；

Map 可以利用累计几何补充门洞证据。

因此：

> **Scan + Map 都保留，但二者只负责提供 Evidence，不再分别拥有完整候选生命周期。**

---

# 4. 阶段 B 新总体架构

推荐调整为：

```text
                              Livox
                    ┌───────────┴───────────┐
                    │                       │
                    ▼                       ▼
                定位后端                 /scan_2d
          Hector → 后续可换 LIO             │
                    │                       ▼
                    │               CorridorEstimator
                    │                       │
                    │             ┌─────────┴─────────┐
                    │             │                   │
                    │       ScanDoorDetector      走廊控制
                    │             │
                    │        DoorEvidence
                    │             │
                    │             ▼
                    │         DoorFusion
                    │             ▲
                    │        DoorEvidence
                    │             │
             /exploration_map → MapDoorDetector
                    │
                    ▼
          LocalizationHealthMonitor
                    │
                    ▼
             DoorCandidateManager
                    │
                    ▼
              SingleFloor FSM
                    │
        ┌───────────┼────────────┐
        │           │            │
   CORRIDOR      ENTER_ROOM   EXIT_ROOM
        │           │            │
        └───────────┴────────────┘
                    │
                 /cmd_vel
                    │
               Unitree A1


RealSense RGB + Depth
          │
          ▼
   DangerDetector
          │
          ▼
      world XYZ
          │
          ▼
     ResultWriter
```

---

# 5. 必须保留的机制

以下机制当前已经证明具有明确价值，不应因为“想简化系统”而删除。

## 5.1 Scan 门洞检测

保留：

- 左右侧窄扇区；
- 只统计有限回波；
- `inf` 只表示 unknown；
- `OPEN_START -> OPEN -> OPEN_END`；
- 必须看到闭合边后再计算完整门宽；
- 门宽约 `0.8 ~ 1.6 m`；
- 大厅、大开口直接排除；
- 机器人驶过门中心后才确认也允许。

---

## 5.2 Map 门洞检测

保留：

- 局部占据地图；
- wall-gap-wall；
- 完整两侧边缘；
- 小缝 closing；
- entrance gate；
- defer zone；
- 门前/门后自由栅格检查。

但 Map Detector 后续不再直接创建 `RoomCandidate`。

---

## 5.3 Exploration Map

继续保留，但重新定义用途：

> **它是门洞和任务语义辅助地图，不是 Frontier 目标地图。**

允许：

- 小缝闭运算；
- 入口 gate；
- defer zone；
- Map 门洞检测。

不再用于：

```text
哪里 unknown 最大
→ 就去哪里
```

---

## 5.4 定位失效停车

现有：

```text
/simnav/odom 超过 1 s 未更新
→ LOCALIZATION_STALE
→ zero cmd_vel
```

继续保留。

这是安全保护，不属于冗余。

---

## 5.5 门前显式状态机控制

继续保留：

```text
GO_TO_PRE_DOOR
ALIGN_TO_DOOR_NORMAL
DOOR_CROSSING
ROOM_SCAN
EXIT_ROOM
```

A1 并不是理想二维全向底盘，门口动作由显式 FSM 控制比完全交给 DWA 更合理。

---

# 6. 第一项核心重构：DoorFusion

当前 Scan 和 Map 都可以形成完整门候选，之后再依赖空间距离和法向去重。

阶段 B 调整后：

```text
ScanDoorDetector
       ↓
 DoorEvidence
       │
       ▼
   DoorFusion
       ▲
       │
 DoorEvidence
       ↑
MapDoorDetector
```

只有：

```text
DoorCandidateManager
```

有权创建：

```text
RoomCandidate
```

---

## 6.1 DoorEvidence 建议字段

```text
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
```

---

## 6.2 DoorHypothesis 建议字段

```text
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
```

---

## 6.3 融合原则

禁止简单使用：

```text
SCAN == YES
AND
MAP == YES
```

正确策略：

### SCAN_STRONG

满足：

- 走廊上下文成立；
- 完整 open-close；
- 门宽合法；
- 两侧边缘可靠。

则允许单独成立。

即使 Map 当前将该区域显示为实墙，也不能因此否决。

---

### MAP_STRONG

满足：

- wall-gap-wall 完整；
- 两侧边界可靠；
- 门宽合法；
- 局部走廊结构可信；
- 定位状态 `GOOD`。

允许单独成立。

如果 Scan 当前为 `UNKNOWN`，不构成否决。

---

### MEDIUM + MEDIUM

Scan 和 Map 都只有中等证据时：

```text
scan medium
+
map medium
→ confirm
```

用于处理 Livox 晚闭合、部分回波和地图不完整等情况。

---

# 7. 第二项核心重构：LocalizationHealthMonitor

当前系统只能检测“定位是否停止发布”。

阶段 B 需要增加：

```text
GOOD
DEGRADED
BAD
STALE
```

---

## 7.1 监测内容

至少监测：

- 单帧平移；
- 单帧角度变化；
- `/simnav/odom` 时间连续性；
- 短时间累计位移；
- 由命令速度推断的物理合理性；
- 是否发生异常 pose jump。

---

## 7.2 对门洞链的影响

### GOOD

```text
Scan 正常
Map 正常
```

### DEGRADED

```text
Scan 正常
Map 降权
```

Map 可以确认已有 hypothesis，但不优先独立新建。

### BAD

```text
Scan 继续工作
Map 禁止独立新建 hypothesis
```

因为此时累计地图坐标不再可靠。

### STALE

```text
STOP
```

不进入普通 recovery。

---

# 8. 第三项核心重构：统一 CorridorEstimator

当前关于“是不是主走廊”的判断分散在多个模块中。

后续统一为：

```text
CorridorEstimator
```

---

## 8.1 输出

建议输出：

```text
valid
confidence
axis_yaw
left_wall_distance
right_wall_distance
corridor_width
center_error
front_clearance
```

---

## 8.2 统一服务对象

所有以下模块使用同一走廊判断：

```text
ScanDoorDetector
MapDoorDetector
CorridorController
ExitDetector
EndOfCorridorDetector
```

禁止每个模块单独实现一套“像不像走廊”。

---

# 9. Map Detector 调整

Map Detector 保留，但减少对当前机器人横向位姿的敏感性。

旧思路：

```text
robot pose
↓
固定窄 corridor band
↓
找 gap
```

调整为：

```text
robot pose
↓
只用于截取较大局部 ROI
↓
在 ROI 中重新估计左右主墙
↓
建立 local corridor frame
↓
在真实墙面序列上寻找 gap
```

这样即使 Hector 存在：

```text
0.3 ~ 0.5 m
```

横向漂移，也不会立即导致地图门洞漏检。

---

# 10. Scan 晚闭合机制继续保留

门洞事件允许：

```text
OPEN_START
↓
机器人继续前进
↓
经过门中心
↓
OPEN_END
↓
完成完整宽度确认
```

此时创建：

```text
PENDING_BEHIND
```

候选即可。

不要求在机器人到达门中心之前完成确认。

已经存在的：

```text
驶过门
→ 180°
→ 使用前进步态回访
```

继续使用。

禁止重新引入倒车回访。

---

# 11. 阶段 B 停止探索其他走廊分支

当前阶段不再处理：

```text
发现侧向宽开放
→ 判断是否新走廊
→ 进入探索
```

阶段 B 主逻辑改成：

```text
CORRIDOR_END
        ↓
VISITED == expected_rooms_per_floor ?
        │
   ┌────┴────┐
   │         │
  YES        NO
   │         │
   ▼         ▼
COMPLETE   TURN_180
             ↓
       REVERSE_SWEEP
```

反向扫掠最多一次。

这样从任务逻辑层直接避免：

- 楼梯大厅；
- 电梯区域；
- 宽开放区域；

继续吸引机器人。

---

# 12. move_base / DWA 在阶段 B 中降级

代码和配置可以保留。

但 Stage B 的房间搜索过程不再依赖：

```text
Navfn
DWA
global planner
```

作为主行为控制。

Stage B 中：

```text
SingleFloor FSM
```

拥有 `/cmd_vel` 控制权。

move_base 后续主要服务：

- 楼层完成后前往楼梯 prepose；
- 第二阶段双层任务；
- 必要的较长距离目标导航。

这样避免当前阶段同时维护两套运动决策系统。

---

# 13. 进门判定调整

当前固定时间穿门继续保留为安全上限，但不再作为成功判据。

新逻辑：

```text
ALIGN
↓
DOOR_CROSSING
↓
执行 minimum_cross_time
↓
检测局部走廊结构消失
↓
持续若干 scan frame
↓
INSIDE_ROOM
```

同时：

```text
front_clearance < emergency_threshold
→ STOP
```

如果一直无法确认：

```text
max_cross_time
→ CROSSING_FAILED
```

原则：

> **成功由传感器事件决定，时间只负责限制最大动作持续时间。**

---

# 14. 退出判定调整

当前固定 `EXIT_TRANSLATING` 时间改为最大超时。

真正成功条件：

```text
CorridorEstimator.valid
+
corridor_width 合理
+
左右双墙重新稳定
+
axis_yaw 稳定
+
持续 0.5 ~ 1.0 sim s
```

然后：

```text
EXIT_SUCCESS
```

这样可以避免：

- 房内家具被误当对侧墙；
- 固定退出时间过短；
- 固定退出时间过长。

---

# 15. Recovery 统一

当前多个恢复预算合并为：

```text
RecoveryManager
```

建议第一版：

```text
door_attempt <= 2
exit_attempt <= 2
reverse_sweep <= 1
```

定位：

```text
BAD
→ 暂停 Map 新候选

STALE
→ STOP
```

不纳入普通重试。

---

# 16. 完成条件调整

当前阶段测试目标明确是一层四房。

使用参数：

```text
expected_rooms_per_floor = 4
```

禁止写死在状态机内部。

完成条件建议：

```text
unique_VISITED == expected_rooms_per_floor
+
当前没有 danger confirmation
+
2 ~ 3 s quiet window
→ FLOOR_COMPLETE
```

不再要求：

```text
所有历史假候选都必须终态
+
长时间等待无新候选
```

这样已经访问四个真实房间后，不会继续为了第五个误候选浪费时间。

---

# 17. 危险源检测调整

当前危险源检测主链继续保留：

```text
HSV
↓
轮廓
↓
圆度/宽高比
↓
Depth ROI
↓
world TF
↓
multi-frame confirm
↓
spatial clustering
```

阶段 B 不重新设计视觉算法。

仅调整：

1. `ROOM_SCAN` 直接消费可获得 RGB-D 帧，不额外追求高于传感器真实频率的处理频率；
2. 定位后端稳定后重新评估 `cluster_radius`；
3. 记录每条危险源轨迹坐标方差；
4. 将定位异常与危险源轨迹分裂关联记录。

---

# 18. 正式结果文件必须在阶段 B 闭环

增加独立：

```text
ResultWriter
```

内部调试数据：

```text
results/detected_danger_debug.json
```

正式评分：

```text
results/detected_danger.json
```

正式结构：

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

`exploration_time` 使用 ROS `/clock` 仿真时间。

---

# 19. Hector 的阶段 B 处理原则

不允许继续无限调 Hector 参数。

但也不在同一次整改中同时：

```text
改 Explorer
+
改门洞
+
改状态机
+
换定位
```

否则无法判断改善来源。

---

## B-1：先收敛行为架构

继续使用 Hector。

完成：

- DoorFusion；
- DoorCandidateManager；
- LocalizationHealthMonitor；
- CorridorEstimator；
- Map Detector ROI 重构；
- 进门事件判定；
- 退出走廊重捕获；
- RecoveryManager；
- 完成条件简化；
- ResultWriter。

然后重新跑当前已知 seed。

---

## B-2：定位后端整改

冻结 B-1 上层接口。

如果仍然出现：

- 明显长走廊纵向漂移；
- >1m 非物理 pose jump；
- Hector crash；
- 房内旋转后位置异常；

则开始替换定位后端。

目标接口保持：

```text
/simnav/odom
/simnav/world_pose
TF
```

上层 Explorer 不因为定位后端更换而重写。

优先方向：

```text
Livox + IMU LIO
```

---

## B-3：多 seed 稳定验收

至少使用 5 个 seed。

每轮必须记录：

```text
commit
seed
启动参数
仿真开始时间
仿真结束时间
VISITED
UNREACHABLE
door hypothesis 数量
每个 hypothesis 来源
scan/map 支持情况
进入次数
退出次数
reverse sweep 次数
最大 pose jump
定位状态变化
danger_count
RTF
结果 JSON
```

---

# 20. 阶段 B 失败分类表

以后任何失败首先归类，不允许直接改参数。

## A. 定位失败

表现：

- pose jump；
- Hector crash；
- 长走廊不累计进度；
- 房内旋转后坐标乱跳。

处理：

```text
先查定位
禁止改门宽阈值
```

---

## B. Scan 漏门

表现：

- 地图有清晰门；
- scan 没有完整 open-close。

处理：

```text
查有限回波比例
查 OPEN 生命周期
查角度扇区
禁止先扩大门宽范围
```

---

## C. Map 漏门

表现：

- scan 能看到真实门；
- map 是连续墙或检测带错位。

处理：

```text
查 Hector
查 local ROI
查主墙重估计
禁止增加第三套 detector
```

---

## D. 重复候选

处理：

```text
先查 DoorFusion / hypothesis 生命周期
禁止继续扩大空间去重半径
```

---

## E. 进门失败

处理顺序：

```text
门法向
↓
对准
↓
front clearance
↓
RL 实际速度
↓
INSIDE_ROOM 判定
```

禁止先调 Hector 地图。

---

## F. 退出失败

处理：

```text
退出方向
↓
局部障碍
↓
CorridorEstimator 是否重新成立
```

禁止简单增加退出时间作为第一解决方案。

---

# 21. 明确禁止的“钻牛角尖”行为

阶段 B 期间禁止以下开发方式：

### 1. 不断增加门洞阈值

例如：

```text
0.8-1.6
失败
→ 0.7-1.7
失败
→ 0.6-1.8
```

除非有多轮数据证明真实门宽超出当前范围。

---

### 2. 不因为一个 seed 失败就增加新的特殊分支

必须至少确认同类失败重复出现。

---

### 3. 不用场景真实房间坐标修正算法

禁止读取：

- layout metadata；
- world；
- danger truth；
- model states。

---

### 4. 不同时修改多个核心模块

每轮重构必须能回答：

> **这次修改到底想验证什么？**

---

### 5. 不继续增加第三套门洞 Detector

当前 Scan + Map 已经覆盖两类主要失效来源。

下一步重点是融合和定位，而不是更多 Detector。

---

### 6. 不用新的 Recovery 掩盖定位错误

如果根因是 pose jump：

```text
多重试一次
```

不是解决方案。

---

### 7. 不把“最终能完成”当成“稳定通过”

必须区分：

```text
最终完成
限时完成
误候选
定位异常
人工干预
```

---

# 22. 阶段 B 验收指标

阶段 B 只有达到稳定验收后才进入双层。

| 指标 | 目标 |
|---|---:|
| 第一层真实房间 | 4/4 |
| `floor_complete` | true |
| `UNREACHABLE` | 0 |
| 额外假门 | 0 |
| 单房进入尝试 | ≤ 2 |
| 退出尝试 | ≤ 2 |
| 反向回扫 | ≤ 1 |
| 定位节点 crash | 0 |
| >1m 非物理 pose jump | 0 |
| 人工干预 | 0 |
| 红方块误报 | 0 |
| 正式 JSON | 可直接评分 |
| 单层时间 | 优先 < 360 sim s |
| 理想时间 | < 300 sim s |
| 多 seed | 至少连续 5 个通过 |

---

# 23. Stage B 结束条件

只有以下条件全部满足，才允许进入双层：

```text
至少 5 个 seed
        ↓
4/4 rooms
        ↓
UNREACHABLE = 0
        ↓
FLOOR_COMPLETE
        ↓
无定位崩溃
        ↓
无大幅 pose jump
        ↓
危险源输出可直接评分
        ↓
无人干预
```

此时冻结：

```text
SingleFloorExplorer
```

后续双层只新增：

```text
FloorTransitionManager
```

逻辑：

```text
SingleFloorExplorer
        ↓
FLOOR_COMPLETE
        ↓
FloorTransitionManager
        ↓
楼梯
        ↓
下一层
        ↓
重新 ACQUIRE_CORRIDOR
        ↓
同一个 SingleFloorExplorer
```

禁止为了双层重新修改一套单层探索策略。

---

# 24. 阶段 B 最终原则

整个阶段 B 后续开发只围绕下面四句话：

> **第一，保留真正互补的机制，不因为“代码多”而盲目删除。**

> **第二，相同职责只能有一个管理者：双感知可以有，但候选管理只能有一个。**

> **第三，定位问题归定位层解决，不能继续通过 Explorer 补丁掩盖。**

> **第四，阶段 B 的结束标准不是“偶尔能四房”，而是“多 seed、固定时限、零人工、无假门、定位稳定、结果可直接评分”。**

只有达到这一状态，才进入下一阶段的双层探索。
